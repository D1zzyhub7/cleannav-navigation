#!/usr/bin/env python3
"""单构建 supervisor；仅向自己创建的独立 session/PGID 发信号。"""
import argparse
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time

TOOLS = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("CLEANNAV_WORKSPACE_ROOT", TOOLS.parents[2])).resolve()
SOURCE_ONLY_RELEASE = False
EVIDENCE = TOOLS.parent / 'safe-build-evidence'
LOCK = ROOT / 'cleannav-build/locks/build.lock'
NAV = ROOT / 'cleannav-worktrees/pc-sim-v2-mapping-localization/src/cleannav-navigation'
CANDIDATE_UNDERLAY = None
# 唯一允许隔离构建的 Admission 消费端：两个 ament_python 包，白名单之外一律拒绝。
MOTION_ADMISSION_SOURCE = ROOT/'cleannav-experiments/motion-admission-safety-closure-20261031/src'
MOTION_ADMISSION_PACKAGES = {
    'cleannav_mission_manager': 'cleannav-mission-manager',
    'cleannav_safety_supervisor': 'cleannav_safety_supervisor',
}
MOTION_ADMISSION_DEST = ROOT/'cleannav-build/step3-2-f/review-fixes-motion-admission-20261031'
MOTION_ADMISSION_PACKAGE = None
GIB = 1024 ** 2  # /proc/meminfo 单位 KiB
COMPILERS = {'cc1plus', 'cc1', 'clang', 'clang++'}
BUILD_TOOLS = {'colcon', 'cmake', 'make', 'gmake', 'ninja', 'gcc', 'g++', 'c++', 'cc'}
RUNTIMES = {'gzserver', 'gzclient', 'gazebo', 'rviz2', 'rtabmap'}


def stamp():
    return dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def append(path, value):
    # 一条 O_APPEND 写入，工具子进程与 supervisor 日志可同时落盘。
    data = (json.dumps(value, ensure_ascii=False) + '\n').encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    try:
        os.write(fd, data)
    finally:
        os.close(fd)


def processes():
    result = {}
    uptime = float(Path('/proc/uptime').read_text().split()[0])
    hz = os.sysconf('SC_CLK_TCK')
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            raw = (p / 'stat').read_text()
            end = raw.rfind(')')
            fields = raw[end + 2:].split()
            args = (p / 'cmdline').read_bytes().split(b'\0')
            args = [a.decode(errors='replace') for a in args if a]
            pid = int(p.name)
            result[pid] = dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]),
                               sid=int(fields[3]), state=fields[0], comm=raw[raw.find('(')+1:end],
                               start_ticks=int(fields[19]), rss_kib=int(fields[21])*os.sysconf('SC_PAGE_SIZE')//1024,
                               cpu_ticks=int(fields[11])+int(fields[12]), args=args)
            age = max(.01, uptime-int(fields[19])/hz)
            result[pid].update(elapsed_s=round(age,3), cpu_pct_lifetime=round((int(fields[11])+int(fields[12]))/hz/age*100,2))
        except (OSError, ValueError, IndexError, ProcessLookupError):
            continue
    return result


def resource():
    m = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        k, v = line.split(':', 1)
        m[k] = int(v.split()[0])
    psi = {}
    for line in Path('/proc/pressure/memory').read_text().splitlines():
        k, *values = line.split()
        psi[k] = {a: float(b) for a, b in (v.split('=') for v in values)}
    return dict(mem_available_kib=m['MemAvailable'], mem_total_kib=m['MemTotal'],
                ram_used_kib=m['MemTotal']-m['MemFree']-m['Buffers']-m['Cached']-m['SReclaimable']+m['Shmem'],
                swap_total_kib=m['SwapTotal'], swap_free_kib=m['SwapFree'],
                swap_used_kib=m['SwapTotal']-m['SwapFree'], memory_psi=psi)


class Policy:
    """每 2 秒决策：C 连续 3 次增长；D full avg10>=1% 连续 3 次。"""
    def __init__(self):
        self.last = None
        self.trend = 0
        self.pressure = 0

    def check(self, r):
        if r['mem_available_kib'] < 1.5 * GIB:
            return 'A: MemAvailable < 1.5 GiB'
        if r['swap_used_kib'] > GIB:
            return 'B: SwapUsed > 1.0 GiB'
        self.pressure = self.pressure + 1 if r['memory_psi']['full']['avg10'] >= 1.0 else 0
        if self.last:
            rising = (r['swap_used_kib']-self.last['swap_used_kib'] >= 64*1024 and
                      self.last['mem_available_kib']-r['mem_available_kib'] >= 64*1024)
            self.trend = self.trend + 1 if rising else 0
        self.last = r
        if self.trend >= 3:
            return 'C: Swap +64 MiB and available -64 MiB per sample, 3 consecutive samples'
        if self.pressure >= 3:
            return 'D: memory PSI full avg10 >= 1%, 3 consecutive samples'
        return None


def bad_jobs(args):
    for i, a in enumerate(args):
        if a in ('-j', '--jobs', '--parallel'):
            if i+1 >= len(args) or args[i+1] != '1':
                return True
        elif re.match(r'^-j.+', a) and a != '-j1':
            return True
        elif a.startswith(('--jobs=', '--parallel=')) and a.split('=', 1)[1] != '1':
            return True
        elif a.startswith('--jobserver'):
            return True
    return False


def tool_main(kind, args):
    run = Path(os.environ['CLEANNAV_SAFE_RUN'])
    record = dict(time=stamp(), event='tool', kind=kind, pid=os.getpid(), pgid=os.getpgrp(),
                  argv=args, MAKEFLAGS=os.environ.get('MAKEFLAGS'),
                  GNUMAKEFLAGS=os.environ.get('GNUMAKEFLAGS'),
                  CMAKE_BUILD_PARALLEL_LEVEL=os.environ.get('CMAKE_BUILD_PARALLEL_LEVEL'),
                  CMAKE_GENERATOR=os.environ.get('CMAKE_GENERATOR'))
    append(run/'commands.jsonl', record)
    reason = None
    if kind in ('make', 'ninja', 'cmake') and bad_jobs(args):
        reason = 'explicit parallel argument rejected'
    # CMake try_compile 会清空 MAKEFLAGS。拒绝不安全值，再无条件强制单任务。
    flags = shlex.split(os.environ.get('MAKEFLAGS', ''))
    if bad_jobs(flags) or os.environ.get('GNUMAKEFLAGS'):
        reason = 'unsafe inherited make flags'
    if reason:
        write_json(run/'violation.json', dict(**record, reason=reason))
        return 86
    os.environ.update(MAKEFLAGS='-j1 -l1', GNUMAKEFLAGS='', CMAKE_BUILD_PARALLEL_LEVEL='1')
    if kind == 'compiler':
        lock = (run/'compiler.lock').open('a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            write_json(run/'violation.json', dict(**record, reason='parallel compiler launch rejected'))
            return 86
        os.set_inheritable(lock.fileno(), True)
        # FD 跨 exec 留给 compiler，退出自动释放；不会等待或隐藏并行违规。
        os.execvpe(args[0], args, os.environ)
    actual = json.loads((run/'real-tools.json').read_text())[kind]
    if kind == 'ninja':
        args = ['-j1'] + args
    if kind == 'make':
        args = ['-j1', '-l1'] + args
    append(run/'commands.jsonl', dict(time=stamp(), event='exec', kind=kind,
                                      actual=[actual]+args, pid=os.getpid(), pgid=os.getpgrp(),
                                      MAKEFLAGS=os.environ['MAKEFLAGS'], GNUMAKEFLAGS=os.environ['GNUMAKEFLAGS']))
    os.execvpe(actual, [actual]+args, os.environ)


def git_info(repo):
    def call(*args):
        p = subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=True, timeout=8)
        return dict(rc=p.returncode, output=p.stdout.strip(), stderr=p.stderr.strip())
    return dict(path=str(repo), head=call('rev-parse', 'HEAD'), status=call('status', '--porcelain=v1'),
                diff_sha256=hashlib.sha256(call('diff', '--binary')['output'].encode()).hexdigest())


def stop_group(child, run, reason):
    """只处理私有 PGID；不按进程名杀人。记录 TERM、宽限 5 秒、必要时 KILL。"""
    pgid = child.pid
    if pgid == os.getpgrp():
        raise RuntimeError('refuse to signal supervisor group')
    members = [p for p in processes().values() if p['pgid'] == pgid and p['sid'] == pgid]
    append(run/'events.jsonl', dict(time=stamp(), event='stop', reason=reason, pgid=pgid, members=members))
    if members:
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        live = [p for p in processes().values() if p['pgid'] == pgid and p['sid'] == pgid and p['state'] != 'Z']
        if not live:
            break
        time.sleep(.1)
    else:
        append(run/'events.jsonl', dict(time=stamp(), event='SIGKILL', pgid=pgid, survivors=live))
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    child.wait(timeout=3)


def wrappers(run):
    real = {k: shutil.which(k) for k in ('make', 'ninja', 'cmake')}
    if not real['make'] or not real['cmake']:
        raise RuntimeError('cmake / GNU Make missing')
    write_json(run/'real-tools.json', real)
    guard = run/'guard-bin'
    guard.mkdir()
    for kind in ('make', 'ninja', 'cmake', 'compiler'):
        p = guard/kind
        p.write_text('#!/bin/sh\nexec /usr/bin/python3 '+shlex.quote(str(Path(__file__).resolve()))+
                     ' --tool '+kind+' "$@"\n')
        p.chmod(0o755)
    return guard


def motion_admission_package(package):
    """只接受白名单内的两个 Admission 消费端包；其余包名直接拒绝。"""
    if package not in MOTION_ADMISSION_PACKAGES:
        raise RuntimeError('package is not a reviewed Admission consumer: ' + repr(package))
    root = MOTION_ADMISSION_SOURCE/MOTION_ADMISSION_PACKAGES[package]
    xml = root/'package.xml'
    if not xml.is_file():
        raise RuntimeError('reviewed Admission source package missing: ' + str(root))
    if '<build_type>ament_python</build_type>' not in xml.read_text():
        raise RuntimeError('Admission consumer must stay ament_python: ' + package)
    return root


def motion_admission_destination():
    """隔离前缀必须独立；已有内容但没有受审收据时拒绝复用。"""
    marker = MOTION_ADMISSION_DEST/'safe-build-success.json'
    if MOTION_ADMISSION_DEST.exists() and any(MOTION_ADMISSION_DEST.iterdir()) and not marker.is_file():
        raise RuntimeError('isolated Admission prefix has no reviewed build receipt: ' + str(MOTION_ADMISSION_DEST))
    return MOTION_ADMISSION_DEST


def package_source_root(pkg):
    """安装校验必须回到本次构建的真实源码根；历史目标行为不变。"""
    if MOTION_ADMISSION_PACKAGE == pkg:
        return MOTION_ADMISSION_SOURCE/MOTION_ADMISSION_PACKAGES[pkg]/pkg
    return (NAV.parent/'cleannav-mission-manager'/pkg
            if pkg == 'cleannav_mission_manager' else NAV/pkg/pkg)


def candidate_destination(target, candidate_root):
    if candidate_root is None:
        return None
    if target not in ('mppi', 'nav2-costmap', 'app-contract-hmi', 'mission-sequence-simulation-patrol', 'navigation-b-facade',
                      'mission-sequence-manager', 'mission-sequence-interfaces', 'navigation-algorithms', 'navigation-planner', 'navigation-executor', 'safety-supervisor', 'rtabmap-tf-conversions', 'rtabmap-tf-slam', 'amcl-tf2-diagnostics', 'amcl-node-diagnostics'):
        raise RuntimeError('--candidate-root is limited to reviewed HMI/simulation/navigation candidates')
    destination = Path(candidate_root).resolve()
    allowed = ROOT/'cleannav-build/step3-2-f'
    if destination.parent != allowed or not destination.name.startswith('review-fixes-'):
        raise RuntimeError('candidate root must be a review-fixes-* directory directly under step3-2-f')
    return destination


def plan(target, run, guard, diagnostic_source=None, candidate_root=None):
    if target == 'nav2-costmap':
        src = NAV
        pkg = 'nav2_costmap_2d'
        dest = Path(candidate_root)
    elif target in ('amcl-tf2-diagnostics', 'amcl-node-diagnostics'):
        src = NAV
        pkg = 'tf2_ros' if target == 'amcl-tf2-diagnostics' else 'nav2_amcl'
        dest = ROOT/'cleannav-build/step3-2-f/review-fixes-amcl-private-buffer-20261030'
    elif target in ('rtabmap-tf-conversions', 'rtabmap-tf-slam'):
        src = NAV
        pkg = 'rtabmap_conversions' if target.endswith('conversions') else 'rtabmap_slam'
        dest = ROOT/'cleannav-build/step3-2-f/review-fixes-rtabmap-internal-tf-20261023'
    elif target == 'safety-supervisor':
        src = NAV
        pkg = 'cleannav_safety_supervisor'
        dest = ROOT/'cleannav-build/step3-2-f'
    elif target == 'smoke':
        src = EVIDENCE/'smoke-src'
        pkg = 'cleannav_safe_build_smoke'
        dest = ROOT/'cleannav-build/safe-build-smoke'
    elif target == 'simulation':
        src = NAV
        pkg = 'cleannav_simulation'
        dest = ROOT/'cleannav-build/step3-2-f'
    elif target in ('m0-planner', 'm0-facade', 'm0-mission'):
        src = NAV if target != 'm0-mission' else NAV.parent/'cleannav-mission-manager'
        pkg = {'m0-planner': 'cleannav_global_planner', 'm0-facade': 'cleannav_navigation',
               'm0-mission': 'cleannav_mission_manager'}[target]
        dest = ROOT/'cleannav-build/campus-navigation-v2-m0'
    elif target in ('mission-sequence-interfaces', 'mission-sequence-manager',
                    'mission-sequence-simulation',
                    'mission-sequence-simulation-patrol'):
        src = (
            NAV.parent/'cleannav-interfaces'
            if target == 'mission-sequence-interfaces'
            else NAV.parent/'cleannav-mission-manager'
            if target == 'mission-sequence-manager'
            else NAV
        )
        pkg = {
            'mission-sequence-interfaces': 'cleannav_interfaces',
            'mission-sequence-manager': 'cleannav_mission_manager',
            'mission-sequence-simulation': 'cleannav_simulation',
            'mission-sequence-simulation-patrol': 'cleannav_simulation',
        }[target]
        dest = (ROOT/'cleannav-build/step3-2-f/patrol-030-candidate'
                if target == 'mission-sequence-simulation-patrol' else
                ROOT/'cleannav-build/mission-sequence-region-candidate')
    elif target == 'app-contract-hmi':
        src = ROOT/'cleannav-hmi'
        pkg = 'cleannav_hmi_gateway'
        dest = ROOT/'cleannav-build/step3-2-f/app-contract-v1'
    elif target in ('navigation-planner', 'navigation-executor', 'navigation-facade',
                    'navigation-b-facade'):
        src = NAV
        pkg = {'navigation-planner': 'cleannav_global_planner',
               'navigation-executor': 'cleannav_path_executor',
               'navigation-facade': 'cleannav_navigation',
               'navigation-b-facade': 'cleannav_navigation'}[target]
        # 新候选只写独立overlay，禁止覆盖Campus已冻结安装产物。
        dest = ROOT/('cleannav-build/campus-navigation-b'
                     if target == 'navigation-b-facade'
                     else 'cleannav-build/campus-navigation-v2')
    elif target in ('navigation-algorithms', 'm0-algorithms'):
        if diagnostic_source is None:
            raise RuntimeError('navigation-algorithms requires --diagnostic-source')
        src = Path(diagnostic_source).resolve()
        if (not src.is_relative_to(ROOT/'cleannav-experiments')
                or not src.parent.name.startswith(('campus-navigation-v2-development-', 'campus-navigation-v2-m0-readiness-',
                                                   'cleannav-patrol-stability-root-cause-'))
                or src.name != 'algorithm-src'):
            raise RuntimeError('diagnostic source must be inside the unique navigation experiment')
        pkg = 'cleannav_nav2_offline_probe'
        if not (src/pkg/'CMakeLists.txt').is_file():
            raise RuntimeError('missing offline probe CMake package')
        dest = ROOT/('cleannav-build/campus-navigation-v2-m0-algorithms' if target == 'm0-algorithms'
                     else 'cleannav-build/campus-navigation-v2-algorithms')
    elif target == 'motion-admission-consumers':
        if MOTION_ADMISSION_PACKAGE is None:
            raise RuntimeError('motion-admission-consumers requires --motion-admission-package')
        src = motion_admission_package(MOTION_ADMISSION_PACKAGE)
        pkg = MOTION_ADMISSION_PACKAGE
        dest = motion_admission_destination()
    else:
        src = NAV if (NAV/'nav2_mppi_controller/CMakeLists.txt').is_file() else TOOLS.parent/'diagnostic-src'
        pkg = 'nav2_mppi_controller'
        dest = ROOT/'cleannav-build/step3-2-f-mppi-diagnostic'
        success = ROOT/'cleannav-build/step3-2-f/safe-build-success.json'
        if not success.exists() or json.loads(success.read_text()).get('exit_code') != 0:
            raise RuntimeError('Simulation must successfully finish before MPPI')
    dest = candidate_destination(target, candidate_root) or dest
    for cache in (dest/'build').glob('*/CMakeCache.txt'):
        text = cache.read_text()
        gen = re.search(r'^CMAKE_GENERATOR:INTERNAL=(.*)$', text, re.M)
        if not gen or gen.group(1) != 'Unix Makefiles':
            raise RuntimeError(f'unsupported cached generator: {cache}; preserve directory, refuse reuse')
    dest.mkdir(parents=True, exist_ok=True)
    marker = dest/'safe-build-success.json'
    if marker.exists():
        shutil.copyfile(marker, run/'previous-success.json')
    write_json(marker, dict(time=stamp(), exit_code=None, state='in-progress', evidence=str(run)))
    command = ['colcon', '--log-base', str(dest/'log'), 'build', '--base-paths', str(src),
               '--build-base', str(dest/'build'), '--install-base', str(dest/'install'),
               '--executor', 'sequential', '--parallel-workers', '1', '--packages-select', pkg]
    if target in ('mppi', 'nav2-costmap') or target.startswith(('navigation-', 'm0-', 'mission-sequence-', 'amcl-',
                                              'motion-admission-')):
        command += ['--allow-overriding', pkg]
    command += ['--cmake-args', '-GUnix Makefiles', '-DBUILD_TESTING=OFF',
                '-DCMAKE_MAKE_PROGRAM='+str(guard/'make'),
                '-DCMAKE_C_COMPILER_LAUNCHER='+str(guard/'compiler'),
                '-DCMAKE_CXX_COMPILER_LAUNCHER='+str(guard/'compiler'),
                '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON']
    if target == 'nav2-costmap':
        command += ['-DCLEANNAV_BUILD_SNAPSHOT_TESTS=ON']
    underlays = [Path('/opt/ros/humble/setup.bash')]
    if target != 'nav2-costmap':
        underlays.append(ROOT/'cleannav-build/pc-sim-v2/install/local_setup.bash')
    if target == 'mppi':
        underlays += [ROOT/'cleannav-build/step3-2-f/install/local_setup.bash']
    if target.startswith(('navigation-', 'm0-')):
        underlays += [ROOT/'cleannav-build/step3-2-f/install/local_setup.bash']
        own_setup = ROOT/(
            'cleannav-build/campus-navigation-v2-m0/install/local_setup.bash'
            if target.startswith('m0-') or target == 'navigation-b-facade'
            else 'cleannav-build/campus-navigation-v2/install/local_setup.bash'
        )
        if own_setup.is_file():
            underlays.append(own_setup)
    if target.startswith(('mission-sequence-', 'app-contract-hmi')):
        underlays += [ROOT/'cleannav-build/step3-2-f/install/local_setup.bash']
        if target in ('mission-sequence-manager', 'mission-sequence-simulation',
                      'mission-sequence-simulation-patrol', 'app-contract-hmi'):
            own_setup = ROOT/'cleannav-build/mission-sequence-region-candidate/install/local_setup.bash'
            if own_setup.is_file():
                underlays.append(own_setup)
    if SOURCE_ONLY_RELEASE:
        underlays = [Path("/opt/ros/humble/setup.bash")]
    if not all(p.is_file() for p in underlays):
        raise RuntimeError('underlay setup missing')
    if CANDIDATE_UNDERLAY is not None:
        underlays.append(CANDIDATE_UNDERLAY)
    # source 后再次设置，覆盖用户 shell/setup 对并发及 PATH 的潜在修改。
    shell = '\n'.join('source '+shlex.quote(str(p)) for p in underlays)
    shell += '\nexport MAKEFLAGS="-j1 -l1" GNUMAKEFLAGS="" CMAKE_BUILD_PARALLEL_LEVEL=1 CMAKE_GENERATOR="Unix Makefiles"'
    shell += '\nexport PATH='+shlex.quote(str(guard))+':"$PATH"'
    shell += '\npython3 -c '+shlex.quote("import json,os; json.dump({k:os.environ.get(k) for k in ['MAKEFLAGS','GNUMAKEFLAGS','CMAKE_BUILD_PARALLEL_LEVEL','CMAKE_GENERATOR','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH']},open(os.environ['CLEANNAV_SAFE_RUN']+'/environment.json','w'),indent=2)")
    shell += '\nexec '+shlex.join(command)
    write_json(run/'plan.json', dict(time=stamp(), target=target, package=pkg, source=str(src),
                                    dest=str(dest), underlay_order=[str(p) for p in underlays],
                                    command=command, generator='Unix Makefiles', git=[git_info(NAV),git_info(NAV.parents[1])]))
    (run/'actual-command.sh').write_text(shell+'\n')
    return ['bash', '-e', '-c', shell], dest, pkg


def validate(dest, pkg, run):
    if pkg == 'cleannav_hmi_gateway':
        return validate_python_package(dest, pkg, run)
    if pkg in ('cleannav_global_planner', 'cleannav_path_executor', 'cleannav_mission_manager', 'cleannav_safety_supervisor'):
        return validate_python_navigation(dest, pkg, run)
    prefix = dest/'install'/pkg
    index = prefix/'share/ament_index/resource_index/packages'/pkg
    rcfile = dest/'build'/pkg/'colcon_build.rc'
    libs = list((prefix/'lib').glob('*.so'))
    required = ['libnav2_costmap_2d_core.so', 'liblayers.so', 'libfilters.so'] if pkg == 'nav2_costmap_2d' else (
        ['libcleannav_laser_self_filter.so'] if pkg == 'cleannav_simulation' else (
        ['libmppi_controller.so', 'libmppi_critics.so'] if pkg == 'nav2_mppi_controller' else
        ['libcleannav_bounded_mppi_controller.so'] if pkg == 'cleannav_navigation' and
        (dest.name in ('campus-navigation-v2-m0', 'campus-navigation-b') or dest.name.startswith('review-fixes-')) else []))
    checks = dict(rc=rcfile.exists() and rcfile.read_text().strip() == '0', ament_index=index.is_file(),
                  package_xml=(prefix/'share'/pkg/'package.xml').is_file(),
                  manifest=(dest/'build'/pkg/'install_manifest.txt').is_file(),
                  libraries=all((prefix/'lib'/n).is_file() for n in required),
                  setup=(dest/'install/local_setup.bash').is_file())
    if pkg == 'cleannav_navigation' and (NAV/pkg/'src/navigation_boundary_layer.cpp').is_file():
        checks['polygon_boundary_plugin'] = (prefix/'lib/libcleannav_navigation_boundary_layer.so').is_file()
    executables = list((prefix/'lib'/pkg).glob('*'))
    if pkg == 'nav2_costmap_2d':
        checks['snapshot_regression'] = (prefix/'lib'/pkg/'snapshot_regression').is_file()
    if pkg == 'cleannav_simulation':
        checks['executable'] = (prefix/'lib'/pkg/'simulation_status_monitor').is_file()
    if pkg == 'cleannav_safe_build_smoke':
        checks['executable'] = (prefix/'lib'/pkg/'safe_smoke').is_file()
    if pkg == 'cleannav_navigation':
        installed = prefix/'lib'/pkg/'navigation_facade_node.py'
        checks['executable'] = installed.is_file() and os.access(installed, os.X_OK)
        checks['source_sha_matches'] = (installed.is_file() and
            hashlib.sha256(installed.read_bytes()).digest() ==
            hashlib.sha256((NAV/pkg/'navigation_facade_node.py').read_bytes()).digest())
    if pkg == 'cleannav_simulation' and (dest.name == 'mission-sequence-region-candidate' or dest.name.startswith('review-fixes-')):
        installed = prefix/'share/cleannav_simulation/launch/cleannav_full_sim.launch.py'
        source = NAV/pkg/'launch/cleannav_full_sim.launch.py'
        checks['sequence_launch_sha_matches'] = (
            installed.is_file() and source.is_file()
            and hashlib.sha256(installed.read_bytes()).digest()
            == hashlib.sha256(source.read_bytes()).digest()
        )
    if pkg == 'cleannav_simulation' and (dest.name == 'patrol-030-candidate' or dest.name.startswith('review-fixes-')):
        profile = prefix/'share'/pkg/'config/motion_profiles/patrol_030_candidate.json'
        source_profile = NAV/pkg/'config/motion_profiles/patrol_030_candidate.json'
        checks['patrol_profile_sha_matches'] = (
            profile.is_file() and source_profile.is_file()
            and hashlib.sha256(profile.read_bytes()).digest()
            == hashlib.sha256(source_profile.read_bytes()).digest()
        )
        installed_launch = prefix/'share'/pkg/'launch/mapping_mode.py'
        source_launch = NAV/pkg/'launch/mapping_mode.py'
        checks['mapping_launch_sha_matches'] = (
            installed_launch.is_file() and source_launch.is_file()
            and hashlib.sha256(installed_launch.read_bytes()).digest()
            == hashlib.sha256(source_launch.read_bytes()).digest()
        )
        installed_teleop = prefix/'lib'/pkg/'simulation_keyboard_teleop'
        source_teleop = NAV/pkg/'scripts/simulation_keyboard_teleop.py'
        checks['teleop_candidate_cap_sha_matches'] = (
            installed_teleop.is_file() and source_teleop.is_file()
            and hashlib.sha256(installed_teleop.read_bytes()).digest()
            == hashlib.sha256(source_teleop.read_bytes()).digest()
        )
    if pkg == 'tf2_ros':
        checks['diagnostic_library'] = (prefix/'lib/libtf2_ros.so').is_file()
        checks['diagnostic_test'] = (prefix/'lib/tf2_ros/private_buffer_test').is_file()
    if pkg == 'nav2_amcl':
        checks['executable'] = (prefix/'lib/nav2_amcl/amcl').is_file()
        checks['core_library'] = (prefix/'lib/libamcl_core.so').is_file()
    if pkg == 'cleannav_interfaces':
        python_roots = (
            list((prefix/'lib').glob('python*/site-packages/cleannav_interfaces'))
            + list((prefix/'local/lib').glob('python*/dist-packages/cleannav_interfaces'))
        )
        generated = (
            'msg/_task_sequence_step.py',
            'msg/_task_sequence_status.py',
            'srv/_submit_task_sequence.py',
        )
        checks['generated_sequence_interfaces'] = (
            len(python_roots) == 1
            and all((python_roots[0]/name).is_file() for name in generated)
        )
    if pkg == 'cleannav_nav2_offline_probe':
        checks['executable'] = (prefix/'lib'/pkg/'nav2_offline_probe').is_file()
    # 验证动态库可解析；不运行 ROS 节点。
    ldd = {}
    env = os.environ.copy()
    overlay_libs = [str(path) for path in (dest/'install').glob('*/lib')]
    if CANDIDATE_UNDERLAY is not None:
        overlay_libs += [str(path) for path in CANDIDATE_UNDERLAY.parent.glob('*/lib')]
    env['LD_LIBRARY_PATH'] = ':'.join(
        overlay_libs + [
            '/opt/ros/humble/lib',
            '/opt/ros/humble/lib/x86_64-linux-gnu',
            '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
            env.get('LD_LIBRARY_PATH', ''),
        ]
    )
    for p in libs:
        x = subprocess.run(['ldd', str(p)], capture_output=True, text=True, timeout=8, env=env)
        ldd[str(p)] = x.stdout+x.stderr
    checks['dynamic_dependencies'] = all('not found' not in v for v in ldd.values())
    commands = [json.loads(l) for l in (run/'commands.jsonl').read_text().splitlines()]
    checks['actual_make_seen'] = any(x.get('event') == 'exec' and x.get('kind') == 'make' for x in commands)
    caches = list((dest/'build'/pkg).glob('CMakeCache.txt'))
    checks['generator'] = bool(caches) and 'CMAKE_GENERATOR:INTERNAL=Unix Makefiles' in caches[0].read_text()
    checks['compiler_launcher_configured'] = bool(caches) and str(run/'guard-bin/compiler') in caches[0].read_text()
    # 增量构建可以没有 compiler 调用；仍要求配置 gate 且实际 Make 已受约束。
    compiler_calls = [x for x in commands if x.get('kind') == 'compiler']
    dependency_prefixes = {}
    if caches:
        for line in caches[0].read_text().splitlines():
            m = re.match(r'([\w]+_DIR):PATH=(.*)',line)
            if m:
                dependency_prefixes[m.group(1)] = m.group(2)
    xml = (prefix/'share'/pkg/'package.xml').read_text() if checks['package_xml'] else ''
    version = re.search(r'<version>(.*?)</version>', xml)
    write_json(run/'artifacts.json', dict(checks=checks, package_prefix=str(prefix), version=version.group(1) if version else None,
                                        libraries=[str(p) for p in libs], executables=[str(p) for p in executables], ldd=ldd,
                                        compiler_launches=len(compiler_calls), dependency_prefixes=dependency_prefixes))
    return all(checks.values())


def validate_python_navigation(dest, pkg, run):
    """ament_python不执行Make/CMake；验证真实入口和逐文件安装SHA。"""
    prefix = dest/'install'/pkg
    executable = ('hybrid_planner_bridge_node' if pkg == 'cleannav_global_planner'
                  else 'cleannav_pc_demo' if pkg == 'cleannav_mission_manager' else 'safety_supervisor_node' if pkg == 'cleannav_safety_supervisor' else 'path_executor_node')
    paths = list((prefix/'lib').glob('python*/site-packages/'+pkg))
    hashes = {}
    source_root = package_source_root(pkg)
    for source in source_root.rglob('*.py'):
        installed = paths[0]/source.relative_to(source_root) if len(paths) == 1 else None
        hashes[str(source.relative_to(source_root))] = bool(installed and installed.is_file() and
            hashlib.sha256(source.read_bytes()).digest() == hashlib.sha256(installed.read_bytes()).digest())
    rcfile = dest/'build'/pkg/'colcon_build.rc'
    checks = dict(rc=rcfile.is_file() and rcfile.read_text().strip() == '0',
        ament_index=(prefix/'share/ament_index/resource_index/packages'/pkg).is_file(),
        package_xml=(prefix/'share'/pkg/'package.xml').is_file(),
        executable=(prefix/'lib'/pkg/executable).is_file() and os.access(prefix/'lib'/pkg/executable, os.X_OK),
        setup=(dest/'install/local_setup.bash').is_file(),
        source_sha_matches=bool(hashes) and all(hashes.values()))
    write_json(run/'artifacts.json', dict(package_prefix=str(prefix), checks=checks,
        installed_python_sources=hashes, build_type='ament_python',
        make_cmake='NOT_APPLICABLE: no compiler/CMake required; environment and resource protections unchanged'))
    return all(checks.values())


def validate_python_package(dest, pkg, run):
    """Validate the APP Gateway ament_python install without CMake artifacts."""
    prefix = dest/'install'/pkg
    source_root = ROOT/'cleannav-hmi/cleannav_hmi_gateway'
    roots = (
        list((prefix/'lib').glob('python*/site-packages/'+pkg))
        + list((prefix/'local/lib').glob('python*/dist-packages/'+pkg))
    )
    hashes = {}
    for source in source_root.rglob('*.py'):
        installed = roots[0]/source.relative_to(source_root) if len(roots) == 1 else None
        hashes[str(source.relative_to(source_root))] = bool(
            installed and installed.is_file()
            and hashlib.sha256(source.read_bytes()).digest()
            == hashlib.sha256(installed.read_bytes()).digest())
    executable = prefix/'lib'/pkg/'cleannav_hmi_gateway'
    checks = dict(
        ament_index=(prefix/'share/ament_index/resource_index/packages'/pkg).is_file(),
        package_xml=(prefix/'share'/pkg/'package.xml').is_file(),
        setup=(dest/'install/local_setup.bash').is_file(),
        executable=executable.is_file() and os.access(executable, os.X_OK),
        source_sha_matches=bool(hashes) and all(hashes.values()),
    )
    write_json(run/'artifacts.json', dict(
        package_prefix=str(prefix), checks=checks,
        installed_python_sources=hashes, build_type='ament_python',
        make_cmake='NOT_APPLICABLE: no C++ targets in this package'))
    return all(checks.values())


def main():
    global NAV, CANDIDATE_UNDERLAY, MOTION_ADMISSION_PACKAGE, SOURCE_ONLY_RELEASE
    if len(sys.argv) >= 3 and sys.argv[1] == '--tool':
        return tool_main(sys.argv[2], sys.argv[3:])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target', choices=['smoke', 'simulation', 'mppi', 'nav2-costmap', 'navigation-planner',
        'navigation-executor', 'navigation-facade', 'navigation-b-facade', 'navigation-algorithms',
        'm0-planner', 'm0-facade', 'm0-mission', 'm0-algorithms',
        'mission-sequence-interfaces', 'mission-sequence-manager',
        'mission-sequence-simulation', 'mission-sequence-simulation-patrol',
        'app-contract-hmi', 'safety-supervisor', 'rtabmap-tf-conversions', 'rtabmap-tf-slam',
        'amcl-tf2-diagnostics', 'amcl-node-diagnostics', 'motion-admission-consumers'],
        nargs='?', default='simulation')
    parser.add_argument('--diagnostic-source')
    parser.add_argument('--motion-admission-package',
                        help='one of the two reviewed Admission consumer packages; nothing else is buildable')
    parser.add_argument('--candidate-source-root', help='snapshot src directory under a unique experiment; build limits unchanged')
    parser.add_argument('--candidate-underlay', help='successful review-fixes-* overlay in this workspace')
    parser.add_argument('--candidate-root', help='isolated review-fixes-* install; all build safety limits remain enforced')
    parser.add_argument('--source-only-release', action='store_true',
                        help='build restored release packages against Humble and the explicit candidate underlay only')
    parser.add_argument('--timeout', type=int, default=1200)
    parser.add_argument('--probe', choices=['hold', 'memory', 'swap', 'trend', 'psi', 'parallel', 'tool-reject', 'compiler-gate', 'stubborn', 'success'])
    parser.add_argument('--tool', choices=['make', 'ninja', 'cmake', 'compiler'])
    opts, rest = parser.parse_known_args()
    if opts.tool:
        # 工具参数不经过构建入口 parser，保留 -j/-G 等原始顺序。
        i = sys.argv.index('--tool')
        return tool_main(sys.argv[i+1], sys.argv[i+2:])
    if rest or not 1 <= opts.timeout <= 3600:
        parser.error('unknown arguments or timeout outside 1..3600 seconds')
    if opts.source_only_release:
        if (opts.target not in ('nav2-costmap', 'mppi', 'mission-sequence-interfaces', 'navigation-b-facade',
                                'navigation-planner', 'navigation-executor')
                or not opts.candidate_source_root or not opts.candidate_root):
            parser.error('source-only release requires one restored release package and isolated source/destination')
        SOURCE_ONLY_RELEASE = True
    if opts.target == 'nav2-costmap' and not (opts.candidate_source_root and opts.candidate_root):
        parser.error('nav2-costmap requires an isolated candidate source and destination')
    if opts.diagnostic_source and opts.target not in ('navigation-algorithms', 'm0-algorithms'):
        parser.error('--diagnostic-source is only valid for navigation-algorithms')
    if opts.motion_admission_package is not None and opts.target != 'motion-admission-consumers':
        parser.error('--motion-admission-package is only valid for motion-admission-consumers')
    if opts.target == 'motion-admission-consumers':
        if opts.motion_admission_package is None:
            parser.error('motion-admission-consumers requires --motion-admission-package')
        if opts.diagnostic_source or opts.candidate_source_root or opts.candidate_root:
            parser.error('motion-admission-consumers accepts no candidate overrides')
        # 白名单在创建任何证据目录之前生效：非白名单包名不会留下半个 Run。
        try:
            motion_admission_package(opts.motion_admission_package)
        except RuntimeError as error:
            parser.error(str(error))
        MOTION_ADMISSION_PACKAGE = opts.motion_admission_package
    if opts.candidate_source_root:
        source_root = Path(opts.candidate_source_root).resolve(strict=True)
        if (not source_root.is_relative_to(ROOT/'cleannav-experiments')
                or source_root.name != 'src'
                or not ((source_root/('nav2_mppi_controller' if opts.target == 'mppi' else 'nav2_costmap_2d')/'CMakeLists.txt').is_file() if opts.target in ('mppi', 'nav2-costmap') else (source_root/('tf2_ros' if opts.target == 'amcl-tf2-diagnostics' else 'nav2_amcl')/'CMakeLists.txt').is_file() if opts.target.startswith('amcl-') else (source_root/'rtabmap_conversions/CMakeLists.txt').is_file() if opts.target.startswith('rtabmap-tf-') else (source_root/'cleannav-navigation/cleannav_navigation/CMakeLists.txt').is_file())):
            parser.error('candidate source must be an experiment snapshot src directory')
        NAV = source_root if opts.target in ('mppi', 'nav2-costmap') or opts.target.startswith(('rtabmap-tf-', 'amcl-')) else source_root/'cleannav-navigation'
    if opts.candidate_underlay:
        destination = Path(opts.candidate_underlay).resolve(strict=True)
        if (not destination.is_relative_to(ROOT/'cleannav-build/step3-2-f')
                or not destination.name.startswith('review-fixes-')
                or json.loads((destination/'safe-build-success.json').read_text()).get('exit_code') != 0):
            parser.error('candidate underlay must have a successful safe-build marker')
        CANDIDATE_UNDERLAY = destination/'install/local_setup.bash'
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    run = EVIDENCE/(dt.datetime.now().strftime('%Y%m%dT%H%M%S')+f'-{os.getpid()}-'+(opts.probe or opts.target))
    run.mkdir()
    print('EVIDENCE='+str(run), flush=True)
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    lock = LOCK.open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        write_json(run/'result.json', dict(time=stamp(), exit_code=75, reason='build/runtime lock busy', started=False))
        print('REFUSED: build/runtime lock busy', flush=True)
        return 75
    child = None
    reason = None
    started = time.monotonic()
    policy = Policy()
    samples = []
    fast_max_compilers = 0
    interrupted = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda s, _f: interrupted.append(s))
    try:
        if Path('/proc/1/comm').read_text().strip().startswith('codex'):
            raise RuntimeError('global /proc invisible in sandbox; run with full WSL process visibility')
        pre = resource()
        allp = processes()
        offenders = [p for p in allp.values() if p['pid'] != os.getpid() and
                     (p['comm'] in COMPILERS | BUILD_TOOLS | RUNTIMES)]
        write_json(run/'preflight.json', dict(time=stamp(), resources=pre, blockers=offenders,
                                             inherited={k:os.environ.get(k) for k in ['MAKEFLAGS','GNUMAKEFLAGS','CMAKE_BUILD_PARALLEL_LEVEL','CMAKE_GENERATOR']},
                                             boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
        if pre['mem_available_kib'] < 2.5*GIB or pre['swap_used_kib'] > GIB or pre['memory_psi']['full']['avg10'] >= 1:
            raise RuntimeError('preflight memory/swap/PSI threshold failed')
        if offenders:
            raise RuntimeError('existing unmanaged build or large runtime; no processes terminated')
        guard = wrappers(run)
        env = os.environ.copy()
        env.update(MAKEFLAGS='-j1 -l1', GNUMAKEFLAGS='', CMAKE_BUILD_PARALLEL_LEVEL='1', CMAKE_GENERATOR='Unix Makefiles',
                   CLEANNAV_SAFE_RUN=str(run), PATH=str(guard)+':'+env['PATH'])
        if opts.probe:
            write_json(run/'probe.json', dict(mode=opts.probe, synthetic=True, note='resource values simulated; no memory allocation stress'))
            if opts.probe == 'tool-reject':
                cmd = [str(guard/'make'), '-j16']
            elif opts.probe == 'parallel':
                sleeper = "import ctypes,time;ctypes.CDLL(None).prctl(15,b'cc1plus',0,0,0);time.sleep(30)"
                cmd = [sys.executable, '-c', 'import subprocess,time;'+
                       'subprocess.Popen('+repr([sys.executable,'-c',sleeper])+');'+
                       'subprocess.Popen('+repr([sys.executable,'-c',sleeper])+');time.sleep(30)']
            elif opts.probe == 'compiler-gate':
                code = ('import subprocess,time,fcntl;from pathlib import Path\n'
                        'cmd='+repr([str(guard/'compiler'),sys.executable,'-c','import time;time.sleep(30)'])+'\n'
                        'first=subprocess.Popen(cmd)\n'
                        'lock=Path('+repr(str(run/'compiler.lock'))+').open("a")\n'
                        'deadline=time.monotonic()+5\n'
                        'while time.monotonic()<deadline:\n'
                        ' try:\n  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.flock(lock,fcntl.LOCK_UN)\n'
                        ' except BlockingIOError:break\n'
                        ' time.sleep(.05)\n'
                        'second=subprocess.Popen(cmd);time.sleep(30)\n')
                cmd = [sys.executable, '-c', code]
            elif opts.probe == 'stubborn':
                cmd = [sys.executable,'-c','import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(30)']
            else:
                cmd = [sys.executable, '-c', 'import time;time.sleep('+('1' if opts.probe == 'success' else '30')+')']
            dest = pkg = None
        else:
            cmd, dest, pkg = plan(opts.target, run, guard, opts.diagnostic_source, opts.candidate_root)
        # 保留锁 FD 给 build：supervisor 意外退出时在途子进程仍持有锁。
        out = (run/'console.log').open('w')
        child = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=out, stderr=subprocess.STDOUT,
                                 start_new_session=True, pass_fds=(lock.fileno(),))
        write_json(run/'ownership.json', dict(supervisor_pid=os.getpid(), supervisor_pgid=os.getpgrp(),
                                              build_pid=child.pid, build_pgid=child.pid, build_sid=child.pid,
                                              command=cmd, time=stamp()))
        print(f'START pid={child.pid} pgid={child.pid}', flush=True)
        next_sample = 0
        count = 0
        while True:
            now = time.monotonic()
            rows = processes()
            owned = [p for p in rows.values() if p['pgid'] == child.pid and p['sid'] == child.pid and p['state'] != 'Z']
            cpp = [p for p in owned if p['comm'] in COMPILERS]
            fast_max_compilers = max(fast_max_compilers,len(cpp))
            runtime = [p for p in rows.values() if p['comm'] in RUNTIMES]
            if interrupted:
                reason = 'interrupted by signal '+str(interrupted[0])
            elif now-started > opts.timeout:
                reason = 'build timeout'
            elif (run/'violation.json').exists():
                reason = 'unsafe tool/parallel compiler invocation'
            elif len(cpp) > 1:
                reason = 'E: multiple owned compiler tasks'
            elif any(p['comm'] in {'make','gmake','ninja','cmake'} and bad_jobs(p['args'][1:]) for p in owned):
                reason = 'observed unsafe actual make/ninja/cmake command'
            elif runtime:
                reason = 'new unmanaged Gazebo/ROS runtime detected; stop owned build only'
            elif any(p['comm'] in COMPILERS | BUILD_TOOLS and p['pgid'] != child.pid for p in rows.values()):
                reason = 'new external build detected; stop owned build only'
            if now >= next_sample or child.poll() is not None or reason:
                actual = resource()
                decision = json.loads(json.dumps(actual))
                if opts.probe in ('memory','swap','trend','psi') and count >= 1:
                    if opts.probe == 'memory': decision['mem_available_kib'] = int(1.4*GIB)
                    if opts.probe == 'swap': decision['swap_used_kib'] = int(1.1*GIB)
                    if opts.probe == 'psi': decision['memory_psi']['full']['avg10'] = 2.0
                    if opts.probe == 'trend':
                        decision['swap_used_kib'] = count*80*1024
                        decision['mem_available_kib'] = int(4*GIB)-count*80*1024
                if not reason:
                    reason = policy.check(decision)
                sample = dict(time=stamp(), elapsed_s=round(now-started,3), **actual,
                              owned_compilers=len(cpp), global_cc1plus=sum(p['comm']=='cc1plus' for p in rows.values()),
                              make_ninja_count=sum(p['comm'] in {'make','ninja','gmake'} for p in owned),
                              tree_rss_kib=sum(p['rss_kib'] for p in owned), processes=owned,
                              simulated_decision=decision if opts.probe else None, trigger=reason)
                append(run/'samples.jsonl', sample)
                samples.append(sample)
                next_sample = now+2
                count += 1
            if reason:
                stop_group(child, run, reason)
                break
            if child.poll() is not None:
                # 子进程退出但有背景后代时，不允许释放锁并宣称成功。
                owned = [p for p in processes().values() if p['pgid'] == child.pid and
                         p['sid'] == child.pid and p['state'] != 'Z']
                if owned:
                    reason = 'build leader exited with surviving group members'
                    stop_group(child, run, reason)
                break
            time.sleep(.2)
        rc = 86 if reason else child.returncode
        if interrupted:
            rc = 128+interrupted[0]
        if rc == 0 and not opts.probe and not validate(dest, pkg, run):
            reason = 'artifact verification failed'
            rc = 87
        result = dict(time=stamp(), target=opts.target, probe=opts.probe, exit_code=rc,
                      build_exit_code=child.returncode, reason=reason, started=True,
                      elapsed_s=round(time.monotonic()-started,3), build_pid=child.pid, pgid=child.pid,
                      max_compilers=fast_max_compilers,
                      max_tree_rss_kib=max((s['tree_rss_kib'] for s in samples),default=0),
                      max_ram_used_kib=max((s['ram_used_kib'] for s in samples),default=0),
                      min_mem_available_kib=min((s['mem_available_kib'] for s in samples),default=0),
                      max_swap_used_kib=max((s['swap_used_kib'] for s in samples),default=0),
                      peak_note='2s samples, not continuous true peak; compiler count checked every 0.2s plus launch gate')
        write_json(run/'result.json', result)
        if dest:
            write_json(dest/'safe-build-success.json', dict(**result, evidence=str(run), package=pkg,
                                                           state='success' if rc == 0 else 'failed',
                                                           package_prefix=str(dest/'install'/pkg)))
        print(json.dumps(result, ensure_ascii=False), flush=True)
        return rc
    except Exception as exc:
        if child is not None:
            stop_group(child, run, 'supervisor exception: '+str(exc))
        write_json(run/'result.json', dict(time=stamp(), exit_code=88, reason=str(exc), started=child is not None))
        print('REFUSED/STOPPED: '+str(exc), file=sys.stderr, flush=True)
        return 88
    finally:
        # 内核 flock 自动释放；没有需删除的 PID 锁文件。
        lock.close()


if __name__ == '__main__':
    sys.exit(main())
