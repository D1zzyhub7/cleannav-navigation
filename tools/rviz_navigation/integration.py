"""Source/installed-package bindings for the restored RViz composition.

Pure validation never launches nodes, creates authorization, or edits map assets.
"""
import hashlib
import importlib.util
import json
import os
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

HERE = Path(__file__).resolve().parent
NAV = HERE.parents[1]
WORKSPACE = NAV.parents[1]
PROFILE_CONTRACTS = {
    'patrol-a-v4': HERE/'map_contract_patrol_a_v4.json',
    'roof-lidar-360': HERE/'map_contract.json',
    'mcity-localization07': HERE/'map_contract_mcity_localization07.json',
    'mcity-localization07-canonical-debug': HERE/'map_contract_mcity_localization07_canonical_debug.json',
    'simple-four-wall-nohit-v1': HERE/'map_contract_simple_four_wall_nohit_v1.json',
    'simple-four-wall-nohit-v1-oracle-short-goal': HERE/'map_contract_simple_four_wall_nohit_v1_oracle_short_goal.json',
    'simple-four-wall-nohit-v1-oracle-static': HERE/'map_contract_simple_four_wall_nohit_v1_oracle_static.json',
    'simple-four-wall-nohit-v1-oracle-localization': HERE/'map_contract_simple_four_wall_nohit_v1_oracle_localization.json',
}


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_oracle_odometry_inputs(model_xacro, controller_yaml, runtime_dir):
    """Create run-private launch inputs that silence the wheel odometry chain."""
    model_xacro = Path(model_xacro).resolve(strict=True)
    controller_yaml = Path(controller_yaml).resolve(strict=True)
    runtime_dir = Path(runtime_dir).resolve(strict=True)

    model_text = model_xacro.read_text()
    replacements = (
        ("<remapping>ackermann_controller/odometry:=odom</remapping>",
         "<remapping>ackermann_controller/odometry:=/wheel/odom</remapping>"),
        ("<remapping>ackermann_controller/tf_odometry:=tf</remapping>",
         "<remapping>ackermann_controller/tf_odometry:=/wheel/tf</remapping>"),
    )
    for source, target in replacements:
        if model_text.count(source) != 1:
            raise ValueError("selected model lacks one exact Ackermann odometry remap")
        model_text = model_text.replace(source, target, 1)

    config = yaml.safe_load(controller_yaml.read_text())
    params = config.get("ackermann_controller", {}).get("ros__parameters")
    if not isinstance(params, dict) or params.get("enable_odom_tf") is not True:
        raise ValueError("selected wheel controller does not expose its expected TF switch")
    params["enable_odom_tf"] = False

    runtime_model = runtime_dir / "oracle_odometry_model.urdf.xacro"
    runtime_controller = runtime_dir / "oracle_odometry_controllers.yaml"
    runtime_model.write_text(model_text)
    runtime_controller.write_text(yaml.safe_dump(config, sort_keys=False))
    identity = {
        "mode": "oracle",
        "ground_truth": {
            "service": "/gazebo/get_entity_state",
            "entity": "cleannav_ackermann::base_footprint",
            "reference_frame": "world",
            "source_timestamp": "GetEntityState response.header.stamp from Gazebo simulation time",
        },
        "wheel_chain": {
            "odometry_topic": "/wheel/odom",
            "tf_topic": "/wheel/tf",
            "enable_odom_tf": False,
            "rear_axle_to_base_footprint_static_tf": False,
        },
        "authoritative_chain": {
            "odometry_topic": "/odom",
            "tf": "odom -> base_footprint",
            "publisher": "gazebo_oracle_odometry",
        },
        "inputs": {
            "model_xacro_source": str(model_xacro),
            "model_xacro_source_sha256": digest(model_xacro),
            "controller_yaml_source": str(controller_yaml),
            "controller_yaml_source_sha256": digest(controller_yaml),
            "model_xacro_runtime": str(runtime_model),
            "model_xacro_runtime_sha256": digest(runtime_model),
            "controller_yaml_runtime": str(runtime_controller),
            "controller_yaml_runtime_sha256": digest(runtime_controller),
        },
    }
    identity_path = runtime_dir / "oracle_odometry_identity.json"
    identity_path.write_text(json.dumps(identity, indent=2) + "\n")
    return {
        "model_xacro": str(runtime_model),
        "controller_yaml": str(runtime_controller),
        "identity_path": str(identity_path),
        "identity": identity,
    }


def verify_source_manifest(manifest_path=HERE/'SOURCE_MANIFEST.json'):
    data=json.loads(Path(manifest_path).read_text())
    for relative, expected in data['files'].items():
        rel=Path(relative)
        if rel.is_absolute() or '..' in rel.parts:
            raise ValueError('RViz source manifest path escapes worktree: '+relative)
        source=(NAV/rel).resolve(strict=True)
        if digest(source)!=expected:
            raise ValueError('RViz source identity mismatch: '+relative)
    # Isolated production sources are explicit workspace bindings, never ../ escapes.
    for relative,expected in data.get('workspace_files',{}).items():
        rel=Path(relative)
        if rel.is_absolute() or '..' in rel.parts or not str(rel).startswith(
                'cleannav-experiments/motion-admission-safety-closure-20261031/src/'):
            raise ValueError('unapproved isolated production source: '+relative)
        source=(WORKSPACE/rel).resolve(strict=True)
        if not source.is_relative_to(WORKSPACE) or digest(source)!=expected:
            raise ValueError('isolated production source identity mismatch: '+relative)
    return data


def profile_install_binding(package, binding, prefix, environment=None):
    """Select the installation binding declared for the active motion profile.

    The default binding stays authoritative for every profile without its own
    binding, so historical profiles keep their reviewed installation. A profile
    binding names the single prefix it authorizes, which keeps any other profile
    from reusing that installation.
    """
    env = os.environ if environment is None else environment
    bindings = binding.get('profile_bindings') or {}
    profile = env.get('CLEANNAV_RVIZ_MOTION_PROFILE')
    for name, candidate in bindings.items():
        if name != profile and candidate['prefix'] == prefix:
            raise ValueError(f'{package} installation is bound to motion profile '
                             f'{name!r}, not {profile!r}')
    selected = bindings.get(profile)
    if selected is None:
        return None, binding
    if selected['prefix'] != prefix:
        raise ValueError(f'{profile} requires the bound {package} installation: '
                         f'{selected["prefix"]}')
    return profile, selected


def verify_packages(resolve_prefix, environment=None):
    env = os.environ if environment is None else environment
    if env.get('ROS_DISTRO') != 'humble':
        raise ValueError('RViz integration requires final ROS_DISTRO=humble')
    records = {}
    manifest=json.loads((HERE/'RUNTIME_DEPENDENCIES.json').read_text())
    expected_ros_prefix=manifest['ros_base_prefix']
    for pkg in manifest['ros_packages']:
        actual=Path(resolve_prefix(pkg)).absolute()
        if str(actual)!=expected_ros_prefix:
            raise ValueError(f'{pkg} prefix mismatch: actual={actual}, expected ROS base={expected_ros_prefix}')
        records[pkg]=str(actual)
    for pkg, binding in manifest['cleannav_packages'].items():
        expected = env.get(binding['prefix_env'])
        actual = Path(resolve_prefix(pkg)).absolute()
        if not expected or str(actual) != expected:
            raise ValueError(f'{pkg} prefix mismatch: actual={actual}, expected={expected!r}')
        _, selected = profile_install_binding(pkg, binding, expected, env)
        for relative, sha in selected['files'].items():
            if digest(actual/relative) != sha:
                raise ValueError(f'installed dependency changed: {pkg}/{relative}')
        for link, sources in selected['linked_python_sources'].items():
            source = Path((actual/link).read_text().splitlines()[0])
            for relative, sha in sources.items():
                if digest(source/relative) != sha:
                    raise ValueError(f'linked Python dependency changed: {pkg}/{relative}')
        if env.get('CLEANNAV_RVIZ_MOTION_PROFILE')=='short-goal':
            sources=json.loads((HERE/'SOURCE_MANIFEST.json').read_text())
            for installed,source in sources.get('runtime_source_files',{}).get(pkg,{}).items():
                if digest(actual/installed)!=sources['workspace_files'][source]:
                    raise ValueError('short-goal requires rebuilt Admission consumer: '+pkg+'/'+installed)
        records[pkg] = str(actual)
    return records


def contract_path_for_profile(profile):
    try:
        return PROFILE_CONTRACTS[profile]
    except (KeyError, TypeError) as error:
        raise ValueError('unsupported RViz map profile') from error


def verify_profile_source_bindings(contract):
    """Bind the restored profile to source model, RViz config and prefix contract."""
    bindings = contract.get('source_bindings')
    if bindings is None:
        return {}
    dependency_path = HERE/'RUNTIME_DEPENDENCIES.json'
    dependency_data = json.loads(dependency_path.read_text())
    expected_packages = sorted(dependency_data['cleannav_packages'])
    if bindings.get('expected_prefix_packages') != expected_packages:
        raise ValueError('profile expected-prefix package set differs from runtime dependency contract')
    if bindings.get('runtime_dependencies_sha256') != digest(dependency_path):
        raise ValueError('profile runtime dependency contract SHA-256 mismatch')
    resolved = {}
    required = {'simulation_model_xacro', 'rviz_config'}
    for name in ('simulation_model_xacro', 'rviz_config', 'amcl_params_source',
                 'rtabmap_localization_launch', 'optimized_pose_source',
                 'initial_physical_truth'):
        item = bindings.get(name)
        if not isinstance(item, dict):
            if name in required:
                raise ValueError(f'profile source binding missing: {name}')
            continue
        relative = Path(item.get('path', ''))
        if not relative.parts or relative.is_absolute() or '..' in relative.parts:
            raise ValueError(f'profile source binding escapes worktree: {name}')
        root = item.get('root', 'navigation')
        if root not in ('navigation', 'workspace'):
            raise ValueError(f'unsupported profile source root: {name}')
        source = ((WORKSPACE if root == 'workspace' else NAV)/relative).resolve(strict=True)
        if digest(source) != item.get('sha256'):
            raise ValueError(f'profile source identity mismatch: {name}')
        resolved[name] = str(source)
    return resolved


def verify_map(directory, world, contract_path=None, *, profile='roof-lidar-360',
               odometry_mode=None):
    selected_path = Path(contract_path) if contract_path is not None else contract_path_for_profile(profile)
    contract = json.loads(selected_path.read_text())
    if contract.get('profile_id', profile) != profile:
        raise ValueError('map profile identity differs from the selected profile')
    directory = Path(directory).resolve(strict=True)
    if directory.name != contract['map_id']:
        raise ValueError('map directory identity differs from the audited map contract')
    if contract.get('map_directory_relative'):
        expected_directory = (WORKSPACE/contract['map_directory_relative']).resolve(strict=True)
        if directory != expected_directory:
            raise ValueError('map directory path differs from the selected profile contract')
    for name, sha in contract['files'].items():
        if Path(name).name != name or digest(directory/name) != sha:
            raise ValueError('map asset identity mismatch: '+name)
    world = Path(world).resolve(strict=True)
    if digest(world) != contract['world_sha256']:
        raise ValueError('world differs from the map generation world')
    if contract.get('world_relative'):
        expected_world = (WORKSPACE/contract['world_relative']).resolve(strict=True)
        if world != expected_world:
            raise ValueError('world path differs from the selected profile contract')
    if contract['scan_topic'] not in ('/scan', '/mapping/scan'):
        raise ValueError('unsupported or unknown localization scan source')
    if contract.get('odom_topic', '/odom') != '/odom':
        raise ValueError('unsupported map profile odometry topic')
    if 'odom_frame_id' not in contract:
        raise ValueError('map profile must declare its RTAB-Map odometry mode')
    odom_mode = contract.get('odom_mode', 'tf' if contract['odom_frame_id'] else 'topic')
    if odom_mode not in ('tf', 'topic'):
        raise ValueError('unsupported RTAB-Map odometry mode')
    if (odom_mode == 'tf') != bool(contract['odom_frame_id']):
        raise ValueError('map profile odometry mode conflicts with odom_frame_id')
    required_odometry_mode = contract.get('required_odometry_mode')
    if required_odometry_mode:
        if required_odometry_mode not in ('wheel', 'oracle'):
            raise ValueError('unsupported map profile required odometry source')
        if odometry_mode != required_odometry_mode:
            raise ValueError('requested odometry source conflicts with the selected map profile')
    geometry = contract.get('map_geometry')
    if geometry:
        occupancy = json.loads((directory/'occupancy_grid.json').read_text())
        map_yaml = yaml.safe_load((directory/'map.yaml').read_text())
        if (occupancy.get('frame_id') != geometry['frame_id']
                or occupancy.get('width') != geometry['width']
                or occupancy.get('height') != geometry['height']
                or map_yaml.get('origin') is None
                or len(map_yaml['origin']) != 3
                or len(geometry['origin']) != 3
                or len(occupancy.get('origin', [])) != 3
                or any(abs(float(a)-float(b)) > 1e-8
                       for a, b in zip(occupancy.get('origin', []), geometry['origin']))
                or any(abs(float(a)-float(b)) > 1e-8
                       for a, b in zip(map_yaml['origin'], geometry['origin']))
                or abs(float(occupancy.get('resolution', 0))-float(geometry['resolution'])) > 1e-8
                or abs(float(map_yaml.get('resolution', 0))-float(geometry['resolution'])) > 1e-8):
            raise ValueError('map profile geometry differs from OccupancyGrid or YAML')
    localization = contract.get('localization_contract')
    if localization:
        if (localization.get('database') != 'rtabmap.db'
                or localization.get('database_sha256') != contract['files'].get('rtabmap.db')
                or localization.get('rtabmap_config') != 'rtabmap_config.ini'
                or localization.get('rtabmap_config_sha256') != contract['files'].get('rtabmap_config.ini')
                or localization.get('rtabmap_params') != 'rtabmap_params.yaml'
                or localization.get('params_sha256') != contract['files'].get('rtabmap_params.yaml')
                or localization.get('frame_id') != contract.get('frame_id')
                or localization.get('scan_topic') != contract.get('scan_topic')
                or localization.get('scan_frame') != contract.get('scan_frame')
                or localization.get('odom_topic') != contract.get('odom_topic')
                or localization.get('odom_frame_id') != contract.get('odom_frame_id')
                or localization.get('odom_mode') != odom_mode
                or localization.get('required_odometry_mode') != required_odometry_mode
                or localization.get('initial_pose') != contract.get('localization_initial_pose')):
            raise ValueError('localization bundle/profile contract bindings disagree')
        map_to_odom = localization.get('map_to_odom', {})
        if (map_to_odom.get('parent_frame') != 'map'
                or map_to_odom.get('child_frame') != contract['odom_frame_id']
                or map_to_odom.get('publisher') != 'rtabmap localization'
                or map_to_odom.get('launch_publish_tf_map') is not True):
            raise ValueError('map-to-odom localization frame contract is inconsistent')
    amcl = contract.get('localization_runtime')
    if amcl:
        if (amcl.get('kind') != 'nav2_amcl'
                or amcl.get('package') != 'nav2_amcl'
                or amcl.get('node_name') != 'amcl'
                or amcl.get('map_to_odom_publisher') != 'amcl'
                or amcl.get('tf_broadcast') is not True
                or not isinstance(amcl.get('transform_tolerance_s'), (int, float))
                or amcl.get('transform_tolerance_s') < 0
                or amcl.get('uses_rtabmap_database') is not False
                or amcl.get('scan_topic') != contract.get('scan_topic')
                or amcl.get('base_frame_id') != contract.get('frame_id')
                or amcl.get('global_frame_id') != 'map'
                or amcl.get('odom_frame_id') != contract.get('odom_frame_id')
                or amcl.get('parameters_source_binding') != 'amcl_params_source'
                or amcl.get('initial_pose_parameter_source') != 'localization_initial_pose'
                or amcl.get('set_initial_pose') is not True):
            raise ValueError('AMCL localization runtime contract is inconsistent')
        if not contract.get('localization_initial_pose'):
            raise ValueError('AMCL profile lacks an evidence-bound initial pose')
        binding = contract.get('source_bindings', {}).get('amcl_params_source')
        if not isinstance(binding, dict):
            raise ValueError('AMCL parameter source binding is missing')
    verify_profile_source_bindings(contract)
    if contract.get('patrol'):
        patrol = contract['patrol']
        registry_path = (WORKSPACE/patrol['registry_relative']).resolve(strict=True)
        if digest(registry_path) != patrol['registry_sha256']:
            raise ValueError('patrol registry identity mismatch')
        if digest(WORKSPACE/patrol['goal_checker_source_relative']) != patrol['goal_checker_source_sha256']:
            raise ValueError('patrol checker configuration identity mismatch')
        registry = json.loads(registry_path.read_text())
        identity = patrol['map_identity']
        if (registry['map'] != identity or identity['map_id'] != contract['map_id']
                or identity['sha256'] != contract['files']['occupancy_grid.json']):
            raise ValueError('patrol registry/map identity mismatch')
        region = next(r for r in registry['regions'] if r['region_id'] == patrol['region_id'])
        route = next(r for r in region['routes'] if r['route_id'] == patrol['route_id'])
        if (patrol['laps'] != 1 or len(route['waypoints'])-1 != patrol['route_segments']
                or route['waypoints'][0] != route['waypoints'][-1]):
            raise ValueError('patrol route is not the bound single closed lap')
    return contract


def verify_localization_runtime(contract, resolve_prefix, installed_params_path):
    """Verify the profile-bound AMCL install and parameter file without launching it."""
    runtime = contract.get('localization_runtime')
    if not isinstance(runtime, dict):
        raise ValueError('selected map profile has no localization runtime contract')
    package = runtime['package']
    base_package = runtime['base_package']
    prefix = Path(resolve_prefix(package)).resolve(strict=True)
    base_prefix = Path(resolve_prefix(base_package)).resolve(strict=True)
    if prefix != base_prefix:
        raise ValueError('AMCL and lifecycle manager resolve to different ROS prefixes')
    package_xml = prefix/'share'/package/'package.xml'
    executable = prefix/runtime['executable_relative']
    if (digest(package_xml) != runtime['package_xml_sha256']
            or digest(executable) != runtime['executable_sha256']):
        raise ValueError('AMCL package or executable identity mismatch')
    try:
        version = ET.parse(package_xml).getroot().findtext('version')
    except (OSError, ET.ParseError) as error:
        raise ValueError('AMCL package version cannot be read') from error
    if version != runtime['package_version']:
        raise ValueError('AMCL package version differs from the profile contract')
    params = Path(installed_params_path).resolve(strict=True)
    binding = contract['source_bindings']['amcl_params_source']
    if digest(params) != binding['sha256']:
        raise ValueError('installed AMCL parameters differ from the profile-bound source')
    return {'package_prefix': str(prefix), 'base_package_prefix': str(base_prefix),
            'package_xml': str(package_xml), 'package_xml_sha256': digest(package_xml),
            'version': version, 'executable': str(executable),
            'executable_sha256': digest(executable), 'params': str(params),
            'params_sha256': digest(params)}


def verify_amcl_navigation_runtime(contract, environment=None):
    if contract.get('profile_id') != 'simple-four-wall-nohit-v1-oracle-short-goal':
        return None
    module = load_module(NAV/'tools/startup_smoke/amcl_diagnostic_profile.py',
                         'navigation_amcl_runtime_identity')
    identity = module.verify_selected_profile('amcl', os.environ if environment is None else environment)
    if identity is None or identity['profile_sha256'] != contract['diagnostic_profile_sha256']:
        raise ValueError('Oracle-AMCL navigation requires the bound isolated AMCL profile')
    runtime_identity=identity['localization_runtime_identity']
    requested=contract['localization_runtime']
    if any(requested.get(key)!=runtime_identity.get(key) for key in
           ('executable_sha256','package_prefix','base_package_prefix')):
        raise ValueError('AMCL navigation executable/prefix identity mismatch')
    if identity['localization_runtime_identity']['params_sha256'] != contract['source_bindings']['amcl_params_source']['sha256']:
        raise ValueError('AMCL navigation parameter identity mismatch')
    return identity


def profile_initial_pose(contract, requested_pose):
    """Use a map-bound native prior; never silently replace a verified prior."""
    profile_pose = contract.get('localization_initial_pose', '')
    requested_pose = requested_pose.strip()
    if profile_pose and requested_pose and requested_pose != profile_pose:
        raise ValueError('localization initial pose conflicts with the selected map profile')
    return profile_pose or requested_pose


def motion_profile_for_mode(profile, motion_profile):
    """Apply the 0.5 m/s Ackermann envelope only to historical RViz motion."""
    selected = deepcopy(profile)
    if motion_profile not in ('historical', 'patrol'):
        return selected
    radius = float(selected.get('min_turning_radius_m',
                                selected['vehicle']['min_turning_radius_m']))
    forward_speed = 0.3 if motion_profile == 'patrol' else 0.5
    angular_speed = forward_speed / radius
    selected['profile_id'] = 'rviz_historical_manual_050'
    selected['max_forward_linear_x'] = forward_speed
    selected['max_angular_z'] = angular_speed
    selected['safety']['max_forward_linear_x'] = forward_speed
    selected['safety']['max_angular_z'] = angular_speed
    selected['mppi']['vx_max'] = forward_speed
    selected['mppi']['wz_max'] = angular_speed
    return selected


def apply_profile_goal_tolerance(params_path, contract):
    """Patch only a profile that declares its own goal checker values.

    The shared Nav2 YAML stays at its default. Profiles without
    navigation_goal_tolerance keep the values already written by the
    costmap and MPPI profile steps.
    """
    tolerance = contract.get('navigation_goal_tolerance')
    if not tolerance:
        return
    document = yaml.safe_load(Path(params_path).read_text())
    checker = document['controller_server']['ros__parameters']['general_goal_checker']
    checker['xy_goal_tolerance'] = tolerance['xy_goal_tolerance_m']
    checker['yaw_goal_tolerance'] = tolerance['yaw_goal_tolerance_rad']
    Path(params_path).write_text(yaml.safe_dump(document, sort_keys=False))


def prepare_inputs(arg, logs, prefixes):
    """Reuse the installed localization asset validator/private-copy workflow."""
    directory = Path(arg('map_directory')).resolve(strict=True)
    map_profile = arg('map_profile')
    contract = verify_map(directory, arg('world'), profile=map_profile,
                          odometry_mode=arg('odometry_mode'))
    initial_pose = profile_initial_pose(contract, arg('localization_initial_pose'))
    sim = Path(prefixes['cleannav_simulation'])
    helper = load_module(sim/'share/cleannav_simulation/launch/localization_mode.py',
                         'cleannav_integrated_localization')
    values = dict(auto_start_task='false', database_path='', maps_root=str(directory.parent),
                  map_name=directory.name, localization_initial_pose=initial_pose,
                  world=arg('world'), campus_security_profile='', map_yaml_path='', nav2_params_file='')
    runtime, _ = helper.prepare(values.__getitem__, str(logs))
    # Reuse the saved-map static layer rewrite and the existing bounded profile.
    source_params = Path(prefixes['cleannav_navigation'])/'share/cleannav_navigation/config/nav2_params_ackermann.yaml'
    params = helper.freeze_costmap(source_params, runtime/'nav2_params.yaml')
    support = load_module(sim/'lib/cleannav_simulation/mapping_support.py', 'rviz_mapping_support')
    profile = motion_profile_for_mode(
        json.loads((HERE/'short_goal_profile.json').read_text()), arg('motion_profile'))
    support.apply_mppi_motion_profile(params, runtime/'nav2_params.yaml', profile)
    apply_profile_goal_tolerance(runtime/'nav2_params.yaml', contract)
    if arg('motion_profile') == 'patrol':
        patrol = contract['patrol']
        retained = yaml.safe_load((WORKSPACE/patrol['goal_checker_source_relative']).read_text())
        document = yaml.safe_load((runtime/'nav2_params.yaml').read_text())
        controller = document['controller_server']['ros__parameters']
        for checker in ('patrol_goal_checker', 'patrol_position_checker'):
            controller[checker] = retained['controller_server']['ros__parameters'][checker]
            if checker not in controller['goal_checker_plugins']:
                controller['goal_checker_plugins'].append(checker)
        document['cleannav_navigation_facade'] = retained['cleannav_navigation_facade']
        (runtime/'nav2_params.yaml').write_text(yaml.safe_dump(document, sort_keys=False))
    session = json.loads((runtime/'session.json').read_text())
    bindings = contract.get('source_bindings') or {}
    session.update(scan_topic=contract['scan_topic'], scan_frame=contract['scan_frame'],
                   map_profile=map_profile, map_contract_path=str(contract_path_for_profile(map_profile)),
                   map_contract_sha256=digest(contract_path_for_profile(map_profile)),
                   localization_initial_pose=initial_pose,
                   odom_topic=contract.get('odom_topic', '/odom'),
                   odom_frame_id=contract['odom_frame_id'],
                   odom_mode=contract.get('odom_mode', 'tf' if contract['odom_frame_id'] else 'topic'),
                   # The selected source, not the localizer, defines the /odom
                   # child frame the monitors and the guard must accept.
                   odometry_mode=arg('odometry_mode'),
                   patrol=contract.get('patrol'), spawn=contract.get('spawn'), spawn_delay_sec=contract.get('spawn_delay_sec'),
                   simulation_model_xacro=str((NAV/bindings['simulation_model_xacro']['path']).resolve())
                       if bindings else str(NAV/'cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro'),
                   rviz_config=str((NAV/bindings['rviz_config']['path']).resolve())
                       if bindings else str(HERE/'navigation.rviz'),
                   runtime_prefixes=prefixes, nav2_yaml_path=str(runtime/'nav2_params.yaml'))
    amcl_identity = verify_amcl_navigation_runtime(contract)
    if amcl_identity is not None:
        localizer = contract['localization_runtime']
        pose = [float(v) for v in initial_pose.split()]
        overrides = dict(use_sim_time=True, scan_topic=contract['scan_topic'],
            global_frame_id=localizer['global_frame_id'], odom_frame_id=localizer['odom_frame_id'],
            base_frame_id=localizer['base_frame_id'], tf_broadcast=localizer['tf_broadcast'],
            transform_tolerance=localizer['transform_tolerance_s'], set_initial_pose=True,
            **{'initial_pose.x':pose[0], 'initial_pose.y':pose[1],
               'initial_pose.z':pose[2], 'initial_pose.yaw':pose[5]})
        overrides.update(amcl_identity['amcl_parameter_overrides'])
        params_path = amcl_identity['localization_runtime_identity']['params']
        defaults = yaml.safe_load(Path(params_path).read_text())['amcl']['ros__parameters']
        for name in ('update_min_d','update_min_a','max_particles'):
            overrides[name]=defaults[name]
        expected_names=('use_sim_time','transform_tolerance','scan_topic','global_frame_id',
            'odom_frame_id','base_frame_id','tf_broadcast','scan_tf_wait_timeout','update_min_d','update_min_a','max_particles')
        session.update(localization_kind='amcl', amcl_overrides=overrides,
            amcl_expected_parameters={name:overrides[name] for name in expected_names},
            amcl_parameter_sources=dict(requested=overrides, yaml_defaults=defaults),
            amcl_runtime_identity=amcl_identity)
    (runtime/'session.json').write_text(json.dumps(session, indent=2)+'\n')
    return runtime, session, profile
