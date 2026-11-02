#!/usr/bin/env python3
"""Read-only setup/prefix identity validation. Never starts ROS nodes or a run."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import integration
from supervisor_policy import validate_environment

HERE = Path(__file__).resolve().parent
PACKAGES = (
    'cleannav_interfaces', 'cleannav_mission_manager', 'cleannav_navigation',
    'cleannav_rtabmap', 'cleannav_safety_supervisor', 'cleannav_simulation',
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_config(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema_version') != 1:
        raise ValueError('unsupported machine overlay configuration schema')
    if set(data.get('expected_prefixes', {})) != set(PACKAGES):
        raise ValueError('exactly six contracted custom package prefixes are required')
    for name in ('ros_setup', 'overlay_setup', 'local_setup'):
        value = data.get(name)
        if not isinstance(value, str) or not Path(value).is_absolute() or '\n' in value or '\t' in value:
            raise ValueError('invalid absolute setup path: ' + name)
        if value not in data.get('source_inputs', {}):
            raise ValueError('setup path is not bound by source_inputs: ' + name)
    for package, value in data['expected_prefixes'].items():
        if not isinstance(value, str) or not Path(value).is_absolute() or '\n' in value or '\t' in value:
            raise ValueError('invalid expected prefix: ' + package)
    for name, expected in data.get('source_inputs', {}).items():
        if not Path(name).is_absolute() or len(expected) != 64 or any(c not in '0123456789abcdef' for c in expected):
            raise ValueError('invalid setup input hash binding')
    for profile, binding in data.get('profile_bindings', {}).items():
        overrides = binding.get('expected_prefixes') if isinstance(binding, dict) else None
        if not overrides or not set(overrides) <= set(PACKAGES):
            raise ValueError('invalid motion profile prefix binding: ' + str(profile))
        for package, value in overrides.items():
            if not isinstance(value, str) or not Path(value).is_absolute() or '\n' in value or '\t' in value:
                raise ValueError('invalid motion profile expected prefix: ' + package)
    return data


def selected_prefixes(data, environment=None):
    """Reviewed expected prefixes for the active motion profile.

    A profile binding overrides only the packages it names; every other package
    keeps the reviewed default, so unlisted profiles cannot drift.
    """
    env = os.environ if environment is None else environment
    binding = (data.get('profile_bindings') or {}).get(env.get('CLEANNAV_RVIZ_MOTION_PROFILE'))
    prefixes = dict(data['expected_prefixes'])
    if binding is not None:
        prefixes.update(binding['expected_prefixes'])
    return prefixes


def verify_inputs(config_path):
    data = load_config(config_path)
    for path, expected in data['source_inputs'].items():
        if digest(path) != expected:
            raise ValueError('reviewed setup input changed: ' + path)
    for filename, key in (('SOURCE_MANIFEST.json', 'source_manifest_sha256'),
                          ('RUNTIME_DEPENDENCIES.json', 'runtime_dependencies_sha256')):
        if digest(HERE / filename) != data.get(key):
            raise ValueError('machine configuration requires reviewed ' + filename)
    integration.verify_source_manifest()
    return data


def resolve_cli(package, runner=subprocess.run):
    result = runner(['ros2', 'pkg', 'prefix', package], check=True,
                    capture_output=True, text=True, timeout=5)
    value = result.stdout.strip()
    if not value or '\n' in value or not Path(value).is_absolute():
        raise ValueError('invalid ros2 pkg prefix output: ' + package)
    return value


def verify_environment(config_path, *, environment=None, cli_resolver=resolve_cli,
                       ros_resolver=None):
    data = verify_inputs(config_path)
    env = os.environ if environment is None else environment
    if env.get('CLEANNAV_RVIZ_OVERLAY_SETUP') != data['local_setup']:
        raise ValueError('selected overlay setup differs from reviewed local_setup')
    dependencies = json.loads((HERE / 'RUNTIME_DEPENDENCIES.json').read_text())
    expected_prefixes = selected_prefixes(data, env)
    expected_env = {
        dependencies['cleannav_packages'][package]['prefix_env']: value
        for package, value in expected_prefixes.items()
    }
    for key, value in expected_env.items():
        if env.get(key) != value:
            raise ValueError(f'{key} differs from reviewed machine configuration')
    public_environment = validate_environment(env, dependencies)
    actual = {package: cli_resolver(package) for package in PACKAGES}
    for package, prefix in actual.items():
        if prefix != expected_prefixes[package]:
            raise ValueError(f'{package} prefix mismatch: actual={prefix!r}, expected={expected_prefixes[package]!r}')
    if ros_resolver is None:
        from ament_index_python.packages import get_package_prefix
        ros_resolver = get_package_prefix
    # Six custom values come from actual CLI output. ROS base lookups and
    # selected installed/linked source hashes reuse the formal entry contract.
    verified = integration.verify_packages(
        lambda package: actual[package] if package in actual else ros_resolver(package), env)
    return {
        'status': 'PASS', 'scope': 'configuration identity only; dynamic NOT RUN',
        'configuration': str(Path(config_path).resolve()),
        'configuration_sha256': digest(config_path),
        'motion_profile': env.get('CLEANNAV_RVIZ_MOTION_PROFILE'),
        'profile_prefix_overrides': {package: value for package, value in expected_prefixes.items()
                                     if data['expected_prefixes'][package] != value},
        'local_setup': data['local_setup'], 'local_setup_sha256': digest(data['local_setup']),
        'source_manifest_sha256': digest(HERE / 'SOURCE_MANIFEST.json'),
        'runtime_dependencies_sha256': digest(HERE / 'RUNTIME_DEPENDENCIES.json'),
        'setup_inputs_verified': len(data['source_inputs']),
        'environment': public_environment,
        'custom_prefixes_cli': actual,
        'ros_base_package_count': len(dependencies['ros_packages']),
        'all_verified_prefixes': verified,
        'selected_installed_file_count': sum(len(x['files']) for x in dependencies['cleannav_packages'].values()),
        'linked_source_file_count': sum(len(files) for x in dependencies['cleannav_packages'].values()
                                        for files in x['linked_python_sources'].values()),
        'formal_entry_source_verified': True,
        'dynamic_entry_calls': 0,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check-inputs', 'values', 'check-environment'))
    parser.add_argument('--config', required=True)
    parser.add_argument('--report')
    args = parser.parse_args(argv)
    if args.action == 'values':
        data = load_config(args.config)
        prefixes = selected_prefixes(data)
        print('\n'.join([data['ros_setup'], data['overlay_setup'],
                         *(prefixes[package] for package in PACKAGES)]))
        return 0
    if args.action == 'check-inputs':
        data = verify_inputs(args.config)
        print(f'SETUP_INPUT_HASHES_PASS count={len(data["source_inputs"])}')
        return 0
    report = verify_environment(args.config)
    text = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.report:
        with Path(args.report).open('x') as stream:
            stream.write(text)
    print(text, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
