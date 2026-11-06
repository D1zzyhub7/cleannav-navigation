"""Configuration failure injection without ROS nodes, setup execution, or sockets."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import overlay_identity as identity


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    dependencies = {'ros_base_prefix': '/opt/ros/humble', 'ros_packages': ['ros_base'],
                    'cleannav_packages': {}}
    prefixes = {}
    for package in identity.PACKAGES:
        prefix = tmp_path / package
        prefix.mkdir()
        (prefix / 'payload').write_text('reviewed package bytes')
        prefixes[package] = str(prefix)
        dependencies['cleannav_packages'][package] = {
            'prefix_env': 'CLEANNAV_EXPECTED_' + package.removeprefix('cleannav_').upper() + '_PREFIX',
            'files': {'payload': identity.digest(prefix / 'payload')},
            'linked_python_sources': {},
        }
    for name in ('ros_setup', 'overlay_setup', 'local_setup'):
        (tmp_path / name).write_text('# audited setup only\n')
    (tmp_path / 'SOURCE_MANIFEST.json').write_text('{}')
    (tmp_path / 'RUNTIME_DEPENDENCIES.json').write_text(json.dumps(dependencies))
    config = {
        'schema_version': 1,
        **{name: str(tmp_path / name) for name in ('ros_setup', 'overlay_setup', 'local_setup')},
        'source_manifest_sha256': identity.digest(tmp_path / 'SOURCE_MANIFEST.json'),
        'runtime_dependencies_sha256': identity.digest(tmp_path / 'RUNTIME_DEPENDENCIES.json'),
        'source_inputs': {str(tmp_path / name): identity.digest(tmp_path / name)
                          for name in ('ros_setup', 'overlay_setup', 'local_setup')},
        'expected_prefixes': prefixes,
    }
    path = tmp_path / 'config.json'
    path.write_text(json.dumps(config))
    env = {'ROS_DISTRO': 'humble', 'ROS_DOMAIN_ID': '204', 'ROS_LOCALHOST_ONLY': '1',
           'CLEANNAV_RVIZ_OVERLAY_SETUP': config['local_setup']}
    env.update({binding['prefix_env']: prefixes[pkg] for pkg, binding in dependencies['cleannav_packages'].items()})
    monkeypatch.setattr(identity, 'HERE', tmp_path)
    monkeypatch.setattr(identity.integration, 'HERE', tmp_path)
    monkeypatch.setattr(identity.integration, 'verify_source_manifest', lambda: {})
    return config, path, env, prefixes


def test_six_cli_prefixes_and_selected_payload_hashes_pass(fixture):
    config, path, env, prefixes = fixture
    called = []
    def resolve(pkg):
        called.append(pkg)
        return prefixes[pkg]
    result = identity.verify_environment(path, environment=env, cli_resolver=resolve,
                                         ros_resolver=lambda _: '/opt/ros/humble')
    assert called == list(identity.PACKAGES)
    assert result['status'] == 'PASS' and result['selected_installed_file_count'] == 6
    assert result['dynamic_entry_calls'] == 0


@pytest.mark.parametrize('key,value', [('ROS_DISTRO', None), ('ROS_DISTRO', 'jazzy'),
                                       ('ROS_DOMAIN_ID', '205'), ('ROS_LOCALHOST_ONLY', '0')])
def test_environment_rejected_before_cli(fixture, key, value):
    _, path, env, _ = fixture
    env[key] = value
    def forbidden(_):
        pytest.fail('must reject before any package resolution')
    with pytest.raises(ValueError):
        identity.verify_environment(path, environment=env, cli_resolver=forbidden)


def test_wrong_cli_prefix_and_changed_payload_fail(fixture):
    _, path, env, prefixes = fixture
    with pytest.raises(ValueError, match='prefix mismatch'):
        identity.verify_environment(path, environment=env, cli_resolver=lambda _: '/wrong')
    (Path(prefixes['cleannav_navigation']) / 'payload').write_text('changed')
    with pytest.raises(ValueError, match='installed dependency changed'):
        identity.verify_environment(path, environment=env, cli_resolver=prefixes.__getitem__,
                                    ros_resolver=lambda _: '/opt/ros/humble')


def test_changed_setup_source_or_manifest_rejected(fixture):
    config, path, _, _ = fixture
    Path(config['overlay_setup']).write_text('changed')
    with pytest.raises(ValueError, match='setup input changed'):
        identity.verify_inputs(path)
    Path(config['overlay_setup']).write_text('# audited setup only\n')
    config['source_manifest_sha256'] = '0' * 64
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='SOURCE_MANIFEST'):
        identity.verify_inputs(path)


def test_wrong_selected_setup_or_expected_prefix_rejected(fixture):
    _, path, env, _ = fixture
    env['CLEANNAV_RVIZ_OVERLAY_SETUP'] = '/another/setup'
    with pytest.raises(ValueError, match='selected overlay'):
        identity.verify_environment(path, environment=env)
    env['CLEANNAV_RVIZ_OVERLAY_SETUP'] = json.loads(path.read_text())['local_setup']
    env.pop('CLEANNAV_EXPECTED_INTERFACES_PREFIX')
    with pytest.raises(ValueError, match='differs from reviewed'):
        identity.verify_environment(path, environment=env)


def test_missing_package_binding_rejected(fixture):
    config, path, _, _ = fixture
    config['expected_prefixes'].pop('cleannav_interfaces')
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='six contracted'):
        identity.load_config(path)


def test_cli_rejects_multiline_output_and_propagates_timeout():
    def multiline(*args, **kwargs):
        assert args[0] == ['ros2', 'pkg', 'prefix', 'cleannav_navigation']
        return subprocess.CompletedProcess(args[0], 0, '/one\n/two\n', '')
    with pytest.raises(ValueError, match='invalid ros2'):
        identity.resolve_cli('cleannav_navigation', multiline)
    def timed_out(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], kwargs['timeout'])
    with pytest.raises(subprocess.TimeoutExpired):
        identity.resolve_cli('cleannav_navigation', timed_out)


def test_bad_input_hash_prevents_any_setup_source(tmp_path):
    marker = tmp_path / 'source-was-run'
    source = tmp_path / 'setup'
    source.write_text('touch ' + str(marker) + '\n')
    config = {'schema_version': 1, 'ros_setup': str(source), 'overlay_setup': str(source),
              'local_setup': str(tmp_path / 'local.bash'),
              'source_inputs': {str(source): '0' * 64, str(tmp_path / 'local.bash'): '0' * 64},
              'expected_prefixes': {p: '/fake/' + p for p in identity.PACKAGES}}
    path = tmp_path / 'config.json'
    path.write_text(json.dumps(config))
    local = tmp_path / 'local.bash'
    local.write_text('export CLEANNAV_RVIZ_MACHINE_CONFIG=' + str(path) + '\nsource ' +
                     str(HERE / 'overlay_setup.template.bash') + '\n')
    result = subprocess.run(['bash', str(HERE / 'check_overlay_identity.sh'), str(local)],
                            capture_output=True, text=True, env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
    assert result.returncode != 0
    assert not marker.exists()
