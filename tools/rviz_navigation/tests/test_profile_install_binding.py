"""Profile-scoped installation identity: both gates, one resolution rule.

Pure configuration tests. No ROS nodes, setup sourcing, goals or authorizations.
"""
import hashlib
import json
from pathlib import Path
import sys

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import integration
import navigation_supervisor as supervisor
import overlay_identity as identity
from supervisor_policy import ContractError

PACKAGES = ('cleannav_mission_manager', 'cleannav_safety_supervisor')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prefix_env(package):
    return 'CLEANNAV_EXPECTED_' + package.removeprefix('cleannav_').upper() + '_PREFIX'


@pytest.fixture
def installations(tmp_path):
    """Historical and isolated installations of the same two packages."""
    prefixes = {}
    for kind in ('historical', 'isolated'):
        for package in PACKAGES:
            prefix = tmp_path / kind / package
            prefix.mkdir(parents=True)
            (prefix / 'payload').write_text(kind + ' bytes')
            prefixes[kind, package] = str(prefix)
    return prefixes


def prefix_set(installations, kind):
    return {package: installations[kind, package] for package in PACKAGES}


def environment(prefixes, profile=None):
    env = {'ROS_DISTRO': 'humble'}
    env.update({prefix_env(package): value for package, value in prefixes.items()})
    if profile is not None:
        env['CLEANNAV_RVIZ_MOTION_PROFILE'] = profile
    return env


def resolved(prefixes):
    return lambda package: prefixes[package]


@pytest.fixture
def contract(tmp_path, monkeypatch, installations):
    dependencies = {'ros_base_prefix': '/opt/ros/humble', 'ros_packages': [],
                    'cleannav_packages': {}}
    for package in PACKAGES:
        isolated = installations['isolated', package]
        dependencies['cleannav_packages'][package] = {
            'prefix_env': prefix_env(package), 'linked_python_sources': {},
            'files': {'payload': digest(Path(installations['historical', package])/'payload')},
            'profile_bindings': {'short-goal': {
                'prefix': isolated, 'linked_python_sources': {},
                'files': {'payload': digest(Path(isolated)/'payload')}}},
        }
    (tmp_path / 'RUNTIME_DEPENDENCIES.json').write_text(json.dumps(dependencies))
    (tmp_path / 'SOURCE_MANIFEST.json').write_text(
        json.dumps({'schema_version': 1, 'files': {}, 'workspace_files': {},
                    'runtime_source_files': {}}))
    monkeypatch.setattr(integration, 'HERE', tmp_path)
    monkeypatch.setattr(supervisor, 'DEPENDENCIES', dependencies)
    return dependencies


@pytest.mark.parametrize('profile,kind,expected', [
    (None, 'historical', 'PASS'),
    ('historical', 'historical', 'PASS'),
    ('short-goal', 'isolated', 'PASS'),
    ('short-goal', 'historical', 'requires the bound'),
    ('none', 'isolated', 'bound to motion profile'),
])
def test_package_verifier_selects_one_installation_per_profile(
        contract, installations, profile, kind, expected):
    prefixes = prefix_set(installations, kind)
    env = environment(prefixes, profile)
    if expected == 'PASS':
        assert integration.verify_packages(resolved(prefixes), env) \
            == prefixes
    else:
        with pytest.raises(ValueError, match=expected):
            integration.verify_packages(resolved(prefixes), env)
    # The historical binding is the default and is never rewritten in place.
    assert contract['cleannav_packages']['cleannav_mission_manager']['files']['payload'] \
        == digest(Path(installations['historical', 'cleannav_mission_manager'])/'payload')



@pytest.mark.parametrize('profile,kind,expected', [
    (None, 'historical', 'PASS'),
    ('historical', 'historical', 'PASS'),
    ('short-goal', 'isolated', 'PASS'),
    ('short-goal', 'historical', 'requires the bound'),
    ('none', 'isolated', 'bound to motion profile'),
])
def test_supervisor_identity_uses_the_same_rule(
        contract, installations, profile, kind, expected):
    prefixes = prefix_set(installations, kind)
    env = environment(prefixes, profile)
    if expected != 'PASS':
        with pytest.raises(ContractError, match=expected):
            supervisor.installed_file_identity(prefixes, env)
        return
    records = supervisor.installed_file_identity(prefixes, env)
    bound = 'short-goal' if kind == 'isolated' else None
    assert {record['profile_binding'] for record in records.values()} == {bound}
    assert {record['prefix'] for record in records.values()} == set(prefixes.values())


@pytest.fixture
def overlay(tmp_path, monkeypatch, installations):
    defaults = {package: installations['historical', package] for package in identity.PACKAGES
                if package in PACKAGES}
    for package in identity.PACKAGES:
        if package not in defaults:
            prefix = tmp_path / 'historical' / package
            prefix.mkdir()
            (prefix / 'payload').write_text('historical bytes')
            defaults[package] = str(prefix)
    overrides = {package: installations['isolated', package] for package in PACKAGES}
    dependencies = {'ros_base_prefix': '/opt/ros/humble', 'ros_packages': [],
                    'cleannav_packages': {}}
    for package in identity.PACKAGES:
        binding = {'prefix_env': prefix_env(package), 'linked_python_sources': {},
                   'files': {'payload': digest(Path(defaults[package])/'payload')}}
        if package in overrides:
            binding['profile_bindings'] = {'short-goal': {
                'prefix': overrides[package], 'linked_python_sources': {},
                'files': {'payload': digest(Path(overrides[package])/'payload')}}}
        dependencies['cleannav_packages'][package] = binding
    (tmp_path / 'RUNTIME_DEPENDENCIES.json').write_text(json.dumps(dependencies))
    (tmp_path / 'SOURCE_MANIFEST.json').write_text('{}')
    for name in ('ros_setup', 'overlay_setup', 'local_setup'):
        (tmp_path / name).write_text('# audited setup only\n')
    config = {'schema_version': 1,
              **{name: str(tmp_path/name) for name in ('ros_setup', 'overlay_setup', 'local_setup')},
              'source_manifest_sha256': identity.digest(tmp_path/'SOURCE_MANIFEST.json'),
              'runtime_dependencies_sha256': identity.digest(tmp_path/'RUNTIME_DEPENDENCIES.json'),
              'source_inputs': {str(tmp_path/name): identity.digest(tmp_path/name)
                                for name in ('ros_setup', 'overlay_setup', 'local_setup')},
              'expected_prefixes': defaults,
              'profile_bindings': {'short-goal': {'expected_prefixes': overrides}}}
    path = tmp_path/'config.json'
    path.write_text(json.dumps(config))
    monkeypatch.setattr(identity, 'HERE', tmp_path)
    monkeypatch.setattr(identity.integration, 'HERE', tmp_path)
    monkeypatch.setattr(identity.integration, 'verify_source_manifest', lambda: {})
    base = {'ROS_DISTRO': 'humble', 'ROS_DOMAIN_ID': '204', 'ROS_LOCALHOST_ONLY': '1',
            'CLEANNAV_RVIZ_OVERLAY_SETUP': config['local_setup']}
    base.update({prefix_env(package): value for package, value in defaults.items()})
    return path, base, defaults, overrides



def verify(path, env, prefixes):
    return identity.verify_environment(path, environment=env,
                                       cli_resolver=resolved(prefixes),
                                       ros_resolver=lambda _: '/opt/ros/humble')


def test_overlay_environment_selects_one_prefix_set_per_profile(overlay):
    path, base, defaults, overrides = overlay
    report = verify(path, dict(base), defaults)
    assert report['motion_profile'] is None and report['profile_prefix_overrides'] == {}
    isolated = dict(base, CLEANNAV_RVIZ_MOTION_PROFILE='short-goal')
    isolated.update({prefix_env(package): value for package, value in overrides.items()})
    isolated_all = {**defaults, **overrides}
    report = verify(path, isolated, isolated_all)
    assert report['motion_profile'] == 'short-goal'
    assert report['profile_prefix_overrides'] == overrides
    # An unlisted profile may not resolve the isolated installation.
    with pytest.raises(ValueError, match='differs from reviewed'):
        verify(path, dict(base, **{prefix_env(package): value
                                   for package, value in overrides.items()}), isolated_all)
    # Nor may the bound profile fall back to the historical installation.
    with pytest.raises(ValueError, match='differs from reviewed machine configuration'):
        verify(path, dict(base, CLEANNAV_RVIZ_MOTION_PROFILE='short-goal'), defaults)


def test_reviewed_contract_binds_exactly_the_two_admission_consumers():
    dependencies = json.loads((HERE/'RUNTIME_DEPENDENCIES.json').read_text())['cleannav_packages']
    bound = {package: binding['profile_bindings']
             for package, binding in dependencies.items() if binding.get('profile_bindings')}
    assert set(bound) == set(PACKAGES)
    for package, bindings in bound.items():
        assert set(bindings) == {'short-goal'}
        isolated = bindings['short-goal']
        assert set(isolated['files']) == set(dependencies[package]['files'])
        assert isolated['linked_python_sources'] == {}
        assert str(Path(isolated['prefix']).resolve(strict=True)) == isolated['prefix']
        assert isolated['files'] != dependencies[package]['files']
    config = json.loads((Path(integration.WORKSPACE)/'cleannav-local/rviz-navigation/'
                         'overlay_config.json').read_text())
    assert config['profile_bindings']['short-goal']['expected_prefixes'] == {
        package: bound[package]['short-goal']['prefix'] for package in PACKAGES}
    assert set(config['expected_prefixes']) <= set(dependencies)


def test_profile_environment_entry_matches_the_contract():
    """The shell entry and the JSON contract cannot drift apart."""
    dependencies = json.loads((HERE/'RUNTIME_DEPENDENCIES.json').read_text())['cleannav_packages']
    bound = {package: binding['profile_bindings']['short-goal']
             for package, binding in dependencies.items() if binding.get('profile_bindings')}
    entry = Path(integration.WORKSPACE)/'cleannav-local/rviz-navigation/' \
        'motion-admission-20261031/environment.bash'
    text = entry.read_text()
    assert '${CLEANNAV_RVIZ_MOTION_PROFILE:-none} == short-goal' in text
    sourced = {line.split('source ', 1)[1].strip() for line in text.splitlines()
               if line.startswith('  source ')}
    assert sourced == {str(Path(binding['prefix']).parent/'local_setup.bash')
                       for binding in bound.values()}
