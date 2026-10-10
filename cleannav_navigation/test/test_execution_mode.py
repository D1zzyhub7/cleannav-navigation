"""Evaluate launch conditions without starting any ROS processes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext


@pytest.mark.parametrize('mode', ['facade', 'legacy'])
def test_execution_modes_are_exclusive_and_keep_safety(monkeypatch, mode):
    source = (
        Path(__file__).resolve().parents[1]
        / 'launch' / 'cleannav_ackermann_navigation.launch.py'
    )
    spec = importlib.util.spec_from_file_location('ackermann_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: '/unused')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: SimpleNamespace(**kwargs))
    monkeypatch.setattr(
        module, 'IncludeLaunchDescription',
        lambda *args, **kwargs: SimpleNamespace(**kwargs),
    )
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['execution_mode'] = mode
    selected = [entity for entity in description.entities
                if isinstance(entity, SimpleNamespace)
                and (not hasattr(entity, 'condition') or entity.condition.evaluate(context))]
    facade = [entity for entity in selected
              if getattr(entity, 'name', '') == 'cleannav_navigation_facade']
    legacy = [entity for entity in selected if hasattr(entity, 'launch_arguments')
              and any(key in ('replan_period_sec', 'completion_window_xy_tolerance')
                      for key, _ in entity.launch_arguments)]
    assert len(facade) == (1 if mode == 'facade' else 0)
    assert len(legacy) == (0 if mode == 'facade' else 2)
    safety = [entity for entity in selected if hasattr(entity, 'launch_arguments')
              and any(key == 'block_all' for key, _ in entity.launch_arguments)]
    assert len(safety) == 1
    declarations = {entity.name: entity for entity in description.entities
                    if hasattr(entity, 'default_value')}
    assert declarations['execution_mode'].default_value[0].text == 'facade'
    assert declarations['safety_block_all'].default_value[0].text == 'true'
