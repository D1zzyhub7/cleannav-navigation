"""Ackermann P1-RV reverse 配置合同测试。"""

from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_FILE = PACKAGE_ROOT / 'config' / 'nav2_params_ackermann.yaml'
LAUNCH_FILE = (
    PACKAGE_ROOT / 'launch' / 'cleannav_ackermann_navigation.launch.py'
)


@pytest.fixture(scope='module')
def params():
    return yaml.safe_load(PARAMS_FILE.read_text(encoding='utf-8'))


def test_smac_uses_forward_biased_reverse_capable_model(params):
    planner = params['planner_server']['ros__parameters']['GridBased']

    assert planner['plugin'] == 'nav2_smac_planner/SmacPlannerHybrid'
    assert planner['motion_model_for_search'] == 'REEDS_SHEPP'
    assert planner['reverse_penalty'] == pytest.approx(2.0)
    assert planner['minimum_turning_radius'] == pytest.approx(1.12)


def test_mppi_allows_only_the_frozen_reverse_candidate_limit(params):
    controller = params['controller_server']['ros__parameters']['FollowPath']

    assert controller['motion_model'] == 'Ackermann'
    assert controller['vx_min'] == pytest.approx(-0.15)
    assert controller['vx_max'] == pytest.approx(0.50)
    assert controller['AckermannConstraints']['min_turning_r'] == pytest.approx(
        1.12)
    # P1-RV must not absorb the separate vx_std A/B tuning.
    assert controller['vx_std'] == pytest.approx(0.02)


def test_mppi_uses_humble_1_1_20_cusp_parameters(params):
    controller = params['controller_server']['ros__parameters']['FollowPath']

    assert controller['enforce_path_inversion'] is True
    assert controller['inversion_xy_tolerance'] == pytest.approx(0.20)
    assert controller['inversion_yaw_tolerance'] == pytest.approx(0.40)


def test_mppi_uses_humble_direction_critics_without_newer_mode(params):
    controller = params['controller_server']['ros__parameters']['FollowPath']
    path_angle = controller['PathAngleCritic']

    assert path_angle['forward_preference'] is False
    assert 'mode' not in path_angle
    assert controller['PathAlignCritic']['use_path_orientations'] is True
    assert 'PreferForwardCritic' in controller['critics']


def test_ackermann_launch_applies_final_reverse_safety_limit():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'safety_max_reverse_linear_x'" in source
    assert "default_value='0.10'" in source
    assert "'max_reverse_linear_x': safety_max_reverse_linear_x" in source
    assert "'max_forward_linear_x': safety_max_forward_linear_x" in source
    assert "'replan_period_sec': replan_period_sec" in source


def test_completion_window_uses_goal_checker_frame_and_xy_tolerance(params):
    controller = params['controller_server']['ros__parameters']
    launch = LAUNCH_FILE.read_text(encoding='utf-8')

    assert controller['general_goal_checker']['xy_goal_tolerance'] == pytest.approx(
        0.25)
    assert "default_value='0.25'" in launch
    assert "default_value='base_link'" in launch
    assert "'robot_base_frame': robot_base_frame" in launch
