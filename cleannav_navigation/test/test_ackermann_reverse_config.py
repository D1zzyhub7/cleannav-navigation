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
    assert controller['time_steps'] == 100
    assert controller['model_dt'] == pytest.approx(0.05)
    assert controller['batch_size'] == 1000
    assert controller['vx_min'] == pytest.approx(-0.15)
    assert controller['vx_max'] == pytest.approx(0.50)
    assert controller['wz_max'] == pytest.approx(0.44)
    assert controller['AckermannConstraints']['min_turning_r'] == (
        pytest.approx(1.12)
    )
    assert controller['vx_std'] == pytest.approx(0.10)
    assert controller['wz_std'] == pytest.approx(0.15)


def test_mppi_tune_b_extends_preview_and_sampling(params):
    controller_server = params['controller_server']['ros__parameters']
    controller = controller_server['FollowPath']
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']

    assert controller_server['controller_frequency'] == pytest.approx(20.0)
    assert controller['time_steps'] * controller['model_dt'] == pytest.approx(
        5.0
    )
    assert (
        controller['time_steps']
        * controller['model_dt']
        * controller['vx_max']
    ) == pytest.approx(2.5)
    assert controller['prune_distance'] == pytest.approx(3.0)
    assert controller['iteration_count'] == 1
    assert controller['temperature'] == pytest.approx(0.3)
    assert controller['gamma'] == pytest.approx(0.015)
    assert local['width'] == 7
    assert local['height'] == 7
    assert type(local['width']) is int
    assert type(local['height']) is int
    assert local['resolution'] == pytest.approx(0.05)

    for costmap in (local, global_):
        inflation = costmap['inflation_layer']
        assert inflation['inflation_radius'] == pytest.approx(0.80)
        assert inflation['cost_scaling_factor'] == pytest.approx(4.0)


def test_ackermann_demo_uses_twenty_hz_safety_output_with_override():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'safety_publish_frequency'" in source
    assert "default_value='20.0'" in source
    assert "'publish_frequency': safety_publish_frequency" in source


def test_mppi_uses_the_polygon_costmap_footprint(params):
    controller = params['controller_server']['ros__parameters']['FollowPath']
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']

    assert controller['CostCritic']['consider_footprint'] is True
    expected_footprint = (
        "[[-0.45, -0.29], [-0.45, 0.29], "
        "[0.45, 0.29], [0.45, -0.29]]"
    )
    assert local['footprint'] == expected_footprint
    assert global_['footprint'] == expected_footprint
    assert 'robot_radius' not in local
    assert 'robot_radius' not in global_


def test_progress_checker_contract_remains_frozen(params):
    progress = params['controller_server']['ros__parameters'][
        'progress_checker'
    ]

    assert progress['required_movement_radius'] == pytest.approx(0.10)
    assert progress['movement_time_allowance'] == pytest.approx(15.0)


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
    assert "default_value='0.05'" in source
    assert "'max_reverse_linear_x': safety_max_reverse_linear_x" in source
    assert "'max_forward_linear_x': safety_max_forward_linear_x" in source
    assert "'replan_period_sec': replan_period_sec" in source


def test_completion_window_uses_goal_checker_frame_and_xy_tolerance(params):
    controller = params['controller_server']['ros__parameters']
    goal_checker = controller['general_goal_checker']
    launch = LAUNCH_FILE.read_text(encoding='utf-8')

    assert controller['goal_checker_plugins'] == ['general_goal_checker']
    assert goal_checker['plugin'] == 'nav2_controller::PositionGoalChecker'
    assert goal_checker['xy_goal_tolerance'] == pytest.approx(0.25)
    assert goal_checker['stateful'] is True
    assert 'yaw_goal_tolerance' not in goal_checker
    assert "default_value='0.25'" in launch
    assert "default_value='base_link'" in launch
    assert "'robot_base_frame': robot_base_frame" in launch
