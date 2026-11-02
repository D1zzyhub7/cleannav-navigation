import hashlib
from pathlib import Path
import sys

import pytest
import yaml

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import integration
import oracle_odometry as oracle


def test_world_pose_is_expressed_in_first_base_footprint_odom_frame():
    half = 2 ** -0.5
    position, orientation = oracle.world_pose_to_odom(
        (3.0, 5.0, 0.4), (0.0, 0.0, half, half),
        (3.0, 4.0, 0.4), (0.0, 0.0, half, half))

    assert position == pytest.approx((1.0, 0.0, 0.0))
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_world_twist_is_rotated_into_base_footprint():
    half = 2 ** -0.5
    linear, angular = oracle.world_twist_to_base(
        (1.0, 0.0, 0.0), (0.0, 0.0, 0.5), (0.0, 0.0, half, half))

    assert linear == pytest.approx((0.0, -1.0, 0.0))
    assert angular == pytest.approx((0.0, 0.0, 0.5))


def test_oracle_inputs_disable_only_the_private_wheel_authority(tmp_path):
    model = tmp_path / "selected.urdf.xacro"
    controller = tmp_path / "controllers.yaml"
    model.write_text("""<robot>
<remapping>ackermann_controller/odometry:=odom</remapping>
<remapping>ackermann_controller/tf_odometry:=tf</remapping>
</robot>
""")
    controller.write_text("""ackermann_controller:
  ros__parameters:
    enable_odom_tf: true
    wheelbase: 0.6
""")
    source_hashes = {
        path: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (model, controller)
    }

    result = integration.prepare_oracle_odometry_inputs(model, controller, tmp_path)

    private_model = Path(result["model_xacro"]).read_text()
    private_controller = yaml.safe_load(Path(result["controller_yaml"]).read_text())
    assert "ackermann_controller/odometry:=/wheel/odom" in private_model
    assert "ackermann_controller/tf_odometry:=/wheel/tf" in private_model
    assert private_controller["ackermann_controller"]["ros__parameters"] == {
        "enable_odom_tf": False, "wheelbase": 0.6}
    assert result["identity"]["authoritative_chain"]["odometry_topic"] == "/odom"
    assert result["identity"]["authoritative_chain"]["tf"] == "odom -> base_footprint"
    assert "response.header.stamp" in result["identity"]["ground_truth"]["source_timestamp"]
    assert hashlib.sha256(model.read_bytes()).hexdigest() == source_hashes[model]
    assert hashlib.sha256(controller.read_bytes()).hexdigest() == source_hashes[controller]


def test_oracle_input_rejects_an_unrecognized_selected_wheel_remap(tmp_path):
    model = tmp_path / "selected.urdf.xacro"
    controller = tmp_path / "controllers.yaml"
    model.write_text("<robot><remapping>something_else</remapping></robot>")
    controller.write_text("""ackermann_controller:
  ros__parameters:
    enable_odom_tf: true
""")

    with pytest.raises(ValueError, match="selected model lacks"):
        integration.prepare_oracle_odometry_inputs(model, controller, tmp_path)
