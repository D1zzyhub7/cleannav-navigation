#!/usr/bin/env python3
"""Refresh the integrated RViz entry's repository-relative source hashes."""
import hashlib
import json
from pathlib import Path

NAV=Path(__file__).resolve().parents[2]
FILES=(
    'tools/startup_smoke/startup_supervisor.py',
    'tools/startup_smoke/startup_smoke.launch.py',
    'tools/startup_smoke/mapping_mode_retry.py',
    'tools/startup_smoke/amcl_diagnostic_profile.py',
    'tools/startup_smoke/amcl_private_buffer_diagnostic_profile.json',
    'tools/startup_smoke/tests/test_amcl_diagnostic_profile.py',
    'tools/startup_smoke/tests/test_startup_smoke_contract.py',
    'tools/startup_smoke/roof_lidar_runtime.py',
    'tools/startup_smoke/publisher_graph_gate.py',
    'tools/startup_smoke/tests/test_publisher_graph_gate.py',
    'cleannav_navigation/launch/cleannav_ackermann_navigation.launch.py',
    'cleannav_rtabmap/launch/cleannav_rtabmap_localization.launch.py',
    'cleannav_simulation/launch/cleannav_full_sim.launch.py',
    'cleannav_simulation/launch/startup_smoke_simulation.launch.py',
    'cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro',
    'tools/rviz_navigation/RUNTIME_DEPENDENCIES.json',
    'tools/rviz_navigation/collision_lifecycle_startup_policy.py',
    'tools/rviz_navigation/collision_monitor.source.yaml',
    'tools/rviz_navigation/collision_monitor_lifecycle_startup.py',
    'tools/rviz_navigation/deadline_watchdog.py',
    'tools/rviz_navigation/guard_policy.py',
    'tools/rviz_navigation/update_source_manifest.py',
    'tools/rviz_navigation/cm_state_poll.py',
    'tools/rviz_navigation/tests/test_cm_startup_policy.py',
    'tools/rviz_navigation/tests/test_integration.py',
    'tools/rviz_navigation/integration.py',
    'tools/rviz_navigation/localization_status_monitor.py',
    'tools/rviz_navigation/mapping_status_monitor.py',
    'tools/rviz_navigation/tests/test_rtab_resume_guard_contract.py',
    'tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_short_goal.json',
    'tools/rviz_navigation/tests/test_simple_oracle_short_goal.py',
    'tools/rviz_navigation/tests/test_profile_install_binding.py',
    'tools/rviz_navigation/oracle_odometry.py',
    'tools/rviz_navigation/map_contract.json',
    'tools/rviz_navigation/map_contract_patrol_a_v4.json',
    'tools/rviz_navigation/map_contract_mcity_localization07.json',
    'tools/rviz_navigation/map_contract_mcity_localization07_canonical_debug.json',
    'tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1.json',
    'tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_static.json',
    'tools/rviz_navigation/map_contract_simple_four_wall_nohit_v1_oracle_localization.json',
    'tools/rviz_navigation/navigation.rviz',
    'tools/rviz_navigation/navigation_oracle_short_goal.rviz',
    'tools/rviz_navigation/navigation_safety_guard.py',
    'tools/rviz_navigation/readiness_inputs.py',
    'tools/rviz_navigation/navigation_supervisor.py',
    'tools/rviz_navigation/process_memory_telemetry.py',
    'tools/rviz_navigation/localization_diagnostics.py',
    'tools/rviz_navigation/tests/test_localization_diagnostics.py',
    'tools/rviz_navigation/tests/test_navigation_safety_guard.py',
    'tools/rviz_navigation/tests/test_navigation_supervisor.py',
    'tools/rviz_navigation/tests/test_oracle_odometry.py',
    'tools/rviz_navigation/tests/test_patrol_integration.py',
    'tools/rviz_navigation/tests/test_process_memory_telemetry.py',
    'tools/rviz_navigation/overlay_identity.py',
    'tools/rviz_navigation/overlay_setup.template.bash',
    'tools/rviz_navigation/check_overlay_identity.sh',
    'tools/rviz_navigation/rviz_navigation.launch.py',
    'tools/rviz_navigation/run_rviz_navigation.sh',
    'tools/rviz_navigation/rviz_goal_input.py',
    'tools/rviz_navigation/tests/test_rviz_goal_input.py',
    'tools/rviz_navigation/short_goal_profile.json',
    'tools/rviz_navigation/short_goal_policy.py',
    'tools/rviz_navigation/tests/test_admission_safety_closure.py',

    'tools/rviz_navigation/short_goal_rviz_adapter.py',
    'tools/rviz_navigation/supervisor_policy.py',
    'tools/rviz_navigation/tests/test_short_goal_policy.py',
    'tools/rviz_navigation/tests/test_historical_motion_restoration.py',
    'tools/rviz_navigation/tests/test_historical_route_budget.py',
    'tools/rviz_navigation/tests/test_odometry_contract_mode.py',
)

def main():
    files={relative:hashlib.sha256((NAV/relative).read_bytes()).hexdigest()
           for relative in FILES}
    workspace_files={}
    runtime_sources={}
    for package,folder,names in [
        ('cleannav_mission_manager','cleannav-mission-manager',[
            'navigation_goal_service.py','adapters/real_navigation.py','adapters/real_safety.py','simulation_runner.py']),
        ('cleannav_safety_supervisor','cleannav_safety_supervisor',['safety_supervisor_node.py'])]:
        runtime_sources[package]={}
        for name in names:
            relative='cleannav-experiments/motion-admission-safety-closure-20261031/src/'+folder+'/'+package+'/'+name
            workspace_files[relative]=hashlib.sha256((NAV.parents[1]/relative).read_bytes()).hexdigest()
            runtime_sources[package]['lib/python3.10/site-packages/'+package+'/'+name]=relative
    target=NAV/'tools/rviz_navigation/SOURCE_MANIFEST.json'
    target.write_text(json.dumps({'schema_version':1,'files':files,'workspace_files':workspace_files,'runtime_source_files':runtime_sources},indent=2)+'\n')
    print(target)
    print(f'{len(files)} source files hashed')

if __name__=='__main__':main()
