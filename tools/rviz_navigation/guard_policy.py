"""No authority: admission/stop decisions for the reused safety interfaces."""
import math
import json
from pathlib import Path
from integration import motion_profile_for_mode

_BASE_PROFILE = json.loads((Path(__file__).parent/'short_goal_profile.json').read_text())
COMMAND_PROFILES = {mode: motion_profile_for_mode(_BASE_PROFILE, mode)
                    for mode in ('none', 'short-goal', 'historical', 'patrol')}


DIAGNOSTIC_HARD_FACTS = (
    'safety_no_estop', 'no_motion_authority',
)
EXPECTED_OPTIONAL_COMMAND_PUBLISHERS = {
    '/cleannav/navigation/cmd_vel_raw': 'controller_server',
    '/cleannav/cmd_vel_candidate': 'collision_monitor',
}
FORBIDDEN_COMMAND_TOPICS = (
    '/cleannav/manual/cmd_vel_raw', '/cleannav/mapping/cmd_vel_raw',
)


def command_error(values, motion_enabled, safety, *, expected_owner=None,
                  minimum_state_ns=None, now_ns=None, motion_profile='short-goal'):
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        return 'invalid command components'
    if any(abs(values[i]) > 1e-8 for i in (1,2,3,4)):
        return 'nonplanar command'
    moving = abs(values[0]) > 1e-8 or abs(values[5]) > 1e-8
    if moving and not motion_enabled:
        return 'nonzero command in no-motion mode'
    profile = COMMAND_PROFILES.get(motion_profile)
    if profile is None:
        return 'unknown command motion profile'
    if (values[0] < -1e-8 or values[0] > profile['max_forward_linear_x']+1e-8
            or abs(values[5]) > profile['max_angular_z']+1e-8):
        return 'command outside forward-only '+motion_profile+' profile'
    if moving and now_ns is not None and safety:
        received = safety.get('monotonic_ns', 0)
        if not 0 <= now_ns-received <= 500_000_000:
            return 'Safety state stale or future dated'
        if minimum_state_ns is not None and received < minimum_state_ns:
            return 'Safety state predates current goal/Lease'
    if moving and safety and safety.get('owner') and expected_owner is not None:
        if safety['owner'] != expected_owner:
            return 'Safety Lease owner mismatch'
    if moving and (not safety or safety['block_all'] or safety['estop']
                   or not safety['enabled'] or not safety['owner']):
        return 'command without valid existing Safety/Lease state'
    return None


def missing_inputs(facts):
    return sorted(k for k,v in facts.items() if v is not True)


def diagnostic_command_graph_errors(publishers):
    """Reject unknown or conflicting command paths while allowing absent idle stages.

    The required final /cmd_vel endpoint is checked separately by the existing
    stable graph gate. Raw controller/Collision Monitor publishers may not yet
    exist while no-motion localization is still starting; an absent endpoint
    keeps readiness false but is not an unknown publisher. Any present endpoint
    must have the exact expected identity, type, and singleton cardinality.
    """
    errors = []
    for topic, expected_node in EXPECTED_OPTIONAL_COMMAND_PUBLISHERS.items():
        records = publishers.get(topic, [])
        if not records:
            continue
        if len(records) != 1:
            errors.append(f'{topic}:publisher_count_not_one')
            continue
        endpoint = records[0]
        if endpoint.get('node_name') in (None, '', '_NODE_NAME_UNKNOWN_', 'UNKNOWN'):
            errors.append(f'{topic}:publisher_identity_unknown')
        elif endpoint.get('node_name') != expected_node:
            errors.append(f'{topic}:unexpected_publisher_node')
        if endpoint.get('node_namespace') != '/':
            errors.append(f'{topic}:unexpected_or_unknown_namespace')
        if endpoint.get('topic_type') != 'geometry_msgs/msg/Twist':
            errors.append(f'{topic}:unexpected_or_unknown_type')
    for topic in FORBIDDEN_COMMAND_TOPICS:
        if publishers.get(topic):
            errors.append(f'{topic}:unmanaged_command_publisher_present')
    return errors


def diagnostic_continuation_decision(facts, publishers, *, nonzero_command_count,
                                     distance_m, maximum_displacement_m=0.05):
    """Keep measured safety violations hard; missing observations are not READY.

    The live callbacks independently stop on ESTOP/authority violations and
    nonzero commands. The graph state machine rejects illegal final endpoints.
    Stale Safety/CM/TF/localization observations keep readiness false while the
    fixed no-motion launch remains bounded by its independent watchdog.
    """
    errors = [f'{key}:explicit_violation' for key in DIAGNOSTIC_HARD_FACTS
              if facts.get(key) is False]
    errors.extend(diagnostic_command_graph_errors(publishers))
    if nonzero_command_count:
        errors.append('nonzero_cmd_vel_observed')
    if (isinstance(distance_m, bool) or not isinstance(distance_m, (int, float))
            or not math.isfinite(distance_m) or distance_m < 0
            or distance_m > maximum_displacement_m):
        # An absent sample keeps readiness false; a measured violation stops.
        if distance_m is not None:
            errors.append('no_motion_displacement_invalid_or_over_limit')
    return {
        'allowed': not errors,
        'status': 'SAFE_TO_CONTINUE' if not errors else 'STOP_REQUIRED',
        'hard_failures': errors,
        'ordinary_readiness_missing': sorted(
            key for key, value in facts.items()
            if value is not True and key not in DIAGNOSTIC_HARD_FACTS),
        'maximum_displacement_m': maximum_displacement_m,
        'distance_m': distance_m,
        'nonzero_command_count': nonzero_command_count,
    }


MOTION_REQUIRED_FACTS = (
    'localization', 'map_odom', 'nav2', 'controller_publisher', 'cm_active',
    'safety', 'cmd_publisher', 'scan_fresh', 'tf', 'authorization', 'scope',
    'lease', 'estop_healthy', 'resources',
)

def motion_admission(facts):
    """Pure fail-closed validation only; never creates authority or commands."""
    missing = [key for key in MOTION_REQUIRED_FACTS if facts.get(key) is not True]
    return {'allowed': not missing, 'missing': missing, 'stage': 'MOTION_ADMISSION'}


def grid_message_valid(message, frame):
    info=message.info;origin=info.origin
    return (message.header.frame_id==frame and info.width>0 and info.height>0
        and math.isfinite(info.resolution) and info.resolution>0
        and len(message.data)==info.width*info.height
        and all(math.isfinite(v) for v in (origin.position.x,origin.position.y,origin.position.z,
            origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w))
        and abs(math.hypot(origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w)-1)<1e-3
        and all(-1<=v<=100 for v in message.data) and any(v==0 for v in message.data))


def point_path_distance(point, points):
    if not points:return math.inf
    if len(points)==1:return math.hypot(point[0]-points[0][0],point[1]-points[0][1])
    best=math.inf
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length)) if length else 0.
        best=min(best,math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy))
    return best
