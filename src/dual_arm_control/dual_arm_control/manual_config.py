"""Manual-mode configuration validated before the launch opens hardware."""
import math
import time

from .xy_calibration import JOINTS, calibration, tool_model, vector, verify_evidence, Kinematics

DEFAULTS = dict(planning_group='arm1', base_frame='arm1_base', tcp_link='arm1_suction_tcp',
                joint_names=JOINTS, controller_name='arm1_controller', tool_yaw_offset=0.0,
                service_timeout=4.0, planning_timeout=5.0, execution_margin=20.0,
                joint_state_max_age=1.0, velocity_scale=0.10, acceleration_scale=0.10)


def configure(config, urdf, simulation):
    c = {**DEFAULTS, **config['task_coordinator']['ros__parameters']}
    m = config['manual_xy']
    b = config['cm530_bridge']['ros__parameters']
    if c['base_frame'] != 'arm1_base' or c['tcp_link'] != 'arm1_suction_tcp' or c['joint_names'] != JOINTS:
        raise ValueError('Manual XY requires the arm1 base, TCP and joint names')
    if c['planning_group'] != 'arm1' or c['controller_name'] != 'arm1_controller' or b.get('arm1.joint_names', JOINTS) != JOINTS:
        raise ValueError('Manual XY requires arm1 planning group/controller and matching bridge joint order')
    if b.get('arms', ['arm1']) != ['arm1']:
        raise ValueError('Manual launch must control arm1 only')
    for key in ('pick_z', 'approach_height', 'service_timeout', 'planning_timeout',
                'execution_margin', 'joint_state_max_age', 'tool_yaw_offset'):
        if not isinstance(c[key], (int, float)) or not math.isfinite(c[key]):
            raise ValueError('Missing finite measured setting: ' + key)
    if any(c[k] <= 0 for k in ('approach_height', 'service_timeout', 'planning_timeout',
                              'execution_margin', 'joint_state_max_age')):
        raise ValueError('Heights/timeouts must be positive')
    for key in ('velocity_scale', 'acceleration_scale'):
        if not 0 < c[key] <= .15:
            raise ValueError('First-test velocity/acceleration scaling must be in (0, 0.15]')
    lo, hi = vector(m['xy_min'], 2, 'xy_min'), vector(m['xy_max'], 2, 'xy_max')
    if any(a >= z for a, z in zip(lo, hi)):
        raise ValueError('XY bounds must be increasing')
    vector(c['table_size'], 3, 'table_size')
    vector(c['table_center'], 3, 'table_center')
    if min(c['table_size']) <= 0:
        raise ValueError('Table dimensions must be positive')
    for box in m['obstacles']:
        vector(box['center'], 3, 'obstacle center')
        if min(vector(box['size'], 3, 'obstacle size')) <= 0:
            raise ValueError('Obstacle dimensions must be positive')
    c['simulation'] = simulation
    c['travel_z'] = c['pick_z'] + c['approach_height']
    c['velocity_scale'] = min(c['velocity_scale'], .10)
    c['acceleration_scale'] = min(c['acceleration_scale'], .10)
    model = tool_model(urdf, m['tool'])
    Kinematics(model).check_limits(calibration(b))
    if not simulation:
        if not c.get('calibrated') or not b.get('hardware_calibrated') or not b.get('serial_port'):
            raise ValueError('Hardware calibration flags and explicit serial port required')
        if m.get('calibration', {}).get('method') == 'legacy_user_confirmed':
            validate_legacy_confirmation(m, b)
        else:
            if not m['obstacles']:
                raise ValueError('Measure and model the stationary arm2/other obstacle envelope')
            verify_evidence(m, b, model)
    return dict(task=c, manual=m, bridge=b, urdf=model), model


def validate_legacy_confirmation(manual, bridge):
    """Use user-tested legacy calibration; never fabricate measurement samples."""
    e = manual['calibration']
    for key in ('joint_mapping_user_confirmed', 'unchanged_installation_user_confirmed',
                'workspace_clear_user_confirmed', 'physical_base_alignment_confirmed'):
        if e.get(key) is not True:
            raise ValueError('Missing legacy confirmation: ' + key)
    expiry = e.get('clearance_valid_until_unix_s', 0)
    now = time.time()
    if not isinstance(expiry, (int, float)) or not math.isfinite(expiry) or not now < expiry <= now + 1800:
        raise ValueError('Operator clearance expired or invalid; inspect workspace again')
    if not e.get('source'):
        raise ValueError('Legacy source required')
    if bridge.get('arm1.zero_raw') != [512]*4 or bridge.get('arm1.direction') != [1]*4:
        raise ValueError('Mapping differs from confirmed legacy source')
    t = manual['tool']
    if t.get('xyz') != [0., 0., 0.] or t.get('rpy') != [0., 0., 0.] or t.get('box_size') != [.01]*3 or t.get('box_center') != [0., 0., .005]:
        raise ValueError('Preserve the legacy end_effector target and tool geometry')


def target_xy(config, x, y):
    vector([x, y], 2, 'XY target')
    m, c = config['manual'], config['task']
    if not all(lo <= v <= hi for lo, v, hi in zip(m['xy_min'], [x, y], m['xy_max'])):
        raise ValueError('XY target outside configured workspace')
    return [x, y, c['travel_z']]


def check_plane(xyz, axis, height):
    from .xy_calibration import angle_between
    if abs(xyz[2] - height) > .001 or angle_between(axis, [0, 0, -1]) > 3:
        raise ValueError('TCP must be downward and within 1 mm of travel plane; preview --prepare first')
