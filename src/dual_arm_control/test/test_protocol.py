import math
import threading
from types import SimpleNamespace as S
import pytest
from dual_arm_control.cm530 import Calibration, CommandError, ProtocolError, SerialCM530, parse_reply
from dual_arm_control.trajectory import sample, validate_trajectory


def calibration():
    return Calibration([512]*4, [1, -1, 1, -1], [-2.0]*4, [2.0]*4, [100]*4, [900]*4)


def test_read_maps_exact_arm_and_four_raw_values():
    assert parse_reply('READ,arm1', 'POS,arm1,512,500,600,510') == [512, 500, 600, 510]
    for reply in ['POS,arm2,512,500,600,510', 'OK,READ,arm1', 'POS,arm1,512,500,600',
                  'POS,arm1,512,500,600,1024', 'POS,arm1,512.0,500,600,510', 'READY,5']:
        with pytest.raises(ProtocolError): parse_reply('READ,arm1', reply)


def test_ack_requires_command_arm_and_value():
    parse_reply('AX,arm1,500,500,500,500', 'OK,AX,arm1')
    for command, reply in [('AX,arm1,1,2,3,4', 'OK,HOME,arm1'), ('TORQUE,arm1,1', 'OK,TORQUE,arm1,0'),
                            ('VERSION', 'VERSION,4')]:
        with pytest.raises(ProtocolError): parse_reply(command, reply)
    with pytest.raises(CommandError): parse_reply('READ,arm1', 'ERR,DXL_TIMEOUT,arm1,3')


def test_calibration_roundtrip_quantization_and_direction():
    c = calibration()
    q = [0.3, 0.5, -0.4, -0.8]
    raw = c.to_raw(q)
    assert raw[0] > 512 and raw[1] < 512
    assert max(abs(a-b) for a,b in zip(q, c.to_rad(raw))) <= 0.5/c.ticks_per_rad
    for invalid in ([math.nan]*4, [3.0]*4, [0.0]*3):
        with pytest.raises(ValueError): c.to_raw(invalid)


def point(q, t):
    return S(positions=q, velocities=[], accelerations=[], effort=[], time_from_start=S(sec=t, nanosec=0))


def test_trajectory_reorders_joint_names_and_interpolates():
    t = S(joint_names=['d','b','a','c'], points=[point([0.0]*4,0), point([0.4,0.2,0.1,0.3],2)])
    times, qs = validate_trajectory(t, ['a','b','c','d'], calibration())
    assert qs[-1] == [0.1,0.2,0.3,0.4]
    assert sample(times, qs, 1.0) == [0.05,0.1,0.15,0.2]


@pytest.mark.parametrize('points', [[point([0.0]*4,1), point([0.1]*4,1)],
                                  [point([float('nan')]*4,0)], [point([3.0]*4,0)], []])
def test_invalid_trajectory_rejected_before_any_write(points):
    with pytest.raises(ValueError): validate_trajectory(S(joint_names=list('abcd'), points=points), list('abcd'), calibration())


class FakeSerial:
    def __init__(self, reply=b''):
        self.buffer = bytearray()
        self.reply, self.writes = reply, []
    @property
    def in_waiting(self): return len(self.buffer)
    def write(self, data):
        self.writes.append(data)
        self.buffer.extend(self.reply)
    def read(self, count):
        data = bytes(self.buffer[:count]); del self.buffer[:count]
        return data


def session(reply=b''):
    s = SerialCM530.__new__(SerialCM530)
    s.port, s.timeout, s.healthy, s.lock = FakeSerial(reply), 0.001, True, threading.RLock()
    return s


def test_fragmented_crlf_and_sequential_transactions():
    s = session(b'OK,AX,arm1\r\n')
    s.transact('AX,arm1,512,512,512,512')
    s.transact('AX,arm1,520,512,512,512')
    assert len(s.port.writes) == 2


def test_timeout_latches_fault_and_does_not_reuse_late_ack():
    s = session()
    with pytest.raises(ProtocolError): s.transact('AX,arm1,512,512,512,512')
    s.port.buffer.extend(b'OK,AX,arm1\r\n')
    with pytest.raises(ProtocolError): s.transact('AX,arm1,520,512,512,512')
    assert len(s.port.writes) == 1 and not s.healthy


def test_stale_nonempty_data_faults_before_writing():
    s = session(); s.port.buffer.extend(b'OK,AX,arm1\r\n')
    with pytest.raises(ProtocolError): s.transact('AX,arm1,512,512,512,512')
    assert not s.port.writes


def test_arm2_protocol5_exact_order_and_arm_mismatch():
    assert parse_reply('GET_HOME,arm2', 'HOME,arm2,512,510,520,530') == [512, 510, 520, 530]
    assert parse_reply('READ,arm2', 'POS,arm2,500,501,502,503') == [500, 501, 502, 503]
    s = session(b'OK,AX,arm2\r\n')
    s.write('arm2', [500, 501, 502, 503])
    assert s.port.writes == [b'AX,arm2,500,501,502,503\n']
    with pytest.raises(ProtocolError):
        parse_reply('AX,arm2,500,501,502,503', 'OK,AX,arm1')
    with pytest.raises(CommandError):
        parse_reply('READ,arm2', 'ERR,DXL_MOTOR,arm2,8,32')


@pytest.mark.parametrize('reply', ['ERR,OVERFLOW','ERR,INIT_FAILED','ERR,DXL_TIMEOUT,arm2,12',
    'ERR,DXL_TIMEOUT,arm1,99','ERR,DXL_MOTOR,arm1,17,0','ERR,UNKNOWN','ERR'])
def test_async_and_mismatched_errors_desynchronize_session(reply):
    s=session((reply+'\r\n').encode())
    with pytest.raises(ProtocolError): s.transact('READ,arm1')
    assert not s.healthy
    with pytest.raises(ProtocolError): s.hold('arm1')
    assert len(s.port.writes)==1


@pytest.mark.parametrize('command',['STOP','AX,arm1,1.0,2,3,4','AX,arm1,1024,2,3,4',
    'TORQUE,arm1,2','LED,arm1,RED','AX,BOTH,1,2,3,4','PING\nHOME,arm1'])
def test_invalid_outgoing_commands_never_touch_serial(command):
    s=session()
    with pytest.raises(ValueError): s.transact(command)
    assert s.healthy and not s.port.writes


@pytest.mark.parametrize('boot,valid', [(b'READY,5\r\n',True),(b'',True),
    (b'READY,4\r\n',False),(b'READY,5',False),(b'ERR,INIT_FAILED\r\n',False),
    (b'OK,AX,arm1\r\n',False)])
def test_startup_accepts_only_ready5_or_silence_and_closes_on_failure(monkeypatch,boot,valid):
    import serial
    ports=[]
    class Port(FakeSerial):
        def __init__(self,**kwargs):
            super().__init__();self.buffer.extend(boot);self.closed=False;ports.append(self)
        def open(self):pass
        def close(self):self.closed=True
        def write(self,data):
            self.reply={b'VERSION\n':b'VERSION,5\r\n',b'PING\n':b'PONG\r\n'}[data]
            super().write(data)
            return len(data)
    monkeypatch.setattr(serial,'Serial',Port)
    if valid:
        c=SerialCM530('fake',timeout=.01,startup_listen=.002)
        assert ports[0].writes==[b'VERSION\n',b'PING\n']
        c.close()
    else:
        with pytest.raises(ProtocolError):SerialCM530('fake',timeout=.01,startup_listen=.002)
        assert not ports[0].writes and ports[0].closed
