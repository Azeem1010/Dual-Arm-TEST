"""CM530 v17/protocol 5. No ROS imports and no implicit motion/retry."""
import math
import re
import logging
import threading
import time
from dataclasses import dataclass


class ProtocolError(RuntimeError):
    pass


class CommandError(RuntimeError):
    """A complete ERR reply; the transaction ended, but its motion failed."""


def parse_reply(command, line):
    cmd = command.split(',')
    fields = line.split(',')
    if fields[0] == 'ERR':
        # These are asynchronous stream/controller faults, not matched replies.
        if line in ('ERR,OVERFLOW', 'ERR,INIT_FAILED'):
            raise ProtocolError(line)
        code = fields[1] if len(fields) > 1 else ''
        sizes = {'BAD_CMD': (2, 3), 'BAD_ARG': (2, 3), 'RANGE': (3,),
                 'NO_TARGET': (3,), 'INIT_FAILED': (3,), 'DXL_TX': (3, 4),
                 'DXL_TIMEOUT': (4,), 'DXL_CORRUPT': (4,), 'DXL_RX': (4,),
                 'DXL_MOTOR': (5,), 'DXL_RANGE': (4,)}
        if len(fields) not in sizes.get(code, ()):
            raise ProtocolError('Malformed error: ' + line)
        if len(fields) >= 3 and (len(cmd) < 2 or fields[2] != cmd[1]):
            raise ProtocolError('Error arm mismatch: ' + line)
        if len(fields) >= 4:
            ids = {'arm1': (17, 3, 2, 15), 'arm2': (12, 1, 8, 16)}
            if (cmd[0] not in ('READ', 'HOLD') or not fields[3].isascii()
                    or not fields[3].isdigit() or int(fields[3]) not in ids.get(cmd[1], ())):
                raise ProtocolError('Unexpected motor error: ' + line)
        if len(fields) == 5 and (not fields[4].isascii() or not fields[4].isdigit()
                                or not 1 <= int(fields[4]) <= 255):
            raise ProtocolError('Invalid motor error mask: ' + line)
        raise CommandError(line)
    if cmd[0] in ('READ', 'GET_HOME'):
        tag = 'POS' if cmd[0] == 'READ' else 'HOME'
        if len(fields) != 6 or fields[:2] != [tag, cmd[1]]:
            raise ProtocolError(f'Mismatched reply: {command!r} -> {line!r}')
        if not all(s.isascii() and s.isdigit() and 0 <= int(s) <= 1023 for s in fields[2:]):
            raise ProtocolError(f'Invalid raw positions: {line}')
        return [int(s) for s in fields[2:]]
    expected = {'PING': ['PONG'], 'VERSION': ['VERSION', '5']}.get(cmd[0])
    if expected is None:
        expected = ['OK', *cmd[:2]]
        if cmd[0] in ('TORQUE', 'LED'):
            expected.append(cmd[2])
    if fields != expected:
        raise ProtocolError(f'Mismatched reply: {command!r} -> {line!r}')
    return None


def validate_command(command):
    """Canonical protocol-5 wire format; reject local mistakes before any I/O."""
    if not isinstance(command, str) or not command.isascii() or len(command) > 95:
        raise ValueError('Expected at most 95 ASCII bytes')
    if command in ('PING', 'VERSION'):
        return
    p = command.split(',')
    counts = {'AX': 6, 'HOME': 2, 'GET_HOME': 2, 'READ': 2, 'HOLD': 2, 'TORQUE': 3, 'LED': 3}
    if len(p) != counts.get(p[0]) or p[1] not in ('arm1', 'arm2'):
        raise ValueError('Unsupported command, arm or field count')
    if p[0] == 'AX' and not all(re.fullmatch(r'[0-9]{1,4}', v) and int(v) <= 1023 for v in p[2:]):
        raise ValueError('AX requires four raw integers in 0..1023')
    if p[0] == 'TORQUE' and p[2] not in ('0', '1'):
        raise ValueError('TORQUE requires 0 or 1')
    if p[0] == 'LED' and p[2] not in ('MOVING', 'STOPPED'):
        raise ValueError('Invalid LED state')


@dataclass
class Calibration:
    zero: list
    direction: list
    lower: list
    upper: list
    raw_min: list
    raw_max: list
    ticks_per_rad: float = 1023.0 / math.radians(300.0)

    def __post_init__(self):
        arrays = [self.zero, self.direction, self.lower, self.upper, self.raw_min, self.raw_max]
        if any(len(a) != 4 for a in arrays) or any(not math.isfinite(float(v)) for a in arrays for v in a):
            raise ValueError('Four finite calibration values are required per arm')
        if any(d not in (-1, 1) for d in self.direction):
            raise ValueError('Joint directions must be -1 or +1')
        if not math.isfinite(self.ticks_per_rad) or self.ticks_per_rad <= 0:
            raise ValueError('Invalid ticks_per_rad')
        for i in range(4):
            if not self.lower[i] < self.upper[i] or not 0 <= self.raw_min[i] < self.raw_max[i] <= 1023:
                raise ValueError('Invalid mechanical limits')
            if not self.raw_min[i] <= self.zero[i] <= self.raw_max[i]:
                raise ValueError('Zero outside raw limits')

    def to_raw(self, positions):
        if len(positions) != 4:
            raise ValueError('Exactly four joint positions required')
        result = []
        for i, q in enumerate(positions):
            if not math.isfinite(q) or not self.lower[i] <= q <= self.upper[i]:
                raise ValueError(f'Joint {i} outside calibrated limits: {q}')
            raw = round(self.zero[i] + self.direction[i] * q * self.ticks_per_rad)
            if not self.raw_min[i] <= raw <= self.raw_max[i]:
                raise ValueError(f'Joint {i} raw target outside calibrated limits: {raw}')
            result.append(raw)
        return result

    def to_rad(self, raw):
        if len(raw) != 4 or any(not isinstance(x, int) or not 0 <= x <= 1023 for x in raw):
            raise ValueError('Invalid READ positions')
        return [(raw[i] - self.zero[i]) / (self.direction[i] * self.ticks_per_rad) for i in range(4)]


class SerialCM530:
    def __init__(self, port, timeout=2.0, startup_listen=6.0):
        import serial
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(startup_listen) or startup_listen < 0:
            raise ValueError('Invalid serial timeouts')
        self.port = serial.Serial(port=None, baudrate=57600, timeout=0.02,
                                  write_timeout=timeout, exclusive=True)
        self.port.dtr = self.port.rts = False
        self.port.port = port
        self.port.open()
        self.timeout = timeout
        self.lock = threading.RLock()
        self.healthy = True
        try:
            deadline = time.monotonic() + startup_listen
            boot = bytearray()
            while time.monotonic() < deadline:
                byte = self.port.read(1)
                if byte in (b'\r', b'\n'):
                    if not boot:
                        continue
                    if boot != b'READY,5':
                        raise ProtocolError(f'Unexpected startup data: {boot!r}')
                    boot.clear()
                    break
                boot.extend(byte)
                if len(boot) > 256:
                    raise ProtocolError('Oversized startup line')
            if boot:
                raise ProtocolError('Incomplete startup response')
            self.transact('VERSION')
            self.transact('PING')
        except Exception:
            self.healthy = False
            self.port.close()
            raise

    def transact(self, command):
        validate_command(command)
        with self.lock:
            if not self.healthy:
                raise ProtocolError('Serial connection faulted; no automatic resynchronization or retry')
            try:
                stale = self.port.read(self.port.in_waiting)
                if stale.strip(b'\r\n'):
                    raise ProtocolError(f'Unexpected data before request: {stale!r}')
                payload = (command + '\n').encode('ascii')
                logging.getLogger('cm530.protocol').info('TX monotonic=%.6f %s', time.monotonic(), command)
                written = self.port.write(payload)
                if written is not None and written != len(payload):
                    raise ProtocolError('Partial serial write; outcome unknown')
                deadline = time.monotonic() + self.timeout
                data = bytearray()
                while time.monotonic() < deadline:
                    byte = self.port.read(1)
                    if not byte:
                        continue
                    if byte in (b'\r', b'\n'):
                        if data:
                            line = data.decode('ascii')
                            logging.getLogger('cm530.protocol').info('RX monotonic=%.6f %s', time.monotonic(), line)
                            return parse_reply(command, line)
                        continue
                    data.extend(byte)
                    if len(data) > 256:
                        raise ProtocolError('Oversized reply')
                raise ProtocolError(f'Command timeout; outcome unknown: {command}')
            except CommandError as e:
                logging.getLogger('cm530.protocol').warning('Command failed: %s: %s', command, e)
                raise
            except Exception as e:
                self.healthy = False
                logging.getLogger('cm530.protocol').error('Session faulted; result unknown: %s: %s', command, e)
                raise

    def read(self, arm):
        return self.transact(f'READ,{arm}')

    def write(self, arm, raw):
        self.transact('AX,' + arm + ',' + ','.join(map(str, raw)))

    def enable(self, arm):
        # Holding the measured pose prevents an old target from being activated.
        with self.lock:
            self.transact(f'HOLD,{arm}')
            self.transact(f'TORQUE,{arm},1')

    def hold(self, arm):
        self.transact(f'HOLD,{arm}')

    def get_home(self, arm):
        return self.transact(f'GET_HOME,{arm}')

    def torque(self, arm, enabled):
        self.transact(f'TORQUE,{arm},{int(enabled)}')

    def led(self, arm, moving):
        self.transact(f'LED,{arm},{"MOVING" if moving else "STOPPED"}')

    def close(self):
        self.port.close()


class MockCM530:
    """Ideal raw-position actuator, not a motor dynamics simulation."""
    def __init__(self, initial):
        self.positions = {arm: list(raw) for arm, raw in initial.items()}
        self.healthy = True
        self.lock = threading.RLock()

    def read(self, arm):
        with self.lock:
            return list(self.positions[arm])

    def write(self, arm, raw):
        with self.lock:
            self.positions[arm] = list(raw)

    def enable(self, arm):
        pass

    def hold(self, arm):
        pass

    def led(self, arm, moving):
        pass

    def torque(self, arm, enabled):
        pass

    def get_home(self, arm):
        return list(self.positions[arm])

    def close(self):
        pass
