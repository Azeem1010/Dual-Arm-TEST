"""Real pyserial over PTY to unchanged v17 firmware parser compiled for host."""
import ctypes
import os
from pathlib import Path
import select
import subprocess
import threading
import tty
import pytest
from dual_arm_control.cm530 import SerialCM530, CommandError


@pytest.fixture
def wire(tmp_path):
    root=Path(__file__).resolve().parents[3]
    firmware=root/'CM530_ROS_BRIDGE-main/17 ROS to CM530 ver. dual arm'
    lib=tmp_path/'bridge.so'
    subprocess.run(['gcc','-shared','-fPIC','-Wall','-Wextra','-Werror','-I',str(firmware/'APP/inc'),
                    str(firmware/'APP/src/bridge.c'),str(Path(__file__).with_name('firmware_peer.c')),
                    '-o',str(lib)],check=True)
    peer=ctypes.CDLL(str(lib)); peer.take_output.restype=ctypes.c_char_p
    peer.BridgeInit(); peer.clear_output() # READY emitted before host opens, as permitted.
    master,slave=os.openpty(); tty.setraw(slave)
    stopped=threading.Event(); received=[]
    def serve():
        pending=bytearray()
        while not stopped.is_set():
            if not select.select([master],[],[],.02)[0]: continue
            data=os.read(master,256)
            for b in data:
                pending.append(b)
                peer.BridgeFeed(b)
                response=peer.take_output()
                if response:
                    received.append(bytes(pending));pending.clear()
                    # Deliberately fragment CRLF/data through the real serial layer.
                    for byte in response: os.write(master,bytes([byte]))
                    peer.clear_output()
    thread=threading.Thread(target=serve);thread.start()
    connection=None
    try:
        connection=SerialCM530(os.ttyname(slave),timeout=.3,startup_listen=0)
        yield connection,peer,received
    finally:
        if connection: connection.close()
        stopped.set();thread.join(timeout=2)
        os.close(master);os.close(slave)


def test_v17_wire_handshake_home_qualification_and_dual_order(wire):
    c,peer,rx=wire
    assert rx[:2]==[b'VERSION\n',b'PING\n']
    # Read the real header values, not stale README examples.
    assert c.get_home('arm1')==[90,567,712,825]
    assert c.get_home('arm2')==[590,567,712,825]
    with pytest.raises(CommandError,match='NO_TARGET'): c.torque('arm1',True)
    c.enable('arm1');c.enable('arm2')
    c.write('arm1',[500,501,502,503]);c.write('arm2',[510,511,512,513])
    assert c.read('arm1')==[500,501,502,503]
    assert c.read('arm2')==[510,511,512,513]
    c.led('arm1',True);c.led('arm2',False)
    c.torque('arm1',False)
    with pytest.raises(CommandError,match='NO_TARGET'): c.torque('arm1',True)
    c.transact('HOME,arm1');c.torque('arm1',True)
    assert c.read('arm1')==c.get_home('arm1')


def test_v17_hold_failure_invalidates_only_that_arm(wire):
    c,peer,rx=wire
    c.enable('arm1');c.enable('arm2')
    peer.fail_read(1)
    with pytest.raises(CommandError,match='DXL_TIMEOUT,arm1,17'): c.hold('arm1')
    peer.fail_read(0)
    with pytest.raises(CommandError,match='NO_TARGET'): c.torque('arm1',True)
    c.torque('arm2',True)
    assert c.healthy


def test_two_threads_use_one_wire_pending_request(wire):
    c,peer,rx=wire
    failures=[]
    def worker(arm):
        try:
            for _ in range(12): assert len(c.read(arm))==4
        except Exception as e: failures.append(e)
    threads=[threading.Thread(target=worker,args=(arm,)) for arm in ('arm1','arm2')]
    for t in threads:t.start()
    for t in threads:t.join(timeout=5)
    assert not failures and all(not t.is_alive() for t in threads)
    assert len(rx)==26
