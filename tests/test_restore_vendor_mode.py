"""Mode recovery must fail closed and send only the documented commands."""

import json
from pathlib import Path
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import restore_vendor_mode as modes


def test_process_probe_ignores_its_own_shell_command_text():
    shell = ['/bin/bash', '-lc', "python3 -c 'if lite3_odom_tf_adapter and navigation.launch.py: pass'"]
    assert not modes.is_project_navigation_process(shell)
    assert not modes.is_project_navigation_process(['python3', '-c',
                                                     'supervised_nav_relay motion_arbiter'])
    assert modes.is_project_navigation_process([
        '/home/ysc/lite3_autonomy_ws/install/lite3_odom_tf_adapter/lib/'
        'lite3_odom_tf_adapter/motion_arbiter', '--ros-args'])
    assert modes.is_project_navigation_process([
        '/usr/bin/python3', '/opt/ros/foxy/bin/ros2', 'launch',
        'lite3_odom_tf_adapter', 'navigation.launch.py'])


def test_preflight_rejects_live_command_publisher(monkeypatch):
    class Result:
        stdout = json.dumps({'cmd_vel_publishers': ['/other'],
                             'project_navigation_processes': []})+'\n'
    monkeypatch.setattr(modes.subprocess, 'run', lambda *args, **kwargs: Result())
    with pytest.raises(RuntimeError, match='Cannot return modes'):
        modes.preflight('192.168.2.1', '192.168.1.103')


def test_manual_restore_sends_exact_packets_in_order(monkeypatch, tmp_path):
    monkeypatch.setattr(modes, 'ROOT', tmp_path)
    monkeypatch.setattr(modes, 'robot_gateway',
                        lambda: ('YSC-JYML-test', '192.168.2.1'))
    monkeypatch.setattr(modes, 'command', lambda *args: 'ip = "192.168.1.103"')
    monkeypatch.setattr(modes, 'preflight',
                        lambda *args: {'cmd_vel_publishers': [],
                                       'project_navigation_processes': []})
    packets = []

    class Socket:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def settimeout(self, _seconds):
            pass

        def sendto(self, payload, address):
            packets.append((payload, address))
            return len(payload)

    monkeypatch.setattr(modes.socket, 'socket', lambda *args: Socket())
    path = modes.restore(assume_neutral=True)
    assert [packet for packet, _ in packets] == [
        struct.pack('<III', 0x21010C02, 0, 0),
        struct.pack('<III', 0x21010D06, 0, 0)]
    assert all(address == ('192.168.2.1', 43893)
               for _, address in packets)
    assert json.loads(path.read_text())['acknowledgement_available'] is False
