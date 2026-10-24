"""Offline checks for the Linux robot connection helper."""

import importlib.util
from pathlib import Path
import pytest


SPEC = importlib.util.spec_from_file_location(
    'linux_connection', Path(__file__).resolve().parents[1] / 'scripts/connect_lite3_linux.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_network_config_requires_one_valid_perception_address():
    assert MODULE.perception_address('ip = 192.168.1.103\ntarget_port = 43897') == '192.168.1.103'
    for content in ('', 'ip = 192.168.1.103\nip = 192.168.1.120', 'ip = 999.1.1.1'):
        with pytest.raises(ValueError):
            MODULE.perception_address(content)


def test_gateway_requires_robot_wifi_and_current_dhcp(monkeypatch):
    responses = {
        ('nmcli', '-t', '-f', 'DEVICE,TYPE,STATE', 'device'): 'wlp0s20f3:wifi:connected',
        ('nmcli', '-g', 'GENERAL.CONNECTION', 'device', 'show', 'wlp0s20f3'): 'CaciNet',
    }
    monkeypatch.setattr(MODULE, 'command', lambda *args: responses[args])
    with pytest.raises(ValueError, match='robot YSC-JYML Wi-Fi'):
        MODULE.robot_gateway()
    responses[('nmcli', '-g', 'GENERAL.CONNECTION', 'device', 'show', 'wlp0s20f3')] = 'YSC-JYML-he2asx-5G'
    responses[('nmcli', '-g', 'IP4.GATEWAY', 'device', 'show', 'wlp0s20f3')] = '192.168.2.1'
    assert MODULE.robot_gateway() == ('YSC-JYML-he2asx-5G', '192.168.2.1')
