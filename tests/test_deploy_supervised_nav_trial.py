from pathlib import Path
import sys

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import deploy_supervised_nav_trial as deploy
import robot_install_supervised_nav_trial as robot_installer


def test_windows_ssh_options_do_not_use_unix_control_socket(monkeypatch):
    monkeypatch.setattr(deploy.os, 'name', 'nt')
    options = deploy.ssh_common_options('/tmp/not-a-windows-socket')
    assert not any('ControlMaster' in value or 'ControlPath' in value
                   for value in options)
    assert 'ConnectTimeout=30' in options
    assert 'ConnectionAttempts=2' in options


def test_posix_ssh_options_keep_connection_multiplexing(monkeypatch):
    monkeypatch.setattr(deploy.os, 'name', 'posix')
    options = deploy.ssh_common_options('/tmp/control')
    assert 'ControlMaster=auto' in options
    assert 'ControlPath=/tmp/control' in options


def test_shell_files_are_transferred_with_unix_line_endings(tmp_path):
    source = tmp_path/'source'
    source.mkdir()
    script = source/'start.sh'
    script.write_bytes(b'#!/bin/bash\r\nset -eo pipefail\r\n')
    transfer = tmp_path/'transfer'
    transfer.mkdir()
    prepared = deploy.prepare_transfer_files((script,), transfer)
    assert prepared == (transfer/'start.sh',)
    assert prepared[0].read_bytes() == b'#!/bin/bash\nset -eo pipefail\n'


def test_robot_launcher_recovery_normalizes_existing_crlf(tmp_path):
    launcher = tmp_path/'start_robot_api.sh'
    launcher.write_bytes(b'#!/bin/bash\r\nset -eo pipefail\r\n')
    assert robot_installer.normalize_shell_launcher(launcher) is True
    assert launcher.read_bytes() == b'#!/bin/bash\nset -eo pipefail\n'
    assert robot_installer.normalize_shell_launcher(launcher) is False
