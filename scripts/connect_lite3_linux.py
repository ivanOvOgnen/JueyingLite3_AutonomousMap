#!/usr/bin/env python3
"""Discover the robot API and expose it on this Linux laptop via SSH.

Run after joining the robot Wi-Fi. Only reads network configuration and API
status; credentials stay in OpenSSH's terminal prompts.
"""

import argparse
import ipaddress
import json
import re
import socket
import subprocess
import sys
import time
import urllib.request
import webbrowser


def command(*args):
    return subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE).stdout.strip()


def robot_gateway():
    rows = command('nmcli', '-t', '-f', 'DEVICE,TYPE,STATE', 'device').splitlines()
    candidates = []
    for row in rows:
        fields = row.split(':')
        if len(fields) < 3 or fields[1:3] != ['wifi', 'connected']:
            continue
        device = fields[0]
        ssid = command('nmcli', '-g', 'GENERAL.CONNECTION', 'device', 'show', device)
        if not ssid.startswith('YSC-JYML-'):
            continue
        gateways = command('nmcli', '-g', 'IP4.GATEWAY', 'device', 'show', device).splitlines()
        if len(gateways) != 1:
            raise ValueError('Robot Wi-Fi has no single IPv4 gateway yet; wait for DHCP and retry')
        gateway = gateways[0]
        candidates.append((ssid, str(ipaddress.IPv4Address(gateway))))
    if len(candidates) != 1:
        raise ValueError('Connect this laptop to the robot YSC-JYML Wi-Fi, then retry. The network was not changed.')
    return candidates[0]


def perception_address(text):
    values = re.findall(r'^\s*ip\s*=\s*["\']?([0-9.]+)["\']?\s*(?:#.*)?$', text, re.M)
    if len(values) != 1:
        raise ValueError('YSC network.toml must contain one unambiguous perception IP')
    return str(ipaddress.IPv4Address(values[0]))


def port_available(port):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))


def wait_for_api(url, tunnel, timeout=100):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if tunnel.poll() is not None:
            raise RuntimeError('SSH tunnel exited before the robot API responded')
        try:
            with opener.open(url + 'api/v1/status', timeout=2) as response:
                status = json.load(response)
            if status.get('ok') is True and isinstance(status.get('components'), dict) and status.get('simulation') is not True:
                return status
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(1)
    raise TimeoutError('Robot API did not answer within 100 seconds; inspect SSH and robot API startup')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-port', type=int, default=18080)
    parser.add_argument('--no-browser', action='store_true')
    options = parser.parse_args()
    if not 1024 <= options.local_port <= 65535:
        parser.error('--local-port must be between 1024 and 65535')
    try:
        ssid, gateway = robot_gateway()
        port_available(options.local_port)
        print(f'Robot Wi-Fi: {ssid}; current gateway: {gateway}', flush=True)
        print('Reading the perception address from YSC. SSH may prompt for its password.', flush=True)
        config = command('ssh', '-o', 'ConnectTimeout=10', f'ysc@{gateway}',
                         "grep -E '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml")
        perception = perception_address(config)
        print(f'Perception address from YSC: {perception}', flush=True)
        print('Opening SSH tunnel. SSH may prompt again; keep this terminal open.', flush=True)
        tunnel = subprocess.Popen(['ssh', '-N', '-o', 'ExitOnForwardFailure=yes',
                                   '-o', 'ConnectTimeout=10', '-L',
                                   f'127.0.0.1:{options.local_port}:{perception}:8080', f'ysc@{gateway}'])
        try:
            url = f'http://127.0.0.1:{options.local_port}/'
            status = wait_for_api(url, tunnel)
            fresh_url = url + '?view=' + str(time.time_ns())
            print(f'LIVE ROBOT APP: {fresh_url}', flush=True)
            contract = status.get('api_contract_version')
            label = str(contract) if contract is not None else 'legacy (no version field)'
            print(f'API contract: {label}. Press Ctrl+C to close the tunnel.', flush=True)
            if not options.no_browser:
                webbrowser.open(fresh_url)
            tunnel.wait()
        finally:
            if tunnel.poll() is None:
                tunnel.terminate()
                try:
                    tunnel.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    tunnel.kill()
                    tunnel.wait()
    except (ValueError, OSError, subprocess.CalledProcessError, TimeoutError, RuntimeError) as error:
        print(f'Connection failed: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
