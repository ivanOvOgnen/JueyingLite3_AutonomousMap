#!/usr/bin/env python3
"""Passive Retroid and vendor-protection observer for the Lite3 motion host.

This is the project-owned source for the motion-host observer installed at
``/opt/emotion-bot-motion/motion_stop_observer.py``.  It never publishes a
velocity command or writes to the vendor controller.  It relays only the
authenticated safety observation consumed by the perception-side supervisor.

The vendor app's STOP action is not represented by the Retroid packet that the
original observer decodes.  The vendor process records the action as
``Soft Emergency Pressed!!!`` and then ``is_robot_lose_control = 1``.  The
optional log watcher turns a *new* occurrence of either message into a
time-limited STOP observation.  It is deliberately an additional signal and
does not claim that the vendor action is an independently verified physical
e-stop.
"""

import argparse
import glob
import os
import secrets
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "lib"))

from emotion_bot_lite3_hw.protocol import (  # noqa: E402
    AXIS_CODES,
    PacketError,
    filter_datagram,
    parse_command,
    parse_ethernet_udp,
    parse_simple_stop,
    retroid_axes_zero,
)
from emotion_bot_lite3_hw.stop_transport import (  # noqa: E402
    StopRelayFrame,
    encode_frame,
    load_key,
)


class VendorStopLogWatcher:
    """Tail only new vendor log lines and latch a short stop indication."""

    PATTERNS = ("Soft Emergency Pressed!!!", "is_robot_lose_control = 1")

    def __init__(self, directory, hold_seconds):
        self.directory = directory
        self.hold_seconds = max(0.5, float(hold_seconds))
        self.path = None
        self.stream = None
        self.stop_until = 0.0
        self._open_latest(seek_end=True)

    def _latest_path(self):
        paths = glob.glob(os.path.join(self.directory, "*", "deeprcs.*.log"))
        if not paths:
            return None
        try:
            return max(paths, key=lambda item: os.stat(item).st_mtime_ns)
        except OSError:
            # Log rotation can remove a candidate between glob and stat.
            return None

    def _open_latest(self, seek_end):
        path = self._latest_path()
        if path == self.path and self.stream is not None:
            return
        if self.stream is not None:
            self.stream.close()
        self.stream = None
        self.path = path
        if path is None:
            return
        try:
            self.stream = open(path, "r", encoding="utf-8", errors="replace")
            if seek_end:
                self.stream.seek(0, os.SEEK_END)
        except (OSError, UnicodeError):
            self.stream = None

    def poll(self, now):
        """Return true when a new vendor protection line was observed."""
        self._open_latest(seek_end=True)
        if self.stream is None:
            return False
        try:
            # Reopen after truncation or replacement without replaying old data.
            if os.fstat(self.stream.fileno()).st_size < self.stream.tell():
                self.stream.close()
                self.stream = None
                self._open_latest(seek_end=True)
                return False
            triggered = False
            for line in self.stream:
                if any(pattern in line for pattern in self.PATTERNS):
                    self.stop_until = max(self.stop_until, now + self.hold_seconds)
                    triggered = True
            return triggered
        except OSError:
            return False

    def active(self, now):
        return now <= self.stop_until

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="p2p0")
    parser.add_argument("--retroid-ip", default="192.168.2.196")
    parser.add_argument("--motion-wifi-ip", default="192.168.2.1")
    parser.add_argument("--command-port", type=int, default=43893)
    parser.add_argument("--source-ip", default="192.168.1.120")
    parser.add_argument("--source-port", type=int, default=43911)
    parser.add_argument("--destination-ip", default="192.168.1.103")
    parser.add_argument("--destination-port", type=int, default=43910)
    parser.add_argument("--key-file", default="/etc/emotion-bot/stop-relay.key")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--retroid-timeout", type=float, default=0.75)
    parser.add_argument("--stop-hold", type=float, default=2.0)
    parser.add_argument("--vendor-log-dir", default="/home/ysc/jy_exe/log")
    parser.add_argument("--vendor-stop-hold", type=float, default=5.0)
    return parser.parse_args()


def main():
    args = parse_args()
    key = load_key(args.key_file)
    session_id = secrets.token_hex(16)
    sequence = 0
    last_packet = None
    last_nonzero_axis = None
    stop_until = 0.0
    axis_state = {code: None for code in AXIS_CODES}
    vendor_log = VendorStopLogWatcher(args.vendor_log_dir, args.vendor_stop_hold)

    raw = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
    raw.bind((args.interface, 0))
    raw.setblocking(False)
    relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    relay.bind((args.source_ip, args.source_port))
    relay_destination = (args.destination_ip, args.destination_port)
    period = 1.0 / args.rate
    next_publish = time.monotonic()

    try:
        while True:
            while True:
                try:
                    datagram = parse_ethernet_udp(raw.recv(65535))
                    payload = filter_datagram(
                        datagram, args.retroid_ip, args.command_port, args.motion_wifi_ip
                    )
                    command = parse_command(payload)
                except BlockingIOError:
                    break
                except PacketError:
                    continue
                now = time.monotonic()
                last_packet = now
                if parse_simple_stop(command):
                    stop_until = max(stop_until, now + args.stop_hold)
                axis_zero = retroid_axes_zero(command)
                if axis_zero is not None:
                    axis_state[command.code] = axis_zero
                    if not axis_zero:
                        last_nonzero_axis = now

            now = time.monotonic()
            if vendor_log.poll(now):
                stop_until = max(stop_until, now + args.vendor_stop_hold)
            if vendor_log.active(now):
                stop_until = max(stop_until, vendor_log.stop_until)
            if now < next_publish:
                time.sleep(min(next_publish - now, 0.01))
                continue
            observed_fresh = last_packet is not None and now - last_packet <= args.retroid_timeout
            quiet = last_nonzero_axis is None or now - last_nonzero_axis > args.retroid_timeout
            axes_zero = observed_fresh and quiet and all(
                value is not False for value in axis_state.values()
            )
            sequence += 1
            try:
                relay.sendto(encode_frame(key, StopRelayFrame(
                    session_id=session_id,
                    sequence=sequence,
                    stop=now <= stop_until,
                    axes_zero=axes_zero,
                    observed_fresh=observed_fresh,
                )), relay_destination)
            except OSError:
                # A restarting receiver is handled fail-closed by its stale gate.
                pass
            next_publish = now + period
    finally:
        vendor_log.close()
        raw.close()
        relay.close()


if __name__ == "__main__":
    main()
