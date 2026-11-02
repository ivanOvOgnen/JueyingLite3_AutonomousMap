#!/usr/bin/env python3
"""Record read-only robot API status while testing without internet access."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import urllib.request


PRIVATE_FIELDS = ('password', 'secret', 'token', 'cookie', 'authorization')


def redact(value):
    if isinstance(value, dict):
        return {key: ('[redacted]' if any(word in key.lower() for word in PRIVATE_FIELDS) else redact(item))
                for key, item in value.items()}
    if isinstance(value, list):
        return [redact(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=18080)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--output', type=Path)
    options = parser.parse_args()
    if not 1024 <= options.port <= 65535 or options.interval < 0.2:
        parser.error('Use a port from 1024–65535 and an interval of at least 0.2 seconds')
    root = Path(__file__).resolve().parents[1]
    output = options.output or root / 'log' / f'robot_test_{datetime.now().strftime("%Y%m%d_%H%M%S")}.jsonl'
    output.parent.mkdir(parents=True, exist_ok=True)
    url = f'http://127.0.0.1:{options.port}/api/v1/status'
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    print(f'Recording read-only status from {url}\nFile: {output}\nPress Ctrl+C when finished.', flush=True)
    count = 0
    try:
        with output.open('x', encoding='utf-8') as file:
            while True:
                event = {'observed_utc': datetime.now(timezone.utc).isoformat(), 'url': url}
                try:
                    with opener.open(url, timeout=3) as response:
                        status = json.load(response)
                    if status.get('simulation') is True:
                        raise ValueError('This is the virtual demo, not the robot API')
                    event['status'] = redact(status)
                    try:
                        with opener.open(f'http://127.0.0.1:{options.port}/api/v1/maps', timeout=3) as response:
                            catalog = json.load(response)
                        event['maps'] = [item.get('name') for item in catalog.get('maps', [])
                                         if isinstance(item, dict) and isinstance(item.get('name'), str)]
                    except (OSError, ValueError, TypeError):
                        event['maps_error'] = 'Map catalog unavailable'
                except (OSError, ValueError, TypeError) as error:
                    event['error'] = str(error)
                file.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
                file.flush()
                count += 1
                time.sleep(options.interval)
    except KeyboardInterrupt:
        print(f'\nSaved {count} observations to {output}', flush=True)


if __name__ == '__main__':
    main()
