"""Print proposed EtherLab bindings; do not change the host or start a service."""
import argparse
import json
from pathlib import Path
import re


def bindings(arms, net=Path('/sys/class/net')):
    result = {}
    for slot, side in enumerate(('left', 'right')):
        if side not in arms:
            continue  # A single-arm configuration is valid.
        arm = arms[side]
        nic = arm['interface']
        if not isinstance(nic, str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', nic) or nic in ('.', '..'):
            raise ValueError(f'{side}: invalid interface name')
        mac = (net / nic / 'address').read_text().strip().lower()
        declared = arm.get('mac', '')
        if (not isinstance(declared, str) or declared.lower() not in ('', mac)
                or not re.fullmatch(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', mac)):
            raise ValueError(f'{nic}: MAC mismatch')
        master = arm.get('master')
        if master is None:
            master = slot
        if type(master) is not int or not 0 <= master <= 7 or master in result or mac in result.values():
            raise ValueError(f'{side}: duplicate or invalid binding')
        result[master] = mac
    if not result:
        raise ValueError('no arm configured')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text())
        plan = bindings(config['arms'])
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    for master, mac in sorted(plan.items()):
        print(f'MASTER{master}_DEVICE="{mac}"')


if __name__ == '__main__':
    main()
