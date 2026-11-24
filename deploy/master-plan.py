"""Print EtherLab master bindings for review, without editing a host or starting a service."""
import argparse
import json
from pathlib import Path
from re import fullmatch
p=argparse.ArgumentParser(description=__doc__);p.add_argument('config',type=Path);args=p.parse_args()
cfg=json.loads(args.config.read_text()); result={}
for slot,side in enumerate(('left','right')):
    arm=cfg['arms'][side]; nic=arm['interface']; declared=arm.get('mac','').lower()
    if not fullmatch(r'[A-Za-z0-9_.:-]{1,15}',nic):p.error('invalid interface name')
    mac=Path('/sys/class/net',nic,'address').read_text().strip().lower()
    if not fullmatch(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}',mac) or declared not in ('',mac):p.error(f'{nic}: MAC mismatch')
    master=arm.get('master');master=slot if master is None else master
    if type(master)!=int or master<0 or master in result or mac in result.values():p.error('duplicate or invalid binding')
    result[master]=mac
for master,mac in sorted(result.items()):print(f'MASTER{master}_DEVICE="{mac}"')
