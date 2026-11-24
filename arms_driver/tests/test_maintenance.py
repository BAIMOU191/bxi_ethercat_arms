"""Isolated CLI tests: never select the hardware backend or modify a shipped config."""
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile

exe, original = sys.argv[1:]
with tempfile.TemporaryDirectory() as tmp:
    config = pathlib.Path(tmp) / 'arms.json'
    shutil.copyfile(original, config)
    def run(*commands, options=(), ok=True, env=None):
        args = [exe, '--mock', '-c', str(config), *options]
        for cmd in commands:
            args += ['-e', cmd]
        proc = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              env=dict(os.environ, **(env or {})), text=True, timeout=25)
        assert (proc.returncode == 0) == ok, (args, proc.stdout)
        return proc.stdout
    out = run('help', 'selftest', 'info 21', 'identify 21', 'status 21 --raw', 'reg 21 0x130')
    for word in ('selftest', 'release', 'identify', 'pn_save', 'TxPDO:', 'RxPDO:', 'serial=', '0x0008'):
        assert word in out, word
    # 维护命令须带 --confirm
    run('release 21', ok=False)
    # 可同时松开多个关节、单臂或全部关节
    out = run('release 21 --confirm', 'release 28 --confirm', 'release left --confirm', 'status 15',
              'lock all --confirm', 'release all --confirm', 'lock all --confirm', 'enable 15')
    assert out.count('已松闸') >= 2 + 7 + 14 and 'released' in out, out
    # 松闸期间拒绝使能
    run('release 21 --confirm', 'enable 22', ok=False)
    # 使能中的关节拒绝松闸
    run('enable 21', 'release 21 --confirm', ok=False)
    out = run('release 21 --confirm','status 21','lock 21 --confirm','enable 21','disable','pn 21 Pn075',
              'pn 21 028 12 --confirm','pn 21 28','sdo 21 0x2028 0',
              'pn_save 21 --confirm','zero 21 --hw --confirm')
    assert 'released' in out and '(12)' in out, out
    run('sdo 21 0x6040 0 15 --confirm', ok=False)
    data=json.loads(config.read_text()); data['safety']['check_motors']=False
    next(j for j in data['joints'] if j['joint']==21)['offset_rad']=0.7
    config.write_text(json.dumps(data)); run('zero 21 --confirm')
    assert next(j for j in json.loads(config.read_text())['joints'] if j['joint']==21)['offset_rad']!=0.7
    env={'ARMS_MOCK_SLAVES':'5'}
    run('check',env=env,options=('--check',),ok=False)
    run('check',env=env,options=('--check-motor','1','--motor-disable','0x18300000'))
    run('check',env=env,options=('--check-motor','0'))
    run('check',options=('--check-motor','2'),ok=False)
