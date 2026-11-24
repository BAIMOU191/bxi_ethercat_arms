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
    def pos(out, joint):
        # status 列: 关节 名称 臂/位置 状态 AL SW ERR pos ...
        rows = [l.split() for l in out.splitlines() if l.startswith(f'{joint} ') and ('L/' in l or 'R/' in l)]
        return float(rows[-1][7])

    out = run('help', 'selftest', 'info 21', 'identify 21', 'status 21 --raw', 'reg 21 0x130', 'brake all')
    for word in ('selftest', 'brake', 'free', 'identify', 'pn_save', 'TxPDO:', 'RxPDO:', 'serial=', '0x0008',
                 'normal  (Pn075=0'):
        assert word in out, word
    # release / lock 已由 brake 取代
    run('release 21 --confirm', ok=False)
    # 修改抱闸模式须带 --confirm
    run('brake 21 release', ok=False)
    # 任意关节组合，三种模式
    out = run('brake 21 release --confirm', 'brake left 1 --confirm', 'status 15', 'brake all normal --confirm',
              'brake right hold --confirm', 'status 22', 'brake all 0 --confirm', 'enable 15')
    assert 'released' in out and 'held' in out, out
    # 非 normal 期间拒绝使能和零力矩拖动
    run('brake 21 release --confirm', 'enable 22', ok=False)
    run('brake 21 hold --confirm', 'free 22', ok=False)
    # 使能中的关节拒绝强制松闸
    run('enable 21', 'brake 21 release --confirm', ok=False)
    # 零力矩拖动：kp = 0；kd 不超过上限
    out = run('free 21 0.2', 'status 21')
    row = next(l.split() for l in out.splitlines() if l.startswith('21 ') and 'L/6' in l)
    assert row[3] == 'enabled' and float(row[11]) == 0.0 and abs(float(row[12]) - 0.2) < 1e-9, row
    run('free 21 99', ok=False)
    # 参数读写、保存、硬件校零
    out = run('pn 21 Pn075', 'pn 21 028 12 --confirm', 'pn 21 28', 'sdo 21 0x2028 0',
              'pn_save all --confirm', 'zero 21 --hw --confirm')
    assert '(12)' in out, out
    run('sdo 21 0x6040 0 15 --confirm', ok=False)
    # 软件校零：立即生效，只改动该关节的 offset_rad，保留其余文本
    before = config.read_text()
    out = run('status 21', 'zero 21 --confirm', 'status 21')
    assert abs(pos(out, 21)) < 1e-3, out
    after = config.read_text()
    diff = [(a, b) for a, b in zip(before.splitlines(), after.splitlines()) if a != b]
    assert len(before.splitlines()) == len(after.splitlines()) and len(diff) == 1, diff
    assert '"joint": 21' in diff[0][1] and '"offset_rad": 0.700000' in diff[0][1], diff
    out = run('status 21')
    assert abs(pos(out, 21)) < 1e-3, out
    # 多关节校零
    run('zero left --confirm')
    data = json.loads(config.read_text())
    assert all(j['offset_rad'] != 0.0 for j in data['joints'] if 15 <= j['joint'] <= 21), data
    env={'ARMS_MOCK_SLAVES':'5'}
    run('check',env=env,options=('--check',),ok=False)
    run('check',env=env,options=('--check-motor','1','--motor-disable','0x18300000'))
    run('check',env=env,options=('--check-motor','0'))
    run('check',options=('--check-motor','2'),ok=False)
