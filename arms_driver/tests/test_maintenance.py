"""Isolated CLI tests: never select the hardware backend or modify a shipped config."""
import json
import os
import pathlib
import shutil
import select
import time
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
                              env=dict(os.environ, **(env or {})), stdin=subprocess.DEVNULL, text=True, timeout=25)
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
    # 字段顺序不影响校零，也不允许误改嵌套元数据或相邻关节。
    joint = next(j for j in data['joints'] if j['joint'] == 21)
    joint.pop('offset_rad')
    joint['metadata'] = {'joint': 21, 'offset_rad': 17.5}
    joint['joint'] = joint.pop('joint')  # joint 位于对象末尾
    config.write_text(json.dumps(data, indent=2))
    run('zero 21 --confirm')
    joint['offset_rad'] = 0.7
    assert json.loads(config.read_text()) == data
    joint['joint'] = joint.pop('joint')  # offset_rad 在 joint 之前
    joint['offset_rad'] = -0.25
    config.write_text(json.dumps(data, indent=2))
    run('zero 21 --confirm')
    joint['offset_rad'] = 0.7
    assert json.loads(config.read_text()) == data
    # SIGTERM 不能退回提示符，即便标准输入保持打开、命令只输入一半。
    with subprocess.Popen([exe, '--mock', '-c', str(config)], stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT) as proc:
        try:
            seen = b''
            deadline = time.monotonic() + 8
            while '输入 help 查看命令'.encode() not in seen:
                assert select.select([proc.stdout], [], [], max(0, deadline - time.monotonic()))[0], seen
                chunk = os.read(proc.stdout.fileno(), 8192)
                assert chunk, seen
                seen += chunk
            proc.stdin.write(b'enable 21')
            proc.stdin.flush()
            proc.terminate()
            assert proc.wait(timeout=3) == 0
        finally:
            if proc.poll() is None:
                proc.kill()
    out = run('watch 21')
    assert 'l_wrist_z_joint' in out and '\x1b[' not in out, out
    for cmd in ('enable 21', 'free 21', 'pn_save 21 --confirm', 'zero 21 --hw --confirm'):
        run(cmd, env={'ARMS_MOCK_PN075_UNREADABLE': '1'}, ok=False)
    # 修改副本中的安全字段类型时，启动必须报配置错误，而不能沿用默认值。
    valid = config.read_text()
    for parent, field in [(data['joints'][0]['limit'], 'vel'), (data['joints'][0], 'offset_rad'),
                          (data, 'safety'), (data['joints'][0], 'gain_max')]:
        previous = parent[field]
        parent[field] = 'bad-type'
        config.write_text(json.dumps(data))
        run('check', ok=False)
        parent[field] = previous
    config.write_text(valid)
    env={'ARMS_MOCK_SLAVES':'5'}
    run('check',env=env,options=('--check',),ok=False)
    run('check',env=env,options=('--check-motor','1','--motor-disable','0x18300000'))
    run('check',env=env,options=('--check-motor','0'))
    run('check',options=('--check-motor','2'),ok=False)
