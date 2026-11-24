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
    run('release 21', ok=False)
    run('brake 21 unknown', ok=False)
    # 任意关节组合，三种模式
    out = run('brake 21 release', 'brake left 1', 'status 15', 'brake all normal',
              'brake right hold', 'status 22', 'brake all 0', 'enable 15')
    assert 'released' in out and 'held' in out, out
    # 非 normal 期间拒绝使能和零力矩拖动
    run('brake 21 release', 'enable 22', ok=False)
    run('brake 21 hold', 'free 22', ok=False)
    # 使能中的关节拒绝强制松闸
    run('enable 21', 'brake 21 release', ok=False)
    # 零力矩拖动：kp = 0；kd 不超过上限
    out = run('free 21 0.2', 'status 21')
    row = next(l.split() for l in out.splitlines() if l.startswith('21 ') and 'L/6' in l)
    assert row[3] == 'enabled' and float(row[11]) == 0.0 and abs(float(row[12]) - 0.2) < 1e-9, row
    run('free 21 99', ok=False)
    # 参数读写、保存、硬件校零
    out = run('pn 21 Pn075', 'pn 21 028 12', 'pn 21 28', 'sdo 21 0x2028 0',
              'pn_save all', 'zero 21 --hw')
    assert '(12)' in out, out
    assert '(13)' in run('sdo 21 0x2028 0 13', 'pn 21 28')
    run('zero 21 --bad', ok=False)
    # 同批包含已失能且强制松闸的 20、正在使能的 21；未选中的 22 不能被失能。
    before_save = config.read_bytes()
    result = run('enable 21,22', 'brake 20 release', 'pn_save 20,21', 'status 20,21,22',
                 'brake 20 hold', 'pn_save 20', 'brake 20')
    states = {p[0]: p[3] for p in (line.split() for line in result.splitlines())
              if len(p) >= 4 and p[2] in ('L/5', 'L/6', 'R/0')}
    assert states == {'20': 'released', '21': 'locked', '22': 'enabled'}, result
    assert result.count('已发送参数保存脉冲并回读（Pn097）') == 3 and 'hold    (Pn075=2' in result, result
    run('free 21', 'disable 21', 'pn_save 21')
    assert config.read_bytes() == before_save
    run('brake 21 hold', 'zero 21 --hw', ok=False)
    # 无失能反馈时不能校零或改文件；正常的硬件校零则按需失能。
    before = config.read_bytes()
    for cmd in ('zero 21', 'zero 21 --hw', 'pn_save 21'):
        result = run('free 21', cmd, env={'ARMS_MOCK_STUCK_ENABLED': '1'}, ok=False)
        assert '未确认失能并静止' in result and '已发送参数保存脉冲' not in result, result
    assert config.read_bytes() == before
    result = run('free 21', 'zero 21 --hw', 'status 21')
    assert any(l.startswith('21 ') and 'locked' in l for l in result.splitlines()), result
    run('sdo 21 0x6040 0 15', ok=False)
    # 软件校零：立即生效，只改动该关节的 offset_rad，保留其余文本
    before = config.read_text()
    out = run('free 21', 'zero 21', 'status 21')
    assert any(l.startswith('21 ') and 'locked' in l for l in out.splitlines()), out
    assert abs(pos(out, 21)) < 1e-3, out
    after = config.read_text()
    diff = [(a, b) for a, b in zip(before.splitlines(), after.splitlines()) if a != b]
    assert len(before.splitlines()) == len(after.splitlines()) and len(diff) == 1, diff
    assert '"joint": 21' in diff[0][1] and '"offset_rad": 0.700000' in diff[0][1], diff
    out = run('status 21')
    assert abs(pos(out, 21)) < 1e-3, out
    # 多关节校零
    mixed = run('free 15,22', 'zero left', 'status 15,22')
    assert any(l.startswith('15 ') and 'locked' in l for l in mixed.splitlines()), mixed
    assert any(l.startswith('22 ') and 'enabled' in l for l in mixed.splitlines()), mixed
    run('free 21', 'disable 21', 'zero 21')
    data = json.loads(config.read_text())
    assert all(j['offset_rad'] != 0.0 for j in data['joints'] if 15 <= j['joint'] <= 21), data
    # 字段顺序不影响校零，也不允许误改嵌套元数据或相邻关节。
    joint = next(j for j in data['joints'] if j['joint'] == 21)
    joint.pop('offset_rad')
    joint['metadata'] = {'joint': 21, 'offset_rad': 17.5}
    joint['joint'] = joint.pop('joint')  # joint 位于对象末尾
    config.write_text(json.dumps(data, indent=2))
    run('zero 21')
    joint['offset_rad'] = 0.7
    assert json.loads(config.read_text()) == data
    joint['joint'] = joint.pop('joint')  # offset_rad 在 joint 之前
    joint['offset_rad'] = -0.25
    config.write_text(json.dumps(data, indent=2))
    run('zero 21')
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
    run('pn_save 21', env={'ARMS_MOCK_PN075_UNREADABLE': '1'})
    for cmd in ('enable 21', 'free 21', 'zero 21 --hw'):
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
