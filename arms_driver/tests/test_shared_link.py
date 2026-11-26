"""双臂共用网口：两臂经分支器接在同一条链上（链上位置 0 为分支器），只使用模拟后端。"""
import json
import os
import pathlib
import subprocess
import sys
import tempfile

exe, original = sys.argv[1:]
with tempfile.TemporaryDirectory() as tmp:
    tmp = pathlib.Path(tmp)
    base = json.loads(pathlib.Path(original).read_text())

    def make(name, edit):
        data = json.loads(json.dumps(base))
        data['arms']['right'] = dict(data['arms']['left'])
        for j in data['joints']:
            j['slave'] = j['joint'] - 14  # 左臂 1..7，右臂 8..14
        edit(data)
        path = tmp / name
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        return path

    def run(config, *commands, ok=True, hub=None):
        args = [exe, '--mock', '-c', str(config)]
        for cmd in commands:
            args += ['-e', cmd]
        env = dict(os.environ, ARMS_MOCK_SLAVES='15')
        if hub is not None:
            env['ARMS_MOCK_HUB'] = hub
        p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=30,
                           env=env, stdin=subprocess.DEVNULL)
        assert (p.returncode == 0) == ok, (args, p.stdout)
        return p.stdout

    shared = make('shared.json', lambda d: None)
    out = run(shared, 'check', 'bus', 'enable 15,22', 'mit 22 0.1 5 0.5', 'status 15,22', 'disable')
    assert 'left+right 臂: 扫描完成，发现 15 个从站（预期 15）' in out, out
    assert '已打开的 14 个关节全部在线' in out, out
    assert 'left+right eth0 主站0' in out and 'right 未打开' not in out, out
    rows = {f[0]: f for f in (l.split() for l in out.splitlines()) if len(f) > 10 and f[0] in ('15', '22') and '/' in f[2]}
    assert rows['15'][2] == 'L/1' and rows['22'][2] == 'R/8', rows
    assert rows['22'][3] == 'enabled' and abs(float(rows['22'][10]) - 0.1) < 1e-9, rows

    # 屏蔽右臂时只打开左臂关节
    masked = make('masked.json', lambda d: d['safety'].update(ignore_mask='0x1fc00000'))
    out = run(masked, 'check')
    assert '已打开的 7 个关节全部在线' in out, out

    # 共用网口时 mac、master 必须一致；同一条链上 slave 不能重复
    bad_mac = make('bad_mac.json', lambda d: d['arms']['right'].update(mac='00:11:22:33:44:55'))
    assert 'mac、master 必须一致' in run(bad_mac, 'check', ok=False)
    dup = make('dup.json', lambda d: d['joints'][7].update(slave=1))
    assert '共用链路上的 slave 1 被多个关节使用' in run(dup, 'check', ok=False)

    # 集线器端口绑定：OUT1 = ESC 端口 3 接左臂，OUT2 = ESC 端口 1 接右臂，OUT3 空
    hub = make('hub.json', lambda d: d['arms'].update(hub={'slave': 0, 'left': 1, 'right': 2}))
    out = run(hub, 'check', hub='3=1,1=8')
    assert '集线器（链上位置 0）端口核对通过: OUT1 → 左臂（链上位置 1～7），OUT2 → 右臂（链上位置 8～14）' in out, out
    # 左右臂插反
    out = run(hub, 'check', hub='3=8,1=1', ok=False)
    assert '左臂应接 OUT1，实际接在 OUT2' in out, out
    # 右臂所在输出口未连接
    out = run(hub, 'check', hub='3=1', ok=False)
    assert 'OUT2 的下一个从站' in out or '右臂应接 OUT2' in out, out
    # 屏蔽右臂时只核对左臂
    hub_masked = make('hub_masked.json', lambda d: (d['arms'].update(hub={'slave': 0, 'left': 1, 'right': 2}),
                                                    d['safety'].update(ignore_mask='0x1fc00000')))
    out = run(hub_masked, 'check', hub='3=1,1=8')
    assert 'OUT1 → 左臂' in out and 'OUT2' not in out.split('端口核对通过')[1].splitlines()[0], out
    # 配置校验
    for bad, msg in [({'slave': 1, 'left': 1}, '与关节 15 的 slave 相同'),
                     ({'slave': 0, 'left': 4}, 'arms.hub.left 必须是集线器输出口 1..3'),
                     ({'slave': 0, 'left': 2, 'right': 2}, '左右臂不能接同一个输出口 OUT2')]:
        cfg = make('bad_hub.json', lambda d, h=bad: d['arms'].update(hub=h))
        assert msg in run(cfg, 'check', ok=False)
print('OK')
