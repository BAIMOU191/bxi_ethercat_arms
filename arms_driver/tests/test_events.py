"""通信事件日志：模拟丢帧和掉电，检查事件与周期时序写入日志文件（只使用模拟后端）。"""
import json
import os
import pathlib
import subprocess
import sys
import tempfile

exe, original = sys.argv[1:]
with tempfile.TemporaryDirectory() as tmp:
    tmp = pathlib.Path(tmp)
    config = tmp / 'arms.json'
    data = json.loads(pathlib.Path(original).read_text())
    data['safety']['wkc_tolerance'] = 3
    data['safety']['offline_action'] = 'power_off'
    config.write_text(json.dumps(data, indent=2, ensure_ascii=False))

    def run(log, env, *commands):
        args = [exe, '--mock', '-c', str(config), '--event-log', str(log)]
        for cmd in commands:
            args += ['-e', cmd]
        p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=30,
                           env=dict(os.environ, **env), stdin=subprocess.DEVNULL)
        return p.stdout

    # 偶发丢帧：记录 WKC 异常，附带周期时序表
    log = tmp / 'drop.log'
    out = run(log, {'ARMS_MOCK_DROP_PPM': '20000'}, 'commtest 2')
    text = log.read_text()
    assert '#### 会话' in text and 'WKC 容忍 3 次' in text, text
    assert 'WKC 异常' in text and '发送→接收us' in text and 'journalctl -k' in text, text
    assert '已记录' in out, out

    # 左臂掉电：记录通信失败和离线保护触发，附带完整 64 个周期
    log = tmp / 'power.log'
    run(log, {'ARMS_MOCK_POWER_OFF': '2500,300'}, 'commtest 4')
    text = log.read_text()
    assert '通信失败  left 臂' in text and '离线保护触发' in text, text
    fail = text[text.index('通信失败  left 臂'):]
    assert '最近 64 个周期' in fail and 'AL' in fail, fail

    # 默认路径：配置目录不是 config/ 时写到配置目录下的 logs/
    out = run('', {}, 'status 21')
    assert str(tmp / 'logs' / 'comm_events.log') in out, out
print('PASS: communication events logged with cycle timing')
