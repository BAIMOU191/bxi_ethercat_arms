"""驱动器监控：ESC 错误计数、温度、驱动器丢帧、紧急报文与过温保护（只使用模拟后端）。"""
import json
import os
import pathlib
import subprocess
import sys
import tempfile

exe, original = sys.argv[1:]
with tempfile.TemporaryDirectory() as tmp:
    tmp = pathlib.Path(tmp)
    data = json.loads(pathlib.Path(original).read_text())
    data['safety']['wkc_tolerance'] = 3
    config = tmp / 'arms.json'
    config.write_text(json.dumps(data, indent=2, ensure_ascii=False))

    def run(name, env, *commands, cfg=config):
        log = tmp / f'{name}.log'
        args = [exe, '--mock', '-c', str(cfg), '--event-log', str(log)]
        for cmd in commands:
            args += ['-e', cmd]
        p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=40,
                           env=dict(os.environ, **env), stdin=subprocess.DEVNULL)
        return p.stdout, (log.read_text() if log.exists() else '')

    def row(out, joint):
        return next(l.split() for l in out.splitlines() if l.startswith(f'{joint} ') and ('L/' in l or 'R/' in l))

    # 温度和驱动器丢帧：status 末列、diag 表都能读到
    out, _ = run('normal', {}, 'commtest 4', 'status 15,28', 'diag 21')
    assert '结果: 通过' in out, out
    assert row(out, 15)[-1] == '35' and row(out, 28)[-1] == '41', out
    assert '线路类 端口0 无效/物理/丢链' in out and row(out, 21)[3] == '41' and row(out, 21)[4] == '0', out

    # ESC 错误计数增加：commtest 列出关节 18（左臂链上位置 3）并判为不通过，日志记录端口 1 的计数变化
    out, log = run('esc', {'ARMS_MOCK_ESC_ERR': '1500'}, 'commtest 4')
    assert '通信测试未通过' in out and any(l.startswith('18 ') for l in out.splitlines()), out
    assert '线路错误计数增加' in log and '关节 18（链上位置 3）计数变化: 端口1无效帧 0→' in log, log
    assert '端口1物理层错误' in log and '最近 16 个周期' in log, log

    # 只有处理类计数（主机报文被拒）：不逐条记录，按窗口汇总一条；commtest 只提示，不判为不通过
    data['safety']['esc_summary_s'] = 2
    proc_cfg = tmp / 'proc.json'
    proc_cfg.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    out, log = run('proc', {'ARMS_MOCK_ESC_PROC': '1200'}, 'commtest 6', cfg=proc_cfg)
    assert '结果: 通过' in out and '只有处理类计数' in out, out
    assert '线路错误计数增加' not in log and '从站拒绝的帧（汇总）' in log, log
    assert '最先拒绝的是关节 15（链上位置 0）；线路类计数未变' in log, log
    del data['safety']['esc_summary_s']

    # 驱动器自报丢帧增加
    out, log = run('loss', {'ARMS_MOCK_DRIVE_LOSS': '1000'}, 'commtest 4')
    assert '驱动器丢帧' in log and '驱动器自报 EtherCAT 丢帧次数' in log, log
    assert '通信测试未通过' in out, out

    # 紧急报文
    out, log = run('emcy', {'ARMS_MOCK_EMCY': '1500'}, 'commtest 2')
    assert '驱动器紧急报文' in log and '关节 15（链上位置 0）错误码 0x7500  错误寄存器 0x10' in log, log

    # 温度告警：只告警，不锁定
    out, log = run('warn', {'ARMS_MOCK_TEMP': '80'}, 'commtest 4', 'bus', 'enable 21')
    assert '温度告警' in log and '执行器温度 80 ℃，阈值 75 ℃' in log, log
    assert '超限保护: 正常' in out, out

    # 过温保护：全部失能并锁定，拒绝使能
    out, log = run('hot', {'ARMS_MOCK_TEMP': '95'}, 'commtest 4', 'bus', 'enable 21')
    assert '过温保护' in log and '全部关节已失能并锁定' in log, log
    assert '温度(℃) 95.0000 超过阈值 90.0000' in out and '超速/超力矩/过温保护已触发' in out, out

    # 关闭驱动器监控：不读取温度，也不记录相关事件
    data['safety']['drive_monitor'] = False
    off = tmp / 'off.json'
    off.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    out, log = run('off', {'ARMS_MOCK_TEMP': '95'}, 'commtest 3', 'status 15', cfg=off)
    assert row(out, 15)[-1] == '-' and '过温' not in log, (out, log)

    # 配置校验
    data['safety'].update(drive_monitor=True, temp_warn_c=95, temp_max_c=90)
    bad = tmp / 'bad.json'
    bad.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    out, _ = run('bad', {}, 'status', cfg=bad)
    assert 'temp_warn_c 不能大于' in out, out
print('PASS: drive monitoring (ESC counters, temperature, drive frame loss, emergency, overtemperature)')
