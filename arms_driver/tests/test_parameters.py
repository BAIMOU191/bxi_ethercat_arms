"""Round-trip and interlock tests against an isolated mock, never the hardware backend."""
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

exe, original = sys.argv[1:]
# V1.2 parameter table, the additional entries named in the mode/alarm chapters,
# and the established save trigger. This tests documentation, not access permissions.
manual = {0,1,2,4,5,6,9,10,11,20,21,24,26,28,31,32,33,34,35,36,39,42,44,45,
          49,50,51,63,64,65,70,71,74,75,77,79,80,82,85,88,91,92,96,97,101,
          106,107,111,148,150}
with tempfile.TemporaryDirectory() as work:
    config = pathlib.Path(work) / 'arms.json'
    shutil.copyfile(original, config)
    before = config.read_bytes()

    def run(*cmds, fail=False, env=None):
        args = [exe, '--mock', '-c', str(config)]
        for cmd in cmds:
            args.extend(['-e', cmd])
        p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                           timeout=20, env={**os.environ, **(env or {})}, stdin=subprocess.DEVNULL)
        assert (p.returncode != 0) == fail, (cmds, p.stdout)
        return p.stdout

    listing = run('pn list')
    assert {int(n) for n in re.findall(r'^Pn(\d+)  0x', listing, re.M)} == manual, listing
    assert '51/81/101' in run('pn help 051') and 'A/N·m' in run('pn help torque_scale')
    out = run('brake_voltage 15 012', 'pn 15 brake_voltage', 'sdo 15 0x2026 0',
              'pn 15 torque_scale 11', 'torque_scale 15', 'sdo 15 0x2082 0 12', 'pn 15 082',
              'pn 15 Pn035 -3', 'sdo 15 0x2035 0', 'sdo 15 0x1018 4')
    assert out.count('0x2026:00 = 0c 00 (12)  12 V') == 2, out
    for word in ('1.1 A/N·m', '1.2 A/N·m', 'fd ff (-3)', '0x1018:04 ='):
        assert word in out, out
    # Same OD addressing above/below the ESI boundary, non-catalogued values and revision.
    out = run('pn 15 199 -7', 'sdo 15 0x2199 0', 'sdo 15 0x2345 0 -32768', 'pn 15 345',
              'pn 15 004 2', 'pn 15 004', env={'ARMS_MOCK_REVISION':'1'})
    for line in ('0x2199:00 = f9 ff (-7)', '0x2345:00 = 00 80 (-32768)', '0x2004:00 = 02 00 (2)'):
        assert line in out, out
    # Non-Pn records, nonzero subindices, explicit wire widths, write-only device object.
    out = run('sdo 15 0x5ffe 1 u32:0xfedcba98', 'sdo 15 0x5ffe 1',
              'sdo 15 0x5ffe 1 hex:01020304', 'sdo 15 0x5ffe 1',
              'sdo 15 0x5ffe 2 u8:255', 'sdo 15 0x5ffe 3 i64:-9223372036854775808', 'sdo 15 0x5ffe 3')
    for word in ('98 ba dc fe', '01 02 03 04', '00 00 00 00 00 00 00 80'):
        assert word in out, out
    for cmd, abort in [('sdo 15 0x1018 4 u32:1','06010002'), ('sdo 15 0x5ffe 2','06010001'),
                       ('sdo 15 0x5ffe 1 u8:1','06070010'), ('sdo 15 0x2075 1 1','06090011'),
                       ('pn 15 999 0','06020000'), ('pn 15 999','06020000'), ('brake_voltage 15 25','06090030')]:
        assert 'abort=0x'+abort in run(cmd, fail=True), cmd
    for cmd in ('pn 15 035 -32769', 'pn 15 035 32768', 'sdo 15 0x2097 0 0', 'sdo 15 0x2101 0 u8:1',
                'sdo 15 0x6040 0 15', 'sdo 15 0x1601 0 u8:0', 'sdo 15 0x5ffe 2 u8:256',
                'sdo 15 0x5ffe 2 hex:0', 'sdo 15 0x5ffe 2 hex:gz', 'sdo 15 0x5ffe 1 u32:-1',
                'pn list 1', 'pn help 199'):
        run(cmd, fail=True)
    assert '未确认失能' in run('enable 15', 'pn 15 199 7', fail=True)
    assert '未确认失能' in run('enable 15', 'sdo 15 0x5ffe 2 u8:1', fail=True)
    assert '拒绝' in run('sdo 15 0x2075 0 1', 'enable 16', fail=True)
    assert '抱闸' in run('pn 15 75 2', 'sdo 15 0x2101 0 1', fail=True)
    out = run('free 15', 'pn 15 097 1', 'sdo 15 0x2097 0', 'sdo 15 0x2101 0 1', 'pn 15 101')
    for word in ('已发送参数保存脉冲', '0x2097:00 = 00 00 (0)', '硬件清零脉冲', '0x2101:00 = 00 00 (0)'):
        assert word in out, out
    assert config.read_bytes() == before
