"""Cross-check the registered CLI against its displayed help; execute only mock sessions."""
import os
from pathlib import Path
import re
import subprocess
import sys

exe, config = sys.argv[1:]
source = (Path(__file__).resolve().parents[1] / 'tools/arms_test/main.c').read_text()

def run(args, *, ok=True, stdin='', env=None):
    p = subprocess.run([exe, '--mock', '-c', config, *args], input=stdin, capture_output=True,
                       text=True, timeout=20, env={**os.environ, **(env or {})})
    assert (p.returncode == 0) == ok, (args, p.returncode, p.stdout, p.stderr)
    return p.stdout + p.stderr

options = source.split('static const struct option long_opts[] = {', 1)[1].split('{NULL', 1)[0]
registered = set(re.findall(r'\{"([\w-]+)"', options))
assert registered and registered <= set(re.findall(r'--([\w-]+)', run(['--help'])))
table = source.split('static const command commands[] = {', 1)[1].split('#define COMMAND_COUNT', 1)[0]
names = re.findall(r'^    \{"(\w+)"', table, re.M)
summary = run(['-e', 'help']).split('[通用]', 1)[1].split('关节写法:', 1)[0]
assert names and set(names) == set(re.findall(r'^  (\w+)\s+', summary, re.M))
for start in range(0, len(names), 8):
    args = []
    group = names[start:start+8]
    for name in group:
        args.extend(['-e', 'help ' + name, '-e', name + ' --help'])
    details = re.findall(r'^用法: (\w+).*$', run(args), re.M)
    assert details == [n for n in group for _ in range(2)], details

# EOF/pipe is not Enter for the following command; watch must not consume input.
out = run([], stdin='watch 21\nversion\nquit\n')
assert 'l_wrist_z_joint' in out and re.search(r'(?m)^arms_test \S+', out), out
assert '未知命令' not in run(['-e', 'q', '-e', 'not-a-command'])
out = run(['-e', 'brake 15', '-e', 'not-a-command'], ok=False, env={'ARMS_MOCK_PN075_UNREADABLE': '1'})
assert '读取失败' in out and '未知命令' not in out, out
out = run(['-e', 'commtest 60', '-e', 'not-a-command'], stdin='\n', ok=False)
assert '未完成' in out and '未知命令' not in out, out
assert '启动时间' not in run(['--freq', '333'], ok=False)
assert '启动时间' not in run(['--rt-cpu', ''], ok=False)
