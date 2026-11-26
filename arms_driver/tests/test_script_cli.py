"""Only exercise help and rejected inputs. No privileges or hardware tools are used."""
from pathlib import Path
import os
import subprocess
import tempfile

scripts = Path(__file__).resolve().parents[1] / 'scripts'
with tempfile.TemporaryDirectory() as tmp:
    # Even if validation regresses, never run an actual privileged command or controller.
    for cmd in ('sudo', 'pgrep'):
        stub = Path(tmp) / cmd
        stub.write_text('#!/bin/sh\nexit 97\n' if cmd == 'sudo' else '#!/bin/sh\nexit 0\n')
        stub.chmod(0o755)
    env = dict(os.environ, PATH=tmp + ':' + os.environ['PATH'])
    for script in ('rt_setup.sh', 'comm_test.sh'):
        result = subprocess.run(['bash', scripts / script, '--help'], env=env, capture_output=True, text=True, check=True)
        assert '用法' in result.stdout and 'set -' not in result.stdout
    for args in (['-t', '2 --enabled'], ['-t', '0'], ['-t', '86401'], ['-f', ''], ['-f', '1000 333']):
        p = subprocess.run(['bash', scripts / 'comm_test.sh', *args], env=env, capture_output=True, text=True)
        assert p.returncode == 2, (args, p)
