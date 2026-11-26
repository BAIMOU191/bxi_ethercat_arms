"""Exercise the real interactive editor via a PTY; all commands use mock and a private config."""
import os
import pathlib
import pty
import select
import shutil
import subprocess
import sys
import tempfile
import time


def expect(fd, needle, timeout=6):
    buf = bytearray()
    deadline = time.monotonic() + timeout
    while needle not in buf:
        assert select.select([fd], [], [], max(0, deadline - time.monotonic()))[0], (needle, bytes(buf))
        buf.extend(os.read(fd, 8192))
    return buf


with tempfile.TemporaryDirectory() as tmp:
    config = pathlib.Path(tmp) / 'arms.json'
    shutil.copyfile(sys.argv[2], config)
    original = config.read_bytes()
    master, slave = pty.openpty()
    p = subprocess.Popen([sys.argv[1], '--mock', '-c', str(config)], stdin=slave,
                         stdout=slave, stderr=slave, start_new_session=True)
    os.close(slave)
    try:
        expect(master, b'> \x1b[0K')
        for typed, shown in [(b'ze\t', b'> zero '), (b'help brake_v\t', b'> help brake_voltage '),
                             (b'pn help torque_s\t', b'> pn help torque_scale '),
                             (b'pn 15 Pn14\t', b'> pn 15 Pn148 '),
                             (b'brake 15 rel\t', b'> brake 15 release '),
                             (b'zero l_shoulder_y\t', b'> zero l_shoulder_y_joint '),
                             (b'status 15,rig\t', b'> status 15,right '),
                             (b'zero 15 --s\t', b'> zero 15 --soft '),
                             (b'stat 15\x01' + b'\x06' * 4 + b'\t', b'> status 15')]:
            os.write(master, typed)
            expect(master, shown + b'\x1b[0K')
            os.write(master, b'\x15\x0b')  # clear both sides of cursor
            expect(master, b'> \x1b[0K')
        os.write(master, b's\t')
        listing = expect(master, b'  \r')
        assert b'status  ' in listing and b'swing  ' in listing, listing
        os.write(master, b'\x15qui\t\r')
        assert p.wait(timeout=5) == 0
        assert config.read_bytes() == original
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()
        os.close(master)
