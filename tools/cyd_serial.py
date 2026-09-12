#!/usr/bin/env python3
"""Read the board's serial output, optionally sending keystrokes first.

MicTest and Sonar are driven by single-character commands over the native USB
CDC port, so a full terminal is overkill and pyserial is not installed on this
machine. termios does the whole job:

    tools/cyd_serial.py                     # 6 s of whatever it is saying
    tools/cyd_serial.py -s c -t 12          # press 'c', watch the chirp report
    tools/cyd_serial.py -s "mq" -t 4        # several keys, in order

Opening /dev/cu.* does not assert DTR, so this never resets the board - what
you read is the run already in progress.

Copyright (c) 2026 Rich Sadowsky. MIT licensed - see LICENSE.
Written with Claude Code (Claude Opus 5).
"""

import argparse
import glob
import os
import select
import sys
import termios
import time


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no /dev/cu.usbmodem* found - is the board plugged in?")
    return ports[-1] if len(ports) == 1 else ports[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-t", "--seconds", type=float, default=6.0)
    ap.add_argument("-s", "--send", default="",
                    help="characters to send, one at a time, before reading")
    ap.add_argument("-d", "--delay", type=float, default=0.25,
                    help="pause between sent characters")
    args = ap.parse_args()

    port = args.port or find_port()
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        # Raw: no canonical mode, no echo, no flow control, no CR/LF mangling.
        attrs[0] = 0                      # iflag
        attrs[1] = 0                      # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                      # lflag
        speed = getattr(termios, f"B{args.baud}", args.baud)
        attrs[4] = attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)

        for ch in args.send:
            os.write(fd, ch.encode())
            time.sleep(args.delay)

        deadline = time.time() + args.seconds
        buf = b""
        while time.time() < deadline:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                buf += chunk
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()
        if not buf:
            print(f"\n(nothing received on {port} in {args.seconds:g}s)",
                  file=sys.stderr)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
