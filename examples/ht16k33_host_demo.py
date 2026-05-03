#!/usr/bin/env python3
"""HT16K33 4-digit / 14-segment display demo for the Trenino firmware.

Drives a Trenino-firmware-running Arduino over its USB serial connection,
configures an HT16K33 display module, and runs a short demo sequence:
static text, a fast counter, a scrolling marquee, brightness fade.

This file doubles as a minimal host-side reference implementation: COBS
framing, configure / write-segments / set-brightness messages, and a
14-segment ASCII font table.

----------------------------------------------------------------------
Usage
----------------------------------------------------------------------

    pip install pyserial
    python examples/ht16k33_host_demo.py --port /dev/cu.usbserial-XXXX

Optional flags:
    --port      serial device path (default: auto-detect first match)
    --address   I2C address of the HT16K33 (default 0x70)
    --digits    number of digits on the display, 4 or 8 (default 4)

If you already use the PlatformIO toolchain, its bundled Python has
pyserial pre-installed:

    ~/.platformio/penv/bin/python examples/ht16k33_host_demo.py

----------------------------------------------------------------------
Wiring (Arduino Nano + HT16K33 4-digit / 14-segment breakout)
----------------------------------------------------------------------

    HT16K33 VCC      -> Arduino 5V
    HT16K33 GND      -> Arduino GND
    HT16K33 SDA / D  -> Arduino A4
    HT16K33 SCL / C  -> Arduino A5

If your breakout has a separate "I2C level / Vi2c" pin, also tie it to
5V. Some Chinese clone boards expose this as a second pin labeled "+";
without it the chip ACKs but the segment drivers won't sink current.
"""

from __future__ import annotations

import argparse
import glob
import struct
import sys
import time
from typing import Optional

import serial

# --- Protocol constants (mirror src/protocol.h) ----------------------------

MSG_IDENTITY_REQUEST = 0
MSG_IDENTITY_RESPONSE = 1
MSG_CONFIGURE = 2
MSG_CONFIGURATION_STORED = 3
MSG_CONFIGURATION_ERROR = 4
MSG_HEARTBEAT = 6
MSG_WRITE_SEGMENTS = 13
MSG_SET_MODULE_BRIGHTNESS = 14
MSG_MODULE_ERROR = 15

MODULE_TYPE_HT16K33 = 4

# --- COBS framing ----------------------------------------------------------

def cobs_encode(data: bytes) -> bytes:
    """Encode `data` with COBS, returning the frame plus its 0x00 delimiter."""
    out = bytearray()
    last_zero = -1
    for i, b in enumerate(data):
        if b == 0:
            out.append(i - last_zero)
            out += data[last_zero + 1:i]
            last_zero = i
    out.append(len(data) - last_zero)
    out += data[last_zero + 1:]
    out.append(0)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode a COBS-framed message (without the trailing 0x00 delimiter)."""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            break
        i += 1
        end = min(i + code - 1, n)
        out += data[i:end]
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


# --- 14-segment font (Adafruit's alphafonttable) ---------------------------
#
# Each entry is a 16-bit segment bitmap. To display character C in digit
# position N: low byte goes to RAM[2*N], high byte goes to RAM[2*N+1].

FONT = {
    ' ': 0x0000, '!': 0x0006, '"': 0x0220, '#': 0x12CE, '$': 0x12ED,
    '%': 0x0C24, '&': 0x235D, "'": 0x0400, '(': 0x2400, ')': 0x0900,
    '*': 0x3FC0, '+': 0x12C0, ',': 0x0800, '-': 0x00C0, '.': 0x4000,
    '/': 0x0C00, '0': 0x0C3F, '1': 0x0006, '2': 0x00DB, '3': 0x008F,
    '4': 0x00E6, '5': 0x2069, '6': 0x00FD, '7': 0x0007, '8': 0x00FF,
    '9': 0x00EF, ':': 0x1200, ';': 0x0A00, '<': 0x2400, '=': 0x00C8,
    '>': 0x0900, '?': 0x1083, '@': 0x02BB, 'A': 0x00F7, 'B': 0x128F,
    'C': 0x0039, 'D': 0x120F, 'E': 0x00F9, 'F': 0x0071, 'G': 0x00BD,
    'H': 0x00F6, 'I': 0x1200, 'J': 0x001E, 'K': 0x2470, 'L': 0x0038,
    'M': 0x0536, 'N': 0x2136, 'O': 0x003F, 'P': 0x00F3, 'Q': 0x203F,
    'R': 0x20F3, 'S': 0x00ED, 'T': 0x1201, 'U': 0x003E, 'V': 0x0C30,
    'W': 0x2836, 'X': 0x2D00, 'Y': 0x1500, 'Z': 0x0C09,
}


def encode_text(text: str, digits: int) -> bytes:
    """Encode `text` to `digits * 2` bytes of HT16K33 display RAM.

    Pads with spaces on the right; truncates if too long. Unknown characters
    become spaces.
    """
    s = (text + ' ' * digits)[:digits].upper()
    out = bytearray()
    for c in s:
        v = FONT.get(c, 0)
        out.append(v & 0xFF)
        out.append((v >> 8) & 0xFF)
    return bytes(out)


# --- Host driver -----------------------------------------------------------

class TreninoHost:
    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=0.5)
        # Arduino resets when DTR/RTS toggles on open; wait for boot
        time.sleep(2.0)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()

    def send(self, payload: bytes) -> None:
        self._ser.write(cobs_encode(payload))

    def read_message(self, timeout: float = 1.0) -> Optional[bytes]:
        self._ser.timeout = timeout
        buf = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            b = self._ser.read(1)
            if not b:
                continue
            if b[0] == 0:
                if buf:
                    return cobs_decode(bytes(buf))
            else:
                buf.append(b[0])
        return None

    def configure_ht16k33(self, address: int, brightness: int, num_digits: int,
                          timeout: float = 3.0) -> None:
        """Send a Configure for an HT16K33 module, raising on error.

        Reads responses for up to `timeout` seconds and reports if the chip
        failed to initialize (likely wrong I2C address or wiring problem).
        """
        cfg_id = 0x12345678
        payload = struct.pack(
            '<BIBBBBBB',
            MSG_CONFIGURE, cfg_id,
            1, 0, MODULE_TYPE_HT16K33,
            address, brightness, num_digits,
        )
        self.send(payload)

        got_stored = False
        got_error = False
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.read_message(0.4)
            if msg is None:
                continue
            if msg[0] == MSG_CONFIGURATION_STORED:
                got_stored = True
            elif msg[0] == MSG_MODULE_ERROR and len(msg) >= 3 and msg[1] == address:
                got_error = True
            elif msg[0] == MSG_CONFIGURATION_ERROR:
                raise RuntimeError("device rejected the Configure message")

        if not got_stored:
            raise RuntimeError("no ConfigurationStored received within timeout")
        if got_error:
            raise RuntimeError(
                f"HT16K33 at 0x{address:02X} failed to initialize "
                f"(check wiring, jumpers, and Vi2c if applicable)"
            )

    def write_segments(self, address: int, data: bytes) -> None:
        """Write up to MAX_SEGMENT_BYTES (16) raw segment bytes."""
        assert len(data) <= 16
        self.send(struct.pack('<BBB', MSG_WRITE_SEGMENTS, address, len(data)) + data)

    def set_brightness(self, address: int, level: int) -> None:
        level = max(0, min(15, level))
        self.send(struct.pack('<BBB', MSG_SET_MODULE_BRIGHTNESS, address, level))

    def show_text(self, address: int, text: str, digits: int) -> None:
        self.write_segments(address, encode_text(text, digits))


# --- Demo flow -------------------------------------------------------------

def run_demo(host: TreninoHost, address: int, digits: int) -> None:
    show = lambda s: host.show_text(address, s, digits)

    print("HI"); show("HI"); time.sleep(2.0)
    print("TEST"); show("TEST"); time.sleep(2.0)

    print("Counting 0000 -> 0050")
    for n in range(0, 51):
        show(f"{n:04d}")
        time.sleep(0.06)
    time.sleep(0.5)

    print("Marquee: 'HELLO ARESTIFO TRENINO 14SEG'")
    text = ' ' * digits + "HELLO ARESTIFO TRENINO 14SEG" + ' ' * digits
    for i in range(len(text) - digits + 1):
        show(text[i:i + digits])
        time.sleep(0.18)

    print("Brightness fade on 8888")
    show("8" * digits)
    for level in range(15, -1, -1):
        host.set_brightness(address, level)
        time.sleep(0.15)

    host.set_brightness(address, 12)
    print("BYE"); show("BYE"); time.sleep(2.0)

    show("")  # clear


# --- CLI -------------------------------------------------------------------

def autodetect_port() -> Optional[str]:
    """Best-effort serial port autodetection across macOS and Linux."""
    candidates = (
        glob.glob("/dev/cu.usbserial*")        # macOS FTDI
        + glob.glob("/dev/cu.usbmodem*")       # macOS native USB (Pro Micro etc.)
        + glob.glob("/dev/cu.wchusbserial*")   # macOS CH340
        + glob.glob("/dev/ttyUSB*")            # Linux FTDI / CH340
        + glob.glob("/dev/ttyACM*")            # Linux native USB
    )
    return candidates[0] if candidates else None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="HT16K33 host demo for Trenino firmware")
    p.add_argument("--port", default=None, help="serial device (autodetected if omitted)")
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x70,
                   help="HT16K33 I2C address (default 0x70)")
    p.add_argument("--digits", type=int, default=4, choices=(4, 8),
                   help="display digits (default 4)")
    p.add_argument("--brightness", type=int, default=12,
                   help="initial brightness 0-15 (default 12)")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    port = args.port or autodetect_port()
    if port is None:
        print("error: no serial port found; pass --port explicitly", file=sys.stderr)
        return 1

    print(f"Connecting to {port}...")
    host = TreninoHost(port)

    try:
        host.configure_ht16k33(args.address, args.brightness, args.digits)
        print(f"HT16K33 @ 0x{args.address:02X} configured "
              f"({args.digits} digits, brightness {args.brightness})")
        run_demo(host, args.address, args.digits)
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        host.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
