#!/usr/bin/env python3
"""Generates the X27 gauge wiring diagrams in this folder.

Run from anywhere: python3 docs/wiring/make_diagrams.py
Pin choices live in DIRECT and VID below; GAUGE_WIRING.md quotes the same pins.
"""

import os
from html import escape

HERE = os.path.dirname(os.path.abspath(__file__))

INK = "#1f2328"
MUTED = "#59636e"
LINE = "#d1d9e0"
BOARD = "#e7f0e9"
BOARD_EDGE = "#2f6f3e"
CHIP = "#eef1f5"
CHIP_EDGE = "#4b5563"
MOTOR = "#f6f2e8"
MOTOR_EDGE = "#8a6d1f"
WARN = "#9a3412"
# One colour per X27 contact; coil A is 1-2 (warm), coil B is 3-4 (cool).
CONTACT = {1: "#c2410c", 2: "#d97706", 3: "#0f766e", 4: "#1d4ed8"}

FONT = "font-family='-apple-system,Segoe UI,Helvetica,Arial,sans-serif'"


def text(x, y, s, size=13, fill=INK, anchor="start", weight="normal"):
    return (f"<text x='{x}' y='{y}' {FONT} font-size='{size}' fill='{fill}' "
            f"text-anchor='{anchor}' font-weight='{weight}'>{escape(s)}</text>")


def rect(x, y, w, h, fill, stroke, r=8, width=1.5):
    return (f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='{r}' fill='{fill}' "
            f"stroke='{stroke}' stroke-width='{width}'/>")


def wire(points, colour, width=3):
    d = " ".join(f"{x},{y}" for x, y in points)
    return (f"<polyline points='{d}' fill='none' stroke='{colour}' stroke-width='{width}' "
            f"stroke-linejoin='round' stroke-linecap='round'/>")


def dot(x, y, colour):
    return f"<circle cx='{x}' cy='{y}' r='4' fill='{colour}'/>"


def svg(width, height, body, title):
    return (f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {width} {height}' "
            f"width='{width}' height='{height}' role='img' aria-label='{escape(title)}'>\n"
            f"<title>{escape(title)}</title>\n"
            f"<rect width='{width}' height='{height}' fill='#ffffff'/>\n"
            + "\n".join(body) + "\n</svg>\n")


def motor(x, y, label="X27.168", sub="rear view, pins down"):
    """X27 body with contacts 1-4 on its left edge. Returns (parts, pin y list)."""
    parts = [rect(x, y, 170, 190, MOTOR, MOTOR_EDGE, r=14),
             f"<circle cx='{x + 100}' cy='{y + 82}' r='38' fill='#fff' stroke='{MOTOR_EDGE}' stroke-width='1.5'/>",
             f"<circle cx='{x + 100}' cy='{y + 82}' r='5' fill='{MOTOR_EDGE}'/>",
             text(x + 100, y + 150, label, 14, INK, "middle", "bold"),
             text(x + 100, y + 170, sub, 11, MUTED, "middle")]
    ys = [y + 40 + 32 * i for i in range(4)]
    for i, py in enumerate(ys, start=1):
        parts.append(f"<rect x='{x - 8}' y='{py - 5}' width='16' height='10' rx='2' fill='#b0b7c0'/>")
        parts.append(text(x + 16, py + 4, str(i), 12, CONTACT[i], "start", "bold"))
    parts.append(text(x + 34, y + 60, "coil A", 10, MUTED))
    parts.append(text(x + 34, y + 124, "coil B", 10, MUTED))
    return parts, ys


def direct_diagram(name, board, pins, notes, header_note):
    """Four board pins straight to X27 contacts 1-4."""
    w, h = 780, 400
    body = [text(30, 38, f"{board}: X27 in direct mode (driver = 1)", 18, INK, weight="bold"),
            text(30, 60, header_note, 12, MUTED)]
    bx, by, bw, bh = 30, 90, 250, 210
    body.append(rect(bx, by, bw, bh, BOARD, BOARD_EDGE, r=10))
    body.append(text(bx + 16, by + 26, board, 15, BOARD_EDGE, weight="bold"))
    body.append(text(bx + 16, by + 44, "5 V logic", 11, MUTED))
    m_parts, m_ys = motor(560, 100)
    for i, (label, number) in enumerate(pins, start=1):
        y = by + 70 + 34 * (i - 1)
        body.append(text(bx + bw - 14, y + 4, f"{label}", 13, INK, "end", "bold"))
        body.append(text(bx + 16, y + 4, f"pins[{i - 1}] = {number}", 12, MUTED))
        ty = m_ys[i - 1]
        mid = 400 + 14 * (i - 1)
        body.append(wire([(bx + bw, y), (mid, y), (mid, ty), (552, ty)], CONTACT[i]))
        body.append(dot(bx + bw, y, CONTACT[i]))
    body += m_parts
    ny = 330
    for n in notes:
        body.append(text(30, ny, n, 12, WARN if n.startswith("!") else MUTED))
        ny += 18
    return name, svg(w, h, [b.replace(">!", ">") for b in body], f"{board} direct X27 wiring")


# Direct-mode pin choices: (header label, Arduino pin number for Configure).
DIRECT = {
    "direct-pro-micro.svg": dict(
        board="SparkFun Pro Micro",
        header_note="Same pins as J5 pins 3-6 on the train-lever gauge board (a 2x2 block there); on a bare Pro Micro use four jumpers.",
        pins=[("A0", 18), ("A1", 19), ("10", 10), ("15", 15)],
        notes=["No 5 V or GND wire: the four pins drive both coils in both directions.",
               "Firmware turns JTAG off at boot, which frees A0 and A1 (PF7, PF6).",
               "! About 20 mA per pin. Up to two direct gauges per board; add BAT54S clamps for long runs."]),
    "direct-leonardo-micro.svg": dict(
        board="Arduino Leonardo / Micro",
        header_note="Digital header D4-D7. D2/D3 stay free for I2C displays.",
        pins=[("D4", 4), ("D5", 5), ("D6", 6), ("D7", 7)],
        notes=["No 5 V or GND wire: the four pins drive both coils in both directions.",
               "Second gauge: D8, D9, D10, D11.",
               "! About 20 mA per pin. Up to two direct gauges per board; add BAT54S clamps for long runs."]),
    "direct-uno-nano.svg": dict(
        board="Arduino Uno / Nano",
        header_note="Digital header D4-D7. D0/D1 carry USB serial and A4/A5 carry I2C, so leave them.",
        pins=[("D4", 4), ("D5", 5), ("D6", 6), ("D7", 7)],
        notes=["No 5 V or GND wire: the four pins drive both coils in both directions.",
               "Second gauge: D8, D9, D10, D11.",
               "! About 20 mA per pin. Up to two direct gauges per board; add BAT54S clamps for long runs."]),
    "direct-mega.svg": dict(
        board="Arduino Mega 2560",
        header_note="Double header D22-D25 (the pair of rows at the board's far end).",
        pins=[("D22", 22), ("D23", 23), ("D24", 24), ("D25", 25)],
        notes=["No 5 V or GND wire: the four pins drive both coils in both directions.",
               "Second gauge: D26, D27, D28, D29.",
               "! About 20 mA per pin. Up to two direct gauges per board; add BAT54S clamps for long runs."]),
}


def vid_diagram_3v3():
    """Due and ESP32 cannot drive the coils from their pins: VID6606 instead."""
    w, h = 820, 470
    body = [text(30, 38, "Arduino Due and ESP32: direct mode is refused, use a VID6606", 18, INK, weight="bold"),
            text(30, 60, "Due pins are rated 3 mA; ESP32 at 3.3 V gives a 260 ohm coil only about 13 mA. An X27 coil wants about 20 mA at 5 V.", 12, MUTED)]
    # board
    body.append(rect(30, 90, 220, 220, BOARD, BOARD_EDGE, r=10))
    body.append(text(46, 116, "Due  /  ESP32", 15, BOARD_EDGE, weight="bold"))
    body.append(text(46, 134, "3.3 V logic", 11, MUTED))
    rows = [("STEP", "D22  /  GPIO25", "pins[0]"), ("DIR", "D23  /  GPIO26", "pins[1]"),
            ("RESET_N", "D24  /  GPIO27", "pins[2]"), ("GND", "GND", ""), ("5 V", "5V  /  VIN", "")]
    chip_x, chip_y = 340, 100
    body.append(rect(chip_x, chip_y, 170, 210, CHIP, CHIP_EDGE, r=6))
    body.append(text(chip_x + 85, chip_y + 26, "VID6606", 15, INK, "middle", "bold"))
    body.append(text(chip_x + 85, chip_y + 44, "channel A shown", 11, MUTED, "middle"))
    chip_pins = ["f(scx)A  28", "CW/CCW A  27", "RESET  26", "VSS  12", "VDD  1, 15"]
    for i, (sig, pin, cfg) in enumerate(rows):
        y = 160 + 30 * i
        colour = [CONTACT[1], CONTACT[3], CONTACT[4], INK, "#b91c1c"][i]
        body.append(text(234, y + 4, pin, 12, INK, "end", "bold"))
        if cfg:
            body.append(text(46, y + 4, cfg, 11, MUTED))
        body.append(wire([(250, y), (chip_x, y)], colour, 2.5))
        body.append(text(chip_x + 8, y + 4, chip_pins[i], 11, INK))
        body.append(text(295, y - 6, sig, 10, MUTED, "middle"))
    m_parts, m_ys = motor(620, 110)
    outs = ["OUT1A", "OUT2A", "OUT3A", "OUT4A"]
    for i, ty in enumerate(m_ys, start=1):
        body.append(text(chip_x + 162, ty + 4, outs[i - 1], 11, INK, "end"))
        body.append(wire([(chip_x + 170, ty), (612, ty)], CONTACT[i], 2.5))
    body += m_parts
    notes = ["VDD must be 4.5-5.5 V. Add 100 nF at each VDD pin and about 5 uF per motor (see the train-lever BOM).",
             "Hold RESET_N low at power-up with 10 k to GND; the firmware raises it when the gauge starts.",
             "Inputs need VIH >= 3.15 V. 3.3 V outputs meet that only just: for long wires add a 74HCT125 at 5 V.",
             "Wire channels B-D the same way; four motors share one chip and one RESET_N pin."]
    y = 350
    for n in notes:
        body.append(text(30, y, n, 12, MUTED))
        y += 20
    return "vid6606-due-esp32.svg", svg(w, h, body, "Due and ESP32 VID6606 X27 wiring")


# train-lever class323-gauge-driver rev 3 pin map (HANDOFF.md "Pin map for the firmware").
VID = [("A", "D8", 8, "D7", 7, "J1"), ("B", "D14", 14, "D16", 16, "J2"),
       ("C", "A3", 21, "A2", 20, "J3"), ("D", "D5", 5, "D4", 4, "J4")]


def gauge_board_diagram():
    w, h = 900, 560
    body = [text(30, 38, "train-lever gauge board (rev 3): firmware pin map", 18, INK, weight="bold"),
            text(30, 60, "Pro Micro on the board drives the VID6606 channels A-D; header J5 can take one direct X27 for prototyping.", 12, MUTED)]
    body.append(rect(30, 90, 230, 330, BOARD, BOARD_EDGE, r=10))
    body.append(text(46, 116, "Pro Micro (socketed)", 15, BOARD_EDGE, weight="bold"))
    chip_x, chip_y = 360, 90
    body.append(rect(chip_x, chip_y, 170, 330, CHIP, CHIP_EDGE, r=6))
    body.append(text(chip_x + 85, chip_y + 26, "U1  VID6606", 15, INK, "middle", "bold"))
    y = 150
    for ch, sl, sn, dl, dn, conn in VID:
        body.append(text(244, y + 4, f"{sl} STEP {ch}", 12, INK, "end", "bold"))
        body.append(wire([(260, y), (chip_x, y)], CONTACT[1], 2.5))
        body.append(text(244, y + 22, f"{dl} DIR {ch}", 12, INK, "end", "bold"))
        body.append(wire([(260, y + 18), (chip_x, y + 18)], CONTACT[3], 2.5))
        body.append(text(46, y + 13, f"{ch}: [{sn}, {dn}, 6, 0xff]", 11, MUTED))
        # channel out to connector
        body.append(wire([(chip_x + 170, y + 9), (620, y + 9)], MUTED, 6))
        body.append(rect(620, y - 8, 70, 34, "#fff", INK, r=4))
        body.append(text(655, y + 14, f"{conn}  {ch}", 12, INK, "middle", "bold"))
        body.append(wire([(690, y + 9), (730, y + 9)], MUTED, 6))
        body.append(text(738, y + 13, "X27 pins 1-4", 12, INK))
        y += 56
    body.append(text(244, y + 4, "D6 RESET_N", 12, INK, "end", "bold"))
    body.append(wire([(260, y), (chip_x, y)], CONTACT[4], 2.5))
    body.append(text(chip_x + 10, y + 4, "R1 10 k to GND", 11, MUTED))
    # J5 direct gauge
    jy = 450
    body.append(rect(30, jy, 230, 70, "#fff", BOARD_EDGE, r=6))
    body.append(text(46, jy + 22, "J5 spare header", 13, BOARD_EDGE, weight="bold"))
    body.append(text(46, jy + 42, "pin 3 A0, 4 A1, 5 D10, 6 D15", 12, INK))
    body.append(text(46, jy + 60, "direct: [18, 19, 10, 15]", 11, MUTED))
    body.append(wire([(260, jy + 35), (620, jy + 35)], CONTACT[2], 2.5))
    body.append(text(440, jy + 28, "2x2 plug, X27 pins 1-4", 11, MUTED, "middle"))
    body.append(text(630, jy + 39, "direct X27 (takes a gauge slot)", 12, INK))
    return "gauge-board-pro-micro.svg", svg(w, h, body, "train-lever gauge board firmware pin map")


def main():
    outputs = [direct_diagram(n, **cfg) for n, cfg in DIRECT.items()]
    outputs.append(vid_diagram_3v3())
    outputs.append(gauge_board_diagram())
    for name, content in outputs:
        with open(os.path.join(HERE, name), "w") as f:
            f.write(content)
        print("wrote", name)


if __name__ == "__main__":
    main()
