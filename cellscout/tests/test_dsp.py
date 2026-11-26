#!/usr/bin/env python3
"""Synthetic LTE downlink signals through the C++ DSP.

The generator is written independently of the decoder, straight from the
definitions in 3GPP TS 36.211 / 36.212, and produces the central 72 subcarriers
at 3.84 MS/s as the radio would deliver them (signed 8 bit I/Q).
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "build/run_dsp"
DATA = ROOT / "data"
RATE = 3_840_000
FFT = 256


def gold(c_init, length):
    n = length + 1600 + 31
    x1 = np.zeros(n, np.uint8)
    x2 = np.zeros(n, np.uint8)
    x1[0] = 1
    x2[:31] = [(c_init >> i) & 1 for i in range(31)]
    for i in range(n - 31):
        x1[i + 31] = x1[i + 3] ^ x1[i]
        x2[i + 31] = x2[i + 3] ^ x2[i + 2] ^ x2[i + 1] ^ x2[i]
    return (x1 ^ x2)[1600:1600 + length]


def pss(nid2):
    u = (25, 29, 34)[nid2]
    n = np.arange(62)
    m = np.where(n < 31, n * (n + 1), (n + 1) * (n + 2))
    return np.exp(-1j * np.pi * u * m / 63)


def m_seq(taps):
    x = [0, 0, 0, 0, 1]
    for i in range(26):
        x.append(sum(x[i + t] for t in taps) % 2)
    return 1 - 2 * np.array(x)


def sss(nid1, nid2, subframe):
    s, c, z = m_seq((2, 0)), m_seq((3, 0)), m_seq((4, 2, 1, 0))
    qp = nid1 // 30
    q = (nid1 + qp * (qp + 1) // 2) // 30
    mp = nid1 + q * (q + 1) // 2
    m0 = mp % 31
    m1 = (m0 + mp // 31 + 1) % 31
    n = np.arange(31)
    s0, s1 = s[(n + m0) % 31], s[(n + m1) % 31]
    c0, c1 = c[(n + nid2) % 31], c[(n + nid2 + 3) % 31]
    z0, z1 = z[(n + m0 % 8) % 31], z[(n + m1 % 8) % 31]
    d = np.zeros(62)
    d[0::2] = s0 * c0 if subframe == 0 else s1 * c0
    d[1::2] = s1 * c1 * z0 if subframe == 0 else s0 * c1 * z1
    return d


def mib_bits(n_rb, sfn, ports):
    payload = [0] * 24
    code = (6, 15, 25, 50, 75, 100).index(n_rb)
    payload[0:3] = [(code >> 2) & 1, (code >> 1) & 1, code & 1]
    payload[3:6] = [0, 1, 0]
    payload[6:14] = [((sfn >> 2) >> (7 - i)) & 1 for i in range(8)]
    reg = 0
    for bit in payload:
        feedback = ((reg >> 15) ^ bit) & 1
        reg = (reg << 1) & 0xFFFF
        if feedback:
            reg ^= 0x1021
    mask = {1: 0x0000, 2: 0xFFFF, 4: 0x5555}[ports]
    reg ^= mask
    return payload + [(reg >> (15 - i)) & 1 for i in range(16)]


def convolve_tail_biting(bits):
    out = []
    for g in (0o133, 0o171, 0o165):
        stream = []
        for k in range(len(bits)):
            value = 0
            for delay in range(7):
                if (g >> (6 - delay)) & 1:
                    value ^= bits[(k - delay) % len(bits)]
            stream.append(value)
        out.append(stream)
    return out


def rate_match(streams, e):
    perm = [1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31,
            0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30]
    w = []
    for stream in streams:
        padded = [None] * 24 + stream
        for column in perm:
            w += [padded[row * 32 + column] for row in range(2)]
    out = []
    k = 0
    while len(out) < e:
        if w[k % len(w)] is not None:
            out.append(w[k % len(w)])
        k += 1
    return np.array(out, np.uint8)


def crs_symbols(ports, port, l, slot, symbols_per_slot):
    """Offset v of the reference signal of one port in symbol l, or None."""
    second = symbols_per_slot - 3
    if port >= ports:
        return None
    if port < 2:
        if l == 0:
            return 0 if port == 0 else 3
        if l == second:
            return 3 if port == 0 else 0
        return None
    if l == 1:
        return 3 * (slot % 2) + (3 if port == 3 else 0)
    return None


def cell_grids(pci, n_rb, ports, tdd, extended, frames, rng):
    """Resource grid per port: [port][frame][slot][symbol][72]."""
    symbols = 6 if extended else 7
    n_cp = 0 if extended else 1
    grid = np.zeros((4, frames, 20, symbols, 72), complex)
    nid1, nid2 = pci // 3, pci % 3
    e = 432 if extended else 480  # PBCH bits per radio frame

    for frame in range(frames):
        # random QPSK load on every resource element, split between the ports
        load = (rng.choice([-1, 1], (20, symbols, 72)) + 1j * rng.choice([-1, 1], (20, symbols, 72))) / np.sqrt(2)
        grid[0, frame] = load

        # reference signals; the other ports are silent on them
        for slot in range(20):
            for l in range(symbols):
                for port in range(ports):
                    v = crs_symbols(ports, port, l, slot, symbols)
                    if v is None:
                        continue
                    c_init = 1024 * (7 * (slot + 1) + l + 1) * (2 * pci + 1) + 2 * pci + n_cp
                    c = gold(c_init, 440).astype(float)
                    r = ((1 - 2 * c[0::2]) + 1j * (1 - 2 * c[1::2])) / np.sqrt(2)
                    k = 6 * np.arange(12) + (v + pci % 6) % 6
                    grid[:, frame, slot, l, k] = 0
                    grid[port, frame, slot, l, k] = r[104:116]

        # PBCH
        sfn = frame
        coded = rate_match(convolve_tail_biting(mib_bits(n_rb, sfn - sfn % 4, ports)), 4 * e)
        scrambled = coded ^ gold(pci, 4 * e)
        part = scrambled[(sfn % 4) * e:(sfn % 4 + 1) * e].astype(float)
        d = ((1 - 2 * part[0::2]) + 1j * (1 - 2 * part[1::2])) / np.sqrt(2)
        where = []
        for l in range(4):
            pilots = l < 2 or (extended and l == 3)
            for k in range(72):
                if not pilots or k % 3 != pci % 3:
                    where.append((l, k))
        assert len(where) == len(d)
        for l, k in where:
            grid[:, frame, 1, l, k] = 0
        if ports == 1:
            for symbol, (l, k) in zip(d, where):
                grid[0, frame, 1, l, k] = symbol
        else:
            for i in range(0, len(d), 2):
                a, b = (0, 1) if ports == 2 else ((0, 2) if (i // 2) % 2 == 0 else (1, 3))
                (l0, k0), (l1, k1) = where[i], where[i + 1]
                x0, x1 = d[i], d[i + 1]
                grid[a, frame, 1, l0, k0] = x0 / np.sqrt(2)
                grid[a, frame, 1, l1, k1] = x1 / np.sqrt(2)
                grid[b, frame, 1, l0, k0] = -np.conj(x1) / np.sqrt(2)
                grid[b, frame, 1, l1, k1] = np.conj(x0) / np.sqrt(2)

        # synchronisation signals, with five unused subcarriers either side
        for half, subframe in enumerate((0, 5)):
            if tdd:
                pss_slot, pss_l = 2 + 10 * half, 2
                sss_slot, sss_l = 1 + 10 * half, symbols - 1
            else:
                pss_slot, pss_l = 10 * half, symbols - 1
                sss_slot, sss_l = 10 * half, symbols - 2
            for slot, l, sequence in ((pss_slot, pss_l, pss(nid2)), (sss_slot, sss_l, sss(nid1, nid2, subframe))):
                grid[:, frame, slot, l, :] = 0
                grid[0, frame, slot, l, 5:67] = sequence
    return grid


def modulate(grid, extended):
    ports, frames, slots, symbols, _ = grid.shape
    out = np.zeros((ports, frames * 10 * 3840), complex)
    bins = np.array([k - 36 if k < 36 else k - 35 for k in range(72)]) % FFT
    for port in range(ports):
        at = 0
        for frame in range(frames):
            for slot in range(slots):
                for l in range(symbols):
                    spectrum = np.zeros(FFT, complex)
                    spectrum[bins] = grid[port, frame, slot, l]
                    useful = np.fft.ifft(spectrum) * FFT
                    prefix = 64 if extended else (20 if l == 0 else 18)
                    symbol = np.concatenate([useful[-prefix:], useful])
                    out[port, at:at + len(symbol)] = symbol
                    at += len(symbol)
        assert at == out.shape[1]
    return out


def make(cells, snr_db, cfo=0.0, offset=0, seed=1, name="test"):
    """cells: list of dicts. snr_db: reference signal to noise ratio of a 0 dB cell."""
    rng = np.random.default_rng(seed)
    frames = 4
    total = np.zeros(frames * 38400, complex)
    for cell in cells:
        grid = cell_grids(cell["pci"], cell.get("n_rb", 50), cell.get("ports", 2),
                          cell.get("tdd", False), cell.get("extended", False), frames, rng)
        signal = modulate(grid, cell.get("extended", False))
        gain = 10 ** (cell.get("power_db", 0) / 20)
        mixed = np.zeros_like(total)
        for port in range(4):
            h = gain * np.exp(2j * np.pi * rng.random())
            echo = 0.25 * gain * np.exp(2j * np.pi * rng.random())
            mixed += h * signal[port] + echo * np.roll(signal[port], 3)
        total += np.roll(mixed, cell.get("delay", 0))
    noise_per_re = 10 ** (-snr_db / 10) if snr_db is not None else 1.0
    sigma = np.sqrt(FFT * noise_per_re / 2)
    if not cells:
        total = np.zeros_like(total)
    total = np.roll(total, -offset)
    total += sigma * (rng.standard_normal(len(total)) + 1j * rng.standard_normal(len(total)))
    total *= np.exp(2j * np.pi * cfo * np.arange(len(total)) / RATE)
    total *= 28 / np.sqrt(np.mean(abs(total) ** 2))
    total += 3 - 2j  # receiver DC offset
    iq = np.empty(2 * len(total))
    iq[0::2], iq[1::2] = total.real, total.imag
    path = DATA / f"{name}.cs8"
    np.clip(np.round(iq), -128, 127).astype(np.int8).tofile(path)
    return path


def run(path, center=0, steps=7, offset=0):
    out = subprocess.run([BINARY, "search", path, str(center), str(steps), str(offset)], check=True, capture_output=True, text=True).stdout
    found = {}
    for line in out.splitlines():
        fields = dict(item.split("=") for item in line.split()[1:])
        found[int(fields["pci"])] = {k: float(v) for k, v in fields.items()}
    return found


failures = []


def certain(found):
    return {p: v for p, v in found.items() if not v["tentative"]}


def check(label, found, expected, cfo=None):
    """Cells expected with mib=False may be reported as tentative."""
    problems = []
    hinted = {c["pci"] for c in expected if not c.get("mib", True)}
    found = {p: v for p, v in found.items() if not v["tentative"] or p in hinted}
    if set(found) != {c["pci"] for c in expected}:
        problems.append(f"PCIs {sorted(found)} instead of {sorted(c['pci'] for c in expected)}")
    for c in expected:
        got = found.get(c["pci"])
        if not got:
            continue
        want = {"tdd": c.get("tdd", False), "extcp": c.get("extended", False)}
        if c.get("mib", True):
            want.update(mib=1, n_rb=c.get("n_rb", 50), ports=c.get("ports", 2))
        for key, value in want.items():
            if got[key] != value:
                problems.append(f"PCI {c['pci']} {key}={got[key]:g}, expected {int(value)}")
        if cfo is not None and abs(got["cfo"] - cfo) > 400:
            problems.append(f"PCI {c['pci']} cfo={got['cfo']:g}, expected {cfo}")
    summary = ", ".join(f"{p}{'?' if v['tentative'] else ''}: sinr {v['sinr']:g} rsrq {v['rsrq']:g} mib {int(v['mib'])}" for p, v in found.items())
    print(f"{'ok  ' if not problems else 'FAIL'} {label:44} {summary}")
    for p in problems:
        print("       ", p)
    if problems:
        failures.append(label)


def main():
    DATA.mkdir(exist_ok=True)
    basic = {"pci": 214, "n_rb": 50, "ports": 2}
    check("FDD, 2 ports, 10 MHz, 15 dB", run(make([basic], 15, cfo=2300, offset=12345)), [basic], cfo=2300)
    for pci, n_rb, ports, snr in ((0, 6, 1, 15), (101, 25, 4, 15), (503, 100, 4, 12), (57, 75, 1, 8), (302, 15, 2, 8)):
        c = {"pci": pci, "n_rb": n_rb, "ports": ports}
        check(f"FDD, {ports} ports, {n_rb} RB, {snr} dB", run(make([c], snr, cfo=-4100, offset=777 * (pci + 1), seed=pci)), [c], cfo=-4100)
    for tdd, extended in ((False, True), (True, False), (True, True)):
        c = {"pci": 333, "tdd": tdd, "extended": extended}
        check(f"tdd={int(tdd)} extended_cp={int(extended)}", run(make([c], 12, cfo=5200, offset=31000)), [c], cfo=5200)
    for cfo in (-41000, 23750, 49000):
        check(f"frequency error {cfo} Hz", run(make([basic], 12, cfo=cfo, offset=999)), [basic], cfo=cfo)
    check("frequency error 143 kHz, wide search", run(make([basic], 12, cfo=143000, offset=5), 0, 27), [basic], cfo=143000)
    for snr in (5, 0, -3):
        c = dict(basic, mib=snr >= 0)
        check(f"weak cell, {snr} dB", run(make([c], snr, cfo=1500, offset=4321)), [c])
    two = [{"pci": 214}, {"pci": 57, "power_db": -6, "delay": 1311, "mib": False}]
    # the weaker cell is a matter of luck here (see the statistics below), so it
    # may or may not show up; the stronger one and nothing else must
    found = run(make(two, 18, cfo=800))
    check("two cells on one channel, 6 dB apart", {p: v for p, v in found.items() if p != two[1]["pci"]}, two[:1])
    two = [{"pci": 90}, {"pci": 91, "power_db": -3, "delay": 700, "mib": False}]
    check("two cells on one channel, 3 dB apart", run(make(two, 18, offset=100)), two)

    # neighbours below a strong cell: how often are they seen at all?
    for down in (3, 6, 9, 12):
        seen = 0
        for seed in range(20):
            pair = [{"pci": 214}, {"pci": 57, "power_db": -down, "delay": 400 + 311 * seed}]
            seen += 57 in run(make(pair, 18, cfo=800, seed=seed, name="pair"))
        print(f"info neighbour {down:2} dB below the serving cell: seen in {seen}/20 captures")

    # false alarms, on noise and next to a real cell
    for label, cells, snr in (("noise only", [], None), ("one strong cell", [{"pci": 214}], 18), ("one weak cell", [{"pci": 214}], 2)):
        false_certain, false_tentative = [], []
        for seed in range(80):
            found = run(make(cells, snr, seed=1000 + seed, name="false"))
            wrong = set(found) - {c["pci"] for c in cells}
            false_certain += [p for p in wrong if p in certain(found)]
            false_tentative += [p for p in wrong if p not in certain(found)]
        repeats = len(false_tentative) - len(set(false_tentative))
        print(f"{'ok  ' if not false_certain else 'FAIL'} {label + ', 80 captures':44} false certain {len(false_certain)}, "
              f"false tentative {len(false_tentative)} (repeated PCIs {repeats})")
        if false_certain:
            failures.append(label)

    # Recordings off air, if there are any. They are the only check here that does
    # not share its idea of LTE with the generator. They and what they should
    # contain are not part of the repository.
    # data/field/expected.json: {"name": [wanted PCI or null, in at least this
    # many of 10 captures, [all PCIs known to be there]]} for data/field/name.cs8.
    # Establish the PCIs by reference signals, not by this decoder alone.
    listing = DATA / "field" / "expected.json"
    field = json.loads(listing.read_text()) if listing.exists() else {}
    for name, (wanted, least, known) in field.items():
        path = DATA / "field" / f"{name}.cs8"
        if not path.exists():
            continue
        results = [certain(run(str(path), 10000, 7, k * 760000)) for k in range(10)]
        seen = sum(wanted in r for r in results)
        wrong = sorted(set().union(*results) - set(known))
        good = seen >= least and not wrong
        print(f"{'ok  ' if good else 'FAIL'} {'recording ' + name:44} " +
              (f"{wanted} in {seen}/10" if wanted else "weak cell only") + (f", unknown PCIs {wrong}" if wrong else ""))
        if not good:
            failures.append(name)

    print("failures:", len(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
