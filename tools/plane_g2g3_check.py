import math
from pathlib import Path


def _parse_words(line: str):
    out = {}
    for part in line.strip().split():
        if not part:
            continue
        k = part[0].upper()
        v = part[1:]
        out[k] = v
    return out


def _validate_file(path: Path):
    if not path.is_file():
        raise SystemExit(f"missing example file: {path}")

    lines = [ln.strip() for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip() and not ln.strip().startswith(";")]
    if not any(ln in {"G17", "G18", "G19"} for ln in lines):
        raise SystemExit(f"missing plane select (G17/18/19) in {path}")

    x = y = z = 0.0
    plane = "G17"
    checked = 0

    for ln in lines:
        up = ln.upper()
        if up in {"G17", "G18", "G19"}:
            plane = up
            continue
        if up.startswith("G0 ") or up.startswith("G1 "):
            words = _parse_words(ln)
            x = float(words.get("X", x))
            y = float(words.get("Y", y))
            z = float(words.get("Z", z))
            continue
        if up.startswith("G2 ") or up.startswith("G3 "):
            words = _parse_words(ln)
            x1 = float(words.get("X", x))
            y1 = float(words.get("Y", y))
            z1 = float(words.get("Z", z))
            i = float(words.get("I", 0.0))
            j = float(words.get("J", 0.0))
            k = float(words.get("K", 0.0))

            if plane == "G17":
                c0, c1 = x + i, y + j
                r0 = math.hypot(x - c0, y - c1)
                r1 = math.hypot(x1 - c0, y1 - c1)
            elif plane == "G18":
                c0, c1 = x + i, z + k
                r0 = math.hypot(x - c0, z - c1)
                r1 = math.hypot(x1 - c0, z1 - c1)
            else:  # G19
                c0, c1 = y + j, z + k
                r0 = math.hypot(y - c0, z - c1)
                r1 = math.hypot(y1 - c0, z1 - c1)

            if abs(r0 - r1) > 1e-6:
                raise SystemExit(f"arc radius mismatch in {path}: {ln}")
            x, y, z = x1, y1, z1
            checked += 1

    if checked < 2:
        raise SystemExit(f"expected at least two arc moves in {path}")
    print(f"OK: checked {checked} arc moves in {path}")


def main():
    files = [
        Path("data/examples/plane_g2g3_3dp_example.gcode"),
        Path("data/examples/plane_g2g3_3dp_example_g18.gcode"),
        Path("data/examples/plane_g2g3_3dp_example_g19.gcode"),
    ]
    for path in files:
        _validate_file(path)


if __name__ == "__main__":
    main()
