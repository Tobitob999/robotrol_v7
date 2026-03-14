#!/usr/bin/env python3
"""Lightweight language checker for English-only project text.

Scans selected text files for:
- German umlaut characters
- common German words

Usage:
  python tools/check_english_text.py
  python tools/check_english_text.py --strict
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
PROJECT_FLAGS = ROOT / "configs" / "project_flags.json"

INCLUDE_SUFFIXES = {".py", ".md", ".json", ".txt"}
EXCLUDE_DIRS = {
    ".git",
    "__pycache__",
    "tmp_eb300_pages",
    "tmp_eb300_pages_hi",
}
EXCLUDE_FILES = {
    "tmp_eb300_assembly.pdf",
    "tmp_eb300_contactsheet.png",
    "check_english_text.py",
}

GERMAN_WORDS = re.compile(
    r"\b("
    r"und|oder|nicht|kein|keine|wenn|dann|wird|wurde|ist|sind|"
    r"schach|kamera|achse|achsen|gelenk|befehl|befehle|"
    r"warnung|fehler|abbruch|ausfuehren|ausführen|zurueck|zurück|"
    r"konfig|gespeichert|geladen"
    r")\b",
    re.IGNORECASE,
)


def _iter_files(base: Path) -> Iterable[Path]:
    for p in base.rglob("*"):
        if not p.is_file():
            continue
        if p.name in EXCLUDE_FILES:
            continue
        if p.suffix.lower() not in INCLUDE_SUFFIXES:
            continue
        parts = set(p.parts)
        if parts & EXCLUDE_DIRS:
            continue
        yield p


def _load_flags() -> dict:
    if not PROJECT_FLAGS.exists():
        return {}
    try:
        return json.loads(PROJECT_FLAGS.read_text(encoding="utf-8"))
    except Exception:
        return {}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true", help="Return non-zero when findings exist.")
    args = parser.parse_args()

    flags = _load_flags()
    enforce = bool(flags.get("enforce_english_text", False))
    strict = args.strict or enforce

    findings = []
    for path in _iter_files(ROOT):
        rel = path.relative_to(ROOT)
        try:
            text = path.read_text(encoding="utf-8")
        except Exception:
            continue
        for lineno, line in enumerate(text.splitlines(), start=1):
            if re.search(r"[ÄÖÜäöüß]", line):
                findings.append((str(rel), lineno, "german-umlaut", line.strip()))
            if GERMAN_WORDS.search(line):
                findings.append((str(rel), lineno, "german-word", line.strip()))

    if findings:
        print(f"Found {len(findings)} language finding(s):")
        for path, line, kind, snippet in findings[:200]:
            print(f"- {path}:{line} [{kind}] {snippet}")
        if len(findings) > 200:
            print(f"... truncated, +{len(findings) - 200} more")
    else:
        print("No language findings.")

    return 1 if (strict and findings) else 0


if __name__ == "__main__":
    raise SystemExit(main())
