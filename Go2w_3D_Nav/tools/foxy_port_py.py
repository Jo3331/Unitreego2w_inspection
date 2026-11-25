#!/usr/bin/env python3
"""Apply Python 3.8 compatibility patches for the Foxy target on the Jetson.

Foxy runs Python 3.8. The launch files below annotate a function with
`tuple[str, str, str]`, which is only evaluatable at runtime from Python 3.9.
`from __future__ import annotations` (available since 3.7) defers that
evaluation, so the same source keeps working on Humble's Python 3.10.

Idempotent: running twice makes no further changes.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

LAUNCH_FILES = (
    "jie_3d_nav/octo_planner/launch/nav.launch.py",
    "jie_3d_nav/octo_planner/launch/web_test.launch.py",
)

FUTURE_LINE = "from __future__ import annotations\n"


def add_future_import(text):
    """Insert the future import at the very top, above any module docstring."""
    if FUTURE_LINE in text:
        return text, 0

    lines = text.splitlines(keepends=True)
    insert_at = 1 if lines and lines[0].startswith("#!") else 0

    # A module docstring must remain the first statement, so the import goes after it.
    rest = "".join(lines[insert_at:])
    m = re.match(r'\s*("""|\'\'\')(?:.|\n)*?\1[ \t]*\n', rest)
    if m:
        insert_at += rest[: m.end()].count("\n")

    lines.insert(insert_at, FUTURE_LINE)
    return "".join(lines), 1


def main():
    total = 0
    for rel in LAUNCH_FILES:
        p = SRC / rel
        if not p.is_file():
            print(f"  MISSING {rel}")
            continue
        original = p.read_text()
        text, n = add_future_import(original)
        if text != original:
            p.write_text(text)
            total += n
            print(f"  {rel}: future-import={n}")

    print(f"total edits: {total}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
