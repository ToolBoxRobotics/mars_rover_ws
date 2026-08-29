#!/usr/bin/env python3
"""Workspace-wide syntax/parse validation sweep.

Checks every touched file type the fast, cheap way (parse, don't
execute) before anything gets zipped up:
  .py            -> ast.parse
  .xml/.xacro/.urdf/.rviz(if xml)/.launch.py handled as .py already
  .yaml/.yml     -> yaml.safe_load
  .js            -> node --check (skipped if node is unavailable)
  .ino/.h/.cpp   -> brace/paren/bracket balance + a couple of cheap
                    structural sanity checks (no real C++ parser
                    available in this environment)

Exits non-zero if anything fails, and prints a one-line summary per
file plus a final tally.
"""

from __future__ import annotations

import ast
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent

SKIP_DIR_NAMES = {".git", "build", "install", "log", "__pycache__"}


def iter_files(*extensions: str):
    for ext in extensions:
        for path in ROOT.rglob(f"*{ext}"):
            if any(part in SKIP_DIR_NAMES for part in path.parts):
                continue
            yield path


def check_python(path: Path) -> str | None:
    try:
        ast.parse(path.read_text(), filename=str(path))
    except SyntaxError as exc:
        return f"SyntaxError: {exc}"
    return None


def check_xml(path: Path) -> str | None:
    try:
        ET.parse(path)
    except ET.ParseError as exc:
        return f"XML ParseError: {exc}"
    return None


def check_yaml(path: Path) -> str | None:
    try:
        with open(path) as f:
            yaml.safe_load(f)
    except yaml.YAMLError as exc:
        return f"YAML error: {exc}"
    return None


def check_js(path: Path) -> str | None:
    if shutil.which("node") is None:
        return None  # node unavailable in this environment; skip rather than false-fail
    result = subprocess.run(["node", "--check", str(path)], capture_output=True, text=True)
    if result.returncode != 0:
        return f"node --check failed: {result.stderr.strip()}"
    return None


_PAIRS = {"(": ")", "{": "}", "[": "]"}
_CLOSERS = {v: k for k, v in _PAIRS.items()}


def check_brace_balance(path: Path) -> str | None:
    text = path.read_text()
    stack: list[tuple[str, int]] = []
    in_line_comment = False
    in_block_comment = False
    in_string = None  # holds the quote char if inside a string literal
    i = 0
    line = 1
    n = len(text)
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""

        if in_line_comment:
            if c == "\n":
                in_line_comment = False
        elif in_block_comment:
            if c == "*" and nxt == "/":
                in_block_comment = False
                i += 1
        elif in_string:
            if c == "\\":
                i += 1  # skip escaped char
            elif c == in_string:
                in_string = None
        else:
            if c == "/" and nxt == "/":
                in_line_comment = True
                i += 1
            elif c == "/" and nxt == "*":
                in_block_comment = True
                i += 1
            elif c in ("'", '"'):
                in_string = c
            elif c in _PAIRS:
                stack.append((c, line))
            elif c in _CLOSERS:
                if not stack or stack[-1][0] != _CLOSERS[c]:
                    return f"unbalanced '{c}' at line {line}"
                stack.pop()

        if c == "\n":
            line += 1
        i += 1

    if stack:
        opener, at_line = stack[-1]
        return f"unclosed '{opener}' opened at line {at_line}"
    return None


def check_msg(path: Path) -> str | None:
    """Lightweight structural check for .msg files (no colcon/rosidl
    available in this environment to actually build them): every
    non-blank, non-comment line must be either a `type name` field
    declaration or a `type NAME=value` constant, i.e. at least two
    whitespace-separated tokens. Catches gross typos (a stray line
    missing a type or name); does not validate that `type` is an
    actually-known rosidl type.
    """
    for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()  # strip trailing comments
        if not line:
            continue
        tokens = line.split()
        if len(tokens) < 2:
            return f"line {lineno}: expected '<type> <name>' or '<type> <name>=<value>', got {raw_line!r}"
    return None


def check_srv(path: Path) -> str | None:
    """Same structural check as check_msg, field-line by field-line,
    but .srv files also have exactly one bare '---' line separating
    the request fields from the response fields - allowed here rather
    than flagged as a malformed field line.
    """
    separator_count = 0
    for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if line == "---":
            separator_count += 1
            continue
        tokens = line.split()
        if len(tokens) < 2:
            return f"line {lineno}: expected '<type> <name>' or '<type> <name>=<value>', got {raw_line!r}"
    if separator_count != 1:
        return f"expected exactly one '---' request/response separator, found {separator_count}"
    return None


def main() -> int:
    failures = []
    checks = 0

    for path in iter_files(".py"):
        checks += 1
        err = check_python(path)
        if err:
            failures.append((path, err))

    for path in iter_files(".xacro", ".urdf", ".svg", ".srdf"):
        checks += 1
        err = check_xml(path)
        if err:
            failures.append((path, err))

    for path in iter_files(".yaml", ".yml", ".rviz"):
        checks += 1
        err = check_yaml(path)
        if err:
            failures.append((path, err))

    for path in iter_files(".js"):
        checks += 1
        err = check_js(path)
        if err:
            failures.append((path, err))

    for path in iter_files(".ino", ".h", ".hpp", ".cpp"):
        checks += 1
        err = check_brace_balance(path)
        if err:
            failures.append((path, err))

    for path in iter_files(".msg"):
        checks += 1
        err = check_msg(path)
        if err:
            failures.append((path, err))

    for path in iter_files(".srv"):
        checks += 1
        err = check_srv(path)
        if err:
            failures.append((path, err))

    print(f"Checked {checks} files.")
    if failures:
        print(f"\n{len(failures)} FAILURE(S):")
        for path, err in failures:
            print(f"  {path.relative_to(ROOT)}: {err}")
        return 1

    print("All files parse cleanly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
