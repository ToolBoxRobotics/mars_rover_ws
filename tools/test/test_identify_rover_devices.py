"""End-to-end smoke test for identify_rover_devices.py, mocked at
exactly the OS boundary (glob.glob, input(), subprocess.run,
shutil.which, Path.write_text) so the entire real control flow -
prompting, hot-plug diffing, udevadm parsing, collision detection, and
final rules rendering - runs for real, just without real hardware
underneath it. This is a thin IO layer wrapping already-unit-tested
pure logic (see test_udev_device_id.py), but the wiring between the
two is exactly what a mistake here would break, so it earns its own
test rather than being left as manual-only verification.
"""

import os
import subprocess
import sys
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import identify_rover_devices as script
from udev_device_id import DEVICES


def _run_scripted_main(plug_order_matches_devices=True):
    """Simulates a full run: each prompt "plugs in" exactly one new
    device of the right kind, udevadm returns realistic output keyed
    off which device is currently being prompted for, and the final
    "write it?" prompt is auto-confirmed. Returns (return_code,
    written_content).
    """
    fake_tty_devices = []
    fake_video_devices = []
    current_spec = [None]
    written = {}

    def fake_glob(pattern):
        if "ttyACM" in pattern:
            return list(fake_tty_devices)
        if "ttyUSB" in pattern:
            return []
        if "video" in pattern:
            return list(fake_video_devices)
        return []

    def fake_input(prompt=""):
        if "Write this to" in prompt:
            return "y"
        if "Plug in ONLY" in prompt:
            spec = current_spec[0]
            if spec.subsystem == "tty":
                fake_tty_devices.append(f"/dev/ttyACM{len(fake_tty_devices)}")
            else:
                fake_video_devices.append(f"/dev/video{len(fake_video_devices)}")
        return ""

    def fake_run(cmd, capture_output, text, check):
        device_path = cmd[-1]
        spec = current_spec[0]
        is_mega = spec.key in ("base", "arm")  # mast is an Uno now, not a third Mega
        vendor, product = ("2341", "0042") if is_mega else ("1a86", "7523")
        # Megas deliberately share an identical (empty) serial, to
        # exercise the KERNELS-fallback collision path; everything
        # else gets a unique serial.
        serial = "" if is_mega else f"SER_{spec.key}"
        port_index = len(fake_tty_devices) + len(fake_video_devices)
        output = f"""
  looking at device '/devices/.../tty/{device_path.split('/')[-1]}':
    KERNEL=="{device_path.split('/')[-1]}"

  looking at parent device '/devices/.../1-{port_index}':
    KERNELS=="1-{port_index}"
    ATTRS{{idVendor}}=="{vendor}"
    ATTRS{{idProduct}}=="{product}"
    ATTRS{{serial}}=="{serial}"
"""
        return subprocess.CompletedProcess(cmd, 0, stdout=output, stderr="")

    def fake_write_text(self, text):
        written["content"] = text

    orig_prompt = script.prompt_for_device

    def wrapped_prompt(spec, known_before):
        current_spec[0] = spec
        return orig_prompt(spec, known_before)

    with mock.patch("glob.glob", side_effect=fake_glob), mock.patch(
        "builtins.input", side_effect=fake_input
    ), mock.patch("subprocess.run", side_effect=fake_run), mock.patch(
        "shutil.which", return_value="/usr/bin/udevadm"
    ), mock.patch.object(script.Path, "write_text", fake_write_text), mock.patch.object(
        script, "prompt_for_device", wrapped_prompt
    ):
        rc = script.main()

    return rc, written.get("content", "")


def test_end_to_end_captures_all_devices_and_writes_rules_file():
    rc, content = _run_scripted_main()
    assert rc == 0
    for spec in DEVICES:
        assert spec.symlink in content


def test_end_to_end_detects_mega_serial_collision_and_uses_kernels():
    _rc, content = _run_scripted_main()
    # base/arm share an empty serial in this simulation - the generated
    # file must fall back to KERNELS for exactly those two. mast is an
    # Uno now (see the is_mega check in _run_scripted_main), not a
    # third Mega, so it must NOT be swept into this fallback even
    # though it's captured in the same run.
    assert 'SYMLINK+="rover/base"' in content
    base_line = next(line for line in content.splitlines() if "rover/base" in line)
    arm_line = next(line for line in content.splitlines() if "rover/arm\"" in line)
    for line in (base_line, arm_line):
        assert "KERNELS==" in line
        assert "ATTRS{serial}" not in line


def test_end_to_end_non_mega_devices_still_use_serial():
    _rc, content = _run_scripted_main()
    gps_line = next(line for line in content.splitlines() if "rover/gps" in line)
    assert 'ATTRS{serial}=="SER_gps"' in gps_line
    # mast is a non-Mega device in this simulation too now - confirm
    # it gets a real serial match, not swept into the KERNELS fallback
    # alongside base/arm.
    mast_line = next(line for line in content.splitlines() if "rover/mast" in line)
    assert 'ATTRS{serial}=="SER_mast"' in mast_line
    assert "KERNELS==" not in mast_line


def test_end_to_end_exits_early_without_udevadm():
    with mock.patch("shutil.which", return_value=None):
        rc = script.main()
    assert rc == 1


def test_list_candidate_devices_rejects_unknown_subsystem():
    import pytest

    with pytest.raises(ValueError):
        script.list_candidate_devices("bluetooth")
