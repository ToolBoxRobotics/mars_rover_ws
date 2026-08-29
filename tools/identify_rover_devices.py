#!/usr/bin/env python3
"""Interactive helper for docs/INSTALL.md Section 8: identifies each
of the rover's 9 serial/video devices by prompting you to plug them in
one at a time, auto-detecting which new /dev entry appeared, pulling
its VID:PID/serial via udevadm, and writing a filled-in udev rules
file at the end - instead of running `udevadm info -a -n ...` by hand
nine times and copy-pasting values into the placeholders.

Run this ON THE ROVER'S OWN COMPUTER (needs `udevadm`, part of systemd
and present on essentially any Ubuntu install - this script exits
early with a clear message if it isn't found, e.g. if accidentally run
somewhere else). Devices can be left plugged in cumulatively as you
go - each prompt just waits for ONE new device of the right kind to
appear, not a clean slate every time.

All the actual parsing/diffing/rule-rendering logic lives in
udev_device_id.py and is unit tested there; this file is intentionally
just the thin interactive/subprocess/filesystem layer around it.

Usage:
    python3 tools/identify_rover_devices.py
"""

from __future__ import annotations

import glob
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, Set

from udev_device_id import DEVICES, DeviceSpec, UdevAttrs, find_newly_appeared, parse_udevadm_info, render_rules_file

OUTPUT_PATH = (
    Path(__file__).resolve().parent.parent / "src" / "rover_bringup" / "config" / "udev" / "99-rover-serial.rules"
)


def list_candidate_devices(subsystem: str) -> Set[str]:
    if subsystem == "tty":
        return set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if subsystem == "video4linux":
        return set(glob.glob("/dev/video*"))
    raise ValueError(f"unknown subsystem {subsystem!r}")


def run_udevadm(device_path: str) -> str:
    result = subprocess.run(
        ["udevadm", "info", "-a", "-n", device_path],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def prompt_for_device(spec: DeviceSpec, known_before: Set[str]) -> str:
    """Prompts for `spec`, waits for Enter, and returns the newly
    appeared device path - retrying on zero or ambiguous results
    rather than guessing.
    """
    while True:
        input(
            f"\nPlug in ONLY the {spec.label} now if it isn't already "
            "connected (everything already identified can stay plugged "
            "in), then press Enter..."
        )
        after = list_candidate_devices(spec.subsystem)
        new_devices = find_newly_appeared(known_before, after)

        if len(new_devices) == 1:
            print(f"  -> detected {new_devices[0]}")
            return new_devices[0]
        if len(new_devices) == 0:
            print("  No new device detected - check the cable/port and try again.")
            continue

        print(f"  Multiple new devices appeared: {new_devices}")
        for i, dev in enumerate(new_devices):
            print(f"    [{i}] {dev}")
        choice = input("  Enter the number of the correct one, or press Enter to retry: ").strip()
        if choice.isdigit() and int(choice) < len(new_devices):
            return new_devices[int(choice)]


def main() -> int:
    if shutil.which("udevadm") is None:
        print(
            "udevadm not found on PATH - this must be run on the rover's "
            "own Linux computer (it's part of systemd), not in a dev "
            "sandbox or over a non-Linux connection."
        )
        return 1

    print("=" * 70)
    print("Rover USB device identification")
    print("=" * 70)
    print(f"Will walk through {len(DEVICES)} devices one at a time.")
    print("Unplug all rover USB devices first, if you haven't already.")
    input("Press Enter when ready...")

    captured: Dict[str, UdevAttrs] = {}
    known_tty = list_candidate_devices("tty")
    known_video = list_candidate_devices("video4linux")

    for spec in DEVICES:
        known_before = known_tty if spec.subsystem == "tty" else known_video
        device_path = prompt_for_device(spec, known_before)

        try:
            output = run_udevadm(device_path)
        except subprocess.CalledProcessError as exc:
            print(f"  udevadm failed for {device_path}: {exc}. Skipping - fill this one in by hand.")
            continue

        attrs = parse_udevadm_info(output)
        if attrs is None:
            print(f"  Could not find idVendor/idProduct for {device_path}. Skipping - fill this one in by hand.")
            continue

        captured[spec.key] = attrs
        print(
            f"  idVendor={attrs.id_vendor} idProduct={attrs.id_product} "
            f"serial={attrs.serial!r} kernels={attrs.kernels}"
        )

        # Refresh the running baseline for this subsystem so the device
        # just captured isn't re-flagged as "new" next time, and so it
        # can be safely left plugged in for the remaining prompts.
        if spec.subsystem == "tty":
            known_tty = list_candidate_devices("tty")
        else:
            known_video = list_candidate_devices("video4linux")

    print("\n" + "=" * 70)
    print(f"Captured {len(captured)}/{len(DEVICES)} devices.")
    if len(captured) < len(DEVICES):
        missing = [d.key for d in DEVICES if d.key not in captured]
        print(f"Missing: {missing} - re-run to fill these in, or edit the rules file by hand for just these.")

    rules_text = render_rules_file(captured)
    print("\n--- Generated rules file ---\n")
    print(rules_text)

    confirm = input(f"Write this to {OUTPUT_PATH}? [y/N] ").strip().lower()
    if confirm == "y":
        OUTPUT_PATH.write_text(rules_text)
        print(f"Wrote {OUTPUT_PATH}")
        print(
            "Next: sudo cp it to /etc/udev/rules.d/, then "
            "sudo udevadm control --reload-rules && sudo udevadm trigger "
            "(see docs/INSTALL.md Section 8.3-8.4)."
        )
    else:
        print("Not written. Re-run when ready, or edit the file by hand.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
