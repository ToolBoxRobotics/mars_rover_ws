"""Pure logic backing tools/identify_rover_devices.py: parsing
`udevadm info -a -n <device>` output, diffing device-node listings to
detect what just got plugged in, spotting the "identical/empty serial
across multiple Mega 2560 boards" collision documented in
99-rover-serial.rules, and rendering the final rules file text.

Kept free of subprocess/interactive-prompt/filesystem-glob concerns so
it can be unit tested with plain synthetic strings and sets, the same
split used throughout this workspace (e.g. rover_sensors's NMEA/BNO086
parsers) between pure logic and the thin hardware-facing IO layer that
calls it.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Set


@dataclass(frozen=True)
class DeviceSpec:
    key: str        # short identifier, e.g. "base"
    label: str      # human-readable prompt text
    subsystem: str  # "tty" or "video4linux"
    symlink: str    # e.g. "rover/base"
    comment: str    # short description embedded above the generated rule


# Order matters: this is the sequence identify_rover_devices.py prompts in.
DEVICES: List[DeviceSpec] = [
    DeviceSpec("base", "Base Mega #1 (drive + 4-corner steering + encoders)", "tty",
               "rover/base", "Base Mega #1 (drive + 4-corner steering + encoders)"),
    DeviceSpec("arm", "Arm Mega #2 (5-axis joint control)", "tty",
               "rover/arm", "Arm Mega #2 (5-axis joint control)"),
    DeviceSpec("mast", "Mast Uno #3 (pan/tilt head + erect/stow lift)", "tty",
               "rover/mast", "Mast Uno #3 (pan/tilt head + erect/stow lift)"),
    DeviceSpec("microscope", "Microscope Uno #4 (focus/zoom, LED, lens cover)", "tty",
               "rover/microscope", "Microscope Uno #4 (focus/zoom, LED, lens cover)"),
    DeviceSpec("antenna", "Antenna gimbal Uno #5 (azimuth/elevation HGA pointing)", "tty",
               "rover/antenna", "Antenna gimbal Uno #5 (azimuth/elevation HGA pointing)"),
    DeviceSpec("imu", "BNO086 IMU (Waveshare USB-TTL converter, UART-RVC mode)", "tty",
               "rover/imu", "BNO086 IMU on a Waveshare Industrial USB-TTL converter (no Arduino)"),
    DeviceSpec("gps", "L76X GPS (Waveshare USB-TTL converter)", "tty",
               "rover/gps", "Waveshare L76X GPS (also direct-to-USB, no Arduino)"),
    DeviceSpec("lidar", "RPLIDAR C1", "tty",
               "rover/lidar", "RPLIDAR C1"),
    DeviceSpec("main_cam", "Main perception USB camera", "video4linux",
               "rover/main_cam", "Main perception USB camera"),
    DeviceSpec("microscope_cam", "Microscope USB camera", "video4linux",
               "rover/microscope_cam", "Microscope USB camera"),
]

# Only these two are physically identical Mega 2560 boards that can
# share a VID:PID (and possibly an empty/identical serial) - see
# find_ambiguous_serial_keys(). Originally just the two Mega 2560
# boards (base, arm - the mast used to be a third Mega here; it's an
# Uno now, with a different USB VID:PID by default) - renamed and
# extended to also cover the two Uno boards (microscope, antenna)
# once there were two of those sharing a VID:PID too. Safe to check
# together: base/arm's idProduct (0042) never matches microscope/
# antenna's (0043), so a collision can only ever be detected within
# each actual matching pair, never across them.
SERIAL_COLLISION_RISK_KEYS = ("base", "arm", "microscope", "antenna")


@dataclass(frozen=True)
class UdevAttrs:
    id_vendor: str
    id_product: str
    serial: str            # "" if the device reports no serial at all
    kernels: Optional[str]  # physical USB port path, e.g. "1-4.2"; None if not found


def find_newly_appeared(before: Set[str], after: Set[str]) -> List[str]:
    """Device paths present in `after` but not `before`, sorted for a
    deterministic return value regardless of set iteration order.
    """
    return sorted(after - before)


def parse_udevadm_info(output: str) -> Optional[UdevAttrs]:
    """Parses the full text of `udevadm info -a -n <device>` and
    returns the attributes from the first (leaf-most) stanza that has
    both idVendor and idProduct set - that's the actual USB device
    node, as opposed to the tty character device or a USB *interface*
    sub-node one level down, neither of which carries these attributes
    on its own. Returns None if no such stanza is found (malformed or
    unexpected udevadm output).
    """
    stanzas = re.split(r"\n(?=\s*looking at)", output)
    for stanza in stanzas:
        id_vendor = _extract_attr(stanza, "ATTRS{idVendor}")
        id_product = _extract_attr(stanza, "ATTRS{idProduct}")
        if id_vendor and id_product:
            serial = _extract_attr(stanza, "ATTRS{serial}")
            kernels = _extract_attr(stanza, "KERNELS")
            return UdevAttrs(id_vendor=id_vendor, id_product=id_product, serial=serial or "", kernels=kernels)
    return None


def _extract_attr(stanza_text: str, attr_name: str) -> Optional[str]:
    escaped = re.escape(attr_name)
    match = re.search(rf'{escaped}=="([^"]*)"', stanza_text)
    return match.group(1) if match else None


def find_ambiguous_serial_keys(captured: Dict[str, UdevAttrs]) -> Set[str]:
    """Returns the subset of SERIAL_COLLISION_RISK_KEYS whose
    (idVendor, idProduct, serial) triple collides with another board's
    of the same type - meaning ATTRS{serial} alone can't tell them
    apart (including two boards that both report an empty serial,
    which is the common case this is actually guarding against - see
    the caveat in 99-rover-serial.rules).
    """
    fingerprint_to_keys: Dict[tuple, List[str]] = {}
    for key in SERIAL_COLLISION_RISK_KEYS:
        attrs = captured.get(key)
        if attrs is None:
            continue
        fingerprint = (attrs.id_vendor, attrs.id_product, attrs.serial)
        fingerprint_to_keys.setdefault(fingerprint, []).append(key)

    ambiguous: Set[str] = set()
    for keys in fingerprint_to_keys.values():
        if len(keys) > 1:
            ambiguous.update(keys)
    return ambiguous


def render_rule_line(spec: DeviceSpec, attrs: UdevAttrs, use_kernels: bool) -> str:
    """Renders one udev rule line. Falls back to KERNELS (physical USB
    port path) instead of ATTRS{serial} when use_kernels is True -
    only meant to be set for entries returned by
    find_ambiguous_serial_keys(), and only if a KERNELS value was
    actually captured.
    """
    if use_kernels and attrs.kernels:
        match_clause = f'KERNELS=="{attrs.kernels}"'
    else:
        match_clause = f'ATTRS{{serial}}=="{attrs.serial}"'
    return (
        f'SUBSYSTEM=="{spec.subsystem}", ATTRS{{idVendor}}=="{attrs.id_vendor}", '
        f'ATTRS{{idProduct}}=="{attrs.id_product}", {match_clause}, SYMLINK+="{spec.symlink}"'
    )


_FILE_HEADER = '''# Stable /dev/rover/* symlinks for every serial device on the rover.
#
# Generated by tools/identify_rover_devices.py from values captured
# live off the actual hardware - not hand-edited placeholders. If you
# add, replace, or re-flash a device, re-run that script rather than
# editing this file directly.
#
# INSTALL:
#   sudo cp 99-rover-serial.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger
'''

_MEGA_COLLISION_NOTE = '''#
# NOTE: two or more of the Mega 2560 boards reported the same (or an
# empty) USB serial number, so ATTRS{{serial}} alone couldn't tell them
# apart - those entries below match on KERNELS (physical USB port
# path) instead. This is less portable (it breaks if you move which
# hub port a board is plugged into) but was the only option available
# without re-flashing each board's ATmega16u2 with a unique serial.
# Affected: {affected}
'''


def render_rules_file(captured: Dict[str, UdevAttrs]) -> str:
    """Renders the complete 99-rover-serial.rules file text from a
    key -> UdevAttrs mapping (expected to have one entry per DEVICES
    entry, though missing keys are simply skipped rather than raising,
    so a partial capture still produces a partial-but-valid file).
    """
    ambiguous = find_ambiguous_serial_keys(captured)

    parts = [_FILE_HEADER]
    if ambiguous:
        affected = ", ".join(sorted(ambiguous))
        parts.append(_MEGA_COLLISION_NOTE.format(affected=affected))
    parts.append("")

    for spec in DEVICES:
        attrs = captured.get(spec.key)
        if attrs is None:
            continue
        parts.append(f"# {spec.comment}")
        parts.append(render_rule_line(spec, attrs, use_kernels=spec.key in ambiguous))
        parts.append("")

    return "\n".join(parts).rstrip() + "\n"
