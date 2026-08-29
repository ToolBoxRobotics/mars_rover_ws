import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from udev_device_id import (
    DEVICES,
    UdevAttrs,
    find_ambiguous_serial_keys,
    find_newly_appeared,
    parse_udevadm_info,
    render_rule_line,
    render_rules_file,
)

# Realistic (structure-accurate) synthetic udevadm output, matching what
# `udevadm info -a -n /dev/ttyACM0` actually prints: a leaf "looking at
# device" stanza for the tty char device, then successive "looking at
# parent device" stanzas walking up toward the physical USB device,
# where idVendor/idProduct/serial actually live.
def _make_udevadm_output(id_vendor, id_product, serial, kernels="1-4.2"):
    serial_line = f'    ATTRS{{serial}}=="{serial}"\n' if serial is not None else ""
    return f"""Udevadm info starts with the device specified by the devpath and then
walks up the chain of parent devices.

  looking at device '/devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4.2/1-4.2:1.0/tty/ttyACM0':
    KERNEL=="ttyACM0"
    SUBSYSTEM=="tty"

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4.2/1-4.2:1.0':
    KERNELS=="1-4.2:1.0"
    SUBSYSTEMS=="usb"
    DRIVERS=="cdc_acm"

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4.2':
    KERNELS=="{kernels}"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{{idVendor}}=="{id_vendor}"
    ATTRS{{idProduct}}=="{id_product}"
{serial_line}    ATTRS{{manufacturer}}=="Arduino (www.arduino.cc)"

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-4':
    KERNELS=="1-4"
    SUBSYSTEMS=="usb"
"""


# -- parse_udevadm_info ---------------------------------------------------
def test_parse_udevadm_info_extracts_all_fields():
    output = _make_udevadm_output("2341", "0042", "95033313938351E0C0", kernels="1-4.2")
    attrs = parse_udevadm_info(output)
    assert attrs == UdevAttrs(id_vendor="2341", id_product="0042", serial="95033313938351E0C0", kernels="1-4.2")


def test_parse_udevadm_info_handles_empty_serial():
    output = _make_udevadm_output("2341", "0042", "")
    attrs = parse_udevadm_info(output)
    assert attrs.serial == ""


def test_parse_udevadm_info_handles_missing_serial_line_entirely():
    output = _make_udevadm_output("2341", "0042", None)
    attrs = parse_udevadm_info(output)
    assert attrs.id_vendor == "2341"
    assert attrs.serial == ""  # absent entirely is treated the same as empty


def test_parse_udevadm_info_returns_none_when_no_usb_device_stanza_found():
    garbage = "this is not udevadm output at all\nno stanzas here\n"
    assert parse_udevadm_info(garbage) is None


def test_parse_udevadm_info_skips_leaf_stanza_lacking_attrs():
    # The leaf tty stanza has no idVendor/idProduct - parser must walk
    # up to the parent stanza that actually has them, not stop early.
    output = _make_udevadm_output("1a86", "55d3", "AB123")
    attrs = parse_udevadm_info(output)
    assert attrs.id_vendor == "1a86"
    assert attrs.id_product == "55d3"


# -- find_newly_appeared ---------------------------------------------------
def test_find_newly_appeared_detects_single_new_device():
    before = {"/dev/ttyACM0"}
    after = {"/dev/ttyACM0", "/dev/ttyACM1"}
    assert find_newly_appeared(before, after) == ["/dev/ttyACM1"]


def test_find_newly_appeared_empty_when_nothing_changed():
    devices = {"/dev/ttyACM0", "/dev/ttyUSB0"}
    assert find_newly_appeared(devices, devices) == []


def test_find_newly_appeared_sorted_for_determinism():
    before = set()
    after = {"/dev/ttyUSB1", "/dev/ttyACM0"}
    assert find_newly_appeared(before, after) == ["/dev/ttyACM0", "/dev/ttyUSB1"]


# -- find_ambiguous_serial_keys ---------------------------------------------
def test_no_collision_when_all_megas_have_distinct_serials():
    captured = {
        "base": UdevAttrs("2341", "0042", "SERIAL_A", "1-4.1"),
        "arm": UdevAttrs("2341", "0042", "SERIAL_B", "1-4.2"),
    }
    assert find_ambiguous_serial_keys(captured) == set()


def test_collision_detected_when_two_megas_share_empty_serial():
    captured = {
        "base": UdevAttrs("2341", "0042", "", "1-4.1"),
        "arm": UdevAttrs("2341", "0042", "", "1-4.2"),
    }
    assert find_ambiguous_serial_keys(captured) == {"base", "arm"}


def test_collision_detected_when_both_megas_share_identical_nonempty_serial():
    captured = {
        "base": UdevAttrs("2341", "0042", "SAME", "1-4.1"),
        "arm": UdevAttrs("2341", "0042", "SAME", "1-4.2"),
    }
    assert find_ambiguous_serial_keys(captured) == {"base", "arm"}


def test_mast_never_flagged_even_with_a_shared_serial():
    # The mast is an Uno now (not a third Mega) and, unlike the two
    # other Uno boards (microscope, antenna), was never added to
    # SERIAL_COLLISION_RISK_KEYS - a "mast" entry must never appear in
    # the result even if it happens to share a serial with base/arm
    # (e.g. a coincidentally-matching value, or a bug elsewhere
    # capturing the wrong device) - this key is structurally excluded,
    # not just unlikely to collide in practice.
    captured = {
        "base": UdevAttrs("2341", "0042", "SAME", "1-4.1"),
        "arm": UdevAttrs("2341", "0042", "UNIQUE", "1-4.2"),
        "mast": UdevAttrs("2341", "0042", "SAME", "1-4.3"),
    }
    assert find_ambiguous_serial_keys(captured) == set()


def test_dissimilar_uno_boards_never_falsely_flagged():
    # microscope and antenna ARE collision-risk keys now (unlike mast,
    # which is structurally excluded above) - this specifically tests
    # that having unique serial values means neither gets flagged,
    # not that they're exempt from consideration entirely. See
    # test_microscope_and_antenna_flagged_when_they_actually_collide
    # below for the case where they should be.
    captured = {
        "base": UdevAttrs("2341", "0042", "UNIQUE1", "1-4.1"),
        "arm": UdevAttrs("2341", "0042", "UNIQUE2", "1-4.2"),
        "mast": UdevAttrs("2341", "0043", "UNIQUE3", "1-4.3"),  # Uno VID:PID, not a Mega's, and structurally excluded regardless
        "microscope": UdevAttrs("2341", "0043", "UNIQUE4", "1-4.4"),
        "antenna": UdevAttrs("2341", "0043", "UNIQUE5", "1-4.5"),
        "imu": UdevAttrs("1a86", "55d3", "", "1-4.6"),
    }
    assert find_ambiguous_serial_keys(captured) == set()


def test_microscope_and_antenna_flagged_when_they_actually_collide():
    # The actual point of extending SERIAL_COLLISION_RISK_KEYS beyond
    # base/arm: two Uno boards sharing 2341:0043 and an empty/matching
    # serial should be caught the same way two Megas already are.
    captured = {
        "microscope": UdevAttrs("2341", "0043", "", "1-4.4"),
        "antenna": UdevAttrs("2341", "0043", "", "1-4.5"),
    }
    assert find_ambiguous_serial_keys(captured) == {"microscope", "antenna"}


def test_missing_entries_do_not_crash():
    # Only "base" captured so far (mid-flow) - must not raise.
    assert find_ambiguous_serial_keys({"base": UdevAttrs("2341", "0042", "X", "1-4.1")}) == set()


# -- render_rule_line -------------------------------------------------------
def test_render_rule_line_uses_serial_by_default():
    spec = DEVICES[0]  # base
    attrs = UdevAttrs("2341", "0042", "MY_SERIAL", "1-4.1")
    line = render_rule_line(spec, attrs, use_kernels=False)
    assert 'ATTRS{serial}=="MY_SERIAL"' in line
    assert "KERNELS" not in line
    assert 'SUBSYSTEM=="tty"' in line
    assert 'SYMLINK+="rover/base"' in line


def test_render_rule_line_uses_kernels_when_requested():
    spec = DEVICES[0]
    attrs = UdevAttrs("2341", "0042", "", "1-4.1")
    line = render_rule_line(spec, attrs, use_kernels=True)
    assert 'KERNELS=="1-4.1"' in line
    assert "ATTRS{serial}" not in line


def test_render_rule_line_falls_back_to_serial_if_kernels_missing():
    spec = DEVICES[0]
    attrs = UdevAttrs("2341", "0042", "FALLBACK", kernels=None)
    line = render_rule_line(spec, attrs, use_kernels=True)
    assert 'ATTRS{serial}=="FALLBACK"' in line


def test_render_rule_line_uses_video_subsystem_for_cameras():
    camera_spec = next(d for d in DEVICES if d.key == "main_cam")
    attrs = UdevAttrs("046d", "082d", "CAM123", "1-4.6")
    line = render_rule_line(camera_spec, attrs, use_kernels=False)
    assert 'SUBSYSTEM=="video4linux"' in line


# -- render_rules_file -------------------------------------------------------
def test_render_rules_file_includes_every_captured_device():
    captured = {spec.key: UdevAttrs("VID", "PID", f"SERIAL_{spec.key}", "1-4.1") for spec in DEVICES}
    text = render_rules_file(captured)
    for spec in DEVICES:
        assert spec.symlink in text
    assert "Generated by tools/identify_rover_devices.py" in text


def test_render_rules_file_handles_partial_capture():
    captured = {"base": UdevAttrs("2341", "0042", "X", "1-4.1")}
    text = render_rules_file(captured)
    assert "rover/base" in text
    assert "rover/arm" not in text


def test_render_rules_file_adds_collision_note_when_needed():
    captured = {
        "base": UdevAttrs("2341", "0042", "", "1-4.1"),
        "arm": UdevAttrs("2341", "0042", "", "1-4.2"),
    }
    text = render_rules_file(captured)
    assert "NOTE" in text
    assert "arm, base" in text  # sorted() alphabetizes: arm < base
    assert 'KERNELS=="1-4.1"' in text
    assert 'KERNELS=="1-4.2"' in text


def test_render_rules_file_omits_collision_note_when_not_needed():
    captured = {"base": UdevAttrs("2341", "0042", "UNIQUE", "1-4.1")}
    text = render_rules_file(captured)
    assert "NOTE" not in text


def test_device_list_has_ten_entries_matching_the_rules_file():
    assert len(DEVICES) == 10
    assert len(set(d.key for d in DEVICES)) == 10  # all keys unique
    assert len(set(d.symlink for d in DEVICES)) == 10  # all symlinks unique
