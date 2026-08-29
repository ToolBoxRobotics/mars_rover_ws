import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rover_protocol.framing import encode_frame
from rover_protocol.serial_link import SerialLink


class FakeSerial:
    """Minimal stand-in for serial.Serial used in tests."""

    def __init__(self, port, baud, timeout, write_timeout=None, fail_open=False):
        if fail_open:
            raise IOError("simulated: device not present")
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.written = []
        self.rx_queue = []
        self.closed = False

    def write(self, data: bytes):
        self.written.append(data)

    def readline(self) -> bytes:
        if self.rx_queue:
            return self.rx_queue.pop(0)
        return b""

    def close(self):
        self.closed = True


def make_factory(fail_open=False, shared_holder=None):
    def factory(port, baud, timeout, write_timeout):
        fake = FakeSerial(port, baud, timeout, write_timeout=write_timeout, fail_open=fail_open)
        if shared_holder is not None:
            shared_holder.append(fake)
        return fake

    return factory


def test_ensure_open_success():
    link = SerialLink("/dev/rover/base", 115200, serial_factory=make_factory())
    assert link.ensure_open() is True
    assert link.connected is True
    assert link.reconnect_count == 1


def test_write_timeout_reaches_the_underlying_connection():
    # FIXED: a real, previously-unbounded hang, not a hypothetical
    # hardening - see serial_link.py's own module docstring for the
    # full incident. This is the regression test for it: confirms
    # write_timeout is actually threaded all the way from SerialLink's
    # own constructor through to the underlying connection, and that
    # it defaults to a real, bounded value rather than None (pyserial's
    # own default, which is exactly what allowed the unbounded block
    # in the first place).
    holder = []
    link = SerialLink("/dev/rover/base", 115200, serial_factory=make_factory(shared_holder=holder))
    link.ensure_open()
    assert holder[0].write_timeout is not None
    assert holder[0].write_timeout == link.write_timeout


def test_write_timeout_custom_value_reaches_the_underlying_connection():
    holder = []
    link = SerialLink(
        "/dev/rover/base", 115200, write_timeout=1.5, serial_factory=make_factory(shared_holder=holder)
    )
    link.ensure_open()
    assert holder[0].write_timeout == 1.5


def test_ensure_open_failure_stays_disconnected():
    link = SerialLink("/dev/rover/base", 115200, serial_factory=make_factory(fail_open=True))
    assert link.ensure_open() is False
    assert link.connected is False
    assert link.reconnect_count == 0


def test_write_frame_round_trip():
    holder = []
    link = SerialLink(
        "/dev/rover/base", 115200, serial_factory=make_factory(shared_holder=holder), boot_grace_sec=0
    )
    frame = encode_frame("D", [1, 2, 3])
    assert link.write_frame(frame) is True
    assert holder[0].written[0] == frame.encode("ascii")


def test_read_decoded_valid_frame():
    holder = []
    link = SerialLink(
        "/dev/rover/base", 115200, serial_factory=make_factory(shared_holder=holder), boot_grace_sec=0
    )
    link.ensure_open()
    frame = encode_frame("E", [10, 20, 30, 40, 50, 60])
    holder[0].rx_queue.append(frame.encode("ascii"))

    result = link.read_decoded()
    assert result == ("E", [10, 20, 30, 40, 50, 60])
    assert link.rx_frame_count == 1
    assert link.checksum_error_count == 0


def test_read_decoded_no_data_returns_none():
    link = SerialLink("/dev/rover/base", 115200, serial_factory=make_factory(), boot_grace_sec=0)
    link.ensure_open()
    assert link.read_decoded() is None


def test_read_decoded_bad_frame_counts_and_swallows():
    holder = []
    link = SerialLink(
        "/dev/rover/base", 115200, serial_factory=make_factory(shared_holder=holder), boot_grace_sec=0
    )
    link.ensure_open()
    holder[0].rx_queue.append(b"garbage-not-a-frame\n")

    result = link.read_decoded()
    assert result is None
    assert link.checksum_error_count == 1
    assert link.rx_frame_count == 0


def test_last_rx_age_sec_updates():
    holder = []
    fake_time = {"t": 100.0}
    link = SerialLink(
        "/dev/rover/base",
        115200,
        serial_factory=make_factory(shared_holder=holder),
        clock=lambda: fake_time["t"],
        boot_grace_sec=0,
    )
    link.ensure_open()
    assert link.last_rx_age_sec() == float("inf")

    frame = encode_frame("H", [])
    holder[0].rx_queue.append(frame.encode("ascii"))
    link.read_decoded()
    assert link.last_rx_age_sec() == 0.0

    fake_time["t"] += 2.5
    assert link.last_rx_age_sec() == 2.5


def test_write_frame_exception_marks_disconnected():
    class BoomOnWrite(FakeSerial):
        def write(self, data):
            raise IOError("cable unplugged")

    def factory(port, baud, timeout, write_timeout):
        return BoomOnWrite(port, baud, timeout)

    link = SerialLink("/dev/rover/base", 115200, serial_factory=factory, boot_grace_sec=0)
    link.ensure_open()
    assert link.write_frame("D,1*00\n") is False
    assert link.connected is False


# -- boot grace period (Arduino auto-reset-on-connect handling) -------------
def test_write_frame_no_ops_during_boot_grace_period():
    holder = []
    fake_time = {"t": 0.0}
    link = SerialLink(
        "/dev/rover/base",
        115200,
        serial_factory=make_factory(shared_holder=holder),
        clock=lambda: fake_time["t"],
        boot_grace_sec=2.0,
    )
    assert link.write_frame(encode_frame("H", [])) is False
    assert holder[0].written == []  # nothing actually sent to the port


def test_write_frame_resumes_once_grace_period_elapses():
    holder = []
    fake_time = {"t": 0.0}
    link = SerialLink(
        "/dev/rover/base",
        115200,
        serial_factory=make_factory(shared_holder=holder),
        clock=lambda: fake_time["t"],
        boot_grace_sec=2.0,
    )
    link.ensure_open()
    fake_time["t"] = 2.1  # just past the grace period
    frame = encode_frame("H", [])
    assert link.write_frame(frame) is True
    assert holder[0].written == [frame.encode("ascii")]


def test_read_decoded_no_ops_during_boot_grace_period():
    holder = []
    fake_time = {"t": 0.0}
    link = SerialLink(
        "/dev/rover/base",
        115200,
        serial_factory=make_factory(shared_holder=holder),
        clock=lambda: fake_time["t"],
        boot_grace_sec=2.0,
    )
    link.ensure_open()
    holder[0].rx_queue.append(encode_frame("E", [1, 2]).encode("ascii"))
    assert link.read_decoded() is None
    assert link.rx_frame_count == 0  # the queued frame is left completely unread


def test_in_boot_grace_period_reflects_elapsed_time():
    fake_time = {"t": 0.0}
    link = SerialLink(
        "/dev/rover/base",
        115200,
        serial_factory=make_factory(),
        clock=lambda: fake_time["t"],
        boot_grace_sec=2.0,
    )
    assert link.in_boot_grace_period() is False  # not connected yet
    link.ensure_open()
    assert link.in_boot_grace_period() is True
    fake_time["t"] = 2.5
    assert link.in_boot_grace_period() is False


def test_reconnect_restarts_the_grace_period():
    fake_time = {"t": 0.0}
    link = SerialLink(
        "/dev/rover/base",
        115200,
        serial_factory=make_factory(),
        clock=lambda: fake_time["t"],
        boot_grace_sec=2.0,
    )
    link.ensure_open()
    fake_time["t"] = 5.0
    assert link.in_boot_grace_period() is False

    link.close()
    fake_time["t"] = 5.1
    link.ensure_open()  # simulates a fresh reconnect - the board resets again
    assert link.in_boot_grace_period() is True
