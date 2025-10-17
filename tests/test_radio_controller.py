import struct
import threading
import unittest
from unittest import mock

from kv4p_python.radio_controller import HostCommand, RadioController, RadioMode


class DummyLock:
    def __init__(self) -> None:
        self.enter_count = 0
        self.exit_count = 0
        self.in_context = False

    def __enter__(self):
        self.enter_count += 1
        if self.in_context:
            raise AssertionError("Lock entered while already held.")
        self.in_context = True
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if not self.in_context:
            raise AssertionError("Lock exit without matching enter.")
        self.exit_count += 1
        self.in_context = False


class FakeSerial:
    def __init__(self, lock: DummyLock) -> None:
        self._lock = lock
        self.is_open = True
        self.writes = []
        self.flush_calls = 0

    def write(self, payload: bytes) -> int:
        if not self._lock.in_context:
            raise AssertionError("Serial.write invoked without acquiring lock.")
        self.writes.append(bytes(payload))
        return len(payload)

    def flush(self) -> None:
        if not self._lock.in_context:
            raise AssertionError("Serial.flush invoked without acquiring lock.")
        self.flush_calls += 1


class SendBytesLockTests(unittest.TestCase):
    def test_send_bytes_acquires_lock(self) -> None:
        lock = DummyLock()
        fake_serial = FakeSerial(lock)

        controller = object.__new__(RadioController)
        controller._lock = lock  # type: ignore[attr-defined]
        controller._serial = fake_serial  # type: ignore[attr-defined]
        controller._error_listeners = []  # type: ignore[attr-defined]
        controller._audio_listeners = []  # type: ignore[attr-defined]
        controller._notify_error = lambda exc: None  # type: ignore[attr-defined]

        payload = b"test-bytes"
        controller._send_bytes_to_esp32(payload)

        self.assertEqual(fake_serial.writes, [payload])
        self.assertEqual(fake_serial.flush_calls, 1)
        self.assertEqual(lock.enter_count, 1)
        self.assertEqual(lock.exit_count, 1)


class DisposeTests(unittest.TestCase):
    def test_dispose_calls_close(self) -> None:
        controller = object.__new__(RadioController)
        called = {"closed": False}

        def close_connection() -> None:
            called["closed"] = True

        controller.close_connection = close_connection  # type: ignore[assignment]
        controller.dispose()

        self.assertTrue(called["closed"])


class TuneCommandTests(unittest.TestCase):
    def test_tune_sends_stop_before_tune(self) -> None:
        controller = object.__new__(RadioController)
        sent = []
        controller._lock = threading.RLock()  # type: ignore[attr-defined]
        controller._mode = RadioMode.RX  # type: ignore[attr-defined]
        controller._audio_listeners = []  # type: ignore[attr-defined]
        controller._error_listeners = []  # type: ignore[attr-defined]
        controller._serial = None  # type: ignore[attr-defined]
        controller._tx_pcm_buffer = bytearray()  # type: ignore[attr-defined]
        controller._opus_encoder = None  # type: ignore[attr-defined]
        controller._encoder_lock = threading.RLock()  # type: ignore[attr-defined]
        controller._frame_overhead = 7  # type: ignore[attr-defined]
        controller._window_available = None  # type: ignore[attr-defined]
        controller._window_condition = threading.Condition()  # type: ignore[attr-defined]

        def send_command(command, params=None) -> None:  # type: ignore[no-redef]
            sent.append((command, params))

        controller._send_command = send_command  # type: ignore[attr-defined]
        controller.stop = RadioController.stop.__get__(controller, RadioController)
        controller._make_safe_2m_freq = RadioController._make_safe_2m_freq.__get__(controller, RadioController)  # type: ignore[attr-defined]

        with mock.patch("kv4p_python.radio_controller.time.sleep", lambda _: None):
            controller.tune_to_frequency("146.520", "146.990", 12, 3)

        self.assertEqual(sent[0][0], HostCommand.STOP)
        self.assertEqual(sent[1][0], HostCommand.GROUP)
        payload = sent[1][1]
        self.assertIsInstance(payload, (bytes, bytearray))
        bw, tx, rx, tone_tx, squelch, tone_rx = struct.unpack("<BffBBB", payload)
        self.assertEqual(bw, 0)
        self.assertAlmostEqual(tx, 146.520, places=3)
        self.assertAlmostEqual(rx, 146.990, places=3)
        self.assertEqual(tone_tx, 12)
        self.assertEqual(tone_rx, 12)
        self.assertEqual(squelch, 3)


class VersionParsingTests(unittest.TestCase):
    def test_extract_version_with_variants(self) -> None:
        self.assertEqual(RadioController._extract_version("VERSION00000005"), 5)
        self.assertEqual(RadioController._extract_version("FWVERSION:00000006"), 6)
        self.assertEqual(RadioController._extract_version("FW_VER 0007\r\n"), 7)
        self.assertIsNone(RadioController._extract_version("no version here"))


class StartupFallbackTests(unittest.TestCase):
    def test_non_text_handshake_falls_back_to_rx(self) -> None:
        controller = object.__new__(RadioController)
        controller._lock = threading.RLock()  # type: ignore[attr-defined]
        controller._mode = RadioMode.STARTUP  # type: ignore[attr-defined]
        controller._startup_fallback_threshold = 8  # type: ignore[attr-defined]
        controller._startup_bytes = 0  # type: ignore[attr-defined]
        controller._handshake_fallback_logged = False  # type: ignore[attr-defined]
        controller._notify_error = lambda exc: None  # type: ignore[attr-defined]
        controller._notify_audio = lambda data: None  # type: ignore[attr-defined]
        controller._audio_listeners = []  # type: ignore[attr-defined]
        controller._error_listeners = []  # type: ignore[attr-defined]
        controller._read_buffer = bytearray()  # type: ignore[attr-defined]
        controller._frame_overhead = 7  # type: ignore[attr-defined]
        controller._window_condition = threading.Condition()  # type: ignore[attr-defined]
        controller._window_available = 0  # type: ignore[attr-defined]

        controller._handle_data(bytes([0x80] * 8))

        self.assertEqual(controller._mode, RadioMode.RX)  # type: ignore[attr-defined]


if __name__ == "__main__":
    unittest.main()
