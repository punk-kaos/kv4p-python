"""Native Python implementation of the KV4P radio controller."""

from __future__ import annotations

import asyncio
import enum
import logging
import struct
import threading
import time
from typing import Callable, Iterable, List, Optional, Tuple, Type

from typing import Tuple, Type, TypeVar


def _load_opus_codec() -> Tuple[bool, Optional[int], Optional[Type], Optional[Type], Optional[type], Optional[BaseException]]:
    codec_err: Optional[BaseException] = None
    try:
        from opuslib import APPLICATION_AUDIO
        from opuslib import Encoder as OpusEncoder
        from opuslib import Decoder as OpusDecoder
        from opuslib import exceptions as opus_exceptions
    except Exception as exc:  # pragma: no cover - runtime dependency
        codec_err = exc
        return False, None, None, None, None, codec_err
    else:  # pragma: no cover - runtime dependency
        return True, APPLICATION_AUDIO, OpusEncoder, OpusDecoder, opus_exceptions.OpusError, None


OPUS_AVAILABLE = False
APPLICATION_AUDIO = None  # type: ignore
OpusEncoder = None  # type: ignore
OpusDecoder = None  # type: ignore
OpusError = RuntimeError
_OPUS_ERROR: Optional[BaseException] = None

(_codec_loaded, _APPLICATION_AUDIO, _OpusEncoder, _OpusDecoder, _OpusError, _codec_err) = _load_opus_codec()
if _codec_loaded:
    OPUS_AVAILABLE = True
    APPLICATION_AUDIO = _APPLICATION_AUDIO  # type: ignore[assignment]
    OpusEncoder = _OpusEncoder  # type: ignore[assignment]
    OpusDecoder = _OpusDecoder  # type: ignore[assignment]
    OpusError = _OpusError  # type: ignore[assignment]
else:
    _OPUS_ERROR = _codec_err

try:  # pragma: no cover - runtime dependency
    import serial
    from serial import Serial
    from serial.serialutil import SerialException
except ImportError as exc:  # pragma: no cover - surfaced during runtime
    serial = None  # type: ignore[assignment]
    Serial = None  # type: ignore[assignment]

    class SerialException(RuntimeError):  # type: ignore[override]
        """Fallback serial exception when pyserial is missing."""

    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


LOGGER = logging.getLogger(__name__)


class RadioControllerError(RuntimeError):
    """Raised for high-level controller failures."""


class RadioMode(enum.Enum):
    STARTUP = "startup"
    RX = "rx"
    TX = "tx"
    SCAN = "scan"


class HostCommand(enum.IntEnum):
    PTT_DOWN = 0x01
    PTT_UP = 0x02
    GROUP = 0x03
    FILTERS = 0x04
    STOP = 0x05
    CONFIG = 0x06
    TX_AUDIO = 0x07
    HL = 0x08
    RSSI = 0x09


class DeviceCommand(enum.IntEnum):
    DEBUG_INFO = 0x01
    DEBUG_ERROR = 0x02
    DEBUG_WARN = 0x03
    DEBUG_DEBUG = 0x04
    DEBUG_TRACE = 0x05
    HELLO = 0x06
    RX_AUDIO = 0x07
    VERSION = 0x08
    WINDOW_UPDATE = 0x09
    SMETER_REPORT = 0x53
    PHYS_PTT_DOWN = 0x44
    PHYS_PTT_UP = 0x55


class RadioController:
    """Controls a KV4P radio over a serial link."""

    COMMAND_DELIMITER = bytes.fromhex("de ad be ef")
    MIN_FIRMWARE_VER = 4
    AUDIO_SAMPLE_RATE = 48_000
    OPUS_FRAME_DURATION_MS = 40

    def __init__(
        self,
        port_name: str,
        *,
        baudrate: int = 115_200,
        read_timeout: float = 1.0,
        write_timeout: float = 1.0,
    ) -> None:
        if _IMPORT_ERROR is not None:
            raise ImportError(
                "pyserial is required to use RadioController; install with `pip install pyserial`."
            ) from _IMPORT_ERROR

        if not port_name:
            raise ValueError("Port name cannot be empty.")

        self._port_name = port_name
        self._baudrate = baudrate
        self._read_timeout = read_timeout
        self._write_timeout = write_timeout
        self._serial: Optional[Serial] = None
        self._mode = RadioMode.STARTUP
        self._lock = threading.RLock()
        self._audio_listeners: List[Callable[[bytes], None]] = []
        self._error_listeners: List[Callable[[Exception], None]] = []
        self._stop_reader = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._startup_bytes = 0
        self._startup_fallback_threshold = 128
        self._handshake_fallback_logged = False
        self._read_buffer = bytearray()
        self._firmware_version: Optional[int] = None
        self._window_size: Optional[int] = None
        self._features: int = 0
        self._opus_frame_size = int(self.AUDIO_SAMPLE_RATE * self.OPUS_FRAME_DURATION_MS / 1000)
        self._opus_frame_bytes = self._opus_frame_size * 2
        if OPUS_AVAILABLE and OpusEncoder is not None and OpusDecoder is not None:
            self._opus_encoder = OpusEncoder(self.AUDIO_SAMPLE_RATE, 1, APPLICATION_AUDIO)  # type: ignore[arg-type]
            self._opus_decoder = OpusDecoder(self.AUDIO_SAMPLE_RATE, 1)  # type: ignore[misc]
        else:
            self._opus_encoder = None
            self._opus_decoder = None
            if _OPUS_ERROR is not None:
                LOGGER.warning("Opus codec unavailable: %s", _OPUS_ERROR)
        self._encoder_lock = threading.Lock()
        self._tx_pcm_buffer = bytearray()
        self._frame_overhead = len(self.COMMAND_DELIMITER) + 3
        self._window_available: Optional[int] = None
        self._window_condition = threading.Condition()

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def open_connection(self) -> None:
        with self._lock:
            if self._serial and self._serial.is_open:
                return

            try:
                self._serial = Serial(
                    self._port_name,
                    self._baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self._read_timeout,
                    write_timeout=self._write_timeout,
                )
            except SerialException as exc:  # pragma: no cover - hardware-specific
                raise RadioControllerError(str(exc)) from exc

            self._stop_reader.clear()
            self._read_buffer.clear()
            self._startup_bytes = 0
            self._handshake_fallback_logged = False
            self._tx_pcm_buffer.clear()
            with self._window_condition:
                self._window_available = None
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

    def close_connection(self) -> None:
        with self._lock:
            self._stop_reader.set()
            serial_obj = self._serial
            self._serial = None

        if serial_obj is not None and serial_obj.is_open:  # pragma: no branch
            try:
                serial_obj.close()
            except SerialException as exc:  # pragma: no cover - hardware-specific
                self._notify_error(exc)

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._read_buffer.clear()
        self._tx_pcm_buffer.clear()
        with self._window_condition:
            self._window_available = None

    # API parity with C# RadioController.Dispose
    def dispose(self) -> None:
        """Mirror the C# Dispose() method for GUI compatibility."""
        self.close_connection()

    def __enter__(self) -> "RadioController":
        self.open_connection()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close_connection()

    # ------------------------------------------------------------------
    # Public API mirroring the C# version
    # ------------------------------------------------------------------
    def initialize(self) -> None:
        with self._lock:
            self._mode = RadioMode.STARTUP
            self._startup_bytes = 0
            self._handshake_fallback_logged = False
            self._firmware_version = None
            self._tx_pcm_buffer.clear()
        self._send_command(HostCommand.STOP)
        self._send_command(HostCommand.CONFIG, bytes([0x00]))

    def start_rx_mode(self) -> None:
        with self._lock:
            self._mode = RadioMode.RX

    def start_tx_mode(self) -> None:
        with self._lock:
            self._mode = RadioMode.TX
            self._tx_pcm_buffer.clear()
        self._send_command(HostCommand.PTT_DOWN)

    def end_tx_mode(self) -> None:
        with self._lock:
            if self._mode != RadioMode.TX:
                return
            self._mode = RadioMode.RX
            self._tx_pcm_buffer.clear()
        self._send_command(HostCommand.PTT_UP)

    def stop(self) -> None:
        with self._lock:
            self._mode = RadioMode.RX
            self._tx_pcm_buffer.clear()
        self._send_command(HostCommand.STOP)

    def tune_to_frequency(
        self,
        tx_frequency: str,
        rx_frequency: str,
        tone: int,
        squelch_level: int,
        *,
        wideband: bool = False,
    ) -> None:
        if not tx_frequency or not rx_frequency:
            raise ValueError("TX and RX frequencies are required.")

        if not 0 <= squelch_level <= 9:
            raise ValueError("Squelch level must be between 0 and 9.")

        tx_freq = self._make_safe_2m_freq(tx_frequency)
        rx_freq = self._make_safe_2m_freq(rx_frequency)
        tone_value = max(0, min(255, tone))
        params = struct.pack(
            "<BffBBB",
            1 if wideband else 0,
            tx_freq,
            rx_freq,
            tone_value,
            squelch_level,
            tone_value,
        )
        # Ensure the ESP32 exits any previous state before retuning; STOP is idempotent.
        LOGGER.debug(
            "Tune request TX=%.4f RX=%.4f tone=%d squelch=%d wideband=%s mode=%s",
            tx_freq,
            rx_freq,
            tone_value,
            squelch_level,
            wideband,
            getattr(self, "_mode", None),
        )
        self.stop()
        time.sleep(0.05)
        self._send_command(HostCommand.GROUP, params)

    def set_filters(self, emphasis: bool, highpass: bool, lowpass: bool) -> None:
        flags = (1 if emphasis else 0) | (2 if highpass else 0) | (4 if lowpass else 0)
        self._send_command(HostCommand.FILTERS, bytes([flags]))

    def send_audio_data(
        self,
        audio_data: bytes | bytearray | memoryview | Iterable[int],
    ) -> None:
        if self._opus_encoder is None:
            LOGGER.warning("opuslib not available; dropping TX audio payload.")
            return
        frames: List[bytes] = []
        with self._lock:
            if self._mode != RadioMode.TX:
                LOGGER.debug("Ignoring audio data while not in TX mode.")
                return

            if isinstance(audio_data, (bytes, bytearray, memoryview)):
                data = bytes(audio_data)
            else:
                data = bytes(audio_data)

            if not data:
                return

            buffer = self._tx_pcm_buffer
            buffer.extend(data)
            frame_bytes = self._opus_frame_bytes
            while len(buffer) >= frame_bytes:
                frame = bytes(buffer[:frame_bytes])
                del buffer[:frame_bytes]
                frames.append(frame)

        for frame in frames:
            try:
                with self._encoder_lock:
                    encoded = self._opus_encoder.encode(frame, self._opus_frame_size)  # type: ignore[union-attr]
            except OpusError as exc:
                LOGGER.error("Failed to encode TX audio: %s", exc)
                self._notify_error(exc)
                break
            self._send_command(HostCommand.TX_AUDIO, encoded)

    async def send_audio_data_async(
        self,
        audio_data: bytes | bytearray | memoryview | Iterable[int],
        *,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ) -> None:
        target_loop = loop or asyncio.get_event_loop()
        await target_loop.run_in_executor(None, self.send_audio_data, audio_data)

    # ------------------------------------------------------------------
    # Listener management
    # ------------------------------------------------------------------
    def add_audio_listener(self, callback: Callable[[bytes], None]) -> None:
        self._audio_listeners.append(callback)

    def remove_audio_listener(self, callback: Callable[[bytes], None]) -> None:
        if callback in self._audio_listeners:
            self._audio_listeners.remove(callback)

    def add_error_listener(self, callback: Callable[[Exception], None]) -> None:
        self._error_listeners.append(callback)

    def remove_error_listener(self, callback: Callable[[Exception], None]) -> None:
        if callback in self._error_listeners:
            self._error_listeners.remove(callback)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        while not self._stop_reader.is_set():
            try:
                serial_obj = self._serial
                if serial_obj is None or not serial_obj.is_open:
                    break

                waiting = serial_obj.in_waiting or 1
                data = serial_obj.read(waiting)
                if data:
                    self._handle_data(data)
            except SerialException as exc:  # pragma: no cover - hardware specific
                self._notify_error(exc)
                break
            except Exception as exc:  # pragma: no cover - defensive
                self._notify_error(exc)
                break

    def _handle_data(self, data: bytes) -> None:
        if not data:
            return

        with self._lock:
            mode = self._mode

        if mode == RadioMode.STARTUP and not self._handshake_fallback_logged:
            LOGGER.debug("Startup chunk (hex): %s", data.hex(" "))
            printable = "".join(chr(b) if 32 <= b <= 126 else "." for b in data)
            LOGGER.debug("Startup chunk (ascii): %s", printable)
            self._startup_bytes += len(data)
            if self._startup_bytes >= self._startup_fallback_threshold:
                LOGGER.warning(
                    "No firmware banner detected after %d bytes; assuming firmware >= %d.",
                    self._startup_bytes,
                    self.MIN_FIRMWARE_VER,
                )
                with self._lock:
                    self._mode = RadioMode.RX
                self._handshake_fallback_logged = True
                with self._window_condition:
                    if self._window_available is None and self._window_size:
                        self._window_available = self._window_size
                    self._window_condition.notify_all()

        self._read_buffer.extend(data)
        self._process_incoming_buffer()

    def _process_incoming_buffer(self) -> None:
        delim = self.COMMAND_DELIMITER
        header_len = len(delim) + 1 + 2

        while True:
            buffer = self._read_buffer
            start = buffer.find(delim)
            if start == -1:
                # Retain only the last delimiter-sized tail in case a frame is split.
                if len(buffer) > len(delim) - 1:
                    del buffer[: len(buffer) - (len(delim) - 1)]
                break

            if start > 0:
                del buffer[:start]

            if len(buffer) < header_len:
                break

            command_byte = buffer[len(delim)]
            length = int.from_bytes(buffer[len(delim) + 1 : len(delim) + 3], "little")
            frame_len = header_len + length
            if len(buffer) < frame_len:
                break

            payload = bytes(buffer[header_len:frame_len])
            del buffer[:frame_len]
            self._handle_frame(command_byte, payload)

    def _handle_frame(self, command_byte: int, payload: bytes) -> None:
        try:
            command = DeviceCommand(command_byte)
        except ValueError:
            LOGGER.debug("Unknown ESP32 command 0x%02X (%d bytes)", command_byte, len(payload))
            return

        if command == DeviceCommand.VERSION:
            self._handle_version(payload)
        elif command == DeviceCommand.RX_AUDIO:
            if self._opus_decoder is None:
                LOGGER.debug("Ignoring RX audio because opuslib is unavailable.")
                return
            try:
                decoded = self._opus_decoder.decode(payload, self._opus_frame_size)
            except OpusError as exc:
                LOGGER.warning("Failed to decode RX audio: %s", exc)
            else:
                self._notify_audio(decoded)
        elif command == DeviceCommand.WINDOW_UPDATE:
            if len(payload) >= 4:
                window = int.from_bytes(payload[:4], "little")
                with self._window_condition:
                    if self._window_available is None:
                        self._window_available = window
                    else:
                        self._window_available += window
                    self._window_condition.notify_all()
                self._window_size = window
                LOGGER.debug("Window update from ESP32: %d (available=%s)", window, self._window_available)
        elif command == DeviceCommand.SMETER_REPORT:
            if payload:
                LOGGER.debug("RSSI report: %d", payload[0])
        elif command in {DeviceCommand.PHYS_PTT_DOWN, DeviceCommand.PHYS_PTT_UP}:
            LOGGER.info("Physical PTT %s", "down" if command == DeviceCommand.PHYS_PTT_DOWN else "up")
        elif command in {
            DeviceCommand.DEBUG_INFO,
            DeviceCommand.DEBUG_WARN,
            DeviceCommand.DEBUG_ERROR,
            DeviceCommand.DEBUG_DEBUG,
            DeviceCommand.DEBUG_TRACE,
        }:
            message = payload.decode("utf-8", errors="ignore") if payload else ""
            if command == DeviceCommand.DEBUG_ERROR:
                LOGGER.error("ESP32 error: %s", message)
            elif command == DeviceCommand.DEBUG_WARN:
                LOGGER.warning("ESP32 warning: %s", message)
            elif command == DeviceCommand.DEBUG_INFO:
                LOGGER.info("ESP32 info: %s", message)
            else:
                LOGGER.debug("ESP32 debug: %s", message)
        elif command == DeviceCommand.HELLO:
            LOGGER.debug("Received HELLO message from radio (len=%d)", len(payload))

    def _handle_version(self, payload: bytes) -> None:
        if len(payload) < 9:
            LOGGER.warning("VERSION payload too short: %d bytes", len(payload))
            return

        version = int.from_bytes(payload[0:2], "little")
        status = payload[2]
        window_size = int.from_bytes(payload[3:7], "little")
        rf_module = payload[7]
        features = payload[8]

        self._firmware_version = version
        self._window_size = window_size
        self._features = features

        with self._lock:
            self._mode = RadioMode.RX
            self._startup_bytes = 0
            self._handshake_fallback_logged = True

        with self._window_condition:
            self._window_available = window_size
            self._window_condition.notify_all()

        LOGGER.info(
            "Firmware v%d status=0x%02X window=%d rf_module=%d features=0x%02X",
            version,
            status,
            window_size,
            rf_module,
            features,
        )

    def _send_command(self, command: HostCommand, params: Optional[bytes] = None) -> None:
        payload = params or b""
        if len(payload) > 0xFFFF:
            raise ValueError("Payload too large for protocol frame (max 65535 bytes).")
        frame_len = self._frame_overhead + len(payload)
        with self._window_condition:
            if self._window_available is not None:
                deadline = time.monotonic() + 2.0
                while frame_len > self._window_available:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        LOGGER.debug(
                            "Waiting for window: need %d bytes, available %d", frame_len, self._window_available
                        )
                        deadline = time.monotonic() + 2.0
                    self._window_condition.wait(timeout=0.05)
                self._window_available -= frame_len

        frame = (
            self.COMMAND_DELIMITER
            + bytes([int(command)])
            + len(payload).to_bytes(2, "little")
            + payload
        )
        LOGGER.debug("Sending command %s (%#04x) len=%d", command.name, int(command), len(payload))
        self._send_bytes_to_esp32(frame)

    def _send_bytes_to_esp32(self, payload: bytes) -> None:
        error: Optional[Exception] = None
        with self._lock:
            serial_obj = self._serial
            if serial_obj is None or not serial_obj.is_open:
                raise RadioControllerError("Serial port is not open.")

            try:
                written = serial_obj.write(payload)
                if written != len(payload):  # pragma: no cover - defensive
                    raise RadioControllerError(
                        f"Short write to serial port (expected {len(payload)}, wrote {written})."
                    )
                serial_obj.flush()
                LOGGER.debug("Wrote %d bytes to serial port.", written)
            except SerialException as exc:  # pragma: no cover - hardware specific
                error = exc

        if error is not None:
            self._notify_error(error)

    def _make_safe_2m_freq(self, freq: str) -> float:
        try:
            value = float(freq)
        except ValueError:
            value = 146.520

        while value > 148.0:
            value /= 10.0

        value = min(148.0, max(144.0, value))
        return value

    def _notify_audio(self, data: bytes) -> None:
        LOGGER.debug("Forwarding %d bytes of audio to listeners.", len(data))
        for callback in list(self._audio_listeners):
            try:
                callback(data)
            except Exception as exc:  # pragma: no cover - defensive
                LOGGER.exception("Audio listener failed: %s", exc)

    def _notify_error(self, exc: Exception) -> None:
        for callback in list(self._error_listeners):
            try:
                callback(exc)
            except Exception:  # pragma: no cover - defensive
                LOGGER.exception("Error listener failed.")

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        serial_obj = self._serial
        return bool(serial_obj and serial_obj.is_open)

    @property
    def mode(self) -> RadioMode:
        with self._lock:
            return self._mode

    # ------------------------------------------------------------------
    # Parsing helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _extract_version(buffer: str) -> Optional[int]:
        import re

        match = re.search(r"(?:FW[_ ]?)?VER(?:SION)?\D*(\d{1,8})", buffer, re.IGNORECASE)
        if not match:
            return None
        digits = match.group(1)
        try:
            return int(digits.lstrip("0") or "0")
        except ValueError:
            return None
