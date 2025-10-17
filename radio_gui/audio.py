"""Audio capture and playback utilities for the Radio GUI."""

from __future__ import annotations

import queue
import threading
from typing import Callable, Optional

import numpy as np

try:
    import sounddevice as sd
except ImportError as exc:  # pragma: no cover - surfaced in runtime usage
    raise ImportError(
        "sounddevice is required for audio features. Install with `pip install sounddevice`."
    ) from exc


class AudioEngine:
    """Manages microphone capture and speaker playback."""

    def __init__(self, sample_rate: int = 48_000, frames_per_buffer: int = 1_920) -> None:
        self.sample_rate = sample_rate
        self.frames_per_buffer = frames_per_buffer
        self._playback_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=128)
        self._playback_buffer = np.empty(0, dtype=np.float32)
        self._output_stream: Optional[sd.OutputStream] = None
        self._input_stream: Optional[sd.InputStream] = None
        self._playback_device: Optional[int] = None
        self._record_device: Optional[int] = None
        self._record_gain = 1.0
        self._playback_gain = 1.0
        self._lock = threading.RLock()
        self._on_chunk: Optional[Callable[[bytes], None]] = None

    # ------------------------------------------------------------------
    # Device enumeration
    # ------------------------------------------------------------------
    @staticmethod
    def list_input_devices() -> list[dict]:
        devices = sd.query_devices()
        return [
            {"index": idx, **device}
            for idx, device in enumerate(devices)
            if device.get("max_input_channels", 0) > 0
        ]

    @staticmethod
    def list_output_devices() -> list[dict]:
        devices = sd.query_devices()
        return [
            {"index": idx, **device}
            for idx, device in enumerate(devices)
            if device.get("max_output_channels", 0) > 0
        ]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def set_devices(self, *, input_index: Optional[int], output_index: Optional[int]) -> None:
        with self._lock:
            self._record_device = input_index
            self._playback_device = output_index

    def set_record_gain(self, value: float) -> None:
        with self._lock:
            self._record_gain = max(0.0, min(2.0, value))

    def set_playback_gain(self, value: float) -> None:
        with self._lock:
            self._playback_gain = max(0.0, min(2.0, value))

    def start_playback(self) -> None:
        with self._lock:
            if self._output_stream is not None:
                return

            self._output_stream = sd.OutputStream(
                samplerate=self.sample_rate,
                channels=1,
                blocksize=self.frames_per_buffer,
                dtype="float32",
                device=self._playback_device,
                callback=self._output_callback,
            )
            self._output_stream.start()

    def stop_playback(self) -> None:
        with self._lock:
            stream = self._output_stream
            self._output_stream = None
        if stream is not None:
            stream.stop()
            stream.close()

    def start_recording(self, on_chunk: Callable[[bytes], None]) -> None:
        with self._lock:
            if self._input_stream is not None:
                return
            self._on_chunk = on_chunk
            self._input_stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                blocksize=self.frames_per_buffer,
                dtype="float32",
                device=self._record_device,
                callback=self._input_callback,
            )
            self._input_stream.start()

    def stop_recording(self) -> None:
        with self._lock:
            stream = self._input_stream
            self._input_stream = None
            self._on_chunk = None
        if stream is not None:
            stream.stop()
            stream.close()

    def enqueue_playback(self, data: bytes) -> None:
        samples = self._pcm_to_float(data)
        try:
            self._playback_queue.put_nowait(samples)
        except queue.Full:  # pragma: no cover - defensive backpressure
            # Drop oldest data to stay real-time
            try:
                _ = self._playback_queue.get_nowait()
            except queue.Empty:
                pass
            self._playback_queue.put_nowait(samples)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _input_callback(self, indata, frames, time_info, status) -> None:  # pragma: no cover - runtime
        if status:
            # sounddevice uses Status flags; we ignore but could log
            pass
        on_chunk = self._on_chunk
        if on_chunk is None:
            return
        with self._lock:
            gain = self._record_gain
        pcm = self._float_to_pcm(indata[:, 0], gain)
        on_chunk(pcm)

    def _output_callback(self, outdata, frames, time_info, status) -> None:  # pragma: no cover - runtime
        if status:
            pass
        with self._lock:
            gain = self._playback_gain

        required = frames
        buffer = self._playback_buffer

        while buffer.shape[0] < required:
            try:
                next_samples = self._playback_queue.get_nowait()
            except queue.Empty:
                break
            buffer = np.concatenate((buffer, next_samples), axis=0)

        if buffer.shape[0] >= required:
            frame = buffer[:required]
            self._playback_buffer = buffer[required:]
        else:
            frame = np.zeros(required, dtype=np.float32)
            self._playback_buffer = np.empty(0, dtype=np.float32)

        outdata[:, 0] = frame * gain

    # ------------------------------------------------------------------
    # Conversion helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _float_to_pcm(samples: np.ndarray, gain: float) -> bytes:
        scaled = np.clip(samples * gain, -1.0, 1.0)
        ints = (scaled * 32767.0).astype(np.int16)
        return ints.tobytes()

    @staticmethod
    def _pcm_to_float(buffer: bytes) -> np.ndarray:
        ints = np.frombuffer(buffer, dtype=np.int16).astype(np.float32)
        return ints / 32768.0
