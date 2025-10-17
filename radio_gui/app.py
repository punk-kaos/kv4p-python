"""Tkinter GUI for the Python KV4P radio controller."""

from __future__ import annotations

import argparse
import logging
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
from typing import Dict, Optional

import numpy as np

try:
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - surfaced when pyserial missing
    raise ImportError("pyserial is required for the Radio GUI. Install with `pip install pyserial`.") from exc

from kv4p_python import RadioController, RadioControllerError
from .audio import AudioEngine


TONE_MAPPINGS: Dict[str, int] = {
    "None": 0,
    "67.0": 1,
    "71.9": 2,
    "74.4": 3,
    "77.0": 4,
    "79.7": 5,
    "82.5": 6,
    "85.4": 7,
    "88.5": 8,
    "91.5": 9,
    "94.8": 10,
    "97.4": 11,
    "100.0": 12,
    "103.5": 13,
    "107.2": 14,
    "110.9": 15,
    "114.8": 16,
    "118.8": 17,
    "123.0": 18,
    "127.3": 19,
    "131.8": 20,
    "136.5": 21,
    "141.3": 22,
    "146.2": 23,
    "151.4": 24,
    "156.7": 25,
    "162.2": 26,
    "167.9": 27,
    "173.8": 28,
    "179.9": 29,
    "186.2": 30,
    "192.8": 31,
    "203.5": 32,
    "210.7": 33,
    "218.1": 34,
    "225.7": 35,
    "233.6": 36,
    "241.8": 37,
    "250.3": 38,
}


class RadioApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("KV4P Radio Controller")
        self.geometry("900x720")
        self.minsize(700, 640)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind_all("<KeyPress>", self.on_key_down, add=True)
        self.bind_all("<KeyRelease>", self.on_key_up, add=True)

        self.controller: Optional[RadioController] = None
        self.audio_engine = AudioEngine()
        self.is_transmitting = False
        self.waiting_for_ptt_key = False
        self.ptt_key = "space"
        self.status_queue: "queue.Queue[str]" = queue.Queue()
        self.vox_enabled = tk.BooleanVar(value=False)
        self.vox_threshold = tk.DoubleVar(value=35.0)
        self.vox_hold = tk.DoubleVar(value=600.0)
        self.vox_level = tk.DoubleVar(value=0.0)
        self._vox_enabled_flag = False
        self._vox_threshold_value = 0.35
        self._vox_hold_seconds = 0.6
        self._vox_active = False
        self._vox_last_activity = 0.0
        self._vox_level_queue: "queue.Queue[float]" = queue.Queue(maxsize=32)
        self._vox_smoothed_level = 0.0
        self._current_tx_reason = "none"
        self._tx_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=64)
        self._tx_thread: Optional[threading.Thread] = None
        self._tx_stop = threading.Event()
        self._ptt_release_timer: Optional[str] = None

        self.port_options: Dict[str, str] = {}
        self.input_devices: Dict[str, int] = {}
        self.output_devices: Dict[str, int] = {}

        self._build_ui()
        self.refresh_ports()
        self.refresh_audio_devices()
        self.toggle_controls(False)
        self.after(500, self._drain_status_queue)
        self.after(50, self._process_vox_levels)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)

        self.status_indicator = tk.Label(self, text="RX", bg="#2e7d32", fg="white", padx=10)
        self.status_indicator.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))

        connection_frame = ttk.LabelFrame(self, text="Connection")
        connection_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        connection_frame.columnconfigure(1, weight=1)

        ttk.Label(connection_frame, text="Serial Port:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection_frame, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=5, pady=5)

        self.open_button = ttk.Button(connection_frame, text="Open", command=self.open_connection)
        self.open_button.grid(row=0, column=2, padx=5, pady=5)
        self.close_button = ttk.Button(connection_frame, text="Close", command=self.close_connection)
        self.close_button.grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(connection_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=4, padx=5, pady=5)

        frequency_frame = ttk.LabelFrame(self, text="Frequency")
        frequency_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        for idx in range(6):
            frequency_frame.columnconfigure(idx, weight=1 if idx % 2 else 0)

        ttk.Label(frequency_frame, text="TX Frequency (MHz):").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.tx_freq_var = tk.StringVar(value="146.520")
        ttk.Entry(frequency_frame, textvariable=self.tx_freq_var).grid(row=0, column=1, sticky="ew", padx=5, pady=5)

        ttk.Label(frequency_frame, text="RX Frequency (MHz):").grid(row=0, column=2, sticky="w", padx=5, pady=5)
        self.rx_freq_var = tk.StringVar(value="146.520")
        ttk.Entry(frequency_frame, textvariable=self.rx_freq_var).grid(row=0, column=3, sticky="ew", padx=5, pady=5)

        ttk.Label(frequency_frame, text="Tone:").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.tone_var = tk.StringVar(value="None")
        self.tone_combo = ttk.Combobox(
            frequency_frame,
            textvariable=self.tone_var,
            values=list(TONE_MAPPINGS.keys()),
            state="readonly",
        )
        self.tone_combo.grid(row=1, column=1, sticky="ew", padx=5, pady=5)

        ttk.Label(frequency_frame, text="Squelch (0-9):").grid(row=1, column=2, sticky="w", padx=5, pady=5)
        self.squelch_var = tk.StringVar(value="1")
        self.squelch_combo = ttk.Combobox(
            frequency_frame,
            textvariable=self.squelch_var,
            values=[str(i) for i in range(10)],
            state="readonly",
        )
        self.squelch_combo.grid(row=1, column=3, sticky="ew", padx=5, pady=5)

        self.wideband_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            frequency_frame,
            text="Wideband (25 kHz)",
            variable=self.wideband_var,
        ).grid(row=0, column=4, columnspan=2, sticky="w", padx=5, pady=5)

        self.tune_button = ttk.Button(frequency_frame, text="Tune", command=self.tune_frequency)
        self.tune_button.grid(row=1, column=4, columnspan=2, padx=5, pady=5, sticky="ew")

        filters_frame = ttk.LabelFrame(self, text="Filters")
        filters_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=5)

        self.emphasis_var = tk.BooleanVar()
        self.highpass_var = tk.BooleanVar()
        self.lowpass_var = tk.BooleanVar()

        ttk.Checkbutton(filters_frame, text="Emphasis", variable=self.emphasis_var).grid(row=0, column=0, padx=5, pady=5)
        ttk.Checkbutton(filters_frame, text="High-pass", variable=self.highpass_var).grid(row=0, column=1, padx=5, pady=5)
        ttk.Checkbutton(filters_frame, text="Low-pass", variable=self.lowpass_var).grid(row=0, column=2, padx=5, pady=5)
        self.filter_button = ttk.Button(filters_frame, text="Apply", command=self.apply_filters)
        self.filter_button.grid(row=0, column=3, padx=5, pady=5)

        audio_frame = ttk.LabelFrame(self, text="Audio")
        audio_frame.grid(row=4, column=0, sticky="ew", padx=10, pady=5)
        for idx in range(4):
            audio_frame.columnconfigure(idx, weight=1 if idx % 2 else 0)

        ttk.Label(audio_frame, text="Recording device:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.record_device_var = tk.StringVar()
        self.record_combo = ttk.Combobox(audio_frame, textvariable=self.record_device_var, state="readonly")
        self.record_combo.grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        self.record_combo.bind("<<ComboboxSelected>>", lambda _: self._on_record_device_change())

        ttk.Label(audio_frame, text="Playback device:").grid(row=0, column=2, sticky="w", padx=5, pady=5)
        self.playback_device_var = tk.StringVar()
        self.playback_combo = ttk.Combobox(audio_frame, textvariable=self.playback_device_var, state="readonly")
        self.playback_combo.grid(row=0, column=3, sticky="ew", padx=5, pady=5)
        self.playback_combo.bind("<<ComboboxSelected>>", lambda _: self._on_playback_device_change())

        ttk.Label(audio_frame, text="Mic gain:").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.record_gain = tk.DoubleVar(value=50.0)
        record_scale = ttk.Scale(
            audio_frame,
            from_=0,
            to=100,
            variable=self.record_gain,
            command=lambda _: self._on_record_gain_change(),
        )
        record_scale.grid(row=1, column=1, sticky="ew", padx=5, pady=5)

        ttk.Label(audio_frame, text="Playback gain:").grid(row=1, column=2, sticky="w", padx=5, pady=5)
        self.playback_gain = tk.DoubleVar(value=50.0)
        playback_scale = ttk.Scale(
            audio_frame,
            from_=0,
            to=100,
            variable=self.playback_gain,
            command=lambda _: self._on_playback_gain_change(),
        )
        playback_scale.grid(row=1, column=3, sticky="ew", padx=5, pady=5)

        self.vox_enabled_check = ttk.Checkbutton(
            audio_frame,
            text="Enable VOX",
            variable=self.vox_enabled,
            command=self._on_vox_toggle,
        )
        self.vox_enabled_check.grid(row=2, column=0, sticky="w", padx=5, pady=5)

        ttk.Label(audio_frame, text="Threshold:").grid(row=2, column=1, sticky="w", padx=5, pady=5)
        self.vox_threshold_scale = ttk.Scale(
            audio_frame,
            from_=0,
            to=100,
            variable=self.vox_threshold,
            command=lambda _: self._on_vox_threshold_change(),
        )
        self.vox_threshold_scale.grid(row=2, column=2, sticky="ew", padx=5, pady=5)
        self.vox_threshold_value_label = ttk.Label(audio_frame, text="0%")
        self.vox_threshold_value_label.grid(row=2, column=3, sticky="w", padx=5, pady=5)

        ttk.Label(audio_frame, text="Hold (ms):").grid(row=3, column=0, sticky="w", padx=5, pady=5)
        self.vox_hold_scale = ttk.Scale(
            audio_frame,
            from_=100,
            to=2000,
            variable=self.vox_hold,
            command=lambda _: self._on_vox_hold_change(),
        )
        self.vox_hold_scale.grid(row=3, column=1, sticky="ew", padx=5, pady=5)
        self.vox_hold_value_label = ttk.Label(audio_frame, text="0 ms")
        self.vox_hold_value_label.grid(row=3, column=2, sticky="w", padx=5, pady=5)

        self.vox_level_bar = ttk.Progressbar(
            audio_frame,
            maximum=100,
            variable=self.vox_level,
        )
        self.vox_level_bar.grid(row=3, column=3, sticky="ew", padx=5, pady=5)

        self.ptt_button = ttk.Button(audio_frame, text="Set PTT Key", command=self.request_ptt_key)
        self.ptt_button.grid(row=4, column=0, columnspan=4, padx=5, pady=5, sticky="ew")

        status_frame = ttk.LabelFrame(self, text="Status")
        status_frame.grid(row=5, column=0, sticky="nsew", padx=10, pady=(5, 10))
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)

        self.status_text = ScrolledText(status_frame, height=12, state="disabled")
        self.status_text.grid(row=0, column=0, sticky="nsew")

        self._on_vox_threshold_change()
        self._on_vox_hold_change()

    # ------------------------------------------------------------------
    # Device discovery
    # ------------------------------------------------------------------
    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        self.port_options = {
            f"{info.device} ({info.description})": info.device for info in ports
        }
        values = list(self.port_options.keys())
        self.port_combo["values"] = values
        if values:
            self.port_combo.current(0)

    def refresh_audio_devices(self) -> None:
        inputs = AudioEngine.list_input_devices()
        outputs = AudioEngine.list_output_devices()

        self.input_devices = {
            f"{device['name']} (#{device['index']})": device["index"] for device in inputs
        }
        self.output_devices = {
            f"{device['name']} (#{device['index']})": device["index"] for device in outputs
        }

        self.record_combo["values"] = list(self.input_devices.keys())
        self.playback_combo["values"] = list(self.output_devices.keys())

        input_index = None
        output_index = None

        if self.input_devices:
            first = list(self.input_devices.keys())[0]
            self.record_device_var.set(first)
            input_index = self.input_devices[first]
        else:
            self.record_device_var.set("")

        if self.output_devices:
            first = list(self.output_devices.keys())[0]
            self.playback_device_var.set(first)
            output_index = self.output_devices[first]
        else:
            self.playback_device_var.set("")

        self.audio_engine.set_devices(input_index=input_index, output_index=output_index)

        self._on_record_gain_change()
        self._on_playback_gain_change()

    # ------------------------------------------------------------------
    # Controller interactions
    # ------------------------------------------------------------------
    def open_connection(self) -> None:
        if self.controller is not None:
            self.append_status("Connection already open.")
            return

        selected = self.port_var.get()
        if not selected:
            self.append_status("Select a serial port.")
            return

        port_name = self.port_options.get(selected, selected)

        try:
            self.controller = RadioController(port_name)
            self.controller.add_error_listener(self._on_controller_error)
            self.controller.add_audio_listener(self._on_audio_data)
            self.controller.open_connection()
            self.controller.initialize()
            self.controller.start_rx_mode()
            self.audio_engine.start_playback()
            if self._vox_enabled_flag:
                try:
                    self.audio_engine.start_recording(self._handle_mic_chunk)
                except Exception as exc:
                    self._vox_enabled_flag = False
                    self.vox_enabled.set(False)
                    self.append_status(f"VOX monitor failed to start: {exc}")
            self.append_status(f"Connection opened on {port_name}.")
            self.toggle_controls(True)
            self.update_status_indicator(False)
        except Exception as exc:
            if self.controller is not None:
                self.controller = None
            messagebox.showerror("Connection error", str(exc))
            self.append_status(f"Failed to open connection: {exc}")

    def close_connection(self) -> None:
        self.stop_transmission()
        if self.controller:
            try:
                self.controller.close_connection()
            finally:
                self.controller = None
        self.audio_engine.stop_recording()
        self.audio_engine.stop_playback()
        self.toggle_controls(False)
        self.update_status_indicator(False)
        self._vox_active = False
        self._vox_last_activity = 0.0
        self.vox_level.set(0.0)
        self.append_status("Connection closed.")

    def tune_frequency(self) -> None:
        controller = self.controller
        if controller is None:
            self.append_status("Open the connection before tuning.")
            return

        tone_key = self.tone_var.get()
        tone_value = TONE_MAPPINGS.get(tone_key, 0)
        squelch = int(self.squelch_var.get())
        try:
            controller.tune_to_frequency(
                self.tx_freq_var.get().strip(),
                self.rx_freq_var.get().strip(),
                tone_value,
                squelch,
                wideband=self.wideband_var.get(),
            )
            controller.start_rx_mode()
            self.append_status(
                f"Tuned TX {self.tx_freq_var.get()} / RX {self.rx_freq_var.get()}"
            )
        except (ValueError, RadioControllerError) as exc:
            messagebox.showerror("Tune error", str(exc))
            self.append_status(f"Tune failed: {exc}")

    def apply_filters(self) -> None:
        controller = self.controller
        if controller is None:
            self.append_status("Open the connection before applying filters.")
            return
        try:
            controller.set_filters(
                self.emphasis_var.get(),
                self.highpass_var.get(),
                self.lowpass_var.get(),
            )
            self.append_status("Filters updated.")
        except RadioControllerError as exc:
            messagebox.showerror("Filter error", str(exc))
            self.append_status(f"Failed to set filters: {exc}")

    # ------------------------------------------------------------------
    # Transmission control
    # ------------------------------------------------------------------
    def start_transmission(self, *, reason: str = "ptt") -> None:
        if self._ptt_release_timer is not None:
            self.after_cancel(self._ptt_release_timer)
            self._ptt_release_timer = None
        if self.controller is None:
            self.append_status("Open the connection before transmitting.")
            return
        if self.is_transmitting:
            return
        self._current_tx_reason = reason
        try:
            self.controller.start_tx_mode()
        except RadioControllerError as exc:
            self._current_tx_reason = "none"
            self.append_status(f"Failed to enter TX: {exc}")
            return

        self.audio_engine.stop_playback()
        self._tx_stop.clear()
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()
        while not self._tx_queue.empty():
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                break
        try:
            self.audio_engine.start_recording(self._handle_mic_chunk)
        except Exception as exc:
            self._tx_stop.set()
            self.append_status(f"Audio capture failed: {exc}")
            try:
                self.controller.end_tx_mode()
                self.controller.start_rx_mode()
            except RadioControllerError:
                pass
            self.audio_engine.start_playback()
            self._current_tx_reason = "none"
            return
        self.is_transmitting = True
        if reason == "vox":
            self._vox_active = True
            self._vox_last_activity = time.monotonic()
        else:
            self._vox_active = False
        self.update_status_indicator(True)
        if reason == "vox":
            self.append_status("Transmission started (VOX active).")
        else:
            self.append_status("Transmission started (hold PTT key to transmit).")

    def stop_transmission(self) -> None:
        if not self.is_transmitting:
            return
        if self._ptt_release_timer is not None:
            self.after_cancel(self._ptt_release_timer)
            self._ptt_release_timer = None
        if not self._vox_enabled_flag:
            self.audio_engine.stop_recording()
        self._tx_stop.set()
        while not self._tx_queue.empty():
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                break
        if self._tx_thread and self._tx_thread.is_alive():
            self._tx_thread.join(timeout=1.0)
        self._tx_thread = None

        if self.controller:
            try:
                self.controller.end_tx_mode()
                self.controller.start_rx_mode()
            except RadioControllerError as exc:
                self.append_status(f"Failed to exit TX cleanly: {exc}")
        self.audio_engine.start_playback()
        self.is_transmitting = False
        if self._current_tx_reason == "vox":
            self._vox_active = False
        self._current_tx_reason = "none"
        self.update_status_indicator(False)
        self.append_status("Transmission stopped.")

    def _handle_mic_chunk(self, data: bytes) -> None:
        if self._vox_enabled_flag:
            samples = np.frombuffer(data, dtype=np.int16)
            if samples.size:
                rms = float(np.sqrt(np.mean(samples.astype(np.float32) ** 2)))
                level = max(0.0, min(rms / 32768.0, 1.0))
            else:
                level = 0.0
            try:
                self._vox_level_queue.put_nowait(level)
            except queue.Full:
                pass
        if self.is_transmitting:
            self._handle_tx_chunk(data)

    def _handle_tx_chunk(self, data: bytes) -> None:
        try:
            self._tx_queue.put_nowait(data)
        except queue.Full:
            pass

    def _tx_loop(self) -> None:
        while not self._tx_stop.is_set():
            try:
                chunk = self._tx_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            controller = self.controller
            if controller is None:
                continue
            try:
                controller.send_audio_data(chunk)
            except Exception as exc:  # pragma: no cover - hardware specific
                self.append_status(f"Audio send error: {exc}")

    # ------------------------------------------------------------------
    # VOX control
    # ------------------------------------------------------------------
    def _process_vox_levels(self) -> None:
        smoothed = self._vox_smoothed_level
        updated = False
        while True:
            try:
                level = self._vox_level_queue.get_nowait()
            except queue.Empty:
                break
            updated = True
            smoothed = (0.6 * smoothed) + (0.4 * level)
            self._evaluate_vox(level)
        if not updated:
            smoothed *= 0.85
        self._vox_smoothed_level = max(0.0, min(smoothed, 1.0))
        self.vox_level.set(self._vox_smoothed_level * 100.0)
        self.after(40, self._process_vox_levels)

    def _evaluate_vox(self, level: float) -> None:
        if not self._vox_enabled_flag:
            return
        if self.controller is None:
            return
        now = time.monotonic()
        threshold = self._vox_threshold_value
        if threshold <= 0.0:
            threshold = 0.0

        if level >= threshold:
            if self.is_transmitting and self._current_tx_reason != "vox":
                return
            self._vox_last_activity = now
            if not self.is_transmitting:
                self.start_transmission(reason="vox")
                if not self.is_transmitting:
                    self._vox_active = False
                    return
            else:
                self._vox_active = self._current_tx_reason == "vox"
            return

        if self._vox_active:
            hold = self._vox_hold_seconds
            if hold <= 0.0:
                hold = 0.0
            if (now - self._vox_last_activity) >= hold:
                self._vox_active = False
                if self.is_transmitting and self._current_tx_reason == "vox":
                    self.stop_transmission()

    def _on_vox_toggle(self) -> None:
        enabled = bool(self.vox_enabled.get())
        self._vox_enabled_flag = enabled
        if enabled:
            self._vox_active = False
            self._vox_last_activity = 0.0
            try:
                self.audio_engine.start_recording(self._handle_mic_chunk)
            except Exception as exc:
                self._vox_enabled_flag = False
                self.vox_enabled.set(False)
                self.append_status(f"Failed to enable VOX: {exc}")
                return
            self.append_status("VOX enabled.")
        else:
            self._vox_active = False
            if self.is_transmitting and self._current_tx_reason == "vox":
                self.stop_transmission()
            elif not self.is_transmitting:
                self.audio_engine.stop_recording()
            self.vox_level.set(0.0)
            self.append_status("VOX disabled.")

    def _on_vox_threshold_change(self) -> None:
        value = float(self.vox_threshold.get())
        value = max(0.0, min(100.0, value))
        self._vox_threshold_value = value / 100.0
        if hasattr(self, "vox_threshold_value_label"):
            self.vox_threshold_value_label.configure(text=f"{value:.0f}%")

    def _on_vox_hold_change(self) -> None:
        value = float(self.vox_hold.get())
        value = max(100.0, min(2000.0, value))
        self._vox_hold_seconds = value / 1000.0
        if hasattr(self, "vox_hold_value_label"):
            self.vox_hold_value_label.configure(text=f"{int(value)} ms")

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------
    def on_key_down(self, event) -> None:
        key = event.keysym.lower()
        if self._ptt_release_timer is not None:
            self.after_cancel(self._ptt_release_timer)
            self._ptt_release_timer = None
        if self.waiting_for_ptt_key:
            self.ptt_key = key
            self.waiting_for_ptt_key = False
            self.append_status(f"PTT key set to: {self.ptt_key}")
            return
        if key == self.ptt_key and not self.is_transmitting:
            self.start_transmission()

    def on_key_up(self, event) -> None:
        key = event.keysym.lower()
        if not self.waiting_for_ptt_key and key == self.ptt_key and self.is_transmitting:
            if self._ptt_release_timer is not None:
                self.after_cancel(self._ptt_release_timer)
            self._ptt_release_timer = self.after(150, self._execute_ptt_release)

    def _execute_ptt_release(self) -> None:
        self._ptt_release_timer = None
        if self.is_transmitting:
            self.stop_transmission()

    def request_ptt_key(self) -> None:
        self.waiting_for_ptt_key = True
        self.append_status("Press a key to assign as PTT.")

    def _on_controller_error(self, exc: Exception) -> None:
        self.append_status(f"Controller error: {exc}")

    def _on_audio_data(self, data: bytes) -> None:
        self.audio_engine.enqueue_playback(data)

    def _on_record_device_change(self) -> None:
        selection = self.record_device_var.get()
        index = self.input_devices.get(selection)
        self.audio_engine.set_devices(
            input_index=index,
            output_index=self.output_devices.get(self.playback_device_var.get()),
        )

    def _on_playback_device_change(self) -> None:
        selection = self.playback_device_var.get()
        index = self.output_devices.get(selection)
        self.audio_engine.set_devices(
            input_index=self.input_devices.get(self.record_device_var.get()),
            output_index=index,
        )

    def _on_record_gain_change(self) -> None:
        gain = self.record_gain.get() / 50.0
        self.audio_engine.set_record_gain(gain)

    def _on_playback_gain_change(self) -> None:
        gain = self.playback_gain.get() / 50.0
        self.audio_engine.set_playback_gain(gain)

    # ------------------------------------------------------------------
    # UI helpers
    # ------------------------------------------------------------------
    def append_status(self, message: str) -> None:
        self.status_queue.put(message)

    def _drain_status_queue(self) -> None:
        flushed = False
        while True:
            try:
                message = self.status_queue.get_nowait()
            except queue.Empty:
                break
            self.status_text.configure(state="normal")
            self.status_text.insert("end", message + "\n")
            self.status_text.configure(state="disabled")
            self.status_text.yview_moveto(1.0)
            flushed = True
        if flushed:
            self.status_text.update_idletasks()
        self.after(200, self._drain_status_queue)

    def toggle_controls(self, enabled: bool) -> None:
        state = "!disabled" if enabled else "disabled"
        for widget in [
            self.close_button,
            self.tone_combo,
            self.squelch_combo,
            self.ptt_button,
            self.tune_button,
            self.filter_button,
            self.vox_enabled_check,
            self.vox_threshold_scale,
            self.vox_hold_scale,
        ]:
            widget.state([state])
        if enabled:
            self.open_button.state(["disabled"])
        else:
            self.open_button.state(["!disabled"])

    def update_status_indicator(self, transmitting: bool) -> None:
        if transmitting:
            self.status_indicator.configure(text="TX", bg="#c62828")
        else:
            self.status_indicator.configure(text="RX", bg="#2e7d32")

    def on_close(self) -> None:
        self.stop_transmission()
        if self.controller:
            self.controller.close_connection()
            self.controller = None
        self.audio_engine.stop_recording()
        self.audio_engine.stop_playback()
        self.destroy()


def run(*, debug: bool = False) -> None:
    if debug:
        logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    app = RadioApp()
    app.mainloop()


__all__ = ["RadioApp", "run"]


if __name__ == "__main__":  # pragma: no cover - module CLI entry
    parser = argparse.ArgumentParser(description="KV4P Radio GUI")
    parser.add_argument("--debug", action="store_true", help="Enable verbose logging")
    args = parser.parse_args()
    run(debug=args.debug)
