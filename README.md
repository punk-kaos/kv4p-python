# KV4P Python Toolkit

This directory holds the pure-Python tooling that talks to the KV4P-HT radio firmware. It contains a reusable library that implements the serial protocol as well as a Tk-based GUI that demonstrates the end-to-end audio path.

## Contents

- `kv4p_python/` – production library with the `RadioController` class and protocol helpers.
- `radio_gui/` – desktop GUI that exercises the controller, audio capture/playback, and PTT workflow.
- `kiss_tnc/` – headless KISS TCP TNC built on top of the KV4P radio controller.
- `tests/` – `unittest`-based regression checks for the protocol layer.
- `requirements.txt` – Python dependencies, including OPUS bindings (`opuslib`) used for 48 kHz audio.

## Prerequisites

- Python 3.11+ (CPython recommended).
- Host computer with a working serial connection to the KV4P hardware.
- libopus installed on the system. macOS example:

  ```bash
  brew install opus
  ```

  If the dynamic library is not discovered automatically, export the path (macOS example):

  ```bash
  export DYLD_LIBRARY_PATH="$(brew --prefix opus)/lib:${DYLD_LIBRARY_PATH}"
  ```

## Environment Setup

From the repository root:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r python/requirements.txt
```

The requirements list includes `pyserial`, `sounddevice`, `numpy`, and `opuslib`. The controller will fall back gracefully if OPUS cannot be initialised, but audio TX/RX will not function without it.

## Quick Start

1. Connect the KV4P radio via USB and note the serial port.
2. Export the port if you want to avoid typing it (optional):

   ```bash
   export KV4P_PORT=/dev/tty.usbserial-xxxx
   ```

3. Launch the GUI:

   ```bash
   python -m radio_gui.app --debug
   ```

   Select the port, open the connection, and use the “Set PTT Key” button to bind a keyboard shortcut. Hold the key to transmit; release to return to RX.

### Headless KISS TNC

Launch the TCP KISS server (defaults to `127.0.0.1:8001`):

```bash
python -m kiss_tnc --port /dev/tty.usbserial-xxxx --tx 144.390 --squelch 0 --log-level DEBUG
```

Attach a KISS-aware client (e.g., `direwolf -lp 8001` or any APRS application). The modem implements 1200 baud AFSK with simple demodulation suited for strong signals.

### Library Example

```python
from kv4p_python import RadioController

with RadioController("COM3") as radio:
    radio.initialize()
    radio.tune_to_frequency("146.520", "146.520", tone=0, squelch_level=1)
    radio.start_rx_mode()
    # Listen for radio.add_audio_listener(...) callbacks
```

The controller sends frames using the KV4P-HT binary protocol (`0xDEADBEEF` delimiter, 48 kHz OPUS audio). Window updates from the firmware regulate the serial throughput automatically.

## Running Tests

```bash
PYTHONPATH=python python -m unittest python.tests.test_radio_controller
```

The suite validates frame construction, window accounting, and startup fallbacks without needing actual hardware.

## Development Notes

- Audio runs at 48 kHz / 40 ms frames. The GUI’s audio engine produces 16-bit PCM data that the controller encodes into OPUS before transmission.
- The controller automatically falls back to RX if the firmware banner is not received but still respects window updates once they arrive.
- Keyboard PTT in the GUI incorporates a 150 ms debounce so OS-level key repeat does not toggle the transmitter.

## Troubleshooting

- **No audio TX/RX** – verify `opuslib` is installed, the system can locate `libopus`, and the log shows “Firmware v…” followed by window updates.
- **Permission errors** – ensure the user account can access the serial device (`/dev/tty.*` on macOS, `COMx` on Windows, `/dev/ttyUSB*` on Linux).
- **Audio device discovery** – the GUI uses `sounddevice` which depends on PortAudio; install the package for your OS if devices are missing.

For protocol reference and firmware updates, see the microcontroller sources under `../kv4p-ht/microcontroller-src/`.
