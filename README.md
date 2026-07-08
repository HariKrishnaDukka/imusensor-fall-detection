# PDM Microphone Capture — nRF52833 DK (MP34DT01-P)

Real-time audio capture firmware for the nRF52833 DK using an external
MP34DT01-P PDM MEMS microphone, built on nRF Connect SDK 2.7.0 (Zephyr RTOS).
Captured 16kHz mono PCM audio is streamed over UART to a host PC for live
waveform visualization, WAV export, and offline analysis.

## Features

- PDM microphone capture via Zephyr's `dmic` driver and `nrfx_pdm` backend
- 16kHz mono, 16-bit PCM output
- Double/multi-buffered capture (8 blocks) for glitch-free continuous streaming
- Framed UART protocol (sync bytes + length + XOR checksum) for reliable
  PC-side parsing
- Onboard LED status indicators:
  - **LED2** — heartbeat, toggles on every audio block received
  - **LED3** — lights up when signal exceeds a low amplitude threshold
  - **LED4** — lights up when signal exceeds a high amplitude threshold
- PC-side Python tools:
  - `pdm.py` — receives framed audio, verifies checksums, saves to WAV,
    and plots the full captured waveform
  - `live.py` — live scrolling oscilloscope-style view with an additional
    band-pass filtered (300Hz–3400Hz voice band) plot for real-time
    signal verification

## Hardware

- **Board:** nRF52833 DK (PCA10100)
- **Microphone:** MP34DT01-P PDM MEMS microphone (external breakout)

### Wiring

| Mic Pin | DK Pin |
|---------|--------|
| CLK     | P0.26  |
| DOUT    | P0.27  |
| SEL     | GND (selects channel/edge) |
| VDD     | 3V3    |
| GND     | GND    |

## Building and Flashing

```bash
west build -b nrf52833dk/nrf52833 --pristine
west flash
```

## Running the PC-Side Tools

Install dependencies:
```bash
pip install pyserial numpy matplotlib scipy --break-system-packages
```

Capture and save a WAV file with waveform plot:
```bash
python3 pdm.py
```

Live scrolling waveform view (raw + band-pass filtered):
```bash
python3 live.py
```

> Note: close any serial terminal (e.g. `picocom`) before running these
> scripts — only one program can hold the serial port at a time.

## Project Structure
