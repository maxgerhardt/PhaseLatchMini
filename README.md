# STM32F103 Dual-ADC IQ USB Streamer

High-rate continuous streaming of interleaved dual ADC (I/Q) samples over USB Full‑Speed (FS) using only the built‑in CDC class (no custom driver) on an STM32F103C8 ("Blue Pill" style) board. Companion Python host tools provide live visualization, FIFO bridging to SDR applications (e.g. GQRX / GNU Radio), raw capture, diagnostics, and throughput benchmarking.

> Status: Actively optimized. Current validated complex sample rate: **~166.667 kI/Q samples/sec** (TIM3 driven) with sustained USB payload throughput >500 KiB/s (≈85% of theoretical CDC 64‑byte bulk efficiency). Recent firmware changes added ISR burst chaining and DMA IRQ "kick" to reduce inter-packet idle; post‑kick benchmark re‑measurement pending.

> Origin / Intended Use: Initially developed as a lightweight streaming engine for the [PhaseLoom](https://github.com/AndersBNielsen/PhaseLoom) project, but fully usable with any dual (I/Q) analog front end producing baseband signals on two STM32F1 ADC channels. Swap or adapt the front-end conditioning (filters, biasing) and reuse the same transport & host tooling.

---
## Hardware Overview

- MCU: STM32F103C8 @ 72 MHz (HSE 8 MHz ×9 PLL)
- ADCs: ADC1 (master) + ADC2 (slave) in *dual regular simultaneous* mode
- Trigger Source: TIM3 Update → TRGO
- USB: Full-Speed (12 Mbps) CDC ACM (64‑byte bulk IN endpoint)
- Inputs: Two analog channels (default PA0 → I, PA1 → Q)
- LED: On-board user LED used for heartbeat/fallback diagnostics
- Power: Via USB (ensure stable 5V and board has 3V3 regulator in spec)

### Analog Front-End Notes
Provide anti-alias filtering / level shifting as required. Inputs should remain within 0–3.3 V. For differential or RF baseband sources, use buffering + bias network so the ADC sees midscale (~1.65V) plus small AC swing.

---
## Feature Summary

- Dual ADC synchronous sampling packed as 32-bit little-endian words (I 12-bit + Q 12-bit within 2×16 containers)
- Circular DMA with half / full transfer interrupts
- Lock-free ring queue of DMA segments feeding USB transmit path
- ISR burst chaining (up to 12 chained 64-byte packets per IN completion) to minimize USB microframe idle gaps
- Immediate DMA IRQ packet scheduling ("kick") to prime first packet of each buffer region
- Optional diagnostic/stat packets suppressed in throughput builds
- Host tools for: diagnostics, throughput, raw capture, live ASCII / sparkline IQ view, FIFO → GQRX bridge, flexible capture & replay
- Simple text command channel over CDC (START/STOP, A=ADCSTAT, F=FEED, legacy STATS)

---
## Repository Layout

```
platformio.ini              # Build environments & macro flags
src/                        # Firmware sources (USB, ADC, main logic)
Inc/ / include/             # Public headers
host_*.py                   # Python host utilities
lib/ test/                  # (Placeholders / future use)
```

Key firmware files:
- `src/main.c` – System init, main loop USB feeder (fallback scheduling & instrumentation)
- `src/iq_adc.c` / `Inc/iq_adc.h` – ADC+DMA+Timer configuration, DMA IRQ scheduling kick
- `src/usbd_conf.c` – Low-level USB callbacks, DataIn burst chaining logic
- `src/usbd_cdc_if.c` – CDC interface, command parser (A/F/STATS) & data write API
- `src/usbd_raw.c` (future / optional) – Raw class scaffolding (if transitioning from CDC for additional margin)

Host scripts (root directory):
- `host_diagnostics.py` – Passive capture, STATS delta decode, periodic monitor, USB descriptor info
- `host_throughput.py` – PyUSB bulk endpoint benchmark (raw vendor class / future); shows packet stats
- `host_raw_capture.py` – Robust raw USB (PyUSB or CDC) capture with quiet mode & BrokenPipe safety
- `host_iq_fifo.py` – Serial CDC → named FIFO adapter (u8 or cf32) for GQRX / GNU Radio
- `host_iq_live.py` – Terminal live IQ stats + sparklines / averages
- `host_test.py` – Simple START/STOP based capture, optional float32 output & basic stats
- `host_probe.py` – (Currently empty placeholder for future probing utilities)

---
## Build & Flash (PlatformIO)

Prerequisites:
- VS Code + PlatformIO extension (or `pio` CLI)

Clone & open project, then build one of the defined environments:

### PlatformIO Quickstart (Basics)

If you're new to PlatformIO, these are the minimal steps to get a firmware onto the board.

#### Option A: VS Code GUI
1. Install the "PlatformIO IDE" extension in VS Code.
2. Open this project folder (where `platformio.ini` resides).
3. Bottom status bar: pick the desired environment (e.g. `adc`).
4. Click the checkmark (Build). Wait for success.
5. Connect the board via USB (ensure it enumerates) and press the right arrow (Upload). If first flash fails, press/hold BOOT (if available) or use a serial/USB boot jumper sequence depending on your Blue Pill variant.
6. Use the plug icon (Monitor) or run a host script to interact.

#### Option B: CLI
Install PlatformIO Core (Python 3.11+ recommended):
```bash
pip install --upgrade platformio
```
From the project root:
```bash
# List environments defined in platformio.ini
pio run --list-targets

# Build the adc environment
pio run -e adc

# Upload (adjust -e if choosing another env)
pio run -e adc -t upload

# Open a serial monitor (Ctrl+C to exit)
pio device monitor -b 115200
```
If you see permission errors on macOS/Linux, you may need to adjust USB device permissions or add your user to the appropriate group (e.g. `dialout` on some Linux distros).

#### Common Upload Notes
- Some Blue Pill clones require setting/removing BOOT0 jumpers for the built-in ROM bootloader; after initial flash with a USB‑TTL adapter or ST-Link, subsequent USB CDC updates normally proceed automatically.
- For ST-Link users, ensure drivers and udev rules (Linux) are installed.
- If the device does not reset into the new firmware, press the hardware RESET button.

#### Serial Device Identification
On macOS you'll typically see `/dev/tty.usbmodem*`. On Linux, it may appear as `/dev/ttyACM0`. Use `pio device list` to enumerate.

Environments (see `platformio.ini`):
- `env:baseline` – Minimal USB baseline with reduced diagnostics (throughput focus pattern)
- `env:diag` – Heavier diagnostics enabled (ASCII stat chatter)
- `env:adc` – Active dual-ADC streaming (ENABLE_IQ + throughput suppressions)
- `env:adc_smoke` – Lower-intensity validation (extra ADC status prints)

Typical flow (using `env:adc`):
1. Select `adc` environment in PlatformIO
2. Build (should report flash usage ~30%, RAM ~55%)
3. Upload (DFU/serial depending on board configuration)
4. Open a serial monitor or run a host script (e.g. `python host_test.py --progress`)

If enumeration stalls: power cycle board or press reset. Heartbeat LED indicates main loop alive; rapid fallback pattern indicates SysTick/USB early issues.

---
## Runtime Commands (CDC)

Send as ASCII (line ending flexible, CR/LF accepted):
- `START` – Begin IQ streaming (if not already running)
- `STOP` – Halt streaming (host_test.py sends on exit)
- `A` – Print `ADCSTAT` line with ADC & DMA counters
- `F` – Print `FEED` line (USB feeder instrumentation)
- `STATS` – Legacy status snapshot (may be suppressed in throughput builds)

---
## Data Format

Each IQ sample pair is one 32-bit little-endian word composed of two 16‑bit containers produced by the STM32F1 dual regular simultaneous ADC mode:
```
Bits  0..11  : I (ADC1 12-bit result, right-aligned)
Bits 12..15  : Unused (always 0)  <-- upper 4 bits of lower 16-bit half-word
Bits 16..27  : Q (ADC2 12-bit result, right-aligned)
Bits 28..31  : Unused (always 0)  <-- upper 4 bits of upper 16-bit half-word
```

So effectively:
```
uint16_t I_lane = word & 0xFFFF;          // lower half-word
uint16_t Q_lane = (word >> 16) & 0xFFFF;  // upper half-word
uint16_t I_raw12 = I_lane & 0x0FFF;       // mask to 12 bits
uint16_t Q_raw12 = Q_lane & 0x0FFF;
```

The current `host_test.py` treats each 16-bit lane as a full-range unsigned sample and subtracts 32768. That overstates magnitude and uses the high (always zero) 4 bits. For correct 12‑bit centered scaling use:
```
float I = ((int)I_raw12 - 2048) / 2048.0f;   // ≈ [-1.0, +1.0)
float Q = ((int)Q_raw12 - 2048) / 2048.0f;
```
If you need signed 16-bit containers for downstream tooling, you can expand by left-shifting 4 (to occupy high bits) or replicate into 16-bit signed with:
```
int16_t I_s16 = ((int16_t)(I_raw12 ^ 0x800) - 0x800) << 4; // optional dynamic range padding
```
But most host paths should just mask & center at 2048.

---
## ADC / Timing Configuration

- TIM3 base clock: 1 MHz
- TIM3 ARR: 5  → update every 6 ticks → 166,667 complex samples/sec (approx.)
- Dual regular simultaneous mode ensures hardware-aligned I & Q capture
- DMA buffer: Circular array of 32-bit words (size defined in `iq_adc.h`), half/full IRQ used to enqueue segments

Potential Adjustments:
- Lower ARR for higher sample rate (watch USB bandwidth + CPU overhead)
- Consider alternate timer or prescaler to reach round-number rates (e.g. 192 kS/s)

---
## USB Transfer Strategy & Optimizations

Chronological tuning steps:
1. Eliminate high-frequency tiny diagnostic packets (switched to periodic on-demand)
2. Ensure maximum 64-byte packet utilization (no short frames during streaming)
3. Introduce ISR burst chaining in `HAL_PCD_DataInStageCallback` (initial chain limit 4 → raised to 12) to reduce host idle gaps
4. Add DMA IRQ "kick" to schedule first packet immediately as half/full buffer becomes available
5. Instrument feeder: `feed_loop_pkts`, `feed_isr_pkts`, `feed_busy_skips`, `feed_chain_max`

Remaining headroom: Full-speed theoretical payload for 64B bulk can exceed 1 MB/s; current architecture leaves margin for modest sample rate increases (target 180–200 kS/s) before exploring tighter packing or vendor-specific endpoints.

---
## Instrumentation & Monitoring

Example FEED output (command `F`):
```
FEED loop=12345 isr=67890 busy=12 chain_max=11
```
Where:
- `loop` – Packets scheduled from main loop (fallback)
- `isr` – Packets scheduled from ISR path (burst + DMA kick)
- `busy` – Attempts skipped because endpoint currently active
- `chain_max` – Highest contiguous packets chained in a single IN completion

ADCSTAT example (`A`):
```
ADCSTAT half=1024 full=1024 drops=0
```
(Exact field names may vary—check live output.)

Use `host_diagnostics.py stats` for legacy STAT delta interpretation, if enabled in build.

---
## Host Utilities

| Script | Purpose |
|--------|---------|
| `host_test.py` | Simple START capture, optional float32 output, stats, graceful Ctrl+C |
| `host_iq_live.py` | Real-time terminal IQ level display, window averages, sparkline |
| `host_iq_fifo.py` | Serial → FIFO converter (u8 or cf32) for GQRX / fifo consumers |
| `host_raw_capture.py` | Raw byte dump with quiet & BrokenPipe-safe pipeline support |
| `host_diagnostics.py` | STATS deltas, passive capture, USB descriptor info |
| `host_throughput.py` | High-rate PyUSB bulk benchmark (raw class future) |
| `host_probe.py` | Placeholder for future probing tools |

### GQRX Integration (via FIFO)
1. Run FIFO bridge:
   ```
   python host_iq_fifo.py --fifo /tmp/iq.fifo --format cf32 --prebuffer 0.25
   ```
2. In GQRX, set input to "UDP / File" or external source capable of reading named FIFO (Linux/macOS). Point to `/tmp/iq.fifo` with sample rate = `166667`.
3. Adjust gain/AGC in GQRX; confirm spectrum updates.

For unsigned 8-bit path (experimental), use `--format u8` and configure GQRX for `uint8 IQ` (if supported) or convert externally.

### Live View
```
python host_iq_live.py --port /dev/tty.usbmodemXYZ --window 512 --spark --interval 0.2
```
Shows min/max, window & running averages, and a small ASCII sparkline.

### Raw Capture Example
```
python host_raw_capture.py --out stream.bin --secs 5 --quiet
```
Then post-process with custom scripts or convert to complex floats.

---
## Performance Measurement

Quick rate check (CDC path):
```
python host_test.py --seconds 3 --progress
```
Or FEED counters before & after a timed interval to estimate packet dispatch accumulation.

PyUSB high-rate test (when using/adding raw vendor class):
```
python host_throughput.py --seconds 5 --progress
```

Interpretation:
- Bytes/sec ÷ 4 = complex samples/sec (given current 32-bit packing)
- Compare chain_max vs. chain_limit (12) to see if raising provides further benefit

---
## Troubleshooting

| Symptom | Suggestions |
|---------|-------------|
| No `/dev/tty.usbmodem*` device | Replug USB, check cable, ensure board enumerates (dmesg / system log) |
| Streaming stalls after START | Check FEED counters (isr incrementing?). Confirm no flood of diagnostics enabled. |
| Throughput lower than expected | Ensure running `env:adc` (not `diag`), verify chain_max near limit, reduce host-side latency (no heavy printing) |
| Mis-scaled IQ values | Mask to 12 bits (0x0FFF) before centering; ensure not interpreting high unused bits |
| Broken pipe in pipeline capture | Use `host_raw_capture.py --quiet` (handles SIGPIPE and flush safety) |

---
## Roadmap / Future Ideas

- Increase sample rate toward 180–200 kS/s (timer & queue tuning)
- Optional tighter 24-bit (3-byte) packing to reduce bandwidth (I12|Q12) – trade complexity
- Explore vendor RAW class endpoint for marginal gains over CDC ACM
- Lightweight CRC / sequence tags for host-side drop detection
- Optional AGC / scaling in firmware (convert to centered 16-bit signed)
- Continuous integration tests for host scripts (lint / basic functional mocks)

---
## License

(Choose and add a SPDX license header & file as appropriate, e.g. MIT or Apache-2.0.)

---
## Attribution / Notes

Developed as an incremental exploration of practical FS USB throughput & latency reduction techniques on resource-constrained MCUs while streaming synchronous dual-ADC data.

Contributions / suggestions welcome.
