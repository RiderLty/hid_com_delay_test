# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-file Go CLI (`main.go`) that measures the end-to-end latency of a [pico-hid-mapper](../pico-hid-mapper-doc) device (RP2350 firmware): it sends a mouse-click command frame over a control link, and times how long until the device emits the resulting touch HID report. Linux-only (hidraw, raw fd reads); the target is a Pi connected to the device's USB ports.

Two links are involved, both located by USB VID:PID via sysfs (`findHidrawByUsbId`):

- **Control link** (sends command frames): either a USB serial port (`-iface serial`, e.g. CH343 CDC-ACM `/dev/ttyACM*`) or the device's PIO HID interface (`-iface hid`, a vendor-defined 64-byte report with no report ID — hidraw write = `[0x00 report-id byte][frame]`).
- **Touch output** (receives the reports being timed): the device's touch-screen HID interface, read directly via hidraw (`/dev/hidraw*`), NOT evdev. Report ID 1, 13 bytes: `[01][tip:bit0][contact_id:u8][pressure:u8][X:u32 LE][Y:u32 LE][count:u8]`.

Both VID:PID pairs are configurable because the firmware can change them. Typical invocation (dual-HID setup):

```bash
./hid_com_delay_test -iface hid -ctrl-vid 2e8a -ctrl-pid c9d0 -report-id 0 \
    -target-vid 035f -target-pid 0ae8 -n 100
```

Serial alternative: `-iface serial -serial /dev/ttyACM0 -baud 921600`. Other flags: `-timeout` (per-read ms), `-v` (print raw reports). hidraw access requires a udev rule (installed at `/etc/udev/rules.d/99-pico-hid-mapper.rules`, grants plugdev group for VIDs 035f and 2e8a) — update it when VIDs change.

## Commands

```bash
go build -o hid_com_delay_test .   # build
go vet ./...                       # lint (no other linter is configured)
```

No tests. Running requires the hardware connected. The device also exposes WebSocket logs at `ws://192.168.73.1:80/ws` (text frames) — useful for verifying device-side behavior; `pip3 install --break-system-packages --user websocket-client` + a small script tails them.

## Wire protocol

pico-hid-mapper HIDAPI frames (see `../pico-hid-mapper-doc/api/hid-api.md`): `[0x55][0xAA][LEN:u8][CMD:u8][payload...]`, `LEN = 1 + len(payload)`, little-endian. The test uses CMD `0xFD` (keyboard report, to send the `~`/KEY_GRAVE toggle key) and CMD `0xFE` (standard 8-byte mouse report, for clicks). The device silently discards invalid frames — no error reply.

## Test flow (in `main()`)

1. Open touch hidraw and the control link (`cmdLink`: `serialLink` | `hidLink`); start a WebSocket log watcher (`wsWatcher`, `-ws` flag).
2. Toggle mapping mode with `~`, state confirmed via device WS logs: the firmware prints `map on/off (switch key 53)` after the switch, and `key event with map off (53,1)` first (old-state log, must be ignored — only `...(switch...)` lines are authoritative).
3. **Mapping-type self-check**: press left, hold 400ms without sending release. If a `tip=0` report arrives spontaneously, the mapping is an auto-release kind (rapid-fire / tap / hold-macro) and the tool aborts with an explanatory error — the test requires hold-type ("同步按下释放") mappings for both buttons.
4. Measure `-n` iterations of the alternating pattern: left down → right down → left up → right up. Buttons are a bitmask (`0x01 → 0x03 → 0x02 → 0x00`), each step exactly one button edge. The two buttons map to two touch contacts: left = contact id 0, right = contact id 1 (slot order = press order). Latency = hidraw read time − write completion time, per step, with contact-id-specific matching (down: `tip=1 && pressure=255 && id` matches; up: `tip=0 && id` matches). Matches <500µs after write are discarded as stale.
5. Toggle mapping OFF, print per-step stats (mean/median/stddev/min/P90/P99/max + 0.1ms histogram) plus overall.

## Gotchas

- **Both buttons must be mapped as hold-type («同步按下释放»)** — the touch contact follows the mouse button state. Rapid-fire, single-tap, or hold-macro mappings auto-release the contact and make the up-edge unmeasurable; the startup self-check rejects them.
- **The touch hidraw MUST be read continuously** by a dedicated goroutine (`touchReader.readLoop` → channel), never only inside measurement windows. The kernel hidraw ring buffer per reader is 64 reports; the device sends duplicate reports per action, and reading on-demand lets leftovers accumulate until the buffer silently drops *new* reports — manifests as ~10% phantom timeouts. A standalone experiment (continuous reader) proved the device emits 100% of reports.
- Do not send mouse *moves* while mapping is ON to "reset the cursor": they trigger the view-drag mapping (`[view] start`), and the idle auto-release (`[view] auto release`) emits tip=0 reports that pollute measurements. Coordinate-based state detection was replaced by WS-log detection for exactly this reason.
- When mapping is OFF the touch emulation still reports clicks (at the virtual cursor position); when ON, the up report is `tip=0`, when OFF it's `tip=1 p=0`.
- `time.Sleep` between actions (`-gap`, default 20ms) separates the four steps; too-small gaps congest the device input queue.
- Chinese is the working language of user-facing output and comments; keep it that way.
