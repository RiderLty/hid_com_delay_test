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

1. Open touch hidraw (non-blocking fd + `select` loop in `touchReader`) and the control link (`cmdLink`: `serialLink` | `hidLink`).
2. Toggle mapping mode ON with `~`, **verified by coordinates**: move the virtual cursor to the top-left corner with small 0xFE moves, click, and read the reported touch position. Mapping OFF → click lands at the corner sentinel `(2147483646, 0)`; mapping ON → click lands inside the user-configured (elliptical) mapping region. Probe clicks 3× and takes the majority, because late reports from a previous probe can pollute the next one.
3. Measure 100 clicks (down + up): latency = hidraw read return time − write completion time. `measure` filters stale reports: down must match `tip=1 && pressure=255` (MOVE reports are `tip=1 p=0`), up must match `tip=0`, and matches arriving <500µs after the write are discarded (USB polling makes real latency ≥ ~0.9ms). One retry per action on timeout.
4. Toggle mapping OFF, verified the same way; print avg/median/min/max stats.

## Gotchas

- When mapping is OFF the touch emulation still reports — click reports just carry the cursor-corner sentinel. When mapping is ON, the *up* report is `tip=0`, but when OFF it's `tip=1 p=0` — probe code blind-fires the release frame and drains instead of waiting for it.
- The X axis in corner reports is the descriptor max (0x7FFFFFFE), suggesting X is right-origin (or it's the firmware default); either way it never collides with a real mapping region.
- Tight iteration spacing (<5ms) congests the device input queue and drops frames — the loop sleeps 5ms between clicks.
- Chinese is the working language of user-facing output and comments; keep it that way.
