# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-file Go CLI (`main.go`) that measures the end-to-end latency of a [pico-hid-mapper](../pico-hid-mapper-doc) device (RP2350 firmware): it sends a control command over one of three control paths, and times how long until the device emits the resulting touch HID report. Linux-only (hidraw, raw fd reads); the target is a Pi connected to the device's USB ports. 本仓库是 pico-hid-mapper 的 submodule（`tools/hid_com_delay_test`），协议与固件强耦合，固件改协议时同一提交里 bump 指针。

Two links are involved:

- **Control link** (sends commands): a USB serial port (`-iface serial`, e.g. CH343 CDC-ACM `/dev/ttyACM*`, fixed 4M baud) or the device's PIO HID interface (`-iface hid`, a vendor-defined 64-byte report with no report ID — hidraw write = `[0x00 report-id byte][frame]`).
- **Touch output** (receives the reports being timed): the device's touch-screen HID interface, read directly via hidraw (`/dev/hidraw*`), NOT evdev. Report ID 1, 13 bytes: `[01][tip:bit0][contact_id:u8][pressure:u8][X:u32 LE][Y:u32 LE][count:u8]`.

Both VID:PID pairs are configurable because the firmware can change them. Typical invocations:

```bash
./hid_com_delay_test -iface hid -ctrl-vid 2e8a -ctrl-pid c9d0 -report-id 0 \
    -target-vid 035f -target-pid 0ae8 -n 100
./hid_com_delay_test -iface serial -proto hurra -serial /dev/ttyACM0 \
    -target-vid 0541 -target-pid 0ce5 -n 100    # -proto vctrl 测 VCTRL 扩展
```

Other flags: `-proto` (serial only: `hurra` default | `vctrl`), `-baud` (default 4000000, firmware-fixed), `-timeout` (per-read ms), `-v` (print raw reports). hidraw access requires a udev rule (installed at `/etc/udev/rules.d/99-pico-hid-mapper.rules`, grants plugdev group for VIDs 035f and 2e8a) — update it when VIDs change.

## Commands

```bash
go build -o hid_com_delay_test .   # build
go vet ./...                       # lint
go test ./...                      # TinyFrame 编解码对拍测试 (基线 = pytester/test_hurra.py)
```

Running requires the hardware connected. The device also exposes WebSocket logs at `ws://192.168.73.1:80/ws` (text frames) — useful for verifying device-side behavior; `pip3 install --break-system-packages --user websocket-client` + a small script tails them.

## Wire protocol — three control paths

三条路径最终都汇到 `core_input_keyboard / core_input_mouse_button`，测量流程共用；区别在引擎入口：

1. **hurra 标准**（串口 4M，注入路径 `input_filter_inject_*`，过授权门控）：TinyFrame 帧
   `[ID:1][LEN:1][TYPE:1][头CRC16:2 BE][载荷][数据CRC16:2 BE]`，CRC poly 0x8005 反射、初值 0、
   CRC 字段大端，无 SOF 字节，`LEN==0` 帧到头 CRC 为止。鼠标按钮 TYPE `0x20..0x24`（左/右/中/
   后退/前进）载荷 `[state]`；键盘 `0x40/0x41`（KB_DOWN/UP）载荷 `[key]`。
2. **hurra 扩展 VCTRL**（串口 4M，无授权门控）：TF type `0xC0`，载荷 `[0xFC][0xFE][btn][down]`
   （→ `core_input_mouse_button`）或 `[0xFC][0xFC][key][down]`（→ `core_input_keyboard`）——
   固件重组 55 AA 帧重放 `handle_control_frame`。
3. **HID**（PIO vendor HID OUT，`hid_dispatch_*`）：55 AA 帧 `[0x55][0xAA][LEN:u8][CMD:u8][payload]`，
   `LEN = 1 + len(payload)`。CMD `0xFD` 键盘报告（`[rid][mod][rsv][keys[6]]`，发 `~`/KEY_GRAVE 切映射）、
   CMD `0xFE` 标准鼠标报告（`[rid][buttons][dx i16][dy i16][wheel][pan]`，按钮是位图状态）。
   旧的串口 55 AA 协议已随固件 2026-09-27「Hurra 单协议」改造删除。

编码权威参照：`pytester/test_hurra.py`（父仓库）的 `crc16/encode_frame/TFParser`；
`tf_test.go` 里存了它的对拍基线，改编码先在 Python 侧重跑再更新测试。

串口链路启动时做 **VERSION 自检**（TF `0x01`，等应答 `"kmbox: Hurra v1"`，应答复用请求 id）；
`hidLink` 无自检（55 AA 无应答帧）。

## Test flow (in `main()`)

1. Open touch hidraw and the control link (`cmdLink` 语义接口：`MouseButton(btn, down)` /
   `KeyDown(key, down)`，三实现各自编码线格式；HID 实现跨调用维护按钮位图)；串口路径先 VERSION 自检。
2. Toggle mapping mode with `~`, state confirmed via device WS logs: the firmware prints `map on/off (switch key 53)` after the switch, and `key event with map off (53,1)` first (old-state log, must be ignored — only `...(switch...)` lines are authoritative).
3. **Mapping-type self-check**: press left, hold 400ms without sending release. If a `tip=0` report arrives spontaneously, the mapping is an auto-release kind (rapid-fire / tap / hold-macro) and the tool aborts with an explanatory error — the test requires hold-type ("同步按下释放") mappings for both buttons.
4. Measure `-n` iterations of the alternating pattern: left down → right down → left up → right up (每步恰好一个按键边沿). The two buttons map to two touch contacts: left = contact id 0, right = contact id 1 (slot order = press order). Latency = hidraw read time − write completion time, per step, with contact-id-specific matching (down: `tip=1 && pressure=255 && id` matches; up: `tip=0 && id` matches). Matches <500µs after write are discarded as stale.
5. Toggle mapping OFF, print per-step stats (mean/median/stddev/min/P90/P99/max + 0.1ms histogram) plus overall.

## Gotchas

- **Both buttons must be mapped as hold-type («同步按下释放»)** — the touch contact follows the mouse button state. Rapid-fire, single-tap, or hold-macro mappings auto-release the contact and make the up-edge unmeasurable; the startup self-check rejects them.
- **The touch hidraw MUST be read continuously** by a dedicated goroutine (`touchReader.readLoop` → channel), never only inside measurement windows. The kernel hidraw ring buffer per reader is 64 reports; the device sends duplicate reports per action, and reading on-demand lets leftovers accumulate until the buffer silently drops *new* reports — manifests as ~10% phantom timeouts. A standalone experiment (continuous reader) proved the device emits 100% of reports.
- Do not send mouse *moves* while mapping is ON to "reset the cursor": they trigger the view-drag mapping (`[view] start`), and the idle auto-release (`[view] auto release`) emits tip=0 reports that pollute measurements. Coordinate-based state detection was replaced by WS-log detection for exactly this reason.
- When mapping is OFF the touch emulation still reports clicks (at the virtual cursor position); when ON, the up report is `tip=0`, when OFF it's `tip=1 p=0`.
- `time.Sleep` between actions (`-gap`, default 20ms) separates the four steps; too-small gaps congest the device input queue.
- 55 AA 0xFE 报文是**位图状态**而非边沿事件——`hidLink` 必须跨 `MouseButton` 调用维护按钮位图；
  串口路径的 TinyFrame 按钮命令本身就是边沿语义，直接发。
- Chinese is the working language of user-facing output and comments; keep it that way.
