package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/gorilla/websocket"
	"go.bug.st/serial"
	"golang.org/x/sys/unix"
)

// pico-hid-mapper 延迟测试工具
// 协议 (../pico-hid-mapper-doc/api/hid-api.md):
//   命令帧: [0x55][0xAA][LEN:u8][CMD:u8][payload...]，LEN = 1 + payload 长度，多字节小端
// 触屏报告 (Report ID 1, 13 字节):
//   [01][tip:1bit|pad:7][contact_id:u8][pressure:u8][X:u32 LE][Y:u32 LE][count:u8]

const KeyGrave = 0x35 // ~ 键，切换映射模式

func main() {
	// ---------- 命令行参数 ----------
	iface := flag.String("iface", "serial", "控制接口: serial | hid")
	serialDev := flag.String("serial", "", "串口设备路径 (iface=serial 必填, 如 /dev/ttyACM0)")
	baud := flag.Int("baud", 921600, "串口波特率")
	ctrlVID := flag.String("ctrl-vid", "", "控制 HID 设备 VID, 4 位 hex (iface=hid 必填)")
	ctrlPID := flag.String("ctrl-pid", "", "控制 HID 设备 PID, 4 位 hex (iface=hid 必填)")
	reportID := flag.Int("report-id", 0, "控制 HID 写入的报告 ID")
	targetVID := flag.String("target-vid", "0541", "触屏设备 VID, 4 位 hex")
	targetPID := flag.String("target-pid", "0ce5", "触屏设备 PID, 4 位 hex")
	iters := flag.Int("n", 100, "测试点击次数")
	gapMs := flag.Int("gap", 20, "动作间隔 (ms): 按下→抬起、点击→点击之间，过小会拥塞设备输入队列")
	timeoutMs := flag.Int("timeout", 2000, "单次等待报告超时 (ms)")
	wsURL := flag.String("ws", "ws://192.168.73.1:80/ws", "设备 WebSocket 日志地址，用于确定映射模式状态")
	verbose := flag.Bool("v", false, "打印每个触屏报告的原始内容")
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, `hid_com_delay_test - pico-hid-mapper 控制链路延迟测试

用法:
  ./hid_com_delay_test -iface hid -ctrl-vid 2e8a -ctrl-pid c9d0 \
      -target-vid 035f -target-pid 0ae8 -n 100
  ./hid_com_delay_test -iface serial -serial /dev/ttyACM0 -baud 921600 \
      -target-vid 0541 -target-pid 0ce5 -n 100

测量: 上位机写命令帧 → 设备输出触屏 HID 报告的端到端延迟
      序列为 左按下 → 右按下 → 左松开 → 右松开, 分别统计

前提 (必须):
  1. 左键与右键都已映射到触屏区域 (左=触点0, 右=触点1)
  2. 映射类型必须是「同步按下释放」——按下保持、松开释放。
     连发/单次点击/长按宏等会自动抬手的类型无法测量, 程序自检会报错退出。

参数:
`)
		flag.PrintDefaults()
	}
	flag.Parse()

	if *iface == "serial" && *serialDev == "" {
		log.Fatalf("iface=serial 需要用 -serial 指定串口设备路径")
	}
	if *iface == "hid" && (*ctrlVID == "" || *ctrlPID == "") {
		log.Fatalf("iface=hid 需要用 -ctrl-vid/-ctrl-pid 指定控制 HID 设备")
	}
	timeout := time.Duration(*timeoutMs) * time.Millisecond

	fmt.Println("=== pico-hid-mapper 延迟测试 ===")
	fmt.Println("前提: 左键/右键已映射到触屏区域, 且映射类型为「同步按下释放」")
	fmt.Println("      (连发/单次点击等自动释放类型会被自检拒绝)")
	fmt.Println()

	// ---------- 打开触屏设备 (hidraw, 中断 IN 报文) ----------
	touchDev := findHidrawByUsbId(*targetVID, *targetPID)
	if touchDev == "" {
		log.Fatalf("未找到触屏设备 %s:%s (按 sysfs hidraw HID_ID 匹配)", *targetVID, *targetPID)
	}
	touch, err := openTouch(touchDev)
	if err != nil {
		log.Fatalf("无法打开触屏设备 %s: %v", touchDev, err)
	}
	defer touch.close()
	fmt.Printf("触屏设备: %s (%s:%s)\n", touchDev, *targetVID, *targetPID)
	touch.verbose = *verbose

	// ---------- 打开控制链路 ----------
	var link cmdLink
	switch *iface {
	case "serial":
		link, err = openSerial(*serialDev, *baud)
		if err != nil {
			log.Fatalf("无法打开串口 %s: %v", *serialDev, err)
		}
		fmt.Printf("控制接口: 串口 %s @ %d\n", *serialDev, *baud)
	case "hid":
		ctrlDev := findHidrawByUsbId(*ctrlVID, *ctrlPID)
		if ctrlDev == "" {
			log.Fatalf("未找到控制 HID 设备 %s:%s", *ctrlVID, *ctrlPID)
		}
		link, err = openHidLink(ctrlDev, byte(*reportID))
		if err != nil {
			log.Fatalf("无法打开控制 HID 设备 %s: %v", ctrlDev, err)
		}
		fmt.Printf("控制接口: HID %s (report id %d)\n", ctrlDev, *reportID)
	default:
		log.Fatalf("未知控制接口: %s (serial | hid)", *iface)
	}
	defer link.Close()

	// ---------- WebSocket 日志监听 (用于确定映射模式状态) ----------
	ws := startWsWatcher(*wsURL)
	defer ws.Close()
	fmt.Printf("WS 日志: %s\n", *wsURL)

	// ---------- 协议帧构造 ----------
	// 标准键盘报告 (CMD 0xFD): [modifiers][reserved][keys[6]]
	keyboardFrame := func(keys ...byte) []byte {
		f := []byte{0x55, 0xAA, 0x09, 0xFD, 0x00, 0x00}
		for i := 0; i < 6; i++ {
			if i < len(keys) {
				f = append(f, keys[i])
			} else {
				f = append(f, 0)
			}
		}
		return f
	}
	// 标准鼠标报告 (CMD 0xFE): [report_id][buttons][x:i16][y:i16][wheel][reserved]
	mouseFrame := func(buttons byte, dx, dy int16) []byte {
		f := []byte{0x55, 0xAA, 0x09, 0xFE, 0x00, buttons}
		f = binary.LittleEndian.AppendUint16(f, uint16(dx))
		f = binary.LittleEndian.AppendUint16(f, uint16(dy))
		return append(f, 0, 0) // wheel, reserved
	}

	// 发送一次按键 (按下 + 释放，产生完整边沿)
	sendKey := func(keycode byte) {
		link.Write(keyboardFrame(keycode))
		time.Sleep(20 * time.Millisecond)
		link.Write(keyboardFrame())
	}

	// ---------- 测量 ----------
	// 写入一帧并等待指定触点的匹配报告，超时返回 false。
	// 多触点场景 (左右键映射到两个触点): 左键=触点0, 右键=触点1 (按下顺序决定 slot)。
	// down 匹配 tip=true 且 pressure=255 且触点 ID 相符；up 匹配 tip=false 且 ID 相符。
	// ID 不符的报告 (另一触点的状态同步/残留) 跳过。
	// 写入前清空缓冲: 设备每个动作会发重复报告，匹配到第一条即返回，剩余的会
	// 积压在 hidraw 环形缓冲 (仅 64 条)，积满后内核静默丢弃新报告 → 假超时。
	// 早于 500µs 的匹配是残留报告 (USB 轮询决定了真实延迟 ≥ ~0.9ms)，丢弃。
	measure := func(frame []byte, wantDown bool, wantID uint8) (time.Duration, uint32, uint32, bool) {
		touch.drain()
		if err := link.Write(frame); err != nil {
			log.Printf("写入失败: %v", err)
			return 0, 0, 0, false
		}
		t0 := time.Now()
		deadline := t0.Add(timeout)
		for {
			rpt, ok := touch.read(time.Until(deadline))
			if !ok {
				return 0, 0, 0, false
			}
			match := rpt.tip == wantDown && rpt.id == wantID && (!wantDown || rpt.pressure == 255)
			if match && rpt.at.Sub(t0) >= 500*time.Microsecond {
				return rpt.at.Sub(t0), rpt.x, rpt.y, true
			}
			if match {
				log.Printf("匹配但被残留过滤 (<500µs): id=%d tip=%v 延迟=%.3fms",
					rpt.id, rpt.tip, float64(rpt.at.Sub(t0).Nanoseconds())/1e6)
			}
			// 不匹配或过快的报告 (其他触点的残留/状态同步) 跳过
		}
	}

	// 排序副本
	sortedCopy := func(ds []time.Duration) []time.Duration {
		s := append([]time.Duration(nil), ds...)
		for i := 1; i < len(s); i++ {
			for j := i; j > 0 && s[j] < s[j-1]; j-- {
				s[j], s[j-1] = s[j-1], s[j]
			}
		}
		return s
	}
	// 百分位 (0-100)，ds 需已排序
	percentile := func(sorted []time.Duration, p float64) time.Duration {
		idx := int(p / 100 * float64(len(sorted)-1))
		return sorted[idx]
	}

	// 标准差
	stddevOf := func(ds []time.Duration, mean time.Duration) float64 {
		if len(ds) < 2 {
			return 0
		}
		var sumSq float64
		for _, d := range ds {
			diff := float64((d - mean).Nanoseconds()) / 1e6
			sumSq += diff * diff
		}
		return math.Sqrt(sumSq / float64(len(ds)-1))
	}

	// 详细统计: 平均/中位/标准差/百分位/直方图
	stats := func(name string, ds []time.Duration) {
		if len(ds) == 0 {
			fmt.Printf("%s: 无数据\n", name)
			return
		}
		sorted := sortedCopy(ds)
		var total time.Duration
		for _, d := range ds {
			total += d
		}
		mean := total / time.Duration(len(ds))
		ms := func(d time.Duration) float64 { return float64(d.Nanoseconds()) / 1e6 }
		fmt.Printf("%s: n=%d\n", name, len(ds))
		fmt.Printf("  平均 %8.4f | 中位 %8.4f | 标准差 %8.4f\n",
			ms(mean), ms(percentile(sorted, 50)), stddevOf(ds, mean))
		fmt.Printf("  最小 %8.4f | P90 %8.4f | P99 %8.4f | 最大 %8.4f\n",
			ms(sorted[0]), ms(percentile(sorted, 90)), ms(percentile(sorted, 99)), ms(sorted[len(sorted)-1]))
		// 直方图: 以 0.1ms 为桶
		fmt.Printf("  直方图 (0.1ms 桶):\n")
		lo := int(ms(sorted[0]) * 10)
		hi := int(ms(sorted[len(sorted)-1]) * 10)
		if hi == lo {
			hi = lo + 1
		}
		const maxBar = 40
		buckets := make([]int, hi-lo+1)
		for _, d := range ds {
			b := int(ms(d)*10) - lo
			if b < 0 {
				b = 0
			}
			if b >= len(buckets) {
				b = len(buckets) - 1
			}
			buckets[b]++
		}
		peak := 0
		for _, c := range buckets {
			if c > peak {
				peak = c
			}
		}
		for i, c := range buckets {
			if c == 0 {
				continue
			}
			w := c * maxBar / peak
			fmt.Printf("    %5.1f-%5.1f ms | %3d | %s\n",
				float64(lo+i)/10, float64(lo+i+1)/10, c, strings.Repeat("█", w))
		}
	}

	// 探测/切换映射模式: 发送 ~ 并监听设备 WS 日志。
	// 固件行为: 映射开时按 ~ → "map off"; 映射关时按 ~ → "map on";
	// 另外 ~ 在映射关时还会打 "key event with map off" (无切换)。
	// 所以发一个 ~ 看日志即可确定当前状态，不对再补发一个。
	// (不再用点击坐标判定——映射关且光标不在角落时的点击坐标与映射开无法区分，
	//  且探测移动会在映射开时触发视角拖拽 auto release，污染测量。)
	toggleMapping := func(wantOn bool) bool {
		for attempt := 0; attempt < 3; attempt++ {
			ws.drain()
			sendKey(KeyGrave)
			state, ok := ws.waitMapState(2 * time.Second)
			if !ok {
				fmt.Println("未收到映射状态日志 (WS 断连?)，重试...")
				continue
			}
			if state == wantOn {
				fmt.Printf("映射模式已确认: %s\n", map[bool]string{true: "开启", false: "关闭"}[state])
				return true
			}
			// 状态反了 (刚才的 ~ 把状态切过去了)，再发一次切回来
			fmt.Printf("映射状态为 %s，再发 ~ 切换...\n", map[bool]string{true: "开启", false: "关闭"}[state])
			ws.drain()
			sendKey(KeyGrave)
			state, ok = ws.waitMapState(2 * time.Second)
			if ok && state == wantOn {
				fmt.Printf("映射模式已确认: %s\n", map[bool]string{true: "开启", false: "关闭"}[state])
				return true
			}
		}
		return false
	}

	// ---------- 测试流程 ----------
	fmt.Println("准备就绪，1秒后开始测试...")
	time.Sleep(1 * time.Second)
	touch.drain()

	fmt.Println("开启映射模式...")
	touch.drain()
	if !toggleMapping(true) {
		log.Fatalf("无法确认映射模式已开启 (探测点击无响应，请检查映射配置)")
	}
	fmt.Println("映射模式已开启")
	time.Sleep(300 * time.Millisecond) // 等设备侧状态稳定
	touch.drain()

	// ---------- 映射类型自检 ----------
	// 前提: 左键/右键必须映射为「同步按下释放」(触点跟随按键状态保持按下)，
	// 不能是连发/单次点击/长按宏等会自动释放的类型——否则测不到「按下→报告」与
	// 「松开→报告」两个独立边沿 (松开时设备早已自动抬手)。
	// 自检: 按下左键后不发松开帧，静置 400ms 观察是否自发出现抬起报告。
	fmt.Println("映射类型自检 (按下左键后静置 400ms，检查是否有自发释放)...")
	touch.drain()
	if d, _, _, ok := measure(mouseFrame(0x01, 0, 0), true, 0); ok {
		fmt.Printf("  左键按下 → 报告 %.3f ms，保持中...\n", float64(d.Nanoseconds())/1e6)
	} else {
		log.Fatalf("左键按下 2s 内未收到触屏报告——请检查左键是否已映射到触屏区域")
	}
	// 静置期间消费所有报告，出现 tip=0 即为自发释放
	spontaneous := false
	hold := time.Now().Add(400 * time.Millisecond)
	for time.Now().Before(hold) {
		rpt, ok := touch.read(time.Until(hold))
		if !ok {
			break
		}
		if !rpt.tip {
			spontaneous = true
			break
		}
	}
	link.Write(mouseFrame(0x00, 0, 0)) // 收尾: 释放左键
	touch.read(time.Second)
	touch.drain()
	if spontaneous {
		log.Fatalf("检测到自发释放报告: 左键映射类型不是「同步按下释放」" +
			"(疑似连发/单次点击/长按等会自动抬手的类型)。\n" +
			"  请在 WebUI 映射配置中改为按下/释放跟随鼠标按键状态的类型后重试，\n" +
			"  否则无法测量按下与松开两个边沿的延迟。")
	}
	fmt.Println("  自检通过: 按下保持不释放 ✓")
	time.Sleep(100 * time.Millisecond)
	touch.drain()

	fmtMs := func(d time.Duration, ok bool) string {
		if !ok {
			return "    超时/失败"
		}
		return fmt.Sprintf("%10.6f ms", float64(d.Nanoseconds())/1e6)
	}

	// 左右交替测试: 左按下 → 右按下 → 左松开 → 右松开。
	// buttons 为位掩码状态，每步只产生一个按钮边沿:
	//   0x01 (左按下) → 0x03 (加右按下) → 0x02 (左松开) → 0x00 (右松开)
	// 触点 ID 按按下顺序分配: 左键先按下 → 触点0，右键 → 触点1
	type stepStat struct {
		name string
		lat  []time.Duration
		miss int
	}
	steps := [4]*stepStat{
		{name: "左键按下"}, {name: "右键按下"}, {name: "左键松开"}, {name: "右键松开"},
	}
	gap := time.Duration(*gapMs) * time.Millisecond
	for i := 1; i <= *iters; i++ {
		seq := []struct {
			buttons  byte
			wantDown bool
			wantID   uint8
			stat     *stepStat
		}{
			{0x01, true, 0, steps[0]},  // 左按下 → 触点0
			{0x03, true, 1, steps[1]},  // 右按下 (左保持) → 触点1
			{0x02, false, 0, steps[2]}, // 左松开 (右保持) → 触点0
			{0x00, false, 1, steps[3]}, // 右松开 → 触点1
		}
		var results [4]time.Duration
		var oks [4]bool
		for k, s := range seq {
			results[k], _, _, oks[k] = measure(mouseFrame(s.buttons, 0, 0), s.wantDown, s.wantID)
			if !oks[k] {
				s.stat.miss++
			} else {
				s.stat.lat = append(s.stat.lat, results[k])
			}
			if k < len(seq)-1 {
				time.Sleep(gap)
			}
		}
		fmt.Printf("[%03d] 左按: %s | 右按: %s | 左松: %s | 右松: %s\n",
			i, fmtMs(results[0], oks[0]), fmtMs(results[1], oks[1]),
			fmtMs(results[2], oks[2]), fmtMs(results[3], oks[3]))
		time.Sleep(gap)
	}

	fmt.Println("关闭映射模式...")
	time.Sleep(300 * time.Millisecond) // 等设备侧输入队列排空，避免探测点击被拥塞延迟
	touch.drain()
	if !toggleMapping(false) {
		log.Printf("警告: 无法确认映射模式已关闭")
	} else {
		fmt.Println("映射模式已关闭")
	}

	fmt.Println("\n===== 统计 =====")
	var all []time.Duration
	totalMiss := 0
	for _, s := range steps {
		stats(s.name+"→触屏报告", s.lat)
		all = append(all, s.lat...)
		totalMiss += s.miss
	}
	stats("整体             ", all)
	if totalMiss > 0 {
		fmt.Printf("失败次数: %d\n", totalMiss)
	}
	fmt.Println("测试完成。")
}

// ---------- 设备查找 ----------

// 按 USB VID:PID 查找 /dev/hidraw* (匹配 uevent 中的 HID_ID=bbbb:vvvvvvvv:pppppppp)
func findHidrawByUsbId(vid, pid string) string {
	uevents, _ := filepath.Glob("/sys/class/hidraw/hidraw*/device/uevent")
	for _, u := range uevents {
		data, err := os.ReadFile(u)
		if err != nil {
			continue
		}
		for _, line := range strings.Split(string(data), "\n") {
			// HID_ID=0003:00000541:00000CE5
			if !strings.HasPrefix(line, "HID_ID=") {
				continue
			}
			fields := strings.Split(strings.TrimPrefix(line, "HID_ID="), ":")
			if len(fields) == 3 &&
				strings.EqualFold(strings.TrimLeft(fields[1], "0"), strings.TrimLeft(vid, "0")) &&
				strings.EqualFold(strings.TrimLeft(fields[2], "0"), strings.TrimLeft(pid, "0")) {
				// u = .../hidrawN/device/uevent
				hidraw := filepath.Base(filepath.Dir(filepath.Dir(u)))
				return "/dev/" + hidraw
			}
		}
	}
	return ""
}

// ---------- 控制链路 ----------

type cmdLink interface {
	Write([]byte) error
	Close() error
}

type serialLink struct {
	port serial.Port
}

func openSerial(dev string, baud int) (cmdLink, error) {
	p, err := serial.Open(dev, &serial.Mode{BaudRate: baud})
	if err != nil {
		return nil, err
	}
	return &serialLink{port: p}, nil
}

func (s *serialLink) Write(b []byte) error {
	_, err := s.port.Write(b)
	return err
}
func (s *serialLink) Close() error         { return s.port.Close() }

type hidLink struct {
	f        *os.File
	reportID byte
}

func openHidLink(dev string, reportID byte) (cmdLink, error) {
	f, err := os.OpenFile(dev, os.O_RDWR, 0)
	if err != nil {
		return nil, err
	}
	return &hidLink{f: f, reportID: reportID}, nil
}

// hidraw 写入: 首字节为报告 ID，其后为报告数据
func (h *hidLink) Write(b []byte) error {
	buf := append([]byte{h.reportID}, b...)
	_, err := h.f.Write(buf)
	return err
}

func (h *hidLink) Close() error { return h.f.Close() }

// ---------- WebSocket 日志监听 ----------

// wsWatcher 连接设备 WebSocket 日志通道，把日志行送入 channel。
// 固件在 ~ 按下时输出 "map on (switch key 53)" / "map off (switch key 53)"，
// 用来可靠地确定映射模式状态 (映射已关时按 ~ 输出 "key event with map off"，
// 只有真的切换才输出 map on/off)。
type wsWatcher struct {
	url    string
	logs   chan string
	done   chan struct{}
	closed chan struct{}
}

func startWsWatcher(url string) *wsWatcher {
	w := &wsWatcher{
		url:    url,
		logs:   make(chan string, 256),
		done:   make(chan struct{}),
		closed: make(chan struct{}),
	}
	go w.run()
	return w
}

func (w *wsWatcher) run() {
	defer close(w.closed)
	// 自动重连，日志通道断了不影响测试 (只是拿不到状态)
	for {
		select {
		case <-w.done:
			return
		default:
		}
		c, _, err := websocket.DefaultDialer.Dial(w.url, nil)
		if err != nil {
			select {
			case <-w.done:
				return
			case <-time.After(2 * time.Second):
			}
			continue
		}
		for {
			_, msg, err := c.ReadMessage()
			if err != nil {
				c.Close()
				break
			}
			select {
			case w.logs <- string(msg):
			case <-w.done:
				c.Close()
				return
			default: // channel 满则丢弃旧日志
				select {
				case <-w.logs:
				default:
				}
				w.logs <- string(msg)
			}
		}
	}
}

func (w *wsWatcher) Close() {
	close(w.done)
	<-w.closed
}

// 清空积压日志
func (w *wsWatcher) drain() {
	for {
		select {
		case <-w.logs:
		default:
			return
		}
	}
}

// 等待映射切换日志。固件在 ~ 按下时先打 "key event with map off (53,1)" (旧状态)，
// 真正切换后才打 "map on (switch key 53)" / "map off (switch key 53)"。
// 所以忽略 key event 行，只认 switch 行; 收到后再等 150ms 吸收同批日志，取最后一条。
func (w *wsWatcher) waitMapState(timeout time.Duration) (bool, bool) {
	deadline := time.After(timeout)
	var state, got bool
	for {
		select {
		case <-deadline:
			return state, got
		case msg := <-w.logs:
			if strings.Contains(msg, "map on (switch") {
				state, got = true, true
			} else if strings.Contains(msg, "map off (switch") {
				state, got = false, true
			}
			if got {
				// 再等 150ms 看是否有同批的后续 switch 行 (不应有，保守处理)
				late := time.After(150 * time.Millisecond)
				for {
					select {
					case <-late:
						return state, true
					case msg := <-w.logs:
						if strings.Contains(msg, "map on (switch") {
							state = true
						} else if strings.Contains(msg, "map off (switch") {
							state = false
						}
					}
				}
			}
		}
	}
}

// ---------- 触屏报告读取 (hidraw) ----------

// 触屏报告布局 (Report ID 1, 13 字节):
//   [0]=0x01 [1]=tip(bit0) [2]=contact_id [3]=pressure
//   [4:8]=X u32 LE [8:12]=Y u32 LE [12]=contact_count
const touchReportLen = 13

type touchReport struct {
	tip      bool
	id       uint8
	pressure uint8
	x, y     uint32
	count    uint8
	raw      []byte
	at       time.Time // read 返回时刻
}

type touchReader struct {
	f       *os.File
	fd      int
	verbose bool
	ch      chan touchReport // 专用读取 goroutine 持续投递
	done    chan struct{}
}

func openTouch(dev string) (*touchReader, error) {
	f, err := os.OpenFile(dev, os.O_RDONLY, 0)
	if err != nil {
		return nil, err
	}
	fd := int(f.Fd())
	unix.SetNonblock(fd, true) // 统一走 select + 原始 read
	t := &touchReader{f: f, fd: fd, ch: make(chan touchReport, 1024), done: make(chan struct{})}
	go t.readLoop()
	return t, nil
}

func (t *touchReader) close() {
	close(t.done)
	t.f.Close()
}

// 专用读取 goroutine: 持续读取 hidraw 报告并投递到 channel。
// 不能只在测量窗口内读——报告会在 hidraw 环形缓冲 (64 条) 中积压，
// 设备发的重复报告将其填满后内核会静默丢弃新报告。
func (t *touchReader) readLoop() {
	buf := make([]byte, 64)
	for {
		select {
		case <-t.done:
			return
		default:
		}
		n, err := unix.Read(t.fd, buf)
		if n > 0 {
			if rpt, ok := parseTouchReport(buf[:n]); ok {
				if t.verbose {
					fmt.Printf("  触屏报告: % X (tip=%v id=%d p=%d x=%d y=%d n=%d)\n",
						rpt.raw, rpt.tip, rpt.id, rpt.pressure, rpt.x, rpt.y, rpt.count)
				}
				rpt.at = time.Now()
				select {
				case t.ch <- rpt:
				case <-t.done:
					return
				default: // channel 满则丢弃最旧报告
					select {
					case <-t.ch:
					default:
					}
					t.ch <- rpt
				}
			}
			continue
		}
		if err != nil && err != unix.EAGAIN {
			return
		}
		var rfds unix.FdSet
		rfds.Set(t.fd)
		tv := unix.NsecToTimeval(int64(50 * time.Millisecond))
		_, serr := unix.Select(t.fd+1, &rfds, nil, nil, &tv)
		if serr != nil && serr != unix.EINTR {
			return
		}
	}
}

// 带超时读取一个触屏报告 (从 channel 消费)
func (t *touchReader) read(timeout time.Duration) (touchReport, bool) {
	select {
	case rpt := <-t.ch:
		return rpt, true
	case <-time.After(timeout):
		return touchReport{}, false
	}
}

// 清空积压的报告
func (t *touchReader) drain() {
	for {
		select {
		case <-t.ch:
		default:
			return
		}
	}
}

func parseTouchReport(b []byte) (touchReport, bool) {
	if len(b) < touchReportLen || b[0] != 0x01 {
		return touchReport{}, false
	}
	return touchReport{
		tip:      b[1]&0x01 == 1,
		id:       b[2],
		pressure: b[3],
		x:        binary.LittleEndian.Uint32(b[4:8]),
		y:        binary.LittleEndian.Uint32(b[8:12]),
		count:    b[12],
		raw:      append([]byte(nil), b...),
	}, true
}
