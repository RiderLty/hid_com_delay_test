package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/gorilla/websocket"
	"go.bug.st/serial"
	"golang.org/x/sys/unix"
)

// pico-hid-mapper 延迟测试工具
//
// 三条控制路径（固件侧最终都汇到 core_input_keyboard / core_input_mouse_button）：
//   1. hurra 标准（串口 4M，TinyFrame 帧）：TYPE_BTN_LEFT/RIGHT (0x20/0x21) [state]、
//      TYPE_KB_DOWN/UP (0x40/0x41) [key] —— 注入路径（input_filter_inject_*，过授权门控）
//   2. hurra 扩展 VCTRL（串口 4M，TF type 0xC0）：payload [0xFC][0xFE][btn][down] /
//      [0xFC][0xFC][key][down] —— 固件重组 55 AA 帧重放 handle_control_frame（无授权门控）
//   3. HID（PIO vendor HID OUT，55 AA 帧）：CMD 0xFD 键盘 / 0xFE 鼠标 —— hid_dispatch_*
//
// TinyFrame 线上格式: [ID:1][LEN:1][TYPE:1][头CRC16:2 BE][载荷:LEN][数据CRC16:2 BE]
// （CRC poly 0x8005 反射、初值 0；无 SOF 字节；LEN==0 帧到头 CRC 为止）
//
// 触屏报告 (Report ID 1, 13 字节):
//   [01][tip:1bit|pad:7][contact_id:u8][pressure:u8][X:u32 LE][Y:u32 LE][count:u8]

const KeyGrave = 0x35 // ~ 键，切换映射模式

// ---------- Hurra TYPE 常量（对齐 src/hurra.c 枚举） ----------

const (
	tfTypeVersion = 0x01
	tfTypeBtnLeft = 0x20 // 0x20..0x24 = 左/右/中/后退/前进
	tfTypeKbDown  = 0x40
	tfTypeKbUp    = 0x41
	tfTypeVctrl   = 0xC0 // 私有扩展：载荷 [0xFC][subcmd][args...]
	vctrlCmdCore  = 0xFC // → handle_control_frame 的 PIO_CMD_CORE_INPUT
	vctrlSubMouse = 0xFE // core_input_mouse_button(btn, down)
	vctrlSubKbd   = 0xFC // core_input_keyboard(key, down)
	hurraIdentity = "kmbox: Hurra v1"
)

func main() {
	// ---------- 命令行参数 ----------
	iface := flag.String("iface", "serial", "控制接口: serial | hid")
	proto := flag.String("proto", "hurra", "串口协议 (仅 iface=serial 有效): hurra 标准 | vctrl 扩展")
	serialDev := flag.String("serial", "", "串口设备路径 (iface=serial 必填, 如 /dev/ttyACM0)")
	baud := flag.Int("baud", 4000000, "串口波特率 (固件固定 4M, 0x05 BAUD 命令只回 ACK 不改速)")
	ctrlVID := flag.String("ctrl-vid", "", "控制 HID 设备 VID, 4 位 hex (iface=hid 必填)")
	ctrlPID := flag.String("ctrl-pid", "", "控制 HID 设备 PID, 4 位 hex (iface=hid 必填)")
	reportID := flag.Int("report-id", 0, "控制 HID 写入的报告 ID")
	targetVID := flag.String("target-vid", "0541", "触屏设备 VID, 4 位 hex")
	targetPID := flag.String("target-pid", "0ce5", "触屏设备 PID, 4 位 hex")
	iters := flag.Int("n", 100, "测试点击次数")
	gapMs := flag.Int("gap", 20, "动作间隔 (ms): 按下→抬起、点击→点击之间，过小会拥塞设备输入队列")
	gapJitterMs := flag.Int("gap-jitter-ms", 0, "动作间隔随机扰动幅度 (ms): 固定整数 ms 间隔与 USB 1ms 帧网格相位锁定，边沿延迟被钉死在离散步；扰动逐轮重掷相位，测得无锁相位的真实分布。注意扰动必须是亚毫秒粒度才有效——上一报告的到达时刻本身锁在轮询网格上，整数 ms 的扰动会被网格吸收")
	timeoutMs := flag.Int("timeout", 2000, "单次等待报告超时 (ms)")
	wsURL := flag.String("ws", "ws://192.168.73.1:80/ws", "设备 WebSocket 日志地址，用于确定映射模式状态")
	verbose := flag.Bool("v", false, "打印每个触屏报告的原始内容")
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, `hid_com_delay_test - pico-hid-mapper 控制链路延迟测试 (三路径)

用法:
  ./hid_com_delay_test -iface hid -ctrl-vid 2e8a -ctrl-pid c9d0 \
      -target-vid 035f -target-pid 0ae8 -n 100
  ./hid_com_delay_test -iface serial -proto hurra -serial /dev/serial/by-id/... \
      -target-vid 0541 -target-pid 0ce5 -n 100     # Hurra 标准命令 (0x20/0x21)
  ./hid_com_delay_test -iface serial -proto vctrl -serial /dev/serial/by-id/... \
      -target-vid 0541 -target-pid 0ce5 -n 100     # Hurra VCTRL 扩展 (0xC0)

测量: 上位机发控制指令 → 设备输出触屏 HID 报告的端到端延迟
      序列为 左按下 → 右按下 → 左松开 → 右松开, 分别统计
      串口两路径最终入口不同 (hurra=注入路径 / vctrl=控制帧重放), 可对比;
      HID 与 vctrl 引擎入口相同, 差异只在传输层 (USB OUT vs UART)。

前提 (必须):
  1. 左键与右键都已映射到触屏区域 (左=触点0, 右=触点1)
  2. 映射类型必须是「同步按下释放」——按下保持、松开释放。
     连发/单次点击/长按宏等会自动抬手的类型无法测量, 程序自检会报错退出。

参数:
`)
		flag.PrintDefaults()
	}
	flag.Parse()

	switch *iface {
	case "serial":
		if *serialDev == "" {
			log.Fatalf("iface=serial 需要用 -serial 指定串口设备路径")
		}
		if *proto != "hurra" && *proto != "vctrl" {
			log.Fatalf("未知串口协议: %s (-proto hurra | vctrl)", *proto)
		}
	case "hid":
		if *ctrlVID == "" || *ctrlPID == "" {
			log.Fatalf("iface=hid 需要用 -ctrl-vid/-ctrl-pid 指定控制 HID 设备")
		}
		if *proto != "hurra" {
			fmt.Printf("提示: -proto 仅对 iface=serial 有效, HID 路径忽略\n")
		}
	default:
		log.Fatalf("未知控制接口: %s (serial | hid)", *iface)
	}
	timeout := time.Duration(*timeoutMs) * time.Millisecond

	fmt.Println("=== pico-hid-mapper 延迟测试 ===")
	fmt.Printf("控制路径: %s\n", func() string {
		switch {
		case *iface == "hid":
			return "HID (55 AA 控制帧, PIO vendor HID OUT)"
		case *proto == "hurra":
			return "Hurra 标准 (TF 0x20/0x21 鼠标按钮, 注入路径)"
		default:
			return "Hurra VCTRL 扩展 (TF 0xC0 → 控制帧重放)"
		}
	}())
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
		sl, err := openSerial(*serialDev, *baud, *proto)
		if err != nil {
			log.Fatalf("无法打开串口 %s: %v", *serialDev, err)
		}
		// 链路自检: VERSION 应答确认对端确实是本固件 (接错口/固件未刷/波特率
		// 不符时在此给出明确诊断, 而不是测量阶段全部超时)。
		ident, err := sl.probeVersion()
		if err != nil {
			sl.Close()
			log.Fatalf("串口链路自检失败: %v\n"+
				"  可能原因: 接错串口 (应为设备 UART, 4M 波特率)、固件未刷 Hurra 版、\n"+
				"  或串口被其他进程占用", err)
		}
		fmt.Printf("控制接口: 串口 %s @ %d (协议 %s)\n", *serialDev, *baud, *proto)
		fmt.Printf("链路自检: 设备应答 VERSION: %q\n", ident)
		link = sl
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
	}
	defer link.Close()

	// ---------- WebSocket 日志监听 (用于确定映射模式状态) ----------
	ws := startWsWatcher(*wsURL)
	defer ws.Close()
	fmt.Printf("WS 日志: %s\n", *wsURL)

	// 发送一次按键 (按下 + 释放，产生完整边沿)
	sendKey := func(keycode byte) {
		link.KeyDown(keycode, true)
		time.Sleep(20 * time.Millisecond)
		link.KeyDown(keycode, false)
	}

	// ---------- 测量 ----------
	// 按下/松开一个鼠标按钮并等待指定触点的匹配报告，超时返回 false。
	// 多触点场景 (左右键映射到两个触点): 左键=触点0, 右键=触点1 (按下顺序决定 slot)。
	// down 匹配 tip=true 且 pressure=255 且触点 ID 相符；up 匹配 tip=false 且 ID 相符。
	// ID 不符的报告 (另一触点的状态同步/残留) 跳过。
	// 写入前清空缓冲: 设备每个动作会发重复报告，匹配到第一条即返回，剩余的会
	// 积压在 hidraw 环形缓冲 (仅 64 条)，积满后内核静默丢弃新报告 → 假超时。
	// 早于 500µs 的匹配是残留报告 (USB 轮询决定了真实延迟 ≥ ~0.9ms)，丢弃。
	measure := func(btn byte, wantDown bool, wantID uint8) (time.Duration, uint32, uint32, bool) {
		touch.drain()
		if err := link.MouseButton(btn, wantDown); err != nil {
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
	if d, _, _, ok := measure(0, true, 0); ok {
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
	link.MouseButton(0, false) // 收尾: 释放左键
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
	// 每步只产生一个按钮边沿。触点 ID 按按下顺序分配: 左键先按下 → 触点0，右键 → 触点1
	type stepStat struct {
		name string
		lat  []time.Duration
		miss int
	}
	steps := [4]*stepStat{
		{name: "左键按下"}, {name: "右键按下"}, {name: "左键松开"}, {name: "右键松开"},
	}
	gap := time.Duration(*gapMs) * time.Millisecond
	// 带扰动的间隔睡眠: gap ± gapJitter 均匀抖动，**µs 级粒度**。
	// 固定间隔下整条链路（报告到达→写→CH343→设备武装→IN 轮询）逐级量化到同一个
	// 1ms 网格，相位锁定；且上一报告的到达时刻本身就落在网格上，整数 ms 的扰动
	// 会被原样吸收——必须亚毫秒扰动才能真正重掷相位。
	sleepGap := func() {
		j := *gapJitterMs * 1000 // µs
		d := gap
		if j > 0 {
			d += time.Duration(rand.Int63n(int64(2*j)+1)) - time.Duration(j)
		}
		time.Sleep(d)
	}
	for i := 1; i <= *iters; i++ {
		seq := []struct {
			btn      byte
			wantDown bool
			wantID   uint8
			stat     *stepStat
		}{
			{0, true, 0, steps[0]},  // 左按下 → 触点0
			{1, true, 1, steps[1]},  // 右按下 (左保持) → 触点1
			{0, false, 0, steps[2]}, // 左松开 (右保持) → 触点0
			{1, false, 1, steps[3]}, // 右松开 → 触点1
		}
		var results [4]time.Duration
		var oks [4]bool
		for k, s := range seq {
			results[k], _, _, oks[k] = measure(s.btn, s.wantDown, s.wantID)
			if !oks[k] {
				s.stat.miss++
			} else {
				s.stat.lat = append(s.stat.lat, results[k])
			}
			if k < len(seq)-1 {
				sleepGap()
			}
		}
		fmt.Printf("[%03d] 左按: %s | 右按: %s | 左松: %s | 右松: %s\n",
			i, fmtMs(results[0], oks[0]), fmtMs(results[1], oks[1]),
			fmtMs(results[2], oks[2]), fmtMs(results[3], oks[3]))
		sleepGap()
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

// ---------- TinyFrame 编解码 (对齐 pytester/test_hurra.py) ----------

// 反射逐字节 CRC 表: crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
// 多项式 0x8005 反射 (0xA001)、初值 0、无 xorout，与 TinyFrame 内置表一致
var crcTable = func() [256]uint16 {
	var t [256]uint16
	for b := 0; b < 256; b++ {
		crc := uint16(0)
		c := byte(b)
		for i := 0; i < 8; i++ {
			if (crc^uint16(c))&1 != 0 {
				crc = (crc >> 1) ^ 0xA001
			} else {
				crc >>= 1
			}
			c >>= 1
		}
		t[b] = crc
	}
	return t
}()

func crc16(data []byte) uint16 {
	var crc uint16
	for _, b := range data {
		crc = crc>>8 ^ crcTable[(crc^uint16(b))&0xFF]
	}
	return crc
}

// 帧格式: [ID:1][LEN:1][TYPE:1][头CRC16:2 BE][载荷:LEN][数据CRC16:2 BE]
// 无 SOF 字节；LEN==0 帧到头 CRC 为止（无数据 CRC）。
func tfEncode(id, typ byte, payload []byte) []byte {
	head := []byte{id, byte(len(payload)), typ}
	hc := crc16(head)
	out := append(head, byte(hc>>8), byte(hc))
	if len(payload) > 0 {
		dc := crc16(payload)
		out = append(out, payload...)
		out = append(out, byte(dc>>8), byte(dc))
	}
	return out
}

type tfFrame struct {
	id, typ byte
	payload []byte
}

// 从字节流解 TinyFrame 帧（头 CRC 不匹配时滑窗逐字节重同步——无 SOF 字节的代价）
type tfParser struct {
	buf []byte
}

func (p *tfParser) feed(data []byte) []tfFrame {
	p.buf = append(p.buf, data...)
	var out []tfFrame
	for {
		if len(p.buf) < 5 {
			return out
		}
		tfLen := p.buf[1]
		if crc16(p.buf[:3]) != binary.BigEndian.Uint16(p.buf[3:5]) {
			p.buf = p.buf[1:]
			continue
		}
		frameLen := 5 + int(tfLen)
		if tfLen > 0 {
			frameLen += 2
		}
		if len(p.buf) < frameLen {
			return out
		}
		frame := p.buf[:frameLen]
		p.buf = p.buf[frameLen:]
		var payload []byte
		if tfLen > 0 {
			payload = frame[5 : 5+tfLen]
			if crc16(payload) != binary.BigEndian.Uint16(frame[5+tfLen:7+tfLen]) {
				continue // 载荷 CRC 错，丢弃该帧继续收
			}
		}
		out = append(out, tfFrame{
			id: frame[0], typ: frame[2],
			payload: append([]byte(nil), payload...),
		})
	}
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

// 语义化控制接口: 三条路径各自实现线格式编码，测量流程与协议解耦。
type cmdLink interface {
	// MouseButton 按下/松开一个鼠标按钮 (0..4 = 左/右/中/后退/前进)。
	MouseButton(btn byte, down bool) error
	// KeyDown 按下/松开一个键盘键 (HID keycode)。
	KeyDown(key byte, down bool) error
	Close() error
}

// ----- 串口链路 (Hurra / TinyFrame, 固定 4M) -----

type serialLink struct {
	port   serial.Port
	proto  string // "hurra" | "vctrl"
	nextID byte
}

func openSerial(dev string, baud int, proto string) (*serialLink, error) {
	p, err := serial.Open(dev, &serial.Mode{BaudRate: baud})
	if err != nil {
		return nil, err
	}
	// VERSION 自检要读应答，给 Read 一个短超时（只在自检时读，测量期只写）
	if err := p.SetReadTimeout(20 * time.Millisecond); err != nil {
		p.Close()
		return nil, err
	}
	return &serialLink{port: p, proto: proto}, nil
}

func (s *serialLink) send(tfType byte, payload []byte) error {
	s.nextID = (s.nextID + 1) & 0x7F // 对齐参考实现：ID 限 7 位
	_, err := s.port.Write(tfEncode(s.nextID, tfType, payload))
	return err
}

// VERSION 链路自检: 发 TYPE_VERSION (0x01)，等固件回 "kmbox: Hurra v1"（应答复用请求 id）
func (s *serialLink) probeVersion() (string, error) {
	var p tfParser
	buf := make([]byte, 256)
	for attempt := 0; attempt < 3; attempt++ {
		s.nextID = (s.nextID + 1) & 0x7F
		id := s.nextID
		_ = s.port.ResetInputBuffer()
		if _, err := s.port.Write(tfEncode(id, tfTypeVersion, nil)); err != nil {
			return "", err
		}
		deadline := time.Now().Add(500 * time.Millisecond)
		for time.Now().Before(deadline) {
			n, err := s.port.Read(buf)
			if err != nil {
				return "", err
			}
			for _, fr := range p.feed(buf[:n]) {
				if fr.id == id && fr.typ == tfTypeVersion {
					return string(fr.payload), nil
				}
			}
		}
	}
	return "", fmt.Errorf("3×500ms 内未收到 VERSION 应答")
}

func (s *serialLink) MouseButton(btn byte, down bool) error {
	if btn > 4 {
		return fmt.Errorf("按钮编号越界: %d (0..4)", btn)
	}
	d := byte(0)
	if down {
		d = 1
	}
	if s.proto == "vctrl" {
		// VCTRL 扩展: TF 0xC0 载荷 [0xFC][0xFE][btn][down] → 控制帧重放 → core_input_mouse_button
		return s.send(tfTypeVctrl, []byte{vctrlCmdCore, vctrlSubMouse, btn, d})
	}
	// Hurra 标准: TF 0x20..0x24 [state]（0x20=左键，编号连续）→ input_filter_inject_button
	return s.send(tfTypeBtnLeft+btn, []byte{d})
}

func (s *serialLink) KeyDown(key byte, down bool) error {
	d := byte(0)
	if down {
		d = 1
	}
	if s.proto == "vctrl" {
		// VCTRL 扩展: TF 0xC0 载荷 [0xFC][0xFC][key][down] → core_input_keyboard
		return s.send(tfTypeVctrl, []byte{vctrlCmdCore, vctrlSubKbd, key, d})
	}
	// Hurra 标准: TF 0x40 (KB_DOWN) / 0x41 (KB_UP) [key] → input_filter_inject_key
	typ := byte(tfTypeKbDown)
	if !down {
		typ = tfTypeKbUp
	}
	return s.send(typ, []byte{key})
}

func (s *serialLink) Close() error { return s.port.Close() }

// ----- HID 链路 (PIO vendor HID OUT, 55 AA 帧) -----

type hidLink struct {
	f        *os.File
	reportID byte
	btns     byte // 55 AA 0xFE 是状态型报文（按钮位图），跨调用维护
}

func openHidLink(dev string, reportID byte) (cmdLink, error) {
	f, err := os.OpenFile(dev, os.O_RDWR, 0)
	if err != nil {
		return nil, err
	}
	return &hidLink{f: f, reportID: reportID}, nil
}

// 55 AA 帧 + hidraw 报告 ID 前缀
func (h *hidLink) write(cmd byte, payload []byte) error {
	f := append([]byte{0x55, 0xAA, byte(len(payload) + 1), cmd}, payload...)
	_, err := h.f.Write(append([]byte{h.reportID}, f...))
	return err
}

func (h *hidLink) MouseButton(btn byte, down bool) error {
	if btn > 4 {
		return fmt.Errorf("按钮编号越界: %d (0..4)", btn)
	}
	bit := byte(1) << btn
	if down {
		h.btns |= bit
	} else {
		h.btns &^= bit
	}
	// CMD 0xFE 标准鼠标报文 8B: [rid][buttons][dx i16][dy i16][wheel][pan]
	return h.write(0xFE, []byte{0x00, h.btns, 0, 0, 0, 0, 0, 0})
}

func (h *hidLink) KeyDown(key byte, down bool) error {
	var keys [6]byte
	if down {
		keys[0] = key
	}
	// CMD 0xFD 标准键盘报文 8B: [rid][modifiers][reserved][keys[6]]
	return h.write(0xFD, []byte{0x00, 0x00, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]})
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
//
//	[0]=0x01 [1]=tip(bit0) [2]=contact_id [3]=pressure
//	[4:8]=X u32 LE [8:12]=Y u32 LE [12]=contact_count
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
