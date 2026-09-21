package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"strings"
	"time"

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
	timeoutMs := flag.Int("timeout", 2000, "单次等待报告超时 (ms)")
	verbose := flag.Bool("v", false, "打印每个触屏报告的原始内容")
	flag.Parse()

	if *iface == "serial" && *serialDev == "" {
		log.Fatalf("iface=serial 需要用 -serial 指定串口设备路径")
	}
	if *iface == "hid" && (*ctrlVID == "" || *ctrlPID == "") {
		log.Fatalf("iface=hid 需要用 -ctrl-vid/-ctrl-pid 指定控制 HID 设备")
	}
	timeout := time.Duration(*timeoutMs) * time.Millisecond

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
	// 写入一帧并等待匹配的触屏报告，超时返回 false。
	// down 匹配 tip=true 且 pressure=255 (排除 tip=true p=0 的 MOVE 报告)；
	// up 匹配 tip=false (映射开启时抬起报告的特征)。
	// 早于 500µs 的匹配是上一轮延迟到达的残留报告 (USB 轮询决定了真实延迟 ≥ ~0.9ms)，丢弃。
	measure := func(frame []byte, wantDown bool) (time.Duration, uint32, uint32, bool) {
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
			match := rpt.tip == wantDown && (!wantDown || rpt.pressure == 255)
			if match && rpt.at.Sub(t0) >= 500*time.Microsecond {
				touch.drain() // 一个动作可能产生多个报告，清掉尾巴
				return rpt.at.Sub(t0), rpt.x, rpt.y, true
			}
			// 不匹配或过快的报告 (残留) 跳过
		}
	}

	stats := func(name string, ds []time.Duration) {
		if len(ds) == 0 {
			fmt.Printf("%s: 无数据\n", name)
			return
		}
		var total, mn, mx time.Duration
		mn = ds[0]
		for _, d := range ds {
			total += d
			if d < mn {
				mn = d
			}
			if d > mx {
				mx = d
			}
		}
		sorted := append([]time.Duration(nil), ds...)
		for i := 1; i < len(sorted); i++ {
			for j := i; j > 0 && sorted[j] < sorted[j-1]; j-- {
				sorted[j], sorted[j-1] = sorted[j-1], sorted[j]
			}
		}
		fmt.Printf("%s: 平均 %.6f ms | 中位 %.6f ms | 最小 %.6f ms | 最大 %.6f ms (n=%d)\n",
			name,
			float64(total.Nanoseconds())/1e6/float64(len(ds)),
			float64(sorted[len(sorted)/2].Nanoseconds())/1e6,
			float64(mn.Nanoseconds())/1e6,
			float64(mx.Nanoseconds())/1e6,
			len(ds))
	}

	// 角点判定: 光标在左上角时触摸模拟点击的坐标 (X 右原点或固件默认值，
	// 与任何用户配置的映射区域都不会重叠)
	const cornerX, cornerY = uint32(2147483646), uint32(0)

	// 探测映射状态: 小步移动鼠标把虚拟光标移到屏幕左上角，然后点击。
	// 实测报告坐标 (用户配置椭圆区域在 x≈3.5~4亿, y≈16.8~17.2亿):
	//   映射关 → 触摸模拟鼠标，点击报告坐标 = 光标角点 (2147483646, 0)
	//   映射开 → 点击走映射配置位置，坐标落在椭圆区域内
	// 注意: 映射关时抬起报告是 tip=true p=0 (不是 tip=false)，无法用 measure 等待，
	// 所以抬起帧盲发后直接清缓冲。
	// 点击 3 次取多数: 上一轮探测迟到的孤立残留报告不会连中 3 次。
	// 返回: 映射是否开启、探测是否有效、样例坐标 (用于日志)。
	probeMapping := func() (bool, bool, uint32, uint32) {
		for i := 0; i < 40; i++ {
			link.Write(mouseFrame(0, -100, -100))
			time.Sleep(2 * time.Millisecond)
		}
		time.Sleep(100 * time.Millisecond)
		touch.drain() // 丢弃移动产生的报告

		var onVotes, offVotes int
		var lastX, lastY uint32
		for i := 0; i < 3; i++ {
			_, x, y, ok := measure(mouseFrame(0x01, 0, 0), true) // 点击
			link.Write(mouseFrame(0x00, 0, 0))                   // 盲发抬起
			time.Sleep(50 * time.Millisecond)
			touch.drain()
			if !ok {
				continue
			}
			lastX, lastY = x, y
			if x == cornerX && y == cornerY {
				offVotes++
			} else {
				onVotes++
			}
		}
		if onVotes == 0 && offVotes == 0 {
			return false, false, 0, 0
		}
		return onVotes > offVotes, true, lastX, lastY
	}

	// 切换映射模式并用坐标校验确认。设备侧状态在程序启动前不确定
	// (上次运行可能未正确关闭)，先探测再决定是否需要发 ~。
	toggleMapping := func(wantOn bool) bool {
		for attempt := 0; attempt < 3; attempt++ {
			isOn, ok, x, y := probeMapping()
			if ok {
				if isOn == wantOn {
					return true
				}
				fmt.Printf("当前映射状态: %s (探测点击坐标 %d,%d)，发送 ~ 切换...\n",
					map[bool]string{true: "开启", false: "关闭"}[isOn], x, y)
			} else {
				fmt.Println("探测无响应，发送 ~ 尝试切换...")
			}
			sendKey(KeyGrave)
			time.Sleep(300 * time.Millisecond)
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

	fmtMs := func(d time.Duration, ok bool) string {
		if !ok {
			return "    超时/失败"
		}
		return fmt.Sprintf("%10.6f ms", float64(d.Nanoseconds())/1e6)
	}
	var downLat, upLat []time.Duration
	var downMiss, upMiss int
	for i := 1; i <= *iters; i++ {
		// 单次重试: 设备偶发拥塞丢帧时补测，避免统计样本缺失
		d, _, _, okD := measure(mouseFrame(0x01, 0, 0), true) // 左键按下
		if !okD {
			d, _, _, okD = measure(mouseFrame(0x01, 0, 0), true)
		}
		u, _, _, okU := measure(mouseFrame(0x00, 0, 0), false) // 左键抬起
		if !okU {
			u, _, _, okU = measure(mouseFrame(0x00, 0, 0), false)
		}
		if !okD {
			downMiss++
		}
		if !okU {
			upMiss++
		}
		if okD {
			downLat = append(downLat, d)
		}
		if okU {
			upLat = append(upLat, u)
		}
		fmt.Printf("[%03d] 按下: %s | 抬起: %s\n", i, fmtMs(d, okD), fmtMs(u, okU))
		time.Sleep(5 * time.Millisecond) // 间隔过小会让设备输入队列拥塞丢帧
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
	stats("左键按下→触屏报告", downLat)
	stats("左键抬起→触屏报告", upLat)
	all := append(downLat, upLat...)
	stats("整体           ", all)
	if downMiss > 0 || upMiss > 0 {
		fmt.Printf("失败次数: 按下 %d, 抬起 %d (已重试)\n", downMiss, upMiss)
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
}

func openTouch(dev string) (*touchReader, error) {
	f, err := os.OpenFile(dev, os.O_RDONLY, 0)
	if err != nil {
		return nil, err
	}
	fd := int(f.Fd())
	unix.SetNonblock(fd, true) // 统一走 select + 原始 read
	return &touchReader{f: f, fd: fd}, nil
}

func (t *touchReader) close() { t.f.Close() }

// 带超时读取一个触屏报告
func (t *touchReader) read(timeout time.Duration) (touchReport, bool) {
	deadline := time.Now().Add(timeout)
	buf := make([]byte, 64)
	for {
		n, err := unix.Read(t.fd, buf)
		if n > 0 {
			if rpt, ok := parseTouchReport(buf[:n]); ok {
				if t.verbose {
					fmt.Printf("  触屏报告: % X (tip=%v id=%d p=%d x=%d y=%d n=%d)\n",
						rpt.raw, rpt.tip, rpt.id, rpt.pressure, rpt.x, rpt.y, rpt.count)
				}
				rpt.at = time.Now()
				return rpt, true
			}
			// 非触屏报告 ID，继续读
		}
		if err != nil && err != unix.EAGAIN {
			return touchReport{}, false
		}
		remain := time.Until(deadline)
		if remain <= 0 {
			return touchReport{}, false
		}
		var rfds unix.FdSet
		rfds.Set(t.fd)
		tv := unix.NsecToTimeval(remain.Nanoseconds())
		nr, serr := unix.Select(t.fd+1, &rfds, nil, nil, &tv)
		if serr != nil || nr == 0 {
			return touchReport{}, false
		}
	}
}

// 清空接收缓冲中的残留报告
func (t *touchReader) drain() {
	buf := make([]byte, 64)
	for {
		if _, err := unix.Read(t.fd, buf); err != nil {
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
