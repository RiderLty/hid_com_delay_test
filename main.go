package main

import (
	"fmt"
	"log"
	"os"
	"path/filepath"
	"time"

	"go.bug.st/serial"
	"golang.org/x/sys/unix"
)

const (
	BaudRate  = 2000000
	IterCount = 10
	EVIOCGRAB = 0x40044590 // Linux input exclusive grab ioctl
)

// 获取当前所有匹配的设备列表
func getDevices(pattern string) map[string]bool {
	devices := make(map[string]bool)
	files, _ := filepath.Glob(pattern)
	for _, f := range files {
		devices[f] = true
	}
	return devices
}

func main() {
	var ttyDev, inputDev string

	fmt.Println("正在死循环扫描设备...")

	// 1. 死循环检测设备插入
	fmt.Println("正在初始化设备快照，请在启动后插入目标设备...")

	// 1. 记录初始状态（白名单）
	initialTTYs := getDevices("/dev/tty*")
	initialInputs := getDevices("/dev/input/event*")

	// 2. 死循环无延迟差分检测
	fmt.Println("正在等待新设备插入...")
	for {
		if ttyDev == "" {
			current := getDevices("/dev/tty*")
			for f := range current {
				if !initialTTYs[f] { // 如果不在初始名单中，说明是新插入的
					ttyDev = f
					fmt.Printf("检测到新串口插入: %s\n", ttyDev)
					break
				}
			}
		}

		if inputDev == "" {
			current := getDevices("/dev/input/event*")
			for f := range current {
				if !initialInputs[f] {
					inputDev = f
					fmt.Printf("检测到新输入设备插入: %s\n", inputDev)
					break
				}
			}
		}

		if ttyDev != "" && inputDev != "" {
			break
		}
		// 紧凑循环，不加 Sleep
	}

	// 2. 配置串口
	mode := &serial.Mode{
		BaudRate: BaudRate,
	}
	port, err := serial.Open(ttyDev, mode)
	if err != nil {
		log.Fatalf("无法打开串口 %s: %v", ttyDev, err)
	}
	defer port.Close()

	// 3. 配置输入设备并设置为独占模式
	inputF, err := os.OpenFile(inputDev, os.O_RDONLY, 0)
	if err != nil {
		log.Fatalf("无法打开输入设备 %s: %v", inputDev, err)
	}
	defer inputF.Close()

	// 调用 ioctl 开启独占模式 (Grab)
	err = unix.IoctlSetInt(int(inputF.Fd()), EVIOCGRAB, 1)
	if err != nil {
		log.Fatalf("无法开启独占模式: %v", err)
	}
	fmt.Printf("已独占访问设备: %s\n", inputDev)

	// 4. 准备就绪，延迟 1 秒
	fmt.Println("准备就绪，1秒后开始测试...")
	time.Sleep(1 * time.Second)

	// 5. 开始循环测试
	//55 aa 0c 00 01 00 ff ff ff 3f ff ff ff 3f 00 0d
	payload := []byte{0x55, 0xaa, 0x0c, 0x00, 0x01, 0x00, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x0d}
	payload_2 := []byte{0x55, 0xaa, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c}
	for i := 1; i <= IterCount; i++ {
		// 清空之前的输入缓冲区（可选）
		// discardExistingData(inputF)

		// 写入数据
		buf := make([]byte, 64) // 足够容纳 input_event 结构体
		_, err := port.Write(payload)
		if err != nil {
			log.Printf("写入失败: %v", err)
			continue
		}
		// 记录精确时间
		startTime := time.Now()

		// 等待输入设备响应
		n, err := inputF.Read(buf)
		if err != nil {
			log.Printf("读取输入失败: %v", err)
			continue
		}

		elapsed := time.Since(startTime)

		// 格式化输出
		fmt.Printf("[%02d] 耗时: %12.6f ms | 数据(Hex): %X\n",
			i,
			float64(elapsed.Nanoseconds())/1e6,
			buf[:n])
		port.Write(payload_2)
		time.Sleep(1 * time.Millisecond)
	}

	// 释放独占模式
	unix.IoctlSetInt(int(inputF.Fd()), EVIOCGRAB, 0)
	fmt.Println("测试完成，已释放设备。")
}
