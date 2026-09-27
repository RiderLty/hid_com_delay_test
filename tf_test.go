package main

import (
	"encoding/hex"
	"testing"
)

// 期望值由参考实现 pytester/test_hurra.py 的 encode_frame/crc16 生成（对拍基线），
// 改 TinyFrame 编码时先在 Python 侧重跑再更新这里：
//
//	from test_hurra import encode_frame, crc16
//	encode_frame(0x11, 0x20, bytes([1])).hex()  ...
func TestTFEncode(t *testing.T) {
	cases := []struct {
		id, typ byte
		payload []byte
		want    string // hex
	}{
		{0x11, 0x01, nil, "110001c591"},                                                     // VERSION 空载荷: 到头 CRC 为止
		{0x11, 0x20, []byte{1}, "1101204d5001c0c1"},                                         // BTN_LEFT 按下
		{0x7f, 0xc0, []byte{0xfc, 0xfe, 0x00, 0x01}, "7f04c04833fcfe00016090"},              // VCTRL 左键按下
		{0x01, 0x40, []byte{0x35}, "010140a0513517c0"},                                      // KB_DOWN ~
		{0x33, 0x01, []byte(hurraIdentity), "330f013f346b6d626f783a2048757272612076314186"}, // VERSION 应答
	}
	for _, c := range cases {
		got := hex.EncodeToString(tfEncode(c.id, c.typ, c.payload))
		if got != c.want {
			t.Errorf("tfEncode(%#x, %#x, % x) = %s, want %s", c.id, c.typ, c.payload, got, c.want)
		}
	}
}

// parser: 喂入拼接/分片的编码帧应完整解出；头 CRC 错应滑窗重同步；载荷 CRC 错应丢帧
func TestTFParser(t *testing.T) {
	f1 := tfEncode(0x11, tfTypeVersion, nil)
	f2 := tfEncode(0x12, tfTypeBtnLeft, []byte{1})
	var p tfParser

	// 分片喂入
	var got []tfFrame
	got = append(got, p.feed(f1[:3])...)
	got = append(got, p.feed(f1[3:])...)
	got = append(got, p.feed(append(f2, 0xAA))...) // 尾部粘一个噪声字节
	if len(got) != 2 || got[0].typ != tfTypeVersion || got[1].typ != tfTypeBtnLeft {
		t.Fatalf("分帧解析失败: %+v", got)
	}

	// 坏头 CRC 前缀 + 好帧: 应逐字节滑窗后解出好帧。
	// (注意不能用 0x00 填噪声: [00 00 00 00 00] 本身就是合法空帧)
	noise := []byte{0xDE, 0xAD, 0xBE, 0xEF, 0x42}
	n := len(p.feed(append(noise, f2...)))
	if n != 1 {
		t.Fatalf("滑窗重同步失败: 解出 %d 帧", n)
	}

	// 坏载荷 CRC: 整帧丢弃
	bad := append([]byte(nil), f2...)
	bad[len(bad)-1] ^= 0xFF
	if len(p.feed(bad)) != 0 {
		t.Fatal("坏载荷 CRC 未被丢弃")
	}
}
