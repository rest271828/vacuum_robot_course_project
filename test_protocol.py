#!/usr/bin/env python3
"""
测试协议打包，验证发送的数据格式
"""
import struct

def pack_tx_cmd(v_l_cmd: float, v_r_cmd: float) -> bytes:
    """打包命令"""
    TX_HEAD = 0xAA
    TX_TAIL = 0x55
    return bytes([TX_HEAD]) + struct.pack(
        "<ff", float(v_l_cmd), float(v_r_cmd)
    ) + bytes([TX_TAIL])

# 测试 0.3 m/s 的情况
v = 0.3  # m/s
w = 0.0  # rad/s
L = 0.2  # wheel_base
cmd_scale = 1000.0

v_l_mps = v - w * L * 0.5
v_r_mps = v + w * L * 0.5
v_l_unit = v_l_mps * cmd_scale
v_r_unit = v_r_mps * cmd_scale

print(f"输入: v={v} m/s, w={w} rad/s")
print(f"计算: v_l={v_l_mps} m/s, v_r={v_r_mps} m/s")
print(f"转换: v_l={v_l_unit} mm/s, v_r={v_r_unit} mm/s")
print()

# 打包
frame = pack_tx_cmd(v_l_unit, v_r_unit)

print(f"数据包长度: {len(frame)} 字节")
print(f"数据包十六进制: {frame.hex()}")
print(f"数据包字节: {list(frame)}")
print()

# 详细分析
print("数据包结构分析:")
print(f"  字节 0 (Header): 0x{frame[0]:02X} ({frame[0]})")
print(f"  字节 1-4 (v_l): {frame[1:5].hex()} -> {struct.unpack('<f', frame[1:5])[0]} mm/s")
print(f"  字节 5-8 (v_r): {frame[5:9].hex()} -> {struct.unpack('<f', frame[5:9])[0]} mm/s")
print(f"  字节 9 (Tail): 0x{frame[9]:02X} ({frame[9]})")
print()

# 验证解包
unpacked = struct.unpack("<ff", frame[1:9])
print(f"解包验证: v_l={unpacked[0]} mm/s, v_r={unpacked[1]} mm/s")
print()

# 如果下位机按旧协议（float16）解析会怎样？
print("⚠️  如果下位机按旧协议（6字节，float16）解析:")
if len(frame) >= 6:
    old_v_l = struct.unpack("<e", frame[1:3])[0] if len(frame) >= 3 else None
    old_v_r = struct.unpack("<e", frame[3:5])[0] if len(frame) >= 5 else None
    print(f"  只读取前6字节: {frame[:6].hex()}")
    print(f"  字节 1-2 (v_l as float16): {frame[1:3].hex()} -> {old_v_l} mm/s")
    print(f"  字节 3-4 (v_r as float16): {frame[3:5].hex()} -> {old_v_r} mm/s")
    print(f"  字节 5 (Tail): 0x{frame[5]:02X}")
    print("  ❌ 这会导致右轮数据错误！")

