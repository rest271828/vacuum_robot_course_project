#!/bin/bash
# 修复串口权限和连接问题的脚本

SERIAL_PORT=${1:-/dev/ttyUSB0}

echo "=========================================="
echo "串口连接问题修复脚本"
echo "=========================================="
echo "串口: $SERIAL_PORT"
echo ""

# 1. 检查设备是否存在
if [ ! -e "$SERIAL_PORT" ]; then
    echo "✗ 错误: 串口设备不存在: $SERIAL_PORT"
    echo "可用的串口设备:"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  未找到串口设备"
    exit 1
fi

echo "1. 检查当前权限..."
ls -l "$SERIAL_PORT"
echo ""

# 2. 检查是否有进程占用
echo "2. 检查是否有进程占用串口..."
if lsof "$SERIAL_PORT" 2>/dev/null; then
    echo "⚠ 发现进程占用串口，正在停止..."
    pkill -f rplidar
    pkill -f sllidar
    sleep 1
else
    echo "✓ 没有进程占用串口"
fi
echo ""

# 3. 修复权限
echo "3. 修复串口权限..."
echo "请运行以下命令（需要 sudo 权限）："
echo ""
echo "sudo chmod 666 $SERIAL_PORT"
echo ""
read -p "是否已手动修复权限？(y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "请先运行: sudo chmod 666 $SERIAL_PORT"
    exit 1
fi

# 4. 验证权限
echo ""
echo "4. 验证权限..."
CURRENT_PERM=$(stat -c "%a" "$SERIAL_PORT" 2>/dev/null || stat -f "%OLp" "$SERIAL_PORT" 2>/dev/null)
if [ "$CURRENT_PERM" = "666" ] || [ "$CURRENT_PERM" = "664" ] || [ "$CURRENT_PERM" = "660" ]; then
    echo "✓ 权限: $CURRENT_PERM"
else
    echo "⚠ 权限: $CURRENT_PERM (建议设置为 666)"
fi
echo ""

# 5. 测试连接
echo "5. 测试串口连接..."
SDK_DIR="/home/rest1/vacuum_robot/src/rplidar_sdk"
if [ -f "$SDK_DIR/output/Linux/Release/ultra_simple" ]; then
    echo "运行 SDK 测试程序..."
    echo "按 Ctrl+C 停止"
    echo ""
    $SDK_DIR/output/Linux/Release/ultra_simple --channel --serial "$SERIAL_PORT" 115200
else
    echo "⚠ SDK 程序未找到，跳过测试"
fi




