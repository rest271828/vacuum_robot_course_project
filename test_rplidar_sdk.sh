#!/bin/bash
# 测试 RPLIDAR SDK 示例程序

SDK_DIR="/home/rest1/vacuum_robot/src/rplidar_sdk"
SERIAL_PORT=${1:-/dev/ttyUSB1}
BAUD_RATE=${2:-256000}

echo "=========================================="
echo "RPLIDAR SDK 测试程序"
echo "=========================================="
echo "串口设备: $SERIAL_PORT"
echo "波特率: $BAUD_RATE"
echo ""

# 检查串口
if [ ! -e "$SERIAL_PORT" ]; then
    echo "✗ 错误: 串口设备不存在: $SERIAL_PORT"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  未找到串口设备"
    exit 1
fi

# 检查权限
if [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; then
    echo "修复串口权限..."
    sudo chmod 666 "$SERIAL_PORT" 2>/dev/null || {
        echo "⚠ 无法自动修复权限，请手动运行:"
        echo "  sudo chmod 666 $SERIAL_PORT"
    }
fi

echo "1. 测试 ultra_simple (简单连接测试)"
echo "   按 Ctrl+C 停止"
echo ""

# ultra_simple 需要 --channel --serial 参数格式
# A1 雷达默认波特率是 115200
$SDK_DIR/output/Linux/Release/ultra_simple --channel --serial $SERIAL_PORT $BAUD_RATE

echo ""
echo "=========================================="
echo ""
echo "2. 如需测试 simple_grabber (获取设备信息)"
echo "   运行: $SDK_DIR/output/Linux/Release/simple_grabber $SERIAL_PORT"
echo ""
echo "3. 如需测试 custom_baudrate (自定义波特率)"
echo "   运行: $SDK_DIR/output/Linux/Release/custom_baudrate $SERIAL_PORT $BAUD_RATE"
echo ""

