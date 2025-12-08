/*
 * 简化版雷达 SDK 测试程序
 * 用于诊断雷达连接问题
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sllidar_ros2/sdk/include/sl_lidar.h"
#include "sllidar_ros2/sdk/include/sl_lidar_driver.h"

using namespace sl;
using namespace sl::hal;

#define RAD2DEG(x) ((x)*180./M_PI)

// 获取设备信息
bool getSLLIDARDeviceInfo(ILidarDriver *drv) {
    sl_result op_result;
    sl_lidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
            fprintf(stderr, "错误: 获取设备信息超时！\n");
        } else {
            fprintf(stderr, "错误: 获取设备信息失败，错误代码: %x\n", op_result);
        }
        return false;
    }

    printf("\n");
    printf("========================================\n");
    printf("雷达设备信息:\n");
    printf("========================================\n");
    printf("设备型号: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.model[15-pos]);
    }
    printf("\n");
    printf("固件版本: %d.%d\n", devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    printf("硬件版本: %d\n", devinfo.hardware_version);
    printf("序列号: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[15-pos]);
    }
    printf("\n");
    printf("========================================\n");
    return true;
}

// 检查雷达健康状态
bool checkSLLIDARHealth(ILidarDriver *drv) {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_FAIL(op_result)) {
        fprintf(stderr, "错误: 获取健康状态失败，错误代码: %x\n", op_result);
        return false;
    }

    printf("\n");
    printf("========================================\n");
    printf("雷达健康状态:\n");
    printf("========================================\n");
    printf("状态: ");
    switch (healthinfo.status) {
        case SL_LIDAR_STATUS_OK:
            printf("正常 (OK)\n");
            break;
        case SL_LIDAR_STATUS_WARNING:
            printf("警告 (WARNING)\n");
            printf("错误代码: 0x%X\n", healthinfo.error_code);
            break;
        case SL_LIDAR_STATUS_ERROR:
            printf("错误 (ERROR)\n");
            printf("错误代码: 0x%X\n", healthinfo.error_code);
            return false;
        default:
            printf("未知状态\n");
            break;
    }
    printf("========================================\n");
    return true;
}

int main(int argc, char *argv[]) {
    const char *opt_com_path = "/dev/ttyUSB0";
    sl_u32 opt_com_baudrate = 115200;

    // 解析命令行参数
    if (argc >= 2) {
        opt_com_path = argv[1];
    }
    if (argc >= 3) {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
    }

    printf("========================================\n");
    printf("SLAMTEC 雷达 SDK 连接测试程序\n");
    printf("========================================\n");
    printf("串口设备: %s\n", opt_com_path);
    printf("波特率: %d\n", opt_com_baudrate);
    printf("\n");

    // 创建驱动实例
    ILidarDriver *drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "错误: 无法创建雷达驱动实例\n");
        return -1;
    }

    // 创建串口通道
    IChannel *_channel = *createSerialPortChannel(opt_com_path, opt_com_baudrate);
    if (!_channel) {
        fprintf(stderr, "错误: 无法创建串口通道\n");
        delete drv;
        return -1;
    }

    // 连接雷达
    printf("正在连接到雷达...\n");
    sl_result op_result = drv->connect(_channel);
    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
            fprintf(stderr, "\n错误: 连接超时！\n");
            fprintf(stderr, "可能的原因:\n");
            fprintf(stderr, "  1. 串口设备路径不正确: %s\n", opt_com_path);
            fprintf(stderr, "  2. 波特率不匹配: %d\n", opt_com_baudrate);
            fprintf(stderr, "  3. 雷达未正确连接或未上电\n");
            fprintf(stderr, "  4. 串口权限不足\n");
        } else {
            fprintf(stderr, "\n错误: 连接失败，错误代码: 0x%X\n", op_result);
        }
        delete drv;
        return -1;
    }

    printf("✓ 连接成功！\n\n");

    // 获取设备信息
    if (!getSLLIDARDeviceInfo(drv)) {
        drv->disconnect();
        delete drv;
        return -1;
    }

    // 检查健康状态
    if (!checkSLLIDARHealth(drv)) {
        fprintf(stderr, "\n警告: 雷达健康状态异常，但继续测试...\n");
    }

    // 获取支持的扫描模式
    printf("\n正在获取支持的扫描模式...\n");
    std::vector<LidarScanMode> allSupportedScanModes;
    op_result = drv->getAllSupportedScanModes(allSupportedScanModes);
    
    if (SL_IS_OK(op_result)) {
        printf("✓ 找到 %zu 个支持的扫描模式:\n", allSupportedScanModes.size());
        for (size_t i = 0; i < allSupportedScanModes.size(); i++) {
            printf("  模式 %d: %s (最大距离: %.1f m, 采样率: %.1f Hz)\n",
                   allSupportedScanModes[i].id,
                   allSupportedScanModes[i].scan_mode,
                   allSupportedScanModes[i].max_distance,
                   1000000.0f / allSupportedScanModes[i].us_per_sample);
        }
    } else {
        fprintf(stderr, "✗ 获取扫描模式失败，错误代码: 0x%X\n", op_result);
    }

    printf("\n========================================\n");
    printf("测试完成！雷达连接正常。\n");
    printf("========================================\n");

    // 清理
    drv->disconnect();
    delete drv;

    return 0;
}






