/**
 * @file laser_interface.hpp
 * @brief 激光雷达接口抽象类，用于模块化传感器接口
 * @details 支持仿真和实机接口切换
 */

#ifndef SENSOR_INTERFACE__LASER_INTERFACE_HPP_
#define SENSOR_INTERFACE__LASER_INTERFACE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace sensor_interface
{

/**
 * @class LaserInterface
 * @brief 激光雷达接口抽象基类
 * 
 * 提供了统一的激光雷达数据接口，支持仿真和实机实现
 */
class LaserInterface
{
public:
  /**
   * @brief 构造函数
   */
  LaserInterface() = default;

  /**
   * @brief 虚析构函数
   */
  virtual ~LaserInterface() = default;

  /**
   * @brief 初始化接口
   * @param node ROS2节点指针
   * @return 是否初始化成功
   */
  virtual bool initialize(std::shared_ptr<rclcpp::Node> node) = 0;

  /**
   * @brief 启动数据发布
   * @return 是否启动成功
   */
  virtual bool start() = 0;

  /**
   * @brief 停止数据发布
   */
  virtual void stop() = 0;

  /**
   * @brief 检查接口是否就绪
   * @return 是否就绪
   */
  virtual bool isReady() const = 0;

  /**
   * @brief 获取激光扫描数据
   * @return 激光扫描消息指针，如果没有数据则返回nullptr
   */
  virtual sensor_msgs::msg::LaserScan::SharedPtr getScan() = 0;
};

} // namespace sensor_interface

#endif // SENSOR_INTERFACE__LASER_INTERFACE_HPP_


