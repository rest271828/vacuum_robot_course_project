/**
 * @file sim_laser_interface.cpp
 * @brief 仿真环境下的激光雷达接口实现
 * @details 从Gazebo发布的/scan话题订阅数据
 */

#include "sensor_interface/laser_interface.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sensor_interface
{

/**
 * @class SimLaserInterface
 * @brief 仿真激光雷达接口实现类
 * 
 * 从Gazebo的激光雷达话题订阅数据
 */
class SimLaserInterface final : public LaserInterface
{
public:
  SimLaserInterface()
    : node_(nullptr), 
      scan_sub_(nullptr),
      ready_(false),
      latest_scan_(nullptr)
  {
  }

  ~SimLaserInterface() override = default;

  /**
   * @brief 初始化接口
   */
  bool initialize(std::shared_ptr<rclcpp::Node> node) override
  {
    node_ = node;
    
    // 订阅Gazebo发布的激光扫描话题
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::QoS(10).best_effort(),
      std::bind(&SimLaserInterface::scanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "仿真激光雷达接口初始化完成，订阅话题: /scan");
    return true;
  }

  /**
   * @brief 启动数据发布
   */
  bool start() override
  {
    ready_ = true;
    RCLCPP_INFO(node_->get_logger(), "仿真激光雷达接口已启动");
    return true;
  }

  /**
   * @brief 停止数据发布
   */
  void stop() override
  {
    ready_ = false;
    RCLCPP_INFO(node_->get_logger(), "仿真激光雷达接口已停止");
  }

  /**
   * @brief 检查接口是否就绪
   */
  bool isReady() const override
  {
    return ready_ && (latest_scan_ != nullptr);
  }

  /**
   * @brief 获取激光扫描数据
   */
  sensor_msgs::msg::LaserScan::SharedPtr getScan() override
  {
    return latest_scan_;
  }

private:
  /**
   * @brief 激光扫描数据回调函数
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  bool ready_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

} // namespace sensor_interface

