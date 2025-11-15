/**
 * @file laser_sensor_node.cpp
 * @brief 激光雷达传感器节点
 * @details 模块化设计，支持仿真和实机接口切换
 */

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_interface/laser_interface.hpp"

// 前向声明
namespace sensor_interface {
class SimLaserInterface;
class RealLaserInterface;
}

namespace sensor_interface
{

/**
 * @class LaserSensorNode
 * @brief 激光雷达传感器节点类
 * 
 * 根据配置参数选择使用仿真或实机接口
 */
class LaserSensorNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  LaserSensorNode()
    : Node("laser_sensor_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("interface_type", "sim");
    this->declare_parameter<std::string>("output_topic", "/scan");
    this->declare_parameter<double>("publish_rate", 10.0);

    // 获取参数
    std::string interface_type = this->get_parameter("interface_type").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // 创建接口实例
    // 注意：实际使用时，SimLaserInterface和RealLaserInterface需要分别实现
    // 这里简化处理，通过工厂模式或动态加载实现
    // 暂时通过参数控制是否直接转发/scan话题（仿真）或连接硬件（实机）
    RCLCPP_INFO(this->get_logger(), "接口类型: %s", interface_type.c_str());
    
    // 对于仿真模式，可以直接订阅/scan话题并转发
    // 对于实机模式，需要实现硬件接口
    if (interface_type != "sim" && interface_type != "real") {
      RCLCPP_ERROR(this->get_logger(), "未知的接口类型: %s，使用默认仿真接口", interface_type.c_str());
      interface_type = "sim";
    }
    
    // TODO: 实现接口工厂模式来创建对应实例
    // 当前版本：仿真模式直接订阅/scan话题
    if (interface_type == "sim") {
      // 在仿真模式下，如果Gazebo已经发布/scan，可以直接订阅转发
      // 否则需要实现SimLaserInterface
      RCLCPP_INFO(this->get_logger(), "使用仿真模式（直接转发/scan话题）");
    } else {
      RCLCPP_INFO(this->get_logger(), "使用实机模式（需要实现硬件接口）");
    }
    
    // 创建发布者
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic,
      rclcpp::QoS(10).best_effort()
    );

    // 临时实现：直接订阅/scan话题（适用于仿真）
    // 实机模式下需要替换为硬件接口
    std::string input_topic = (interface_type == "sim") ? "/scan" : "/scan_raw";
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      rclcpp::QoS(10).best_effort(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_pub_->publish(*msg);
      }
    );

    RCLCPP_INFO(this->get_logger(), "激光雷达传感器节点已启动，发布话题: %s", output_topic.c_str());
  }

  /**
   * @brief 析构函数
   */
  ~LaserSensorNode() = default;

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

} // namespace sensor_interface

/**
 * @brief 主函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_interface::LaserSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

