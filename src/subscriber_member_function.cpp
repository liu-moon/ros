#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// 创建类MinimalSubscriber，继承自rclcpp::Node
class MinimalSubscriber : public rclcpp::Node
{
  public:
    // 构造函数，初始化节点名，名称为minimal_subscriber
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      // 创建一个订阅者，订阅std_msgs::msg::String类型的消息，名称为topic，队列长度为10，回调函数为topic_callback
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    // 订阅者回调函数
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      // 打印消息内容到控制台
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    // 定义一个订阅者subscription_
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);
  // 创建MinimalSubscriber类的实例
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // 关闭ROS2节点
  rclcpp::shutdown();
  return 0;
}