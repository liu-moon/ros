// 引入标准C++库
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>

// ros2的头文件
#include "rclcpp/rclcpp.hpp"

// 用于发布的消息类型
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

// 创建类MinimalPublisher，继承自rclcpp::Node
class MinimalPublisher : public rclcpp::Node
{
public:
  // 构造函数，初始化节点名，名称为minimal_publisher，初始化计数器count_为0
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    // 创建一个发布者，发布std_msgs::msg::String类型的消息，名称为topic，队列长度为10
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // 创建一个定时器，定时500ms，回调函数为timer_callback
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  // 定时器回调函数
  void timer_callback()
  {
    // 创建一个std_msgs::msg::String类型的消息
    auto message = std_msgs::msg::String();
    // 消息内容为"Hello, world! " + count_的值
    message.data = "Hello, world! " + std::to_string(count_++);
    // 打印消息内容到控制台
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // 发布消息
    publisher_->publish(message);
  }
  // 定义一个定时器timer_
  rclcpp::TimerBase::SharedPtr timer_;
  // 定义一个发布者publisher_
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // 定义一个计数器count_
  size_t count_;
};



int usb_camera()
{
  // 打开默认摄像头
    cv::VideoCapture capture(0);

    // 检查摄像头是否成功打开
    if (!capture.isOpened()) {
        std::cerr << "ERROR: 摄像头无法打开" << std::endl;
        return 1;
    }

    cv::Mat frame;
    while (true) {
        // 从摄像头读取一帧
        capture >> frame;

        // 检查帧是否为空
        if (frame.empty()) {
            std::cerr << "ERROR: 捕获到空帧" << std::endl;
            break;
        }

        // 显示帧
        cv::imshow("摄像头实时画面", frame);

        double fps = capture.get(cv::CAP_PROP_FPS);
        std::cout << "摄像头的帧率: " << fps << " FPS" << std::endl;


        // 按 'q' 退出循环
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // 释放摄像头资源
    capture.release();
    cv::destroyAllWindows();
    return 0;
}

int main(int argc, char * argv[])
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);
  // 创建MinimalPublisher类的实例
  // rclcpp::spin()会阻塞程序，直到ROS2节点关闭
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  usb_camera();
  

  // 关闭ROS2节点
  rclcpp::shutdown();
  return 0;
}
