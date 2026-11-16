// C++ 对照示例 (对应 01 / 05 节的 Python 版本): 订阅 /lowstate，打印姿态和 0 号电机。
//
//   ros2 run unitree_lab_cpp lowstate_reader --ros-args -p robot:=go2 -p count:=5
//
// Go2 与 G1 的 LowState 是两个不同的类型 (unitree_go / unitree_hg)，字段名相同的部分
// (imu_state, motor_state, tick) 可以用同一个模板函数处理 —— 这正是两种机器人的
// "同与不同" 在类型系统里的样子。
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

class LowStateReader : public rclcpp::Node
{
public:
  LowStateReader() : Node("lowstate_reader")
  {
    robot_ = declare_parameter<std::string>("robot", "go2");
    count_ = declare_parameter<int>("count", 0);          // 0 = 一直打印
    every_ = declare_parameter<int>("every", 50);         // 每收到多少条打印一次
    if (robot_ == "g1") {
      sub_hg_ = create_subscription<unitree_hg::msg::LowState>(
        "/lowstate", 10, [this](unitree_hg::msg::LowState::SharedPtr m) {print(*m);});
    } else {
      sub_go_ = create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10, [this](unitree_go::msg::LowState::SharedPtr m) {print(*m);});
    }
    RCLCPP_INFO(get_logger(), "waiting for /lowstate (robot=%s)", robot_.c_str());
  }

private:
  template<typename LowStateT>
  void print(const LowStateT & m)
  {
    if (received_++ % every_ != 0) {return;}
    const auto & rpy = m.imu_state.rpy;
    std::printf(
      "tick=%u rpy=[%+.3f %+.3f %+.3f] motor0: q=%+.3f dq=%+.3f tau=%+.3f\n",
      m.tick, rpy[0], rpy[1], rpy[2], m.motor_state[0].q, m.motor_state[0].dq, m.motor_state[0].tau_est);
    std::fflush(stdout);
    if (count_ > 0 && ++printed_ >= count_) {rclcpp::shutdown();}
  }

  std::string robot_;
  int count_{0}, every_{50}, printed_{0};
  long received_{0};
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_go_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr sub_hg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowStateReader>());
  rclcpp::shutdown();
  return 0;
}
