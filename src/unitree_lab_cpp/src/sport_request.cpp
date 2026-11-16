// C++ 对照示例 (对应 05 / 08 节): 不用 SDK，直接往 /api/sport/request 发一条 Request，
// 等 /api/sport/response 里 id 相同的 Response。这就是 SportClient 在底下做的事。
//
//   ros2 run unitree_lab_cpp sport_request --ros-args -p api_id:=1004      # Go2 StandUp
//
// 安全: 默认 dry_run=true，只打印将要发送的内容。真的发送需要 -p dry_run:=false
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sport_request");
  const auto service = node->declare_parameter<std::string>("service", "sport");
  const auto api_id = node->declare_parameter<int>("api_id", 1004);
  const auto parameter = node->declare_parameter<std::string>("parameter", "{}");
  const auto dry_run = node->declare_parameter<bool>("dry_run", true);

  unitree_api::msg::Request req;
  req.header.identity.id = node->now().nanoseconds();   // 任意唯一数，用来配对应答
  req.header.identity.api_id = api_id;
  req.parameter = parameter;

  if (dry_run) {
    std::printf("[dry_run] would publish to /api/%s/request: api_id=%ld parameter=%s\n",
      service.c_str(), static_cast<long>(api_id), parameter.c_str());
    rclcpp::shutdown();
    return 0;
  }

  bool done = false;
  auto pub = node->create_publisher<unitree_api::msg::Request>("/api/" + service + "/request", 10);
  auto sub = node->create_subscription<unitree_api::msg::Response>(
    "/api/" + service + "/response", 10,
    [&](unitree_api::msg::Response::SharedPtr res) {
      if (res->header.identity.id != req.header.identity.id) {return;}
      std::printf("response: api_id=%ld code=%d data=%s\n",
        static_cast<long>(res->header.identity.api_id), res->header.status.code, res->data.c_str());
      done = true;
    });

  // 等对端的订阅者出现再发，否则第一条会丢
  auto t0 = node->now();
  while (rclcpp::ok() && pub->get_subscription_count() == 0 && (node->now() - t0) < 5s) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(50ms);
  }
  pub->publish(req);
  t0 = node->now();
  while (rclcpp::ok() && !done && (node->now() - t0) < 3s) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(10ms);
  }
  if (!done) {std::printf("no response within 3 s\n");}
  rclcpp::shutdown();
  return done ? 0 : 1;
}
