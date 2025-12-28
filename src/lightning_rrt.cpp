#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "lightning_rrt_interfaces/msg/rrt_request.hpp"

using geometry_msgs::msg::PoseStamped;
using lightning_rrt_interfaces::msg::RRTRequest;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;
using std::placeholders::_1;

struct RRTNode
{
  int x;
  int y;
  std::unique_ptr<RRTNode> parent;
};

class LightningRRT : public rclcpp::Node
{
public:
  LightningRRT() : Node("lightning_rrt") {}

private:
  RRTNode start_node;
  RRTNode goal_node;
  Path rrt_path;
  OccupancyGrid map;

  rclcpp::Subscription<RRTRequest>::SharedPtr rrt_service_ =
      create_subscription<RRTRequest>(
          "rrt_request",
          10,
          std::bind(&LightningRRT::rrt_cb, this, _1));

  void rrt_cb(RRTRequest::SharedPtr request)
  {
    int iterations = request->iterations;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Iterations: %d", iterations);

    float step_size = request->step_size;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Step size: %f", step_size);

    start_node.x = request->start.pose.position.x;
    start_node.y = request->start.pose.position.y;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Start: (%d, %d)", start_node.x, start_node.y);

    goal_node.x = request->goal.pose.position.x;
    goal_node.y = request->goal.pose.position.y;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Goal: (%d, %d)", goal_node.x, goal_node.y);

    map.data = request->map.data;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Map received with size: %zu", map.data.size());
  }

  float get_distance(const PoseStamped &a, const PoseStamped &b)
  {
    return std::hypot(
        a.pose.position.x - b.pose.position.x,
        a.pose.position.y - b.pose.position.y);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightningRRT>());
  rclcpp::shutdown();
  return 0;
}
