#include <memory>
#include <random>
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
  LightningRRT() : Node("lightning_rrt")
  {
    // Initialize random number generator
    engine = std::mt19937(rd());
  }

private:
  std::random_device rd;
  std::mt19937 engine;
  int random_x;
  int random_y;
  Path rrt_path;

  rclcpp::Subscription<RRTRequest>::SharedPtr rrt_service_ =
      create_subscription<RRTRequest>(
          "rrt_request",
          10,
          std::bind(&LightningRRT::rrt_cb, this, _1));

  void rrt_cb(RRTRequest::SharedPtr request)
  {
    uint32_t iterations = request->iterations;
    double step_size = request->step_size;
    PoseStamped start = request->start;
    PoseStamped goal = request->goal;
    OccupancyGrid map = request->map;

    RCLCPP_INFO(this->get_logger(), "Received RRT request");

    float x_start = start.pose.position.x;
    float y_start = start.pose.position.y;
    float x_goal = goal.pose.position.x;
    float y_goal = goal.pose.position.y;
    float bounds_x = static_cast<float>(map.info.width);
    float bounds_y = static_cast<float>(map.info.height);

    if (!check_bounds(x_start, y_start, bounds_x, bounds_y) ||
        !check_bounds(x_goal, y_goal, bounds_x, bounds_y))
    {
      RCLCPP_ERROR(this->get_logger(), "Start or goal position out of bounds");
      return;
    }

    std::uniform_int_distribution<int> x_distrib(0, map.info.width - 1);
    std::uniform_int_distribution<int> y_distrib(0, map.info.height - 1);
    random_x = x_distrib(engine);
    random_y = y_distrib(engine);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Random point: (%d, %d)", random_x, random_y);
  }

  bool check_bounds(float x, float y, float bounds_x, float bounds_y)
  {
    return x >= 0 && x < bounds_x && y >= 0 && y < bounds_y;
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
