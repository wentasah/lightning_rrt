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

struct RRTNode
{
  float x;
  float y;
  std::shared_ptr<RRTNode> parent;

  RRTNode(float x_, float y_, std::shared_ptr<RRTNode> parent_)
      : x(x_), y(y_), parent(std::move(parent_)) {}
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
  float random_x;
  float random_y;
  std::vector<std::shared_ptr<RRTNode>> nodes;

  rclcpp::Publisher<Path>::SharedPtr path_publisher_ =
      create_publisher<Path>("rrt_path", 10);

  rclcpp::Subscription<RRTRequest>::SharedPtr rrt_service_ =
      create_subscription<RRTRequest>(
          "rrt_request",
          10,
          std::bind(&LightningRRT::rrt_cb, this, std::placeholders::_1));

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

    // RRT Algorithm
    nodes.clear();
    nodes.push_back(std::make_shared<RRTNode>(static_cast<int>(x_start), static_cast<int>(y_start), nullptr));
    bool path_found = false;

    for (uint32_t i = 0; i < iterations; ++i)
    {
      // Initialize new_x and new_y
      float new_x = nodes.back()->x;
      float new_y = nodes.back()->y;

      // Check if a straight line to goal is possible
      if (!check_straight_line_collision(new_x, new_y, x_goal, y_goal, map))
      {
        path_found = true;
        RCLCPP_INFO(this->get_logger(), "Path found!");
        break;
      }

      generate_random_point(map);

      // Find nearest node
      std::shared_ptr<RRTNode> nearest_node;
      float min_dist = std::numeric_limits<float>::max();
      for (auto &node : nodes)
      {
        float dist = std::hypot(node->x - random_x, node->y - random_y);
        if (dist < min_dist)
        {
          min_dist = dist;
          nearest_node = node;
        }
      }

      // Steer towards random point
      float theta = std::atan2(random_y - nearest_node->y, random_x - nearest_node->x);
      new_x = nearest_node->x + step_size * std::cos(theta);
      new_y = nearest_node->y + step_size * std::sin(theta);

      // Check for collision
      int int_y = static_cast<int>(new_y);
      int int_x = static_cast<int>(new_x);
      if (map.data[int_y * map.info.width + int_x] == 0) // Free space
      {
        nodes.push_back(std::make_shared<RRTNode>(new_x, new_y, nearest_node));
      }
    }

    Path rrt_path;
    rrt_path.header.frame_id = "map";
    if (path_found)
    {
      // Add goal node
      nodes.push_back(std::make_shared<RRTNode>(x_goal, y_goal, nodes.back()));

      // Reconstruct path
      rrt_path.poses.clear();
      std::shared_ptr<RRTNode> current_node = nodes.back();
      while (current_node != nullptr)
      {
        PoseStamped pose;
        pose.pose.position.x = static_cast<double>(current_node->x);
        pose.pose.position.y = static_cast<double>(current_node->y);
        rrt_path.poses.push_back(pose);
        current_node = current_node->parent;
      }
      std::reverse(rrt_path.poses.begin(), rrt_path.poses.end());
      path_publisher_->publish(rrt_path);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No path found within the given iterations");
    }
  }

  void generate_random_point(const OccupancyGrid &map)
  {
    std::uniform_int_distribution<int> dist_x(0, map.info.width - 1);
    std::uniform_int_distribution<int> dist_y(0, map.info.height - 1);
    random_x = static_cast<float>(dist_x(engine));
    random_y = static_cast<float>(dist_y(engine));
  }

  bool check_bounds(float x, float y, float bounds_x, float bounds_y)
  {
    return x >= 0 && x < bounds_x && y >= 0 && y < bounds_y;
  }

  bool check_straight_line_collision(float x1, float y1,
                                     float x2, float y2,
                                     const OccupancyGrid &map)
  {
    int steps = static_cast<int>(std::hypot(x2 - x1, y2 - y1));
    for (int i = 0; i <= steps; ++i)
    {
      float t = static_cast<float>(i) / static_cast<float>(steps);
      int x = static_cast<int>(x1 + t * (x2 - x1));
      int y = static_cast<int>(y1 + t * (y2 - y1));
      if (map.data[y * map.info.width + x] != 0) // Not free space
      {
        return true; // Collision detected
      }
    }
    return false; // No collision
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
