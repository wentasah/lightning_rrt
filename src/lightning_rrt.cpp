#include <memory>
#include <random>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
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
    engine = std::mt19937(rd()); // Initialize random number generator
  }

private:
  // Initialize random number generation
  std::random_device rd;
  std::mt19937 engine;
  float random_x;
  float random_y;

  // Publisher for the RRT path
  rclcpp::Publisher<Path>::SharedPtr path_publisher_ =
      create_publisher<Path>("rrt_path", 10);

  // Subscription that runs the RRT algorithm upon receiving a request
  rclcpp::Subscription<RRTRequest>::SharedPtr rrt_subscriber_ =
      create_subscription<RRTRequest>(
          "rrt_request",
          10,
          std::bind(&LightningRRT::rrt_cb, this, std::placeholders::_1));

  void rrt_cb(RRTRequest::SharedPtr request)
  {
    // RRT Algorithm
    RCLCPP_INFO(get_logger(), "Received RRT request");

    // Extract parameters from the request
    const uint32_t iterations = request->iterations;
    const double step_size = request->step_size;
    const OccupancyGrid map = request->map;
    const float x_start = request->start.x;
    const float y_start = request->start.y;
    const float x_goal = request->goal.x;
    const float y_goal = request->goal.y;

    // Check if start and goal are within map bounds
    if (!check_bounds(x_start, y_start, map) ||
        !check_bounds(x_goal, y_goal, map))
    {
      RCLCPP_ERROR(get_logger(), "Start or goal is out of bounds");
      return;
    }

    // Check if start or goal is on an obstacle
    // Normally this checks between points, but here it checks the same point
    if (check_collision(x_start, y_start, x_start, y_start, map) ||
        check_collision(x_goal, y_goal, x_goal, y_goal, map))
    {
      RCLCPP_ERROR(get_logger(), "Start or goal is in an obstacle");
      return;
    }

    // Initialize vector to store pointers to RRT nodes
    bool path_found = false;
    std::vector<std::shared_ptr<RRTNode>> nodes = {std::make_shared<RRTNode>(x_start,
        y_start, nullptr)};

    // RRT main loop
    for (uint32_t i = 0; i < iterations; ++i)
    {
      // Initialize new_x and new_y
      float new_x = nodes.back()->x;
      float new_y = nodes.back()->y;

      // Check if a straight line to goal is possible
      if (!check_collision(new_x, new_y, x_goal, y_goal, map))
      {
        nodes.push_back(std::make_shared<RRTNode>(x_goal, y_goal, nodes.back()));
        path_found = true;
        RCLCPP_INFO(get_logger(), "Path found!");
        break;
      }

      update_random_point(map);

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
      const float theta = std::atan2(random_y - nearest_node->y,
                               random_x - nearest_node->x);
      new_x = nearest_node->x + step_size * std::cos(theta);
      new_y = nearest_node->y + step_size * std::sin(theta);

      // Check for collision
      if (!check_collision(nearest_node->x, nearest_node->y, new_x, new_y, map))
      {
        nodes.push_back(std::make_shared<RRTNode>(new_x, new_y, nearest_node));
      }
    }

    // Publish the resulting path if found
    if (path_found)
    {
      // Reconstruct path
      Path rrt_path;
      rrt_path.header.frame_id = "map";
      std::shared_ptr<RRTNode> current_node = nodes.back();
      while (current_node != nullptr)
      {
        PoseStamped pose;
        pose.pose.position.x = current_node->x;
        pose.pose.position.y = current_node->y;
        rrt_path.poses.push_back(pose);
        current_node = current_node->parent;
      }
      std::reverse(rrt_path.poses.begin(), rrt_path.poses.end());
      path_publisher_->publish(rrt_path);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "No path found within %d iterations", iterations);
    }
  }

  void update_random_point(const OccupancyGrid &map)
  {
    std::uniform_int_distribution<int> dist_x(0, map.info.width - 1);
    std::uniform_int_distribution<int> dist_y(0, map.info.height - 1);
    random_x = static_cast<float>(dist_x(engine));
    random_y = static_cast<float>(dist_y(engine));
  }

  bool check_bounds(float x, float y, const OccupancyGrid &map)
  {
    // Check if (x, y) is within map bounds
    const float origin_x = map.info.origin.position.x;
    const float origin_y = map.info.origin.position.y;
    const float bounds_x = static_cast<float>(map.info.width);
    const float bounds_y = static_cast<float>(map.info.height);
    return (x > origin_x && x < origin_x + bounds_x &&
            y > origin_y && y < origin_y + bounds_y);
  }

  bool check_collision(float x1, float y1, float x2, float y2,
                       const OccupancyGrid &map)
  {
    // Check for collision between two points
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    const float distance = std::hypot(dx, dy);
    const float resolution = map.info.resolution;
    int steps = static_cast<int>(distance / resolution);
    steps = std::max(steps, 1); // Ensure at least one check

    for (int i = 0; i <= steps; ++i)
    {
      // Interpolate points along the line from (x1, y1) to (x2, y2)
      const float t = static_cast<float>(i) / static_cast<float>(steps);
      const float world_x = x1 + t * dx;
      const float world_y = y1 + t * dy;
      const float origin_x = map.info.origin.position.x;
      const float origin_y = map.info.origin.position.y;
      const int grid_x = static_cast<int>((world_x - origin_x) / resolution);
      const int grid_y = static_cast<int>((world_y - origin_y) / resolution);

      // Grid bounds check
      if (grid_x < 0 || grid_y < 0 ||
          grid_x >= static_cast<int>(map.info.width) ||
          grid_y >= static_cast<int>(map.info.height))
      {
        return true;
      }

      // Check occupancy grid cell value
      const int index = grid_y * map.info.width + grid_x;
      const int8_t cell = map.data[index];

      if (cell != 0)
      {
        return true; // Collision detected
      }
    }

    return false; // No collision detected
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
