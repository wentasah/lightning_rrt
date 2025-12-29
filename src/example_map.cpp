#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lightning_rrt_interfaces/msg/rrt_request.hpp"

using geometry_msgs::msg::Point;
using lightning_rrt_interfaces::msg::RRTRequest;
using nav_msgs::msg::OccupancyGrid;

class ExampleMap : public rclcpp::Node
{
public:
  ExampleMap() : Node("example_map")
  {
    // Set the max iterations and step size
    iterations = 10000;
    step_size = 0.5;

    // Create an occupancy grid map
    map.header.frame_id = "map";
    map.info.resolution = 0.1;
    map.info.width = 100;
    map.info.height = 100;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.resize(map.info.width * map.info.height, 0);

    // Add "L" obstacle to the map
    for (int y = 20; y < 40; ++y)
    {
      for (int x = 20; x < 80; ++x)
      {
        map.data[y * map.info.width + x] = 100; // Occupied
      }
    }
    for (int y = 40; y < 60; ++y)
    {
      for (int x = 60; x < 80; ++x)
      {
        map.data[y * map.info.width + x] = 100; // Occupied
      }
    }
  }

  void timer_cb()
  {
    // Set the start pose
    start.x = 1.0;
    start.y = 1.0;

    // Set the goal pose
    goal.x = 9.0;
    goal.y = 9.0;

    // Generate the request message
    message.iterations = iterations;
    message.step_size = step_size;
    message.start = start;
    message.goal = goal;
    message.map = map;

    // Publish the request
    request_publisher_->publish(message);
  }

private:
  int iterations;
  double step_size;
  Point start;
  Point goal;
  OccupancyGrid map;

  RRTRequest message;

  rclcpp::Publisher<RRTRequest>::SharedPtr request_publisher_ =
      create_publisher<RRTRequest>("rrt_request", 10);

  rclcpp::TimerBase::SharedPtr timer_ =
      create_wall_timer(std::chrono::milliseconds(50),
                        std::bind(&ExampleMap::timer_cb, this));
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleMap>());
  rclcpp::shutdown();
  return 0;
}
