#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using geometry_msgs::msg::PoseStamped;

using nav_msgs::msg::OccupancyGrid;

class LightningRRT : public rclcpp::Node
{
public:
  LightningRRT() : Node("lightning_rrt")
  {
    map.header.frame_id = "map";
    map.info.resolution = 0.1;
    map.info.width = 100;
    map.info.height = 100;
    map.info.origin.position.x = -5.0;
    map.info.origin.position.y = -5.0;
    map.data.resize(map.info.width * map.info.height, 0);

    // Create some obstacles
    for (int y = 40; y < 60; ++y)
    {
      for (int x = 20; x < 80; ++x)
      {
        map.data[y * map.info.width + x] = 100; // Occupied
      }
    }
  }

private:
  void timer_cb()
  {
    // Publish the map
    map.header.stamp = this->now();
    map_pub_->publish(map);

    // Publish start and goal poses
    PoseStamped start;
    start.header.frame_id = "map";
    start.header.stamp = this->now();
    start.pose.position.x = -4.0;
    start.pose.position.y = -4.0;
    start.pose.orientation.w = 1.0;
    start_pub_->publish(start);

    PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position.x = 4.0;
    goal.pose.position.y = 4.0;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);
  }

  OccupancyGrid map;

  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_ =
      create_publisher<OccupancyGrid>("map", 10);

  rclcpp::Publisher<PoseStamped>::SharedPtr start_pub_ =
      create_publisher<PoseStamped>("start", 10);

  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_ =
      create_publisher<PoseStamped>("goal", 10);

  rclcpp::TimerBase::SharedPtr timer_ =
      create_wall_timer(std::chrono::milliseconds(500),
                        std::bind(&LightningRRT::timer_cb, this));
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightningRRT>());
  rclcpp::shutdown();
  return 0;
}
