#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using nav_msgs::msg::OccupancyGrid;
using std::placeholders::_1;

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
  void publish_map()
  {
    map.header.stamp = this->now();
    map_pub_->publish(map);
  }

  OccupancyGrid map;

  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_ =
      create_publisher<OccupancyGrid>("map", 10);

  rclcpp::TimerBase::SharedPtr timer_ =
      create_wall_timer(std::chrono::milliseconds(500),
                        std::bind(&LightningRRT::publish_map, this));
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightningRRT>());
  rclcpp::shutdown();
  return 0;
}
