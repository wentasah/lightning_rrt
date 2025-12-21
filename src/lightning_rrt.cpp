#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using geometry_msgs::msg::PoseStamped;
using std::placeholders::_1;
using std::placeholders::_2;
using std_srvs::srv::Empty;

class LightningRRT : public rclcpp::Node
{
public:
  LightningRRT() : Node("lightning_rrt") {}

private:
  void rrt_cb(Empty::Request::SharedPtr, Empty::Response::SharedPtr)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RRT service called");
  }

  void start_cb(PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "'%f'", msg->pose.position.x);
  }

  void map_cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map received: '%d' x '%d'",
                msg->info.width, msg->info.height);
  }

  rclcpp::Service<Empty>::SharedPtr rrt_service_ =
      create_service<Empty>(
          "rrt",
          std::bind(&LightningRRT::rrt_cb, this, _1, _2));

  rclcpp::Subscription<PoseStamped>::SharedPtr start_sub_ =
      create_subscription<PoseStamped>(
          "start",
          10,
          std::bind(&LightningRRT::start_cb, this, _1));

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
          "map",
          10,
          std::bind(&LightningRRT::map_cb, this, _1));
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightningRRT>());
  rclcpp::shutdown();
  return 0;
}
