#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "lightning_rrt_interfaces/msg/rrt_request.hpp"

using geometry_msgs::msg::PoseStamped;
using lightning_rrt_interfaces::msg::RRTRequest;
using nav_msgs::msg::OccupancyGrid;
using visualization_msgs::msg::Marker;

class VisualizeRRT : public rclcpp::Node
{
public:
  VisualizeRRT() : Node("visualize_rrt") {}

  void sub_cb(const RRTRequest::SharedPtr msg)
  {
    // Publish the received map
    map_publisher_->publish(msg->map);

    // Publish start marker
    Marker start_marker;
    start_marker.header.stamp = get_clock()->now();
    start_marker.header.frame_id = "map";
    start_marker.ns = "start_goal";
    start_marker.id = 0;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.pose.position = msg->start;
    start_marker.scale.x = 0.2;
    start_marker.scale.y = 0.2;
    start_marker.scale.z = 0.2;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;
    start_marker_publisher_->publish(start_marker);

    // Publish goal marker
    Marker goal_marker;
    goal_marker.header.stamp = get_clock()->now();
    goal_marker.header.frame_id = "map";
    goal_marker.ns = "start_goal";
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position = msg->goal;
    goal_marker.scale.x = 0.2;
    goal_marker.scale.y = 0.2;
    goal_marker.scale.z = 0.2;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 1.0;
    goal_marker_publisher_->publish(goal_marker);
  }

private:
  rclcpp::Subscription<RRTRequest>::SharedPtr request_publisher_ =
      create_subscription<RRTRequest>(
          "rrt_request",
          10,
          std::bind(&VisualizeRRT::sub_cb, this, std::placeholders::_1));

  rclcpp::Publisher<Marker>::SharedPtr start_marker_publisher_ =
      create_publisher<Marker>("rrt_start_marker", 10);

  rclcpp::Publisher<Marker>::SharedPtr goal_marker_publisher_ =
      create_publisher<Marker>("rrt_goal_marker", 10);

  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_publisher_ =
      create_publisher<OccupancyGrid>("rrt_map", 10);
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizeRRT>());
  rclcpp::shutdown();
  return 0;
}
