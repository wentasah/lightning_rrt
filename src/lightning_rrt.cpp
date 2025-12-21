#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;
using std::placeholders::_1;
using std::placeholders::_2;
using std_srvs::srv::Empty;
using visualization_msgs::msg::Marker;

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
    start_marker_.pose = msg->pose;
    start_marker_.header.frame_id = "map";
    start_marker_.ns = "start";
    start_marker_.id = 0;
    start_marker_.type = Marker::SPHERE;
    start_marker_.action = Marker::ADD;
    start_marker_.scale.x = 0.2;
    start_marker_.scale.y = 0.2;
    start_marker_.scale.z = 0.2;
    start_marker_.color.a = 1.0;
    start_marker_.color.r = 0.0;
    start_marker_.color.g = 1.0;
    start_marker_.color.b = 0.0;
    start_marker_pub_->publish(start_marker_);
  }

  void goal_cb(PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "'%f'", msg->pose.position.x);
    goal_marker_.pose = msg->pose;
    goal_marker_.header.frame_id = "map";
    goal_marker_.ns = "goal";
    goal_marker_.id = 1;
    goal_marker_.type = Marker::SPHERE;
    goal_marker_.action = Marker::ADD;
    goal_marker_.scale.x = 0.2;
    goal_marker_.scale.y = 0.2;
    goal_marker_.scale.z = 0.2;
    goal_marker_.color.a = 1.0;
    goal_marker_.color.r = 1.0;
    goal_marker_.color.g = 0.0;
    goal_marker_.color.b = 0.0;
    goal_marker_pub_->publish(goal_marker_);
  }

  void map_cb(OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map received: '%d' x '%d'",
                msg->info.width, msg->info.height);
  }

  void timer_cb()
  {
    path_pub_->publish(Path());
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

  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_ =
      create_subscription<PoseStamped>(
          "goal",
          10,
          std::bind(&LightningRRT::goal_cb, this, _1));

  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_ =
      create_subscription<OccupancyGrid>(
          "map",
          10,
          std::bind(&LightningRRT::map_cb, this, _1));

  rclcpp::Publisher<Path>::SharedPtr path_pub_ =
      create_publisher<Path>("path", 10);

  rclcpp::Publisher<Marker>::SharedPtr start_marker_pub_ =
      create_publisher<Marker>("start_marker", 10);

  rclcpp::Publisher<Marker>::SharedPtr goal_marker_pub_ =
      create_publisher<Marker>("goal_marker", 10);

  rclcpp::TimerBase::SharedPtr timer_ =
      create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&LightningRRT::timer_cb, this));

  Marker start_marker_;
  Marker goal_marker_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightningRRT>());
  rclcpp::shutdown();
  return 0;
}
