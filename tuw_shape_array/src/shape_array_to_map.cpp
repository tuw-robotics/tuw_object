#include <filesystem>
#include <tuw_object_msgs/shape_array.hpp>
#include "tuw_shape_array/shape_array_to_map.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2::toMsg

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace tuw_shape_array;

ShapeArrayToMap::ShapeArrayToMap(const std::string& node_name) : Node(node_name)
{
  declare_parameters();
  read_static_parameters();
  read_dynamic_parameters();

  pub_occupancy_grid_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name_map_to_provide_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  sub_shapes_ = create_subscription<tuw_object_msgs::msg::ShapeArray>(
      topic_name_shapes_to_subscribe_, 10, std::bind(&ShapeArrayToMap::callback_shapes, this, _1));

  timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ShapeArrayToMap::on_timer, this));
  trigger_request();
}

void ShapeArrayToMap::on_timer()
{
  // RCLCPP_INFO(this->get_logger(), "on_timer");
}

void ShapeArrayToMap::trigger_request()
{
  // No trigger call if the timeout_trigger_publisher_ is zero or less
  if (timeout_trigger_publisher_ <= 0) 
    return;
  
  auto client = this->create_client<std_srvs::srv::Trigger>(
      service_name_trigger_);

  // Wait for the service to be available
  int timeout = timeout_trigger_publisher_;
  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service '%s' not available, waiting %d sec ... ", service_name_trigger_.c_str(), timeout--);
    if ((timeout_trigger_publisher_ > 0) && (timeout <= 0))
    {
      return;
    }
  }
  // Create a request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result_future = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    return;
  }
  auto result = result_future.get();
  RCLCPP_INFO(this->get_logger(), "Successfuly trigger request service: '%s'", result->message.c_str());
}


void ShapeArrayToMap::callback_shapes(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "callback_shapes");
  start_process(msg);
}

void ShapeArrayToMap::start_process(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "start_process");
}


void ShapeArrayToMap::declare_parameters()
{
  declare_parameters_with_description("timeout_trigger_publisher", 5,
                                      "Timeout on the service to trigger the publisher on startup [sec]. "
                                      "On zero not service will be called on startup",
                                      0, 600, 1);
  declare_parameters_with_description("frame_map", "", "Used to overwrite the map frame_id in the occupancy grid");
  declare_parameters_with_description("resolution", 0.1, "Resolution of the generated map [m/pix]");
  declare_parameters_with_description("show_map", true, "Shows the map in a opencv window");
}

bool ShapeArrayToMap::read_dynamic_parameters()
{
  static bool first_call = true;  /// varible to identify the first time the fnc was called to init all variables
  bool changes = false;           /// used to identify changes

  update_parameter_and_log("show_map", show_map_, changes, first_call);

  first_call = false;
  return changes;
}

void ShapeArrayToMap::read_static_parameters()
{
  get_parameter_and_log("timeout_trigger_publisher", timeout_trigger_publisher_);
  get_parameter_and_log("resolution", resolution_);
  get_parameter_and_log("frame_map", frame_map_);
}
