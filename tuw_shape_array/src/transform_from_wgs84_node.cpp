#include <filesystem>
#include <json/json.h>
#include <tuw_json/json.hpp>
#include <tuw_object_msgs/shape_array_json.hpp>
#include <tuw_std_msgs/parameter_array_json.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include "tuw_shape_array/transform_from_wgs84_node.hpp"
#include "tuw/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2::toMsg

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace tuw_shape_array;

FromWGS84Node::FromWGS84Node(const std::string& node_name) : Node(node_name)
{
  declare_parameters();
  read_static_parameters();
  read_dynamic_parameters();

  if (!debug_root_folder_.empty())
  {
    if (debug_root_folder_.back() != '/')
      debug_root_folder_ += '/';
    debug_dest_folder_ = debug_root_folder_ + get_name() + "/";
    if (!std::filesystem::is_directory(debug_dest_folder_) || !std::filesystem::exists(debug_dest_folder_))
      std::filesystem::create_directories(debug_dest_folder_);  // create debug destination folder for this node
    RCLCPP_INFO(this->get_logger(), "debug_dest_folder: %s", debug_dest_folder_.c_str());
  }

  pub_shapes_transformed_ = this->create_publisher<tuw_object_msgs::msg::ShapeArray>(topic_name_shapes_to_provide_, 10);
  sub_shapes_ = create_subscription<tuw_object_msgs::msg::ShapeArray>(
      topic_name_shapes_to_subscribe_, 10, std::bind(&FromWGS84Node::callback_shapes, this, _1));

  timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&FromWGS84Node::on_timer, this));
}

void FromWGS84Node::on_timer()
{
  // RCLCPP_INFO(this->get_logger(), "on_timer");
}
void FromWGS84Node::callback_shapes(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "callback_shapes");
  start_process(msg);
}

void FromWGS84Node::start_process(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg)
{
  msg_shapes_received_ = msg;

  if (tuw::equal(msg_shapes_received_->header.frame_id, "WGS84") == false)
  {
    RCLCPP_ERROR(this->get_logger(), "shapes are not in the global WSG84 systems, the frame_id is: %s",
                 msg_shapes_received_->header.frame_id.c_str());
    return;
  }

  // find utm zone by checking the first shape
  // auto wgs84 = msg_shapes_received_->shapes[0].poses[0].position;
  tuw_std_msgs::ParameterArray& params =
      static_cast<tuw_std_msgs::ParameterArray&>(msg_shapes_received_->shapes[0].params_poses[0]);
  int utm_zone;
  bool utm_northp = true;
  double gamma, k;
  tuw_geometry_msgs::Point utm;
  double latitude;
  double longitude;
  if (params.get<double>("latitude", latitude) == false)
  {
    RCLCPP_ERROR(this->get_logger(), "latitude on shape %d point %d missing", 0, 0);
  }
  if (params.get<double>("longitude", longitude) == false)
  {
    RCLCPP_ERROR(this->get_logger(), "longitude on shape %d point %d missing", 0, 0);
  }
  GeographicLib::UTMUPS::Forward(latitude, longitude, utm_zone, utm_northp, utm.x, utm.y, gamma, k);
  utm_meridian_convergence_ = gamma * M_PI / 180.0;
  RCLCPP_INFO(this->get_logger(), "gamma: %f Deg, utm_meridian_convergence: %f rad, scale: %f", gamma,
              utm_meridian_convergence_, k);

  msg_shapes_utm_ = std::make_shared<tuw_object_msgs::msg::ShapeArray>(*msg);
  transform_wgs84_to_utm(msg_shapes_utm_, utm_zone);
  tf2::Transform tf;
  compute_map_frame(msg_shapes_utm_, map_border_, tf);
  msg_shapes_map_ = std::make_shared<tuw_object_msgs::msg::ShapeArray>(*msg_shapes_utm_);
  transform_utm_to_map(msg_shapes_map_, tf);
  pub_shapes_transformed_->publish(*msg_shapes_map_);
  publish_transforms();
}

void FromWGS84Node::transform_wgs84_to_utm(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, int utm_zone)
{
  bool utm_northp = true;
  double gamma, k;
  for (auto& shape : shapes->shapes)
  {
    shape.poses.resize(shape.params_poses.size());
    for (size_t i = 0; i < shape.poses.size(); i++)
    {
      auto& p = shape.poses[i];
      tuw_std_msgs::ParameterArray& params = static_cast<tuw_std_msgs::ParameterArray&>(shape.params_poses[i]);
      double latitude;
      double longitude;
      double altitude;
      if (params.get<double>("latitude", latitude) == false)
      {
        RCLCPP_ERROR(this->get_logger(), "latitude on shape %ld point %ld missing", shape.id, i);
      }
      if (params.get<double>("longitude", longitude) == false)
      {
        RCLCPP_ERROR(this->get_logger(), "longitude on shape %ld point %ld missing", shape.id, i);
      }
      if (params.get<double>("altitude", altitude) == false)
      {
        RCLCPP_ERROR(this->get_logger(), "altitude on shape %ld point %ld missing", shape.id, i);
      }
      GeographicLib::UTMUPS::Forward(latitude, longitude, utm_zone, utm_northp, p.position.x, p.position.y, gamma, k,
                                     utm_zone);
      p.position.z = altitude;
    }
  }
  shapes->header.frame_id = GeographicLib::UTMUPS::EncodeZone(utm_zone, utm_northp);
}

void FromWGS84Node::transform_utm_to_map(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, const tf2::Transform& tf)
{
  tf2::Transform tf_inv = tf.inverse();
  for (auto& shape : shapes->shapes)
  {
    for (auto& pose : shape.poses)
    {
      auto& p = pose.position;
      tf2::Vector3 des = tf_inv * tf2::Vector3(p.x, p.y, p.z);
      p.x = des[0], p.y = des[1], p.z = des[2];
    }
  }
  shapes->header.frame_id = frame_map_;
}

void FromWGS84Node::compute_map_frame(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, double border,
                                      tf2::Transform& tf)
{
  /// crates a rectangular shape reprecenting the frame with border
  map_shape_ = std::make_shared<tuw_object_msgs::Shape>(shapes->shapes.size());
  map_shape_->shape = tuw_object_msgs::Shape::SHAPE_RECTANGLE;
  map_shape_->type = tuw_object_msgs::Shape::TYPE_MAP;
  map_shape_->poses.resize(2);
  map_shape_->poses[0] = shapes->shapes[0].poses[0];
  map_shape_->poses[1] = shapes->shapes[0].poses[0];
  for (auto& shape : shapes->shapes)
  {
    if (shape.type == tuw_object_msgs::Shape::TYPE_MAP)
    {
      RCLCPP_ERROR(this->get_logger(), "A map frame does allready exist");
      return;
    }
    for (auto& p : shape.poses)
    {
      map_shape_->poses[0].position.x = std::min(map_shape_->poses[0].position.x, p.position.x);
      map_shape_->poses[0].position.y = std::min(map_shape_->poses[0].position.y, p.position.y);
      map_shape_->poses[0].position.z = std::min(map_shape_->poses[0].position.z, p.position.z);
      map_shape_->poses[1].position.x = std::max(map_shape_->poses[1].position.x, p.position.x);
      map_shape_->poses[1].position.y = std::max(map_shape_->poses[1].position.y, p.position.y);
      map_shape_->poses[1].position.z = std::max(map_shape_->poses[1].position.z, p.position.z);
    }
  }
  map_shape_->poses[0].position.x -= border;
  map_shape_->poses[0].position.y -= border;
  map_shape_->poses[0].position.z -= border;
  map_shape_->poses[1].position.x += border;
  map_shape_->poses[1].position.y += border;
  map_shape_->poses[1].position.z += border;
  /// make the map frame size nice
  double dx = std::ceil(map_shape_->poses[1].position.x - map_shape_->poses[0].position.x);
  double dy = std::ceil(map_shape_->poses[1].position.y - map_shape_->poses[0].position.y);
  double dz = std::ceil(map_shape_->poses[1].position.z - map_shape_->poses[0].position.z);
  map_shape_->poses[1].position.x = map_shape_->poses[0].position.x + dx;
  map_shape_->poses[1].position.y = map_shape_->poses[0].position.y + dy;
  map_shape_->poses[1].position.z = map_shape_->poses[0].position.z + dz;

  shapes->shapes.push_back(*map_shape_);

  tf2::Quaternion q(0, 0, 0, 1);
  if (publish_tf_rotation_)
  {
    q.setEuler(0., 0., utm_meridian_convergence_);
  }
  tf.setOrigin(
      tf2::Vector3(map_shape_->poses[0].position.x, map_shape_->poses[0].position.y, map_shape_->poses[0].position.z));
  tf.setRotation(q);

  // Compute Transform msg
  if(!tf_utm_map_){
    tf_utm_map_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
  } 
  tf_utm_map_->header.frame_id = frame_utm_;
  tf_utm_map_->child_frame_id = frame_map_;
  tf_utm_map_->transform = tf2::toMsg(tf);
}

void FromWGS84Node::publish_transforms()
{
  if (!publish_tf_) return;
  if (!tf_utm_map_) return;
  // Used for publishing the static utm->map
  static std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_utm_;
  if(!broadcaster_utm_){
    broadcaster_utm_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  }
  RCLCPP_INFO(this->get_logger(), "publish_transforms_utm_map");
  
  tf_utm_map_->header.stamp = this->get_clock()->now();
  broadcaster_utm_->sendTransform(*tf_utm_map_);
  
}

void FromWGS84Node::declare_parameters()
{
  declare_parameters_with_description("timeout_service_call", 5,
                                      "Timeout on the GetGraph servide after startup in seconds. If 0, the timeout "
                                      "will be infinity",
                                      0, 600, 1);
  declare_parameters_with_description("debug_folder", "/tmp/ros",
                                      "Debug root folder, if set it information is stored in a subfolder of "
                                      "debug_folder with the node name.");
  declare_parameters_with_description("publish_tf", true, "On true a tf from frame_utm to frame_map is published");
  declare_parameters_with_description("publish_tf_rotation", false,
                                      "On true it adds a rotiation to the tf published caused by the projection. Only "
                                      "in combinaltion with publish_tf. ");
  declare_parameters_with_description("publish_marker", true, "On true objects are published using marker msgs");
  declare_parameters_with_description("frame_map", "map", "Name of the map frame, only need if publish_tf == true");
  declare_parameters_with_description("frame_utm", "utm", "Name of the utm frame, only need if publish_tf == true");
  declare_parameters_with_description("map_border", 10.1, "Border on the created map [meter]");
}

bool FromWGS84Node::read_dynamic_parameters()
{
  static bool first_call = true;  /// varible to identify the first time the fnc was called to init all variables
  bool changes = false;           /// used to identify changes

  update_parameter_and_log("map_border", map_border_, changes, first_call);
  update_parameter_and_log("publish_marker", publish_marker_, changes, first_call);
  update_parameter_and_log("publish_tf", publish_tf_, changes, first_call);
  update_parameter_and_log("publish_tf_rotation", publish_tf_rotation_, changes, first_call);

  first_call = false;
  return changes;
}

void FromWGS84Node::read_static_parameters()
{
  get_parameter_and_log("debug_folder", debug_root_folder_);
  get_parameter_and_log("frame_utm", frame_utm_);
  get_parameter_and_log("frame_map", frame_map_);
}
