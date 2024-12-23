#ifndef TUW_SHAPE_ARRAY__SHAPE_ARRAY_TO_MAP_HPP_
#define TUW_SHAPE_ARRAY__SHAPE_ARRAY_TO_MAP_HPP_

#include <memory>
#include <thread>
#include <tf2/LinearMath/Transform.h>
#include <tuw/node.hpp>
#include <tuw_object_msgs/shape.hpp>
#include <tuw_object_msgs/msg/shape_array.hpp>
#include <tuw_object_msgs/srv/get_shape_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace tuw_shape_array
{
  class ShapeArrayToMap : public tuw::Node
  {
  public:
    ShapeArrayToMap(const std::string &node_name);

  private:
    const std::string topic_name_shapes_to_subscribe_{"shapes"};   /// topic name to subscribe
    const std::string topic_name_map_to_provide_{"map"};           /// topic name to provide objects with computed map_points

    /// subscriber incomming shapes
    rclcpp::Subscription<tuw_object_msgs::msg::ShapeArray>::ConstSharedPtr sub_shapes_;

    /// callback for the incomming shapes
    void callback_shapes(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg);

    /// publisher for shapes transformed
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_map_;

    // last received shapes
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_received_;

    // timer for loop_rate
    rclcpp::TimerBase::SharedPtr timer_;

    // callbacks
    void on_timer();
    
    std::string frame_map_;         /// static parameter: Used to overwrite the map frame_id in the occupancy grid 
    double resolution_;             /// static parameter: Resolution of the generated map [m/pix]
    bool show_map_;                 /// dynamic parameter: Shows the map in a opencv window

    /// starts the computation
    void start_process(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg);

    void declare_parameters();      // declare parameters
    void read_static_parameters();  // ready the static parameters
    bool read_dynamic_parameters(); // ready the dynamic parameters and returns true on changes
  };
}
#endif // TUW_SHAPE_ARRAY__SHAPE_ARRAY_TO_MAP_HPP_
