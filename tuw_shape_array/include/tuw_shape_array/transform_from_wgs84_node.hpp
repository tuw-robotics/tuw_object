#ifndef TUW_SHAPE_ARRAY__SHAPE_MAP_NODE_HPP_
#define TUW_SHAPE_ARRAY__SHAPE_MAP_NODE_HPP_

#include <memory>
#include <thread>
#include <tf2/LinearMath/Transform.h>
#include <tuw/node.hpp>
#include <tuw_object_msgs/shape.hpp>
#include <tuw_object_msgs/msg/shape_array.hpp>
#include <tuw_object_msgs/srv/get_shape_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace tuw_shape_array
{
  class FromWGS84Node : public tuw::Node
  {
  public:
    FromWGS84Node(const std::string &node_name);

  private:
    const std::string topic_name_shapes_to_subscribe_{"shapes_input"};   /// topic name to subscribe
    const std::string topic_name_shapes_to_provide_{"shapes"};           /// topic name to provide objects with computed map_points

    /// subscriber incomming shapes
    rclcpp::Subscription<tuw_object_msgs::msg::ShapeArray>::ConstSharedPtr sub_shapes_;

    /// callback for the incomming shapes
    void callback_shapes(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg);

    /// publisher for shapes transformed
    rclcpp::Publisher<tuw_object_msgs::msg::ShapeArray>::SharedPtr pub_shapes_transformed_;

    // last received shapes
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_received_;

    // msgs transformed into utm
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_utm_;

    // msgs transformed into map frame
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_map_;

    // last processed shapes
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_processed_;

    // last processed shapes
    std::shared_ptr<tuw_object_msgs::Shape> map_shape_;

    // Transform from utm to map
    std::shared_ptr<geometry_msgs::msg::TransformStamped> tf_utm_map_;

    // timer for loop_rate
    rclcpp::TimerBase::SharedPtr timer_;

    // callbacks
    void on_timer();

    /// Object map objecet need to compute the occupancy grid as well as marker and transforms
    double utm_meridian_convergence_;

    std::string frame_map_;         /// static parameter: Name of the map frame, only need if publish_tf == true
    std::string frame_utm_;         /// static parameter: Name of the utm frame, only need if publish_tf == true
    std::string debug_root_folder_; /// static parameter: folder name with debug information from tuw_ tools
    std::string debug_dest_folder_; /// generated parameter: based on debug_root_folder_ + node name
    bool publish_tf_;               /// dynamic parameter: on true a tf from frame_utm to frame_map is published
    bool publish_tf_rotation_;      /// dynamic parameter: On true it adds a rotiation to the tf published caused by the projection. Only in combinaltion with publish_tf.
    bool publish_marker_;           /// dynamic parameter: on true shapes are published using marker msgs
    double map_border_;             /// dynamic parameter: Border on the created map [meter]
    double utm_z_offset_;           /// dynamic parameter: z offset on the location of the map

    /// starts the computation
    void start_process(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg);
    void transform_wgs84_to_utm(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, int utm_zone);
    void transform_utm_to_map(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, const tf2::Transform &tf);
    void compute_map_frame(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, double border, tf2::Transform &tf);

    void declare_parameters();      // declare parameters
    void read_static_parameters();  // ready the static parameters
    bool read_dynamic_parameters(); // ready the dynamic parameters and returns true on changes
    void publish_transforms();      // publishes the utm -> map transfrom if it exists
  };
}
#endif // TUW_SHAPE_ARRAY__SHAPE_MAP_NODE_HPP_
