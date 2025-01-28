#ifndef TUW_OBJECT_MAP_SERVER__SHAPE_ARRAY_HPP_
#define TUW_OBJECT_MAP_SERVER__SHAPE_ARRAY_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <tuw_object_msgs/shape_array.hpp>

#include <opencv2/core.hpp>

namespace tuw_shape_array
{

  class ShapeArrayToOccupancyGrid
  {
  public:
    ShapeArrayToOccupancyGrid();
    /**
     * @return false on error or not found
     */
    bool find_frame(const tuw_object_msgs::ShapeArray &shape_array);

    bool create_occupancy_grid(double resolution, const std_msgs::msg::Header &header);

    bool draw(const tuw_object_msgs::ShapeArray &shape_array);
 
    nav_msgs::msg::OccupancyGrid &occupancy_grid(){
      return *occupancy_grid_;
    }

  private:
    double min_x, min_y, min_z, max_x, max_y, max_z;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> occupancy_grid_;
    cv::Mat map_;

  };
}
#endif // TUW_OBJECT_MAP_SERVER__SHAPE_ARRAY_HPP_
