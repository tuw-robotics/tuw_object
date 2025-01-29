
#include "tuw_shape_array/shape_array_occupancy_grid.hpp"
#include <tuw_object_msgs/shape.hpp>

#include <opencv2/opencv.hpp>

using namespace tuw_shape_array;

ShapeArrayToOccupancyGrid::ShapeArrayToOccupancyGrid()
{
}

/**
 * @return false on error or not found
 */
bool ShapeArrayToOccupancyGrid::find_frame(const tuw_object_msgs::ShapeArray &shape_array)
{

  const tuw_object_msgs::Shape *frame_shape = NULL;
  for (auto &shape : shape_array.shapes)
  {
    if ((shape.type == tuw_object_msgs::Shape::TYPE_MAP) &&
        (shape.shape == tuw_object_msgs::Shape::SHAPE_RECTANGLE))
    {
      if (frame_shape != NULL)
      {
        return false; // more than one frame entry
      }
      frame_shape = static_cast<const tuw_object_msgs::Shape *>(&shape);
    }
  }
  if (frame_shape == NULL)
  {
    return false; // no frame entry
  }
  if (frame_shape->poses.size() != 2)
  {
    return false; // shape incorrect defined
  }

  min_x = frame_shape->poses[0].position.x;
  min_y = frame_shape->poses[0].position.y;
  min_z = frame_shape->poses[0].position.z;
  max_x = frame_shape->poses[1].position.x;
  max_y = frame_shape->poses[1].position.y;
  max_z = frame_shape->poses[1].position.z;

  return true;
}

bool ShapeArrayToOccupancyGrid::create_occupancy_grid(double resolution, const std_msgs::msg::Header &header)
{
  if (!occupancy_grid_)
  {
    occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }
  occupancy_grid_->header = header;
  occupancy_grid_->info.resolution = resolution;
  double dx = max_x - min_x;
  double dy = max_y - min_y;
  occupancy_grid_->info.width = std::ceil(dx / resolution);
  occupancy_grid_->info.height = std::ceil(dy / resolution);
  occupancy_grid_->info.origin.position.x = 0;
  occupancy_grid_->info.origin.position.y = 0;
  occupancy_grid_->info.origin.position.z = 0;
  occupancy_grid_->info.origin.orientation.x = 0;
  occupancy_grid_->info.origin.orientation.y = 0;
  occupancy_grid_->info.origin.orientation.z = 0;
  occupancy_grid_->info.origin.orientation.w = 1.;

  if ((static_cast<unsigned int>(map_.rows) != occupancy_grid_->info.height) &&
      (static_cast<unsigned int>(map_.cols) != occupancy_grid_->info.width))
  {
    occupancy_grid_->data.resize(occupancy_grid_->info.height * occupancy_grid_->info.width);
    map_ = cv::Mat(occupancy_grid_->info.height, occupancy_grid_->info.width, CV_8S, &occupancy_grid_->data[0]);
  }
  return true;
}
cv::Point ShapeArrayToOccupancyGrid::project(const tuw_geometry_msgs::Point &point){

}

bool ShapeArrayToOccupancyGrid::draw(const tuw_object_msgs::ShapeArray &shape_array){

  map_.setTo(0);
  for (auto &shape : shape_array.shapes){
    if (shape.type == tuw_object_msgs::Shape::SHAPE_LINE) {
      
    }
  }
  return true;
}