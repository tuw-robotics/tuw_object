
#include "tuw_shape_array/shape_array_occupancy_grid.hpp"
#include <tuw_object_msgs/shape.hpp>
#include <tuw_geometry_msgs/pose.hpp>
#include <tuw_std_msgs/parameter_array.hpp>
#include <tuw_std_msgs/parameter.hpp>

#include <opencv2/opencv.hpp>

using namespace tuw_shape_array;

ShapeArrayToOccupancyGrid::ShapeArrayToOccupancyGrid()
{
}

cv::Point ShapeArrayToOccupancyGrid::w2m(const geometry_msgs::msg::Point &pw) const {
  cv::Point pm( (pw.x-min_x)/resolution_, 
                (pw.y-min_y)/resolution_);
  return pm;
}
double ShapeArrayToOccupancyGrid::w2m(double d) const {
  return d / resolution_;
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
  resolution_ = resolution;
  occupancy_grid_->header = header;
  occupancy_grid_->info.resolution = resolution_;
  double dx = max_x - min_x;
  double dy = max_y - min_y;
  occupancy_grid_->info.width = std::ceil(dx / resolution_);
  occupancy_grid_->info.height = std::ceil(dy / resolution_);
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
  draw_type_plant_wine_row(shape_array);
  draw_type_transit_street(shape_array);
  draw_type_obstacle_tree(shape_array);
  return true;
}
bool ShapeArrayToOccupancyGrid::draw_type_plant_wine_row(const tuw_object_msgs::ShapeArray &shape_array){

  for (auto &shape : shape_array.shapes){
    if ((shape.type == tuw_object_msgs::Shape::TYPE_PLANT_WINE_ROW) && (shape.shape == tuw_object_msgs::Shape::SHAPE_LINE_STRIP)) {
      for(unsigned int i = 0; i < shape.poses.size() - 1; i++){
        cv::Point p0 = w2m(shape.poses[i].position);
        cv::Point p1 = w2m(shape.poses[i+1].position);
        const tuw_std_msgs::ParameterArray& params_p0 = static_cast<const tuw_std_msgs::ParameterArray &>(shape.params_poses[i]);
        const tuw_std_msgs::ParameterArray& params_p1 = static_cast<const tuw_std_msgs::ParameterArray &>(shape.params_poses[i+1]);
        const tuw_std_msgs::Parameter *param_p0 = params_p0.get("occupied");
        const tuw_std_msgs::Parameter *param_p1 = params_p1.get("occupied");
        if(param_p0 && param_p1){
          double d0 = w2m(param_p0->get<double>());
          double d1 = w2m(param_p1->get<double>());
          if((d0 > 0) && (d1 > 0)){
            cv::line(map_, p0, p1, cv::Scalar(0xFF), d0 + d1);
          }
        }
      }      
    }
  }
  for (auto &shape : shape_array.shapes){
    if ((shape.type == tuw_object_msgs::Shape::TYPE_PLANT_WINE_ROW) 
    && (shape.shape == tuw_object_msgs::Shape::SHAPE_LINE_STRIP)) {
      for(unsigned int i = 0; i < shape.poses.size() - 1; i++){
        cv::Point p0 = w2m(shape.poses[i].position);
        cv::Point p1 = w2m(shape.poses[i+1].position);
        const tuw_std_msgs::ParameterArray& params_p0 = static_cast<const tuw_std_msgs::ParameterArray &>(shape.params_poses[i]);
        const tuw_std_msgs::ParameterArray& params_p1 = static_cast<const tuw_std_msgs::ParameterArray &>(shape.params_poses[i+1]);
        const tuw_std_msgs::Parameter *param_p0 = params_p0.get("free");
        const tuw_std_msgs::Parameter *param_p1 = params_p1.get("free");
        if(param_p0 && param_p1){
          double d0 = w2m(param_p0->get<double>());
          double d1 = w2m(param_p1->get<double>());
          if((d0 > 0) && (d1 > 0)){
            cv::line(map_, p0, p1, cv::Scalar(0x00), d0 + d1);
          }
        }
      }      
    }
  }
  return true;
}

bool ShapeArrayToOccupancyGrid::draw_type_transit_street(const tuw_object_msgs::ShapeArray &shape_array){
  for (auto &shape : shape_array.shapes){
    if ((shape.type == tuw_object_msgs::Shape::TYPE_TRANSIT_STREET) 
        && (shape.shape == tuw_object_msgs::Shape::SHAPE_LINE_STRIP)) {
      for(unsigned int i = 0; i < shape.poses.size() - 1; i++){
        cv::Point p0 = w2m(shape.poses[i].position);
        cv::Point p1 = w2m(shape.poses[i+1].position);
        const tuw_std_msgs::ParameterArray& params_p0 = static_cast<const tuw_std_msgs::ParameterArray &>(shape.params_poses[i]);
        const tuw_std_msgs::ParameterArray& params_p1 = static_cast<const tuw_std_msgs::ParameterArray &>(shape.params_poses[i+1]);
        const tuw_std_msgs::Parameter *param_p0 = params_p0.get("occupied");
        const tuw_std_msgs::Parameter *param_p1 = params_p1.get("occupied");
        if(param_p0 && param_p1){
          double d0 = w2m(param_p0->get<double>());
          double d1 = w2m(param_p1->get<double>());
          if((d0 > 0) && (d1 > 0)){
            cv::line(map_, p0, p1, cv::Scalar(0xFF), d0 + d1);
          }
        }
      }      
    }
  }
  return true;
}

bool ShapeArrayToOccupancyGrid::draw_type_obstacle_tree(const tuw_object_msgs::ShapeArray &shape_array){
  for (auto &shape : shape_array.shapes){
    if ((shape.type == tuw_object_msgs::Shape::TYPE_OBSTACLE_TREE) 
      && (shape.shape == tuw_object_msgs::Shape::SHAPE_CIRCLE)) {
      cv::Point p0 = w2m(shape.poses[0].position);
      const tuw_std_msgs::ParameterArray& params_p0 = static_cast<const tuw_std_msgs::ParameterArray &>(shape.params_poses[0]);
      const tuw_std_msgs::Parameter *param_p0 = params_p0.get("free");
      if(param_p0){
        double d0 = w2m(param_p0->get<double>());
        if((d0 > 0)){
          cv::circle(map_, p0, d0 + d0, cv::Scalar(0xFF), d0 + d0);
        }
      }     
    }
  }
  return true;
}
