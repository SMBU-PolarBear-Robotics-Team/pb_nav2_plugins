// Copyright 2024 Polaris Xia
// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
#define PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_

#include <vector>

#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pb_nav2_plugins/layers/intensity_obstacle_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pb_nav2_costmap_2d
{

/**
  * @class IntensityVoxelLayer
  * @brief Takes laser and pointcloud data to populate a 3D voxel representation of the environment
  */
class IntensityVoxelLayer : public pb_nav2_costmap_2d::IntensityObstacleLayer
{
public:
  /**
    * @brief Voxel Layer constructor
    */
  IntensityVoxelLayer() : voxel_grid_(0, 0, 0)
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class's parent class Costmap2D
  }

  /**
    * @brief Voxel Layer destructor
    */
  virtual ~IntensityVoxelLayer();

  /**
    * @brief Initialization process of layer on startup
    */
  void onInitialize() override;

  /**
    * @brief Update the bounds of the master costmap by this layer's update dimensions
    * @param robot_x X pose of robot
    * @param robot_y Y pose of robot
    * @param robot_yaw Robot orientation
    * @param min_x X min map coord of the window to update
    * @param min_y Y min map coord of the window to update
    * @param max_x X max map coord of the window to update
    * @param max_y Y max map coord of the window to update
    */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
    * @brief Update the layer's origin to a new pose, often when in a rolling costmap
    */
  void updateOrigin(double new_origin_x, double new_origin_y) override;

  /**
    * @brief If layer is discretely populated
    */
  bool isDiscretized() { return true; }

  /**
    * @brief Match the size of the master costmap
    */
  void matchSize() override;

  /**
    * @brief Reset this costmap
    */
  void reset() override;

  /**
    * @brief If clearing operations should be processed on this layer or not
    */
  bool isClearable() override { return true; }

protected:
  /**
    * @brief Reset internal maps
    */
  void resetMaps() override;

  /**
    * @brief Use raycasting between 2 points to clear freespace
    */
  void raytraceFreespace(
    const nav2_costmap_2d::Observation & clearing_observation, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  bool publish_voxel_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_pub_;
  nav2_voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;
  int unknown_threshold_, mark_threshold_, size_z_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    clearing_endpoints_pub_;

  /**
    * @brief Covert world coordinates into map coordinates
    */
  inline bool worldToMap3DFloat(
    double wx, double wy, double wz, double & mx, double & my, double & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }
    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);
    return mx < size_x_ && my < size_y_ && mz < size_z_;
  }

  /**
    * @brief Covert world coordinates into map coordinates
    */
  inline bool worldToMap3D(
    double wx, double wy, double wz, unsigned int & mx, unsigned int & my, unsigned int & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    mz = static_cast<unsigned int>((wz - origin_z_) / z_resolution_);

    return mx < size_x_ && my < size_y_ && mz < (unsigned int)size_z_;
  }

  /**
    * @brief Covert map coordinates into world coordinates
    */
  inline void mapToWorld3D(
    unsigned int mx, unsigned int my, unsigned int mz, double & wx, double & wy, double & wz)
  {
    // returns the center point of the cell
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  }

  /**
    * @brief Find L2 norm distance in 3D
    */
  inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
  }

  /**
    * @brief Get the height of the voxel sizes in meters
    */
  double getSizeInMetersZ() const { return (size_z_ - 1 + 0.5) * z_resolution_; }

  /**
    * @brief Callback executed when a parameter change is detected
    * @param event ParameterEvent message
    */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace pb_nav2_costmap_2d

#endif  // PB_NAV2_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
