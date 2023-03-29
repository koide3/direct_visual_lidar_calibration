#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <camera/generic_camera_base.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/points_color_updater.hpp>

namespace vlcal {

/**
 * @brief A class to visualize LiDAR-camera dataset while painting LiDAR points with camera images
 * @note  What a poor class name!
 */
class VisualLiDARVisualizer {
public:
  /**
   * @brief Constructor
   * @param proj          Camera model
   * @param dataset       LiDAR-camera data pairings
   * @param draw_sphere   If true, draw unit sphere as a projection screen in addition to LiDAR points
   * @param show_image_cv If ture, do cv::imshow in addition to LiDAR points rendering
   */
  VisualLiDARVisualizer(
    const camera::GenericCameraBase::ConstPtr& proj,
    const std::vector<VisualLiDARData::ConstPtr>& dataset,
    const bool draw_sphere,
    const bool show_image_cv = false);
  ~VisualLiDARVisualizer();

  void set_blend_weight(float weight) { blend_weight = weight; }
  void set_T_camera_lidar(const Eigen::Isometry3d& T_camera_lidar);

  int get_selected_bag_id() const { return selected_bag_id; }
  double get_image_display_scale() const { return image_display_scale; }

  bool spin_once();

private:
  void ui_callback();
  void color_update_task();

private:
  const bool draw_sphere;
  const bool show_image_cv;

  const camera::GenericCameraBase::ConstPtr proj;
  const std::vector<VisualLiDARData::ConstPtr> dataset;

  double image_display_scale;
  Eigen::Isometry3d T_camera_lidar;

  int selected_bag_id;
  float blend_weight;

  std::mutex updater_mutex;
  std::unique_ptr<PointsColorUpdater> sphere_updater;
  std::unique_ptr<PointsColorUpdater> color_updater;

  std::atomic_bool kill_switch;
  std::thread color_update_thread;
};

}  // namespace vlcal
