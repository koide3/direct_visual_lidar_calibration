#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <camera/generic_camera_base.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/points_color_updater.hpp>

namespace vlcal {

class VisualLiDARVisualizer {
public:
  VisualLiDARVisualizer(const camera::GenericCameraBase::ConstPtr& proj, const std::vector<VisualLiDARData::ConstPtr>& dataset, const bool draw_sphere);
  ~VisualLiDARVisualizer();

  void set_T_camera_lidar(const Eigen::Isometry3d& T_camera_lidar);

  bool spin_once();

private:
  void ui_callback();
  void color_update_task();

private:
  const bool draw_sphere;

  const camera::GenericCameraBase::ConstPtr proj;
  const std::vector<VisualLiDARData::ConstPtr> dataset;

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
