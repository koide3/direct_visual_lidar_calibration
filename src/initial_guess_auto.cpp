#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose3.h>

#include <dfo/nelder_mead.hpp>
#include <vlcal/common/console_colors.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/common/estimate_pose.hpp>
#include <vlcal/common/visual_lidar_data.hpp>

#include <glk/primitives/primitives.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

class InitialGuessAuto {
public:
  InitialGuessAuto(const std::string& data_path) : data_path(data_path) {
    std::ifstream ifs(data_path + "/calib.json");
    if (!ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path << "/calib.json" << vlcal::console::reset << std::endl;
      abort();
    }

    const int pick_window_size = 1;
    for (int i = -pick_window_size; i <= pick_window_size; i++) {
      for (int j = -pick_window_size; j <= pick_window_size; j++) {
        if (i == 0 && j == 0) {
          continue;
        }
        pick_offsets.emplace_back(i, j);
      }
    }
    std::sort(pick_offsets.begin(), pick_offsets.end(), [](const auto& lhs, const auto& rhs) { return lhs.squaredNorm() < rhs.squaredNorm(); });

    ifs >> config;

    const std::string camera_model = config["camera"]["camera_model"];
    const std::vector<double> intrinsics = config["camera"]["intrinsics"];
    const std::vector<double> distortion_coeffs = config["camera"]["distortion_coeffs"];
    proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);

    const std::vector<std::string> bag_names = config["meta"]["bag_names"];
    for (const auto& bag_name : bag_names) {
      dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
      auto corrs = read_correspondences(data_path, bag_name, dataset.back()->points);
      correspondences.insert(correspondences.end(), corrs.begin(), corrs.end());
    }
  }

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>
  read_correspondences(const std::string& data_path, const std::string& bag_name, const Frame::ConstPtr& points) {
    cv::Mat point_indices_8uc4 = cv::imread(data_path + "/" + bag_name + "_lidar_indices.png", -1);
    cv::Mat point_indices = cv::Mat(point_indices_8uc4.rows, point_indices_8uc4.cols, CV_32SC1, reinterpret_cast<int*>(point_indices_8uc4.data));

    std::ifstream matches_ifs(data_path + "/" + bag_name + "_matches.json");
    if(!matches_ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/" + bag_name + "_matches.json" << vlcal::console::reset << std::endl;
      abort();
    }

    nlohmann::json matching_result;
    matches_ifs >> matching_result;

    std::vector<int> kpts0 = matching_result["kpts0"];
    std::vector<int> kpts1 = matching_result["kpts1"];
    std::vector<int> matches = matching_result["matches"];
    std::vector<double> confidence = matching_result["confidence"];

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
    for (int i = 0; i < matches.size(); i++) {
      if (matches[i] < 0) {
        continue;
      }

      const Eigen::Vector2i kp0(kpts0[2 * i], kpts0[2 * i + 1]);
      const Eigen::Vector2i kp1(kpts1[2 * matches[i]], kpts1[2 * matches[i] + 1]);

      std::int32_t point_index = point_indices.at<std::int32_t>(kp1.y(), kp1.x());

      const int pick_window_size = 2;
      if(point_index < 0) {
        for(const auto& offset : pick_offsets) {
          point_index = point_indices.at<std::int32_t>(kp1.y() + offset.y(), kp1.x() + offset.x());

          if (point_index >= 0) {
            break;
          }
        }

        if (point_index < 0) {
          std::cerr << vlcal::console::bold_yellow << "warning: ignore keypoint in a blank region!!" << vlcal::console::reset << std::endl;
        }
        continue;
      }

      correspondences.emplace_back(kp0.cast<double>(), points->points[point_index]);
    }

    return correspondences;
  }

  void estimate_and_save(const boost::program_options::variables_map& vm) {
    PoseEstimationParams params;
    params.ransac_iterations = vm["ransac_iterations"].as<int>();
    params.ransac_error_thresh = vm["ransac_error_thresh"].as<double>();
    params.robust_kernel_width = vm["robust_kernel_width"].as<double>();

    PoseEstimation pose_estimation(params);

    std::vector<bool> inliers;
    Eigen::Isometry3d T_camera_lidar = pose_estimation.estimate(proj, correspondences, &inliers);

    const Eigen::Isometry3d T_lidar_camera = T_camera_lidar.inverse();
    const Eigen::Vector3d trans = T_lidar_camera.translation();
    const Eigen::Quaterniond quat = Eigen::Quaterniond(T_lidar_camera.linear()).normalized();
    const std::vector<double> values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};

    config["results"]["init_T_lidar_camera_auto"] = values;

    std::ofstream ofs(data_path + "/calib.json");
    if(!ofs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/calib.json"
                << " for writing" << vlcal::console::reset << std::endl;
      return;
    }

    ofs << config.dump(2) << std::endl;
  }

private:
  const std::string data_path;
  nlohmann::json config;

  std::vector<Eigen::Vector2i> pick_offsets;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
};
}

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("initial_guess_auto");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains preprocessed data")
    ("ransac_iterations", value<int>()->default_value(8192), "iterations for RANSAC")
    ("ransac_error_thresh", value<double>()->default_value(10.0), "reprojection error threshold [pix]")
    ("robust_kernel_width", value<double>()->default_value(10.0), "Cauchy kernel width for fine estimation [pix]")
  ;
  // clang-format on

  positional_options_description p;
  p.add("data_path", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("data_path")) {
    std::cout << description << std::endl;
    return 0;
  }

  const std::string data_path = vm["data_path"].as<std::string>();
  vlcal::InitialGuessAuto init_guess(data_path);
  init_guess.estimate_and_save(vm);

  return 0;
}