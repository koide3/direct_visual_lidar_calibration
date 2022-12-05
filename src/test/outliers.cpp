#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <gtsam/geometry/Pose3.h>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_ext/util/expressions.hpp>

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

class OutliersViewer {
public:
  OutliersViewer(const std::string& data_path) : data_path(data_path) {
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

    const std::vector<double> T_lidar_camera_values = config["results"]["T_lidar_camera"];
    Eigen::Isometry3d T_lidar_camera = Eigen::Isometry3d::Identity();
    T_lidar_camera.translation() << T_lidar_camera_values[0], T_lidar_camera_values[1], T_lidar_camera_values[2];
    T_lidar_camera.linear() = Eigen::Quaterniond(T_lidar_camera_values[6], T_lidar_camera_values[3], T_lidar_camera_values[4], T_lidar_camera_values[5]).toRotationMatrix();
    const Eigen::Isometry3d T_camera_lidar = T_lidar_camera.inverse();

    std::vector<cv::Mat> lidar_images;
    for (const auto& bag_name : bag_names) {
      lidar_images.emplace_back(cv::imread(data_path + "/" + bag_name + "_lidar_intensities.png", 0));
    }

    cv::Mat camera_image, lidar_image;
    cv::cvtColor(dataset.front()->image, camera_image, cv::COLOR_GRAY2BGR);
    cv::cvtColor(lidar_images.front(), lidar_image, cv::COLOR_GRAY2BGR);
    // cv::rotate(camera_image.clone(), camera_image, cv::ROTATE_90_CLOCKWISE);

    double sx = camera_image.cols / static_cast<double>(lidar_image.cols);
    double sy = camera_image.rows / static_cast<double>(lidar_image.rows);
    cv::resize(lidar_image.clone(), lidar_image, cv::Size(camera_image.cols, camera_image.rows));

    cv::Mat canvas(camera_image.rows, camera_image.cols * 2, CV_8UC3);
    camera_image.copyTo(cv::Mat(canvas, cv::Rect(0, 0, camera_image.cols, camera_image.rows)));
    lidar_image.copyTo(cv::Mat(canvas, cv::Rect(camera_image.cols, 0, camera_image.cols, camera_image.rows)));

    for (const auto& [pt_2d_camera, pt_2d_lidar, pt_3d] : correspondences) {
      const Eigen::Vector2d projected = proj->project((T_camera_lidar * pt_3d).head<3>());
      const double error = (projected - pt_2d_camera).norm();

      const double p = std::min(error / 20.0, 1.0);
      const cv::Scalar color(0.0, 255.0 * (1 - p), 255.0 * p);

      // const cv::Point2d pt_c(camera_image.cols - pt_2d_camera.y(), pt_2d_camera.x());
      const cv::Point2d pt_c(pt_2d_camera.x(), pt_2d_camera.y());
      const cv::Point2d pt_l(camera_image.cols + sx * pt_2d_lidar.x(), sy + pt_2d_lidar.y());

      cv::circle(canvas, pt_c, 5, cv::Scalar::all(255));
      cv::circle(canvas, pt_l, 5, cv::Scalar::all(255));
      cv::line(canvas, pt_c, pt_l, color, 2);
    }

    cv::imwrite("/home/koide/corrs.jpg", canvas);
    cv::imshow("canvas", canvas);
    cv::waitKey(0);
  }

  std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector4d>>
  read_correspondences(const std::string& data_path, const std::string& bag_name, const gtsam_ext::Frame::ConstPtr& points) {
    cv::Mat point_indices_8uc4 = cv::imread(data_path + "/" + bag_name + "_lidar_indices.png", -1);
    cv::Mat point_indices = cv::Mat(point_indices_8uc4.rows, point_indices_8uc4.cols, CV_32SC1, reinterpret_cast<int*>(point_indices_8uc4.data));

    std::ifstream matches_ifs(data_path + "/" + bag_name + "_matches.json");
    if (!matches_ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/" + bag_name + "_matches.json" << vlcal::console::reset << std::endl;
      abort();
    }

    nlohmann::json matching_result;
    matches_ifs >> matching_result;

    std::vector<int> kpts0 = matching_result["kpts0"];
    std::vector<int> kpts1 = matching_result["kpts1"];
    std::vector<int> matches = matching_result["matches"];
    std::vector<double> confidence = matching_result["confidence"];

    std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector4d>> correspondences;
    for (int i = 0; i < matches.size(); i++) {
      if (matches[i] < 0) {
        continue;
      }

      const Eigen::Vector2i kp0(kpts0[2 * i], kpts0[2 * i + 1]);
      const Eigen::Vector2i kp1(kpts1[2 * matches[i]], kpts1[2 * matches[i] + 1]);

      std::int32_t point_index = point_indices.at<std::int32_t>(kp1.y(), kp1.x());

      const int pick_window_size = 2;
      if (point_index < 0) {
        for (const auto& offset : pick_offsets) {
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

      correspondences.emplace_back(kp0.cast<double>(), kp1.cast<double>(), points->points[point_index]);
    }

    return correspondences;
  }


private:
  const std::string data_path;
  nlohmann::json config;

  std::vector<Eigen::Vector2i> pick_offsets;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;
  std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d,  Eigen::Vector4d>> correspondences;
};
}  // namespace vlcal

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("outliers_viewer");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains preprocessed data");
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
  vlcal::OutliersViewer viewer(data_path);

  return 0;
}