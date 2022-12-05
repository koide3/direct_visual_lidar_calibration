#include <iostream>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <glim/util/ros_cloud_converter.hpp>

#include <nlohmann/json.hpp>
#include <camera/create_camera.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/estimate_pose.hpp>
#include <vlcal/common/points_color_updater.hpp>

#include <glk/texture_opencv.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <vlcal/common/console_colors.hpp>

#include <test/detect_sphere.hpp>

namespace vlcal {

void mouse_callback(int event, int x, int y, int flags, void* userdata);

class ReferenceCalibration {
public:
  ReferenceCalibration(const std::string& data_path) : data_path(data_path) {
    std::ifstream ifs(data_path + "/calib.json");
    if (!ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path << "/calib.json" << vlcal::console::reset << std::endl;
      abort();
    }

    ifs >> config;

    const std::string camera_model = config["camera"]["camera_model"];
    const std::vector<double> intrinsics = config["camera"]["intrinsics"];
    const std::vector<double> distortion_coeffs = config["camera"]["distortion_coeffs"];
    proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);

    const std::vector<std::string> bag_names = config["meta"]["bag_names"];
    for (const auto& bag_name : bag_names) {
      dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));

      auto mask_image = cv::imread(data_path + "/" + bag_name + "_mask.png");
      cv::Mat mask(mask_image.rows, mask_image.cols, CV_8UC1, cv::Scalar::all(0));
      for (int i = 0; i < mask_image.rows; i++) {
        for (int j = 0; j < mask_image.cols; j++) {
          const auto& bgr = mask_image.at<cv::Vec3b>(i, j);

          if(bgr[0] < 100 && bgr[1] > 150 && bgr[2] < 100) {
            mask.at<std::uint8_t>(i, j) = 255;
          }
        }
      }

      cv::Mat labels;
      cv::Mat stats;
      cv::Mat centroids;
      cv::connectedComponentsWithStats(mask, labels, stats, centroids);

      std::vector<Eigen::Vector2d> pts(centroids.rows);
      for (int i = 0; i < centroids.rows; i++) {
        pts[i].x() = centroids.at<double>(i, 0);
        pts[i].y() = centroids.at<double>(i, 1);
      }
      this->centroids.emplace_back(pts);
    }

    selected_data_id = -1;

    blend_weight = 0.8f;

    auto viewer = guik::LightViewer::instance();
    viewer->register_ui_callback("ui", [this] { ui_callback(); });
  }

  void ui_callback() {
    const int prev_selected_data_id = selected_data_id;

    selected_data_id -= ImGui::ArrowButton("##LEFT", ImGuiDir_Left);
    ImGui::SameLine();
    selected_data_id += ImGui::ArrowButton("##RIGHT", ImGuiDir_Right);
    ImGui::SameLine();
    ImGui::DragInt("selected", &selected_data_id, 1, 0, dataset.size() - 1);
    selected_data_id = std::max<int>(0, std::min<int>(dataset.size() - 1, selected_data_id));

    auto viewer = guik::LightViewer::instance();
    if (selected_data_id != prev_selected_data_id) {
      const auto& data = dataset[selected_data_id];
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(data->points->points, data->points->size());
      cloud_buffer->add_intensity(glk::COLORMAP::TURBO, data->points->intensities, data->points->size());
      viewer->update_drawable("points", cloud_buffer, guik::VertexColor());

      cv::imshow("image", data->image);
      cv::setMouseCallback("image", &mouse_callback, this);
    }

    if (ImGui::Button("Add")) {
      std::stringstream sst;
      sst << "Add " << sphere_2d.transpose() << " and " << sphere_3d.transpose();
      viewer->append_text(sst.str());

      correspondences.emplace_back(sphere_2d, sphere_3d);
    }

    if(ImGui::Button("Pop")) {
      viewer->append_text("Popped");
      correspondences.pop_back();
    }

    if (ImGui::Button("Calibrate")) {
      PoseEstimationParams params;
      PoseEstimation estimation(params);

      T_camera_lidar = estimation.estimate(proj, correspondences);

      const auto& data = dataset[selected_data_id];
      color_updater.reset(new PointsColorUpdater(proj, data->image, data->points));
      color_updater->update(T_camera_lidar, blend_weight);

      auto sub = viewer->sub_viewer("colored");
      sub->update_drawable("points", color_updater->cloud_buffer, guik::VertexColor());

      std::cout << "--- T_camera_lidar ---" << std::endl << T_camera_lidar.matrix() << std::endl;
    }

    if (ImGui::Button("save")) {
      const Eigen::Isometry3d T_lidar_camera = T_camera_lidar.inverse();
      const Eigen::Vector3d trans = T_lidar_camera.translation();
      const Eigen::Quaterniond quat(T_lidar_camera.linear());

      const std::vector<double> values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};
      config["results"]["T_lidar_camera"] = values;

      std::ofstream ofs(data_path + "/calib.json");
      ofs << config.dump(2) << std::endl;
      viewer->append_text("Saved!!");
    }

    if (color_updater) {
      if(ImGui::DragFloat("blend_weight", &blend_weight, 0.01f, 0.0f, 1.0f)) {
        color_updater->update(T_camera_lidar, blend_weight);
      }
    }

    auto& io = ImGui::GetIO();
    if (!io.WantCaptureMouse && io.MouseClicked[1]) {
      const auto mouse_pos = io.MousePos;
      const float depth = viewer->pick_depth({mouse_pos.x, mouse_pos.y});
      if(depth < 1.0f && depth > -1.0f) {
        const Eigen::Vector3f pt = viewer->unproject({mouse_pos.x, mouse_pos.y}, depth);
        detect_sphere(pt);
      }
    }
  }

  void on_mouse(int event, int x, int y) {
    if(event != cv::EVENT_RBUTTONDOWN) {
      return;
    }

    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d closest_centroid;
    for (const auto& centroid : centroids[selected_data_id]) {
      const double dist = (centroid - Eigen::Vector2d(x, y)).norm();
      if(dist < min_dist) {
        min_dist = dist;
        closest_centroid = centroid;
      }
    }

    sphere_2d = closest_centroid;

    cv::Mat canvas;
    cv::cvtColor(dataset[selected_data_id]->image, canvas, cv::COLOR_GRAY2BGR);
    cv::circle(canvas, cv::Point(closest_centroid.x(), closest_centroid.y()), 10, cv::Scalar(0, 255, 0));
    cv::imshow("image", canvas);
  }

  void spin() {
    while (guik::LightViewer::instance()->spin_once()) {
      cv::waitKey(1);
    }
  }

private:
  void detect_sphere(const Eigen::Vector3f& clicked_pt_) {
    const Eigen::Vector3d clicked_pt = clicked_pt_.cast<double>();
    const auto filtered = gtsam_ext::filter(dataset[selected_data_id]->points, [&](const Eigen::Vector4d& p) { return (p.head<3>() - clicked_pt).norm() < 0.2; });
    const auto coeffs = detect_sphere_ransac(filtered);

    auto viewer = guik::LightViewer::instance();
    viewer->update_drawable("selected", std::make_shared<glk::PointCloudBuffer>(filtered->points, filtered->size()), guik::FlatOrange().add("point_scale", 2.0f));
    viewer->update_drawable(
      "sphere",
      glk::Primitives::sphere(),
      guik::FlatColor(1.0f, 0.0f, 0.0f, 0.5f).translate(coeffs.head<3>().cast<float>()).scale(coeffs.w()).make_transparent());

    sphere_3d << coeffs.head<3>(), 1.0;
  }

private:
  const std::string data_path;
  nlohmann::json config;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;
  std::vector<std::vector<Eigen::Vector2d>> centroids;

  int selected_data_id;

  Eigen::Vector4d sphere_3d;
  Eigen::Vector2d sphere_2d;

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;

  float blend_weight;
  Eigen::Isometry3d T_camera_lidar;
  std::unique_ptr<PointsColorUpdater> color_updater;
};

void mouse_callback(int event, int x, int y, int flags, void* userdata) {
  auto calib = reinterpret_cast<ReferenceCalibration*>(userdata);
  calib->on_mouse(event, x, y);
}

}  // namespace vlcal

int main(int argc, char** argv) {
  vlcal::ReferenceCalibration calib(argv[1]);
  calib.spin();

  return 0;
}