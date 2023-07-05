#include <atomic>
#include <thread>
#include <fstream>
#include <iostream>
#include <boost/format.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>

#include <glk/primitives/primitives.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/console_colors.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/visual_lidar_visualizer.hpp>
#include <vlcal/common/estimate_pose.hpp>

namespace vlcal {

struct PickingPoseEstimation {
public:
  PickingPoseEstimation(const camera::GenericCameraBase::ConstPtr& proj) : proj(proj) {}
  ~PickingPoseEstimation() {}

  void pick_point_2d(const Eigen::Vector2d& pt) {
    guik::LightViewer::instance()->append_text((boost::format("picked_2d: %.1f %.1f") % pt.x() % pt.y()).str());
    picked_pt_2d = pt;
  }

  void pick_point_3d(const Eigen::Vector4d& pt) {
    guik::LightViewer::instance()->append_text((boost::format("picked_3d: %.1f %.1f %.1f") % pt.x() % pt.y() % pt.z()).str());
    picked_pt_3d = pt;
  }

  void add() {
    if (!picked_pt_2d || !picked_pt_3d) {
      guik::LightViewer::instance()->append_text("2D/3D points are not picked yet");
      return;
    }

    guik::LightViewer::instance()->append_text("2D/3D correspondence added");
    correspondences.emplace_back(*picked_pt_2d, *picked_pt_3d);
    picked_pt_2d = std::nullopt;
    picked_pt_3d = std::nullopt;
  }

  std::optional<Eigen::Isometry3d> estimate() {
    if (correspondences.size() < 3) {
      guik::LightViewer::instance()->append_text("At least 3 correspondences are necessary!!");
      return std::nullopt;
    }

    PoseEstimationParams params;
    PoseEstimation est(params);
    return est.estimate(proj, correspondences);
  }

public:
  const camera::GenericCameraBase::ConstPtr proj;

  std::optional<Eigen::Vector2d> picked_pt_2d;
  std::optional<Eigen::Vector4d> picked_pt_3d;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> correspondences;
};

class InitialGuessManual {
public:
  InitialGuessManual(const std::string& data_path) : data_path(data_path), camera_image_rotate_code(-1) {
    std::ifstream ifs(data_path + "/calib.json");
    if (!ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path << "/calib.json" << vlcal::console::reset << std::endl;
      abort();
    }

    ifs >> config;

    const std::string camera_model = config["camera"]["camera_model"];
    const std::vector<double> intrinsics = config["camera"]["intrinsics"];
    const std::vector<double> distortion_coeffs = config["camera"]["distortion_coeffs"];

    std::cout << "camera_model:" << camera_model << std::endl;
    std::cout << "intrinsics  :" << intrinsics.size() << std::endl;
    std::cout << "distortion  :" << distortion_coeffs.size() << std::endl;

    proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);

    const std::vector<std::string> bag_names = config["meta"]["bag_names"];
    for (const auto& bag_name : bag_names) {
      dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
    }

    const auto image_size = dataset[0]->image.size();
    std::cout << "image_size:" << image_size.width << "x" << image_size.height << std::endl;
    std::cout << "camera_fov:" << estimate_camera_fov(proj, {image_size.width, image_size.height}) * 180.0 / M_PI << "[deg]" << std::endl;

    vis.reset(new VisualLiDARVisualizer(proj, dataset, true, true));
    vis->set_blend_weight(0.1f);

    picking.reset(new PickingPoseEstimation(proj));

    auto viewer = guik::LightViewer::instance();
    viewer->invoke([] {
      ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
      ImGui::Begin("texts");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 60}, ImGuiCond_Once);
      ImGui::Begin("visualizer");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 150}, ImGuiCond_Once);
      ImGui::Begin("control");
      ImGui::End();
    });

    cv::namedWindow("image");
    cv::setMouseCallback("image", &InitialGuessManual::mouse_callback, this);
  }

  void spin() {
    const std::string camera_model = config["camera"]["camera_model"];
    Eigen::Isometry3d init_T_lidar_camera = Eigen::Isometry3d::Identity();
    if (camera_model != "equirectangular") {
      init_T_lidar_camera.linear() = (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    } else {
      init_T_lidar_camera.linear() = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
    }

    auto viewer = guik::LightViewer::instance();
    guik::ModelControl T_lidar_camera_gizmo("T_lidar_camera", init_T_lidar_camera.matrix().cast<float>());
    viewer->register_ui_callback("gizmo", [&] {
      auto& io = ImGui::GetIO();
      if (!io.WantCaptureMouse && io.MouseClicked[1]) {
        const float depth = viewer->pick_depth({io.MousePos[0], io.MousePos[1]});
        if (depth > -1.0f && depth < 1.0f) {
          const Eigen::Vector3f pt_3d = viewer->unproject({io.MousePos[0], io.MousePos[1]}, depth);
          picking->pick_point_3d(Eigen::Vector4d(pt_3d.x(), pt_3d.y(), pt_3d.z(), 1.0));

          guik::HoveredDrawings hovered;
          hovered.add_cross(pt_3d, IM_COL32(64, 64, 64, 255), 15.0f, 4.0f);
          hovered.add_cross(pt_3d, IM_COL32(0, 255, 0, 255), 15.0f, 3.0f);
          viewer->register_ui_callback("hovered", hovered.create_callback());
        }
      }

      ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

      ImGui::Text("Rotate camera image");
      if (ImGui::Button("0 deg")) {
        camera_image_rotate_code = -1;
        update_image();
      }
      ImGui::SameLine();
      if (ImGui::Button("90 deg")) {
        camera_image_rotate_code = cv::ROTATE_90_CLOCKWISE;
        update_image();
      }
      ImGui::SameLine();
      if (ImGui::Button("180 deg")) {
        camera_image_rotate_code = cv::ROTATE_180;
        update_image();
      }
      ImGui::SameLine();
      if (ImGui::Button("270 deg")) {
        camera_image_rotate_code = cv::ROTATE_90_COUNTERCLOCKWISE;
        update_image();
      }

      ImGui::Separator();

      ImGui::Text("T_lidar_camera");
      T_lidar_camera_gizmo.draw_gizmo();
      T_lidar_camera_gizmo.draw_gizmo_ui();
      vis->set_T_camera_lidar(Eigen::Isometry3d(T_lidar_camera_gizmo.model_matrix().cast<double>().inverse()));

      ImGui::Separator();

      if (ImGui::Button("Add picked points")) {
        picking->add();
      }

      if (ImGui::Button("Estimate")) {
        const auto T_camera_lidar = picking->estimate();
        if (T_camera_lidar) {
          T_lidar_camera_gizmo.set_model_matrix(T_camera_lidar->inverse().matrix().cast<float>());
        }
      }

      ImGui::SameLine();
      if (ImGui::Button("Save")) {
        const Eigen::Isometry3d T_lidar_camera(T_lidar_camera_gizmo.model_matrix().cast<double>());
        const Eigen::Vector3d trans = T_lidar_camera.translation();
        const Eigen::Quaterniond quat(T_lidar_camera.linear());

        const std::vector<double> values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};
        config["results"]["init_T_lidar_camera"] = values;

        std::ofstream ofs(data_path + "/calib.json");
        if (!ofs) {
          std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/calib.json"
                    << " for writing" << vlcal::console::reset << std::endl;
        } else {
          ofs << config.dump(2) << std::endl;
        }

        std::stringstream sst;
        sst << "--- T_lidar_camera ---" << std::endl;
        sst << T_lidar_camera.matrix() << std::endl;
        sst << "saved to " << data_path + "/calib.json";

        viewer->append_text(sst.str());
      }

      ImGui::End();
    });

    while (vis->spin_once()) {
      cv::waitKey(1);
    }
  }

  void update_image() {
    const double scale = vis->get_image_display_scale();

    cv::Mat resized, canvas;
    cv::resize(dataset[vis->get_selected_bag_id()]->image, resized, cv::Size(), scale, scale);
    cv::cvtColor(resized, canvas, cv::COLOR_GRAY2BGR);

    if (camera_image_rotate_code >= 0) {
      cv::rotate(canvas.clone(), canvas, camera_image_rotate_code);
    }

    cv::imshow("image", canvas);
  }

  Eigen::Vector2d transform_camera_point(const Eigen::Vector2d& pt) {
    const cv::Size image_size = dataset[vis->get_selected_bag_id()]->image.size();

    std::cout << "image_size=" << image_size << std::endl;
    switch (camera_image_rotate_code) {
      case -1:
        return pt;
      case cv::ROTATE_90_CLOCKWISE:
        return {pt.y(), image_size.height - pt.x()};
      case cv::ROTATE_180:
        return {image_size.width - pt.x(), image_size.height - pt.y()};
      case cv::ROTATE_90_COUNTERCLOCKWISE:
        return {image_size.width - pt.y(), pt.x()};
    }

    std::cerr << "error: unknown rotate code " << camera_image_rotate_code << std::endl;
    return pt;
  }

  void on_mouse(int event, int x, int y, int flags) {
    if (event != cv::EVENT_RBUTTONDOWN) {
      return;
    }

    const double scale = vis->get_image_display_scale();

    cv::Mat resized, canvas;
    cv::resize(dataset[vis->get_selected_bag_id()]->image, resized, cv::Size(), scale, scale);
    cv::cvtColor(resized, canvas, cv::COLOR_GRAY2BGR);

    if (camera_image_rotate_code >= 0) {
      cv::rotate(canvas.clone(), canvas, camera_image_rotate_code);
    }

    const int cross_size = 15;
    cv::line(canvas, {x - cross_size, y - cross_size}, {x + cross_size, y + cross_size}, cv::Scalar(64, 64, 64), 4, cv::LINE_AA);
    cv::line(canvas, {x + cross_size, y - cross_size}, {x - cross_size, y + cross_size}, cv::Scalar(64, 64, 64), 4, cv::LINE_AA);
    cv::line(canvas, {x - cross_size, y - cross_size}, {x + cross_size, y + cross_size}, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(canvas, {x + cross_size, y - cross_size}, {x - cross_size, y + cross_size}, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::imshow("image", canvas);

    const Eigen::Vector2d pt = transform_camera_point({x / scale, y / scale});
    picking->pick_point_2d(pt);
  }

  static void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    InitialGuessManual* inst = reinterpret_cast<InitialGuessManual*>(userdata);
    inst->on_mouse(event, x, y, flags);
  }

private:
  const std::string data_path;
  nlohmann::json config;

  int camera_image_rotate_code;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;

  std::unique_ptr<VisualLiDARVisualizer> vis;
  std::unique_ptr<PickingPoseEstimation> picking;
};

}  // namespace vlcal

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("initial_guess_manual");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains preprocessed data")
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
  vlcal::InitialGuessManual init_guess(data_path);
  init_guess.spin();

  return 0;
}