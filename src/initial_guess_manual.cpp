#include <atomic>
#include <thread>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>
#include <glim/util/console_colors.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/visual_lidar_visualizer.hpp>
#include <vlcal/common/estimate_pose.hpp>

namespace vlcal {

class InitialGuessManual {
public:
  InitialGuessManual(const std::string& data_path) : data_path(data_path) {
    std::ifstream ifs(data_path + "/calib.json");
    if (!ifs) {
      std::cerr << glim::console::bold_red << "error: failed to open " << data_path << "/calib.json" << glim::console::reset << std::endl;
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
    }

    const auto image_size = dataset[0]->image.size();
    std::cout << "camera_fov:" << estimate_camera_fov(proj, {image_size.width, image_size.height}) * 180.0 / M_PI << "[deg]" << std::endl;
  }

  void spin() {
    VisualLiDARVisualizer vis(proj, dataset, true);

    const std::string camera_model = config["camera"]["camera_model"];
    Eigen::Isometry3d init_T_lidar_camera = Eigen::Isometry3d::Identity();
    if (camera_model != "equirectangular") {
      init_T_lidar_camera.linear() = (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    } else {
      init_T_lidar_camera.linear() = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) ).toRotationMatrix();
    }

    auto viewer = guik::LightViewer::instance();
    guik::ModelControl T_lidar_camera_gizmo("T_lidar_camera", init_T_lidar_camera.matrix().cast<float>());
    viewer->register_ui_callback("gizmo", [&] {
      ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

      T_lidar_camera_gizmo.draw_gizmo();
      T_lidar_camera_gizmo.draw_gizmo_ui();

      vis.set_T_camera_lidar(Eigen::Isometry3d(T_lidar_camera_gizmo.model_matrix().cast<double>().inverse()));

      if (ImGui::Button("save")) {
        const Eigen::Isometry3d T_lidar_camera(T_lidar_camera_gizmo.model_matrix().cast<double>());
        const Eigen::Vector3d trans = T_lidar_camera.translation();
        const Eigen::Quaterniond quat(T_lidar_camera.linear());

        const std::vector<double> values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};
        config["results"]["init_T_lidar_camera"] = values;

        std::ofstream ofs(data_path + "/calib.json");
        if (!ofs) {
          std::cerr << glim::console::bold_red << "error: failed to open " << data_path + "/calib.json"
                    << " for writing" << glim::console::reset << std::endl;
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

    while (vis.spin_once()) {
    }
  }

private:
  const std::string data_path;
  nlohmann::json config;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;
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