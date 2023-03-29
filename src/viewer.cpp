#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/console_colors.hpp>
#include <vlcal/common/visual_lidar_visualizer.hpp>

#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

class Viewer {
public:
  Viewer(const std::string& data_path) : data_path(data_path) {
    std::ifstream ifs(data_path + "/calib.json");
    if (!ifs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path << "/calib.json" << vlcal::console::reset << std::endl;
      abort();
    }

    ifs >> config;

    auto viewer = guik::LightViewer::instance();
    viewer->invoke([] {
      ImGui::SetNextWindowPos({55, 60}, ImGuiCond_Once);
      ImGui::Begin("visualizer");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 150}, ImGuiCond_Once);
      ImGui::Begin("data selection");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
      ImGui::Begin("texts");
      ImGui::End();
      ImGui::SetNextWindowPos({1260, 60}, ImGuiCond_Once);
      ImGui::Begin("images");
      ImGui::End();
    });

    const auto tum2pose = [](const std::vector<double>& values) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() << values[0], values[1], values[2];
      pose.linear() = Eigen::Quaterniond(values[6], values[3], values[4], values[5]).toRotationMatrix();
      return pose;
    };

    if (config.count("results") && config["results"].count("init_T_lidar_camera_auto")) {
      viewer->append_text("Automatic initial guess result found");
      std::cout << "Automatic initial guess result found" << std::endl;
      const std::vector<double> values = config["results"]["init_T_lidar_camera_auto"];
      T_labels.emplace_back("INIT_GUESS (AUTO)");
      T_lidar_camera.emplace_back(tum2pose(values));
      std::cout << "--- T_lidar_camera ---" << std::endl << T_lidar_camera.back().matrix() << std::endl;
    }

    if (config.count("results") && config["results"].count("init_T_lidar_camera")) {
      viewer->append_text("Manual initial guess result found");
      std::cout << "Manual initial guess result found" << std::endl;
      const std::vector<double> values = config["results"]["init_T_lidar_camera"];
      T_labels.emplace_back("INIT_GUESS (MANUAL)");
      T_lidar_camera.emplace_back(tum2pose(values));
      std::cout << "--- T_lidar_camera ---" << std::endl << T_lidar_camera.back().matrix() << std::endl;
    }

    if (config.count("results") && config["results"].count("T_lidar_camera")) {
      viewer->append_text("Calibration result found");
      std::cout << "Calibration result found" << std::endl;
      const std::vector<double> values = config["results"]["T_lidar_camera"];
      T_labels.emplace_back("CALIBRATION_RESULT");
      T_lidar_camera.emplace_back(tum2pose(values));
      std::cout << "--- T_lidar_camera ---" << std::endl << T_lidar_camera.back().matrix() << std::endl;
    }

    if (T_labels.empty()) {
      viewer->append_text("error: no transformation found in calib.json!!");
      std::cerr << vlcal::console::bold_red << "error: no transformation found in calib.json!!" << vlcal::console::reset << std::endl;
      T_labels.emplace_back("NONE");
      T_lidar_camera.emplace_back(Eigen::Isometry3d::Identity());
    }

    selected_transformation = T_labels.size() - 1;

    const std::string camera_model = config["camera"]["camera_model"];
    const std::vector<double> intrinsics = config["camera"]["intrinsics"];
    const std::vector<double> distortion_coeffs = config["camera"]["distortion_coeffs"];
    proj = camera::create_camera(camera_model, intrinsics, distortion_coeffs);

    const std::vector<std::string> bag_names = config["meta"]["bag_names"];
    for (const auto& bag_name : bag_names) {
      dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
    }

    vis.reset(new VisualLiDARVisualizer(proj, dataset, false));
    vis->set_T_camera_lidar(T_lidar_camera[selected_transformation].inverse());

    viewer->register_ui_callback("callback", [this] { ui_callback(); });
  }

  void ui_callback() {
    ImGui::Begin("data selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    std::vector<const char*> labels(T_labels.size());
    std::transform(T_labels.begin(), T_labels.end(), labels.begin(), [](const auto& x) { return x.c_str(); });

    if (ImGui::Combo("Transformation", &selected_transformation, labels.data(), labels.size())) {
      vis->set_T_camera_lidar(T_lidar_camera[selected_transformation].inverse());
    }

    ImGui::End();
  }

  void spin() {
    while (guik::LightViewer::instance()->spin_once()) {
    }
  }

private:
  const std::string data_path;
  nlohmann::json config;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;

  std::unique_ptr<VisualLiDARVisualizer> vis;

  int selected_transformation;
  std::vector<std::string> T_labels;
  std::vector<Eigen::Isometry3d> T_lidar_camera;
};

}  // namespace vlcal

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("viewer");

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
  vlcal::Viewer viewer(data_path);
  viewer.spin();
}