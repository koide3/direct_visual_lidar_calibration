#include <atomic>
#include <thread>
#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>

#include <gtsam/geometry/Pose3.h>

#include <dfo/nelder_mead.hpp>
#include <dfo/directional_direct_search.hpp>

#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/costs/nid_cost.hpp>
#include <vlcal/common/console_colors.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/points_color_updater.hpp>
#include <vlcal/common/visual_lidar_visualizer.hpp>
#include <vlcal/calib/visual_camera_calibration.hpp>

namespace vlcal {

class VisualLiDARCalibration {
public:
  VisualLiDARCalibration(const std::string& data_path, const boost::program_options::variables_map& vm) : data_path(data_path) {
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

    std::vector<std::string> bag_names = config["meta"]["bag_names"];
    for (const auto& bag_name : bag_names) {
      dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
    }

    if (vm.count("first_n_bags")) {
      const int first_n_bags = vm["first_n_bags"].as<int>();
      dataset.erase(dataset.begin() + first_n_bags, dataset.end());
      std::cout << "use only the first " << first_n_bags << " bags" << std::endl;
    }
  }

  void calibrate(const boost::program_options::variables_map& vm) {
    std::vector<double> init_values;
    if (config.count("results") && config["results"].count("init_T_lidar_camera")) {
      std::cout << "use manually estimated initial guess" << std::endl;
      const std::vector<double> values = config["results"]["init_T_lidar_camera"];
      init_values.assign(values.begin(), values.end());
    } else if (config.count("results") && config["results"].count("init_T_lidar_camera_auto")) {
      std::cout << "use automatically estimated initial guess" << std::endl;
      const std::vector<double> values = config["results"]["init_T_lidar_camera_auto"];
      init_values.assign(values.begin(), values.end());
    }

    if (init_values.empty()) {
      std::cerr << vlcal::console::bold_red << "error: initial guess of T_lidar_camera must be computed before calibration!!" << vlcal::console::reset << std::endl;
      abort();
    }

    Eigen::Isometry3d init_T_lidar_camera = Eigen::Isometry3d::Identity();
    init_T_lidar_camera.translation() << init_values[0], init_values[1], init_values[2];
    init_T_lidar_camera.linear() = Eigen::Quaterniond(init_values[6], init_values[3], init_values[4], init_values[5]).normalized().toRotationMatrix();

    const Eigen::Isometry3d init_T_camera_lidar = init_T_lidar_camera.inverse();

    auto viewer = guik::LightViewer::instance(Eigen::Vector2i(-1, -1), vm.count("background"));
    viewer->set_draw_xy_grid(false);
    viewer->use_arcball_camera_control();

    viewer->invoke([] {
      ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
      ImGui::Begin("texts");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 60}, ImGuiCond_Once);
      ImGui::Begin("visualizer");
      ImGui::End();
      ImGui::SetNextWindowPos({1260, 60}, ImGuiCond_Once);
      ImGui::Begin("images");
      ImGui::End();
    });

    VisualLiDARVisualizer vis(proj, dataset, false);
    vis.set_T_camera_lidar(init_T_camera_lidar);

    VisualCameraCalibrationParams params;
    params.disable_z_buffer_culling = vm.count("disable_culling");
    params.nid_bins = vm["nid_bins"].as<int>();
    params.nelder_mead_init_step = vm["nelder_mead_init_step"].as<double>();
    params.nelder_mead_convergence_criteria = vm["nelder_mead_convergence_criteria"].as<double>();

    const std::string registration_type = vm["registration_type"].as<std::string>();
    if (registration_type == "nid_bfgs") {
      params.registration_type = RegistrationType::NID_BFGS;
    } else if (registration_type == "nid_nelder_mead") {
      params.registration_type = RegistrationType::NID_NELDER_MEAD;
    } else {
      std::cerr << vlcal::console::bold_yellow << "warning: unknown registration type " << registration_type << vlcal::console::reset << std::endl;
    }

    params.callback = [&](const Eigen::Isometry3d& T_camera_lidar) { vis.set_T_camera_lidar(T_camera_lidar); };
    VisualCameraCalibration calib(proj, dataset, params);

    std::atomic_bool optimization_terminated = false;
    Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar;
    std::thread optimization_thread([&] {
      T_camera_lidar = calib.calibrate(init_T_camera_lidar);
      optimization_terminated = true;
    });

    while (!optimization_terminated) {
      vis.spin_once();
    }

    optimization_thread.join();

    const Eigen::Isometry3d T_lidar_camera = T_camera_lidar.inverse();
    const Eigen::Vector3d trans(T_lidar_camera.translation());
    const Eigen::Quaterniond quat(T_lidar_camera.linear());

    const std::vector<double> T_lidar_camera_values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};
    config["results"]["T_lidar_camera"] = T_lidar_camera_values;

    std::ofstream ofs(data_path + "/calib.json");
    if (!ofs) {
      std::cerr << vlcal::console::bold_red << "error: failed to open " << data_path + "/calib.json"
                << "for writing" << vlcal::console::reset << std::endl;
    }
    ofs << config.dump(2) << std::endl;

    std::stringstream sst;
    sst << "--- T_lidar_camera ---" << std::endl;
    sst << T_lidar_camera.matrix() << std::endl;
    sst << "saved to " << data_path + "/calib.json";

    viewer->append_text(sst.str());
    viewer->spin_once();

    if (!vm.count("auto_quit")) {
      viewer->spin();
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
  options_description description("calibrate");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains preprocessed data")
    ("first_n_bags", value<int>(), "use only the first N bags (just for evaluation)")
    ("disable_culling", "disable depth buffer-based hidden points removal")
    ("nid_bins", value<int>()->default_value(16), "Number of histogram bins for NID")
    ("registration_type", value<std::string>()->default_value("nid_bfgs"), "nid_bfgs or nid_nelder_mead")
    ("nelder_mead_init_step", value<double>()->default_value(1e-3), "Nelder-mead initial step size")
    ("nelder_mead_convergence_criteria", value<double>()->default_value(1e-8), "Nelder-mead convergence criteria")
    ("auto_quit", "automatically quit after calibration")
    ("background", "hide viewer and run calibration in background")
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

  vlcal::VisualLiDARCalibration calib(data_path, vm);
  calib.calibrate(vm);

  return 0;
}