#include <atomic>
#include <thread>
#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>

#include <gtsam/geometry/Pose3.h>
#include <glim/util/console_colors.hpp>

#include <dfo/nelder_mead.hpp>
#include <dfo/directional_direct_search.hpp>

#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/points_color_updater.hpp>
#include <vlcal/calib/cost_calculator.hpp>
#include <vlcal/calib/cost_calculator_nid.hpp>
#include <vlcal/calib/cost_calculator_edge.hpp>
#include <vlcal/calib/cost_scaller.hpp>

#include <vlcal/common/visual_lidar_visualizer.hpp>

namespace vlcal {

class VisualLiDARCalibration {
public:
  VisualLiDARCalibration(const std::string& data_path) : data_path(data_path) {
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

    for (const auto& data : dataset) {
      auto nid_cost = std::make_shared<CostCalculatorNID>(proj, data);
      auto edge_cost = std::make_shared<CostCalculatorEdge>(proj, data);

      costs.emplace_back(std::make_shared<CostScaler>(nid_cost, 100.0));
      costs.emplace_back(std::make_shared<CostScaler>(edge_cost));
    }
  }

  void calibrate() {
    if (!config.count("results") || !config["results"].count("init_T_lidar_camera")) {
      std::cerr << glim::console::bold_red << "error: initial guess of T_lidar_camera must be computed before calibration!!" << glim::console::reset << std::endl;
      abort();
    }
    const std::vector<double> init_values = config["results"]["init_T_lidar_camera"];

    Eigen::Isometry3d init_T_lidar_camera = Eigen::Isometry3d::Identity();
    init_T_lidar_camera.translation() << init_values[0], init_values[1], init_values[2];
    init_T_lidar_camera.linear() = Eigen::Quaterniond(init_values[6], init_values[3], init_values[4], init_values[5]).normalized().toRotationMatrix();

    const Eigen::Isometry3d init_T_camera_lidar = init_T_lidar_camera.inverse();

    VisualLiDARVisualizer vis(proj, dataset, false);
    vis.set_T_camera_lidar(init_T_camera_lidar);

    double best_cost = std::numeric_limits<double>::max();
    Eigen::Isometry3d best_T_camera_lidar = init_T_camera_lidar;

    auto viewer = guik::LightViewer::instance();
    viewer->set_draw_xy_grid(false);
    viewer->use_arcball_camera_control();

    const auto evaluate = [&](const gtsam::Vector6& x) {
      const Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(x).matrix());

      double sum = 0.0;

// #pragma omp parallel for reduction(+ : sum)
      for (int i = 0; i < costs.size(); i++) {
        sum += costs[i]->calculate(T_camera_lidar);
      }

      if (sum < best_cost) {
        best_cost = sum;
        best_T_camera_lidar = T_camera_lidar;
        vis.set_T_camera_lidar(T_camera_lidar);
        viewer->append_text("best_cost:" + std::to_string(sum));
      }

      return sum;
    };

    const auto callback = [&](const gtsam::Vector6& x) {
      for(int i=0; i<costs.size(); i++) {
        const Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(x).matrix());
        costs[i]->update_correspondences(T_camera_lidar);
      }
    };

    // dfo::NelderMead<6>::Params params;
    // dfo::NelderMead<6> optimizer(params);

    // dfo::OptimizationResult<6> result;

    dfo::DirectionalDirectSearch<6>::Params params;
    params.init_alpha = 1e-2;
    dfo::DirectionalDirectSearch<6> optimizer(params);
    optimizer.set_callback(callback);
    dfo::OptimizationResult<6> result;

    std::atomic_bool optimization_terminated = false;
    std::thread optimization_thread([&] {
      result = optimizer.optimize(evaluate, gtsam::Vector6::Zero());
      optimization_terminated = true;
    });

    while (!optimization_terminated) {
      vis.spin_once();
    }

    optimization_thread.join();

    const Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(result.x).matrix());
    const Eigen::Isometry3d T_lidar_camera = T_camera_lidar.inverse();
    const Eigen::Vector3d trans(T_lidar_camera.translation());
    const Eigen::Quaterniond quat(T_lidar_camera.linear());

    const std::vector<double> T_lidar_camera_values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};
    config["results"]["T_lidar_camera"] = T_lidar_camera_values;

    std::ofstream ofs(data_path + "/calib.json");
    if (!ofs) {
      std::cerr << glim::console::bold_red << "error: failed to open " << data_path + "/calib.json"
                << "for writing" << glim::console::reset << std::endl;
    }
    ofs << config.dump(2) << std::endl;

    std::stringstream sst;
    sst << "--- T_lidar_camera ---" << std::endl;
    sst << T_lidar_camera.matrix() << std::endl;
    sst << "saved to " << data_path + "/calib.json";

    viewer->append_text(sst.str());
    viewer->spin();
  }

private:
  const std::string data_path;
  nlohmann::json config;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::ConstPtr> dataset;

  std::vector<CostCalculator::Ptr> costs;
};

}  // namespace vlcal

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("calibrate");

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

  vlcal::VisualLiDARCalibration calib(data_path);
  calib.calibrate();

  return 0;
}