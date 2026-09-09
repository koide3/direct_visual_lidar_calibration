#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glk/io/ply_io.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <camera/create_camera.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/visual_lidar_visualizer.hpp>
#include <vlcal/calib/visual_camera_calibration.hpp>

namespace vlcal {
namespace {

void save_config(const std::string& data_path, const nlohmann::json& config) {
  const std::string path = data_path + "/calib.json";
  const std::string temporary = path + ".tmp";
  std::ofstream output(temporary);
  if (!output) throw std::runtime_error("failed to open " + temporary + " for writing");
  output << config.dump(2) << std::endl;
  output.close();
  if (!output || std::rename(temporary.c_str(), path.c_str()) != 0) {
    throw std::runtime_error("failed to write " + path);
  }
}

nlohmann::json diagnostics_json(const VisualCameraCalibrationDiagnostics& diagnostics) {
  nlohmann::json output = {
    {"success", diagnostics.success},
    {"converged", diagnostics.converged},
    {"inner_converged", diagnostics.inner_converged},
    {"outer_iterations", diagnostics.outer_iterations},
    {"inner_iterations", diagnostics.inner_iterations},
    {"termination_reason", diagnostics.termination_reason},
    {"visible_points", diagnostics.visible_points}
  };
  output["initial_cost"] = std::isfinite(diagnostics.initial_cost) ? nlohmann::json(diagnostics.initial_cost) : nlohmann::json(nullptr);
  output["final_cost"] = std::isfinite(diagnostics.final_cost) ? nlohmann::json(diagnostics.final_cost) : nlohmann::json(nullptr);
  return output;
}

// Covers input/constructor errors as well as optimization failures. Never leave a stale successful transform.
void record_failure(const std::string& data_path, const std::string& reason) {
  try {
    std::ifstream input(data_path + "/calib.json");
    if (!input) return;
    nlohmann::json config;
    input >> config;
    if (!config.contains("results") || !config["results"].is_object()) config["results"] = nlohmann::json::object();
    config["results"].erase("T_lidar_camera");
    config["results"]["calibration_status"] = "failed";
    config["results"]["calibration_diagnostics"]["success"] = false;
    config["results"]["calibration_diagnostics"]["converged"] = false;
    config["results"]["calibration_diagnostics"]["termination_reason"] = reason;
    save_config(data_path, config);
  } catch (const std::exception& error) {
    std::cerr << "failed to save failure diagnostics: " << error.what() << std::endl;
  }
}

}  // namespace

class VisualLiDARCalibration {
public:
  VisualLiDARCalibration(const std::string& data_path, const boost::program_options::variables_map& vm) : data_path(data_path) {
    std::ifstream input(data_path + "/calib.json");
    if (!input) throw std::runtime_error("failed to open " + data_path + "/calib.json");
    input >> config;
    if (!config.contains("results")) config["results"] = nlohmann::json::object();
    if (!config["results"].is_object()) throw std::runtime_error("calib.json results must be an object");
    config["results"].erase("T_lidar_camera");
    config["results"]["calibration_status"] = "running";
    config["results"]["calibration_diagnostics"] = nlohmann::json::object();
    save_config(data_path, config);

    const auto& camera_config = config.at("camera");
    const std::string model = camera_config.at("camera_model");
    const std::vector<double> intrinsics = camera_config.at("intrinsics");
    const std::vector<double> distortion = camera_config.at("distortion_coeffs");
    cv::Mat mask;
    if (camera_config.contains("mask_path") && !camera_config.at("mask_path").is_null()) {
      const std::string mask_name = camera_config.at("mask_path");
      if (!mask_name.empty()) {
        const auto mask_path = std::filesystem::path(mask_name).is_absolute() ? std::filesystem::path(mask_name) : std::filesystem::path(data_path) / mask_name;
        mask = cv::imread(mask_path.string(), cv::IMREAD_GRAYSCALE);
        if (mask.empty()) throw std::runtime_error("failed to load camera mask: " + mask_path.string());
      }
    }
    if (camera_config.contains("max_theta_deg") || !mask.empty()) {
      camera::CameraValidity validity;
      validity.max_theta_deg = camera_config.value("max_theta_deg", 95.0);
      validity.image_mask = mask;
      proj = camera::create_camera(model, intrinsics, distortion, validity);
    } else {
      proj = camera::create_camera(model, intrinsics, distortion);
    }
    if (!proj) throw std::runtime_error("failed to create camera model: " + model);

    std::vector<std::string> bag_names = config.at("meta").at("bag_names");
    if (vm.count("first_n_bags")) {
      const int count = vm["first_n_bags"].as<int>();
      if (count <= 0 || static_cast<std::size_t>(count) > bag_names.size()) throw std::runtime_error("first_n_bags is outside the dataset range");
      bag_names.resize(count);
    }
    if (bag_names.empty()) throw std::runtime_error("calibration dataset is empty");
    for (const auto& name : bag_names) {
      const std::string prefix = data_path + "/" + name;
      std::cout << "loading " << prefix << ".(png|ply)" << std::endl;
      cv::Mat image = cv::imread(prefix + ".png", cv::IMREAD_GRAYSCALE);
      if (image.empty()) throw std::runtime_error("failed to load image: " + prefix + ".png");
      if (!mask.empty() && mask.size() != image.size()) throw std::runtime_error("camera mask dimensions differ from image: " + prefix + ".png");
      auto ply = glk::load_ply(prefix + ".ply");
      if (!ply || ply->vertices.empty() || ply->intensities.size() != ply->vertices.size()) {
        throw std::runtime_error("PLY must contain nonempty vertices and one intensity per vertex: " + prefix + ".ply");
      }
      for (std::size_t i = 0; i < ply->vertices.size(); i++) {
        if (!ply->vertices[i].allFinite() || !std::isfinite(ply->intensities[i])) {
          throw std::runtime_error("PLY contains non-finite coordinates or intensities: " + prefix + ".ply");
        }
      }
      auto points = std::make_shared<FrameCPU>(ply->vertices);
      points->add_intensities(ply->intensities);
      dataset.emplace_back(std::make_shared<VisualLiDARData>(image, points));
    }
  }

  void calibrate(const boost::program_options::variables_map& vm) {
    std::vector<double> initial;
    if (config["results"].contains("init_T_lidar_camera")) {
      initial = config["results"]["init_T_lidar_camera"].get<std::vector<double>>();
    } else if (config["results"].contains("init_T_lidar_camera_auto")) {
      initial = config["results"]["init_T_lidar_camera_auto"].get<std::vector<double>>();
    }
    if (initial.size() != 7 || !std::all_of(initial.begin(), initial.end(), [](double x) { return std::isfinite(x); })) {
      throw std::runtime_error("initial T_lidar_camera must contain 7 finite values: tx,ty,tz,qx,qy,qz,qw");
    }
    Eigen::Quaterniond quaternion(initial[6], initial[3], initial[4], initial[5]);
    if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-12) throw std::runtime_error("initial transform quaternion has invalid norm");
    Eigen::Isometry3d initial_lidar_camera = Eigen::Isometry3d::Identity();
    initial_lidar_camera.translation() << initial[0], initial[1], initial[2];
    initial_lidar_camera.linear() = quaternion.normalized().toRotationMatrix();
    const Eigen::Isometry3d initial_camera_lidar = initial_lidar_camera.inverse();

    VisualCameraCalibrationParams params;
    params.disable_z_buffer_culling = vm.count("disable_culling");
    params.nid_bins = vm["nid_bins"].as<int>();
    params.max_outer_iterations = vm["max_outer_iterations"].as<int>();
    params.max_inner_iterations = vm["max_inner_iterations"].as<int>();
    params.nelder_mead_init_step = vm["nelder_mead_init_step"].as<double>();
    params.nelder_mead_convergence_criteria = vm["nelder_mead_convergence_criteria"].as<double>();
    if (!std::isfinite(params.nelder_mead_init_step) || params.nelder_mead_init_step <= 0.0 ||
        !std::isfinite(params.nelder_mead_convergence_criteria) || params.nelder_mead_convergence_criteria <= 0.0) {
      throw std::runtime_error("Nelder-Mead step and convergence criterion must be finite and positive");
    }
    const std::string registration = vm["registration_type"].as<std::string>();
    if (registration == "nid_bfgs") params.registration_type = RegistrationType::NID_BFGS;
    else if (registration == "nid_nelder_mead") params.registration_type = RegistrationType::NID_NELDER_MEAD;
    else throw std::runtime_error("unknown registration type: " + registration);

    const bool headless = vm.count("headless");
    decltype(guik::LightViewer::instance()) viewer = nullptr;
    std::unique_ptr<VisualLiDARVisualizer> vis;
    if (!headless) {
      viewer = guik::LightViewer::instance(Eigen::Vector2i(-1, -1), vm.count("background"));
      viewer->set_draw_xy_grid(false);
      viewer->use_arcball_camera_control();
      viewer->invoke([] {
        ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
        ImGui::Begin("texts"); ImGui::End();
        ImGui::SetNextWindowPos({55, 60}, ImGuiCond_Once);
        ImGui::Begin("visualizer"); ImGui::End();
        ImGui::SetNextWindowPos({1260, 60}, ImGuiCond_Once);
        ImGui::Begin("images"); ImGui::End();
      });
      vis = std::make_unique<VisualLiDARVisualizer>(proj, dataset, false);
      vis->set_T_camera_lidar(initial_camera_lidar);
      params.callback = [&](const Eigen::Isometry3d& transform) { vis->set_T_camera_lidar(transform); };
      params.log_callback = [&](const std::string& message) { viewer->append_text(message); };
    }

    VisualCameraCalibration calibration(proj, dataset, params);
    Eigen::Isometry3d camera_lidar;
    try {
      if (headless) {
        camera_lidar = calibration.calibrate(initial_camera_lidar);
      } else {
        std::atomic_bool terminated{false};
        std::exception_ptr optimization_error;
        std::thread optimization([&] {
          try { camera_lidar = calibration.calibrate(initial_camera_lidar); }
          catch (...) { optimization_error = std::current_exception(); }
          terminated.store(true);
        });
        try {
          while (!terminated.load()) vis->spin_once();
        } catch (...) {
          optimization.join();
          throw;
        }
        optimization.join();
        if (optimization_error) std::rethrow_exception(optimization_error);
      }
    } catch (...) {
      config["results"].erase("T_lidar_camera");
      config["results"]["calibration_status"] = "failed";
      config["results"]["calibration_diagnostics"] = diagnostics_json(calibration.diagnostics());
      save_config(data_path, config);
      throw;
    }
    const Eigen::Isometry3d lidar_camera = camera_lidar.inverse();
    const Eigen::Vector3d translation(lidar_camera.translation());
    const Eigen::Quaterniond rotation(lidar_camera.linear());
    config["results"]["T_lidar_camera"] = {translation.x(), translation.y(), translation.z(), rotation.x(), rotation.y(), rotation.z(), rotation.w()};
    config["results"]["calibration_status"] = "success";
    config["results"]["calibration_diagnostics"] = diagnostics_json(calibration.diagnostics());
    save_config(data_path, config);
    std::stringstream output;
    output << "--- T_lidar_camera (camera to LiDAR) ---" << std::endl << lidar_camera.matrix() << std::endl;
    output << "saved to " << data_path + "/calib.json";
    std::cout << output.str() << std::endl;
    if (viewer) {
      viewer->append_text(output.str());
      viewer->spin_once();
      if (!vm.count("auto_quit")) viewer->spin();
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
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains preprocessed data")
    ("first_n_bags", value<int>(), "use only the first N bags")
    ("disable_culling", "disable depth buffer-based hidden points removal")
    ("nid_bins", value<int>()->default_value(16), "number of histogram bins for NID")
    ("registration_type", value<std::string>()->default_value("nid_bfgs"), "nid_bfgs or nid_nelder_mead")
    ("max_outer_iterations", value<int>()->default_value(10), "maximum visibility/optimization rounds")
    ("max_inner_iterations", value<int>()->default_value(256), "maximum BFGS or Nelder-Mead iterations per round")
    ("nelder_mead_init_step", value<double>()->default_value(1e-3), "Nelder-Mead initial step size")
    ("nelder_mead_convergence_criteria", value<double>()->default_value(1e-8), "Nelder-Mead convergence criteria")
    ("auto_quit", "automatically quit after calibration")
    ("background", "hide viewer (still requires an OpenGL display)")
    ("headless", "run without constructing any viewer or OpenGL resources");
  std::string data_path;
  try {
    positional_options_description positional;
    positional.add("data_path", 1);
    variables_map vm;
    store(command_line_parser(argc, argv).options(description).positional(positional).run(), vm);
    notify(vm);
    if (vm.count("help") || !vm.count("data_path")) {
      std::cout << description << std::endl;
      return vm.count("help") ? 0 : 1;
    }
    data_path = vm["data_path"].as<std::string>();
    vlcal::VisualLiDARCalibration calibration(data_path, vm);
    calibration.calibrate(vm);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "calibration failed: " << error.what() << std::endl;
    if (!data_path.empty()) vlcal::record_failure(data_path, error.what());
    return 1;
  } catch (...) {
    std::cerr << "calibration failed: unknown exception" << std::endl;
    if (!data_path.empty()) vlcal::record_failure(data_path, "unknown exception");
    return 1;
  }
}
