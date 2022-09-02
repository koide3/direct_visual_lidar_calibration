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

#include <vlcal/common/create_camera.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/common/points_color_updater.hpp>

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
    proj = create_camera(camera_model, intrinsics, distortion_coeffs);

    const std::vector<std::string> bag_names = config["meta"]["bag_names"];
    for (const auto& bag_name : bag_names) {
      dataset.emplace_back(std::make_shared<VisualLiDARData>(data_path, bag_name));
    }
  }

  void spin() {
    auto viewer = guik::LightViewer::instance();
    viewer->set_draw_xy_grid(false);
    viewer->use_arcball_camera_control();

    std::mutex updater_mutex;
    std::unique_ptr<PointsColorUpdater> sphere_updater;
    std::unique_ptr<PointsColorUpdater> color_updater;

    const std::string camera_model = config["camera"]["camera_model"];
    Eigen::Isometry3d init_T_lidar_camera = Eigen::Isometry3d::Identity();
    if (camera_model != "equirectangular") {
      init_T_lidar_camera.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }

    // WARNING!! I don't literally take care of thread-safety here!!
    int selected_bag_id = -1;
    float blend_weight = 0.7f;
    guik::ModelControl T_lidar_camera_gizmo("T_lidar_camera", init_T_lidar_camera.matrix().cast<float>());

    std::atomic_bool kill_switch = false;
    std::thread color_upate_thread([&] {
      while (!kill_switch) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const Eigen::Isometry3d T_camera_lidar(T_lidar_camera_gizmo.model_matrix().cast<double>().inverse());

        std::lock_guard<std::mutex> lock(updater_mutex);
        if (color_updater) {
          color_updater->update(T_camera_lidar, blend_weight);
        }
        if (sphere_updater) {
          sphere_updater->update(T_camera_lidar, 1.0);
        }
      }
    });

    viewer->register_ui_callback("ui", [&] {
      ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      const int prev_selected_bag_id = selected_bag_id;
      if (ImGui::ArrowButton("##Left", ImGuiDir_Left)) {
        selected_bag_id--;
      }
      ImGui::SameLine();
      if (ImGui::ArrowButton("##Right", ImGuiDir_Right)) {
        selected_bag_id++;
      }
      ImGui::SameLine();
      ImGui::DragInt("bag_id", &selected_bag_id, 1, 0, dataset.size() - 1);
      selected_bag_id = std::max<int>(0, std::min<int>(dataset.size() - 1, selected_bag_id));

      if (prev_selected_bag_id != selected_bag_id) {
        std::lock_guard<std::mutex> lock(updater_mutex);

        sphere_updater.reset(new PointsColorUpdater(proj, dataset[selected_bag_id]->image));
        color_updater.reset(new PointsColorUpdater(proj, dataset[selected_bag_id]->image, dataset[selected_bag_id]->points));

        viewer->update_drawable("sphere", sphere_updater->cloud_buffer, guik::VertexColor());
        viewer->update_drawable("points", color_updater->cloud_buffer, guik::VertexColor());
      }

      T_lidar_camera_gizmo.draw_gizmo();
      T_lidar_camera_gizmo.draw_gizmo_ui();

      ImGui::DragFloat("blend_weight", &blend_weight, 0.01f, 0.0f, 1.0f);

      if (ImGui::Button("save")) {
        const Eigen::Isometry3d T_lidar_camera(T_lidar_camera_gizmo.model_matrix().cast<double>());
        const Eigen::Vector3d trans = T_lidar_camera.translation();
        const Eigen::Quaterniond quat(T_lidar_camera.linear());

        const std::vector<double> values = {trans.x(), trans.y(), trans.z(), quat.x(), quat.y(), quat.z(), quat.w()};
        config["results"]["init_T_lidar_camera"] = values;

        std::ofstream ofs(data_path + "/calib.json");
        if(!ofs) {
          std::cerr << glim::console::bold_red << "error: failed to open " << data_path + "/calib.json"
                    << " for writing" << glim::console::reset << std::endl;
        } else {
          ofs << config.dump(2) << std::endl;
        }

        viewer->append_text("saved");
      }

      ImGui::End();
    });

    viewer->spin();

    kill_switch = true;
    color_upate_thread.join();
  }

private:
  const std::string data_path;
  nlohmann::json config;

  camera::GenericCameraBase::ConstPtr proj;
  std::vector<VisualLiDARData::Ptr> dataset;
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