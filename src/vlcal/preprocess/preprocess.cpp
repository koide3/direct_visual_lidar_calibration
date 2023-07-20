#include <vlcal/preprocess/preprocess.hpp>

#include <fstream>
#include <iostream>
#include <unordered_set>
#include <filesystem>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <nlohmann/json.hpp>

#include <vlcal/common/frame_cpu.hpp>
#include <vlcal/common/time_keeper.hpp>
#include <vlcal/common/console_colors.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/preprocess/generate_lidar_image.hpp>
#include <vlcal/preprocess/static_point_cloud_integrator.hpp>
#include <vlcal/preprocess/dynamic_point_cloud_integrator.hpp>

#include <glk/io/ply_io.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

Preprocess::Preprocess() {}

Preprocess::~Preprocess() {}

bool Preprocess::run(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("preprocess");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("data_path", value<std::string>(), "directory that contains rosbags for calibration")
    ("dst_path", value<std::string>(), "directory to save preprocessed data")
    ("bag_id", value<int>(), "specify the bag to use (just for evaluation)")
    ("first_n_bags", value<int>(), "use only the first N bags (just for evaluation)")
    ("auto_topic,a", "automatically select topics")
    ("dynamic_lidar_integration,d", "create target point cloud from dynamic LiDAR data (for velodyne-like LiDARs)")
    ("intensity_channel,i", value<std::string>()->default_value("auto"), "auto or channel name")
    ("camera_info_topic", value<std::string>())
    ("image_topic", value<std::string>())
    ("points_topic", value<std::string>())
    ("camera_model", value<std::string>()->default_value("auto"), "auto, atan, plumb_bob, fisheye(=equidistant), omnidir, or equirectangular")
    ("camera_intrinsics", value<std::string>(), "camera intrinsic parameters [fx,fy,cx,cy(,xi)] (don't put spaces between values!!)")
    ("camera_distortion_coeffs", value<std::string>(), "camera distortion parameters [k1,k2,p1,p2,k3] (don't put spaces between values!!)")
    ("k_neighbors", value<int>()->default_value(20), "num of neighbor points used for point covariance estimation of CT-ICP")
    ("voxel_resolution", value<double>()->default_value(0.002), "voxel grid resolution")
    ("min_distance", value<double>()->default_value(1.0), "minimum point distance. Points closer than this value will be discarded")
    ("verbose", "if true, print out optimization status")
    ("visualize,v", "if true, show extracted images and points")
  ;
  // clang-format on

  positional_options_description p;
  p.add("data_path", 1);
  p.add("dst_path", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("data_path") || !vm.count("dst_path")) {
    std::cout << description << std::endl;
    return true;
  }

  const std::string data_path = vm["data_path"].as<std::string>();
  const std::string dst_path = vm["dst_path"].as<std::string>();
  std::cout << "data_path: " << data_path << std::endl;
  std::cout << "dst_path : " << dst_path << std::endl;
  std::filesystem::create_directories(dst_path);

  std::vector<std::string> bag_filenames;
  for (const auto& path : std::filesystem::directory_iterator(data_path)) {
    if (!valid_bag(path.path().string())) {
      continue;
    }

    bag_filenames.emplace_back(path.path().string());
  }

  std::cout << "input_bags:" << std::endl;
  for (const auto& bag_filename : bag_filenames) {
    std::cout << "- " << bag_filename << std::endl;
  }

  if (bag_filenames.empty()) {
    std::cerr << vlcal::console::bold_red << "error: no input bags!!" << vlcal::console::reset << std::endl;
    return 1;
  }

  std::sort(bag_filenames.begin(), bag_filenames.end());

  if (vm.count("bag_id")) {
    const int bag_id = vm["bag_id"].as<int>();
    std::cerr << vlcal::console::bold_yellow << "use only " << bag_filenames[bag_id] << vlcal::console::reset << std::endl;
    const std::string bag_filename = bag_filenames[bag_id];
    bag_filenames = {bag_filename};
  }

  if (vm.count("first_n_bags")) {
    const int first_n_bags = vm["first_n_bags"].as<int>();
    bag_filenames.erase(bag_filenames.begin() + first_n_bags, bag_filenames.end());

    std::cerr << vlcal::console::bold_yellow << "use only the following rosbags:" << vlcal::console::reset << std::endl;
    for (const auto& bag_filename : bag_filenames) {
      std::cerr << vlcal::console::bold_yellow << "- " << bag_filename << vlcal::console::reset << std::endl;
    }
  }

  // topics
  // why omp causes errors for structured bindings?
  const auto topics = get_topics(vm, bag_filenames.front());
  const auto camera_info_topic = std::get<0>(topics);
  const auto image_topic = std::get<1>(topics);
  const auto points_topic = std::get<2>(topics);
  std::cout << "selected topics:" << std::endl;
  std::cout << "- camera_info: " << camera_info_topic << std::endl;
  std::cout << "- image      : " << image_topic << std::endl;
  std::cout << "- points     : " << points_topic << std::endl;

  // intensity channel
  const std::string intensity_channel = get_intensity_channel(vm, bag_filenames.front(), points_topic);
  std::cout << "intensity_channel: " << intensity_channel << std::endl;

  // camera params
  auto [camera_model, image_size, intrinsics, distortion_coeffs] = get_camera_params(vm, bag_filenames.front(), camera_info_topic, image_topic);
  std::cout << "camera_model: " << camera_model << std::endl;
  std::cout << "image_size  : " << image_size.width << " " << image_size.height << std::endl;
  std::cout << "intrinsics  : " << Eigen::Map<const Eigen::VectorXd>(intrinsics.data(), intrinsics.size()).transpose() << std::endl;
  std::cout << "dist_coeffs : " << Eigen::Map<const Eigen::VectorXd>(distortion_coeffs.data(), distortion_coeffs.size()).transpose() << std::endl;

  // process bags
  int num_threads_per_bag = omp_get_max_threads();
  std::cout << "processing images and points (num_threads_per_bag=" << num_threads_per_bag << ")" << std::endl;

  std::vector<Frame::ConstPtr> lidar_points(bag_filenames.size());

  // omp_set_max_active_levels(2);
  // #pragma omp parallel for
  for (int i = 0; i < bag_filenames.size(); i++) {
    std::cout << "start processing " << bag_filenames[i] << std::endl;

    const auto& bag_filename = bag_filenames[i];
    auto [image, points] = get_image_and_points(vm, bag_filename, image_topic, points_topic, intensity_channel, num_threads_per_bag);

    lidar_points[i] = points;

    const std::string bag_name = std::filesystem::path(bag_filename).filename();
    cv::imwrite(dst_path + "/" + bag_name + ".png", image);

    glk::PLYData ply;
    ply.vertices.resize(points->size());
    ply.intensities.resize(points->size());

    std::transform(points->points, points->points + points->size(), ply.vertices.begin(), [](const Eigen::Vector4d& p) { return p.cast<float>().head<3>(); });
    std::copy(points->intensities, points->intensities + points->size(), ply.intensities.begin());
    glk::save_ply_binary(dst_path + "/" + bag_name + ".ply", ply);

    std::cout << "processed " << bag_filename << std::endl;
  }

  std::cout << "lidar_points:" << lidar_points.size() << std::endl;
  std::cout << "lidar_points[0]:" << lidar_points.front()->size() << std::endl;

  // Generate LiDAR images
  const double lidar_fov = vlcal::estimate_lidar_fov(lidar_points.front());
  std::cout << "LiDAR FoV: " << lidar_fov * 180.0 / M_PI << "[deg]" << std::endl;
  Eigen::Vector2i lidar_image_size;
  std::string lidar_camera_model;
  std::vector<double> lidar_camera_intrinsics;

  Eigen::Isometry3d T_lidar_camera = Eigen::Isometry3d::Identity();

  if (lidar_fov < 150.0 * M_PI / 180.0) {
    lidar_image_size = {1024, 1024};
    T_lidar_camera.linear() = (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    const double fx = lidar_image_size.x() / (2.0 * std::tan(lidar_fov / 2.0));
    lidar_camera_model = "plumb_bob";
    lidar_camera_intrinsics = {fx, fx, lidar_image_size[0] / 2.0, lidar_image_size[1] / 2.0};
  } else {
    lidar_image_size = {1920, 960};
    T_lidar_camera.linear() = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
    lidar_camera_model = "equirectangular";
    lidar_camera_intrinsics = {static_cast<double>(lidar_image_size[0]), static_cast<double>(lidar_image_size[1])};
  }

  auto lidar_proj = camera::create_camera(lidar_camera_model, lidar_camera_intrinsics, {});

  std::cout << "save LiDAR images" << std::endl;
#pragma omp parallel for
  for (int i = 0; i < bag_filenames.size(); i++) {
    const std::string bag_name = std::filesystem::path(bag_filenames[i]).filename();
    auto [intensities, indices] = vlcal::generate_lidar_image(lidar_proj, lidar_image_size, T_lidar_camera.inverse(), lidar_points[i]);

    cv::Mat indices_8uc4(indices.rows, indices.cols, CV_8UC4, reinterpret_cast<cv::Vec4b*>(indices.data));

    intensities.clone().convertTo(intensities, CV_8UC1, 255.0);
    cv::imwrite(dst_path + "/" + bag_name + "_lidar_intensities.png", intensities);
    cv::imwrite(dst_path + "/" + bag_name + "_lidar_indices.png", indices_8uc4);
  }

  //
  std::vector<std::string> bag_names(bag_filenames);
  std::transform(bag_filenames.begin(), bag_filenames.end(), bag_names.begin(), [](const auto& path) { return std::filesystem::path(path).filename(); });

  std::cout << "save meta data" << std::endl;

  nlohmann::json config;
  config["meta"]["data_path"] = data_path;
  config["meta"]["camera_info_topic"] = camera_info_topic;
  config["meta"]["image_topic"] = image_topic;
  config["meta"]["points_topic"] = points_topic;
  config["meta"]["intensity_channel"] = intensity_channel;
  config["meta"]["bag_names"] = bag_names;
  config["camera"]["camera_model"] = camera_model;
  config["camera"]["intrinsics"] = intrinsics;
  config["camera"]["distortion_coeffs"] = distortion_coeffs;

  std::ofstream ofs(dst_path + "/calib.json");
  ofs << config.dump(2) << std::endl;

  if (vm.count("visualize")) {
    auto viewer = guik::LightViewer::instance();
    viewer->clear();
    viewer->use_arcball_camera_control();

    viewer->invoke([] {
      ImGui::SetNextWindowPos({1260, 60}, ImGuiCond_Once);
      ImGui::Begin("images");
      ImGui::End();
      ImGui::SetNextWindowPos({55, 300}, ImGuiCond_Once);
      ImGui::Begin("texts");
      ImGui::End();
    });

    for (const auto& bag_filename : bag_filenames) {
      const std::string bag_name = std::filesystem::path(bag_filename).filename();
      const cv::Mat image = cv::imread(dst_path + "/" + bag_name + ".png");
      const auto points = glk::load_ply(dst_path + "/" + bag_name + ".ply");

      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->vertices);
      cloud_buffer->add_intensity(glk::COLORMAP::TURBO, points->intensities);

      viewer->append_text(bag_filename);
      viewer->update_image("image", glk::create_texture(image));
      viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
      viewer->spin_until_click();
    }
  }

  return true;
}

std::tuple<std::string, std::string, std::string> Preprocess::get_topics(const boost::program_options::variables_map& vm, const std::string& bag_filename) {
  std::string camera_info_topic;
  std::string image_topic;
  std::string points_topic;

  if (vm.count("auto_topic")) {
    const auto topics_and_types = get_topics_and_types(bag_filename);
    std::cout << "topics in " << bag_filename << ":" << std::endl;
    for (const auto& [topic, type] : topics_and_types) {
      std::cout << "- " << topic << " : " << type << std::endl;

      if (type.find("CameraInfo") != std::string::npos) {
        if (!camera_info_topic.empty()) {
          std::cerr << vlcal::console::bold_yellow << "warning: bag constains multiple camera_info topics!!" << vlcal::console::reset << std::endl;
        }
        camera_info_topic = topic;
      } else if (type.find("Image") != std::string::npos) {
        if (!image_topic.empty()) {
          std::cerr << vlcal::console::bold_yellow << "warning: bag constains multiple image topics!!" << vlcal::console::reset << std::endl;
        }
        image_topic = topic;
      } else if (type.find("PointCloud2") != std::string::npos) {
        if (!points_topic.empty()) {
          std::cerr << vlcal::console::bold_yellow << "warning: bag constains multiple points topics!!" << vlcal::console::reset << std::endl;
        }
        points_topic = topic;
      }
    }
  }

  if (camera_info_topic.empty() && vm.count("camera_info_topic")) {
    camera_info_topic = vm["camera_info_topic"].as<std::string>();
  }
  if (image_topic.empty() && vm.count("image_topic")) {
    image_topic = vm["image_topic"].as<std::string>();
  }
  if (points_topic.empty() && vm.count("points_topic")) {
    points_topic = vm["points_topic"].as<std::string>();
  }

  if (camera_info_topic.empty()) {
    std::cerr << console::bold_yellow << "warning: failed to get camera_info topic!!" << console::reset << std::endl;
  }
  if (image_topic.empty()) {
    std::cerr << console::bold_yellow << "warning: failed to get image topic!!" << console::reset << std::endl;
  }
  if (points_topic.empty()) {
    std::cerr << console::bold_yellow << "warning: failed to get points topic!!" << console::reset << std::endl;
  }

  return {camera_info_topic, image_topic, points_topic};
}

std::string Preprocess::get_intensity_channel(const boost::program_options::variables_map& vm, const std::string& bag_filename, const std::string& points_topic) {
  std::string intensity_channel = vm["intensity_channel"].as<std::string>();
  if (intensity_channel != "auto") {
    return intensity_channel;
  }

  std::unordered_map<std::string, int> channel_priorities;
  channel_priorities["auto"] = -1;
  channel_priorities["intensity"] = 1;
  channel_priorities["reflectivity"] = 2;

  const auto point_fields = get_point_fields(bag_filename, points_topic);
  for (const auto& field : point_fields) {
    if (!channel_priorities.count(field)) {
      continue;
    }

    if (channel_priorities[intensity_channel] < channel_priorities[field]) {
      intensity_channel = field;
    }
  }

  if (intensity_channel == "auto") {
    std::cerr << vlcal::console::bold_red << "error: failed to determine point intensity channel automatically" << vlcal::console::reset << std::endl;
    std::cerr << vlcal::console::bold_red << "     : you must specify the intensity channel to be used manually" << vlcal::console::reset << std::endl;
  }

  return intensity_channel;
}

std::tuple<std::string, cv::Size, std::vector<double>, std::vector<double>> Preprocess::get_camera_params(
  const boost::program_options::variables_map& vm,
  const std::string& bag_filename,
  const std::string& camera_info_topic,
  const std::string& image_topic) {
  //
  cv::Size image_size = get_image_size(bag_filename, image_topic);
  if (image_size.width == 0 && image_size.height == 0) {
    std::cerr << vlcal::console::bold_yellow << "warning: image size is not set (image_topic=" << image_topic << ")" << vlcal::console::reset << std::endl;
  }

  std::string camera_model = vm["camera_model"].as<std::string>();
  if (camera_model != "auto") {
    const std::unordered_set<std::string> valid_camera_models = {"plumb_bob", "fisheye", "equidistant", "omnidir", "equirectangular"};
    if (!valid_camera_models.count(camera_model)) {
      std::cerr << vlcal::console::bold_red << "error: invalid camera model " << camera_model << vlcal::console::reset << std::endl;

      std::stringstream sst;
      for (const auto& model : valid_camera_models) {
        sst << " " << model;
      }
      std::cerr << vlcal::console::bold_red << "     : supported camera models are" << sst.str() << vlcal::console::reset << std::endl;

      abort();
    }

    std::vector<double> intrinsics;
    std::vector<double> distortion_coeffs;
    if (camera_model == "equirectangular") {
      intrinsics = {static_cast<double>(image_size.width), static_cast<double>(image_size.height)};
    } else {
      if (!vm.count("camera_intrinsics")) {
        std::cerr << vlcal::console::bold_red << "error: camera_intrinsics has not been set!!" << vlcal::console::reset << std::endl;
      }
      if (!vm.count("camera_distortion_coeffs")) {
        std::cerr << vlcal::console::bold_red << "error: camera_distortion_coeffs has not been set!!" << vlcal::console::reset << std::endl;
      }

      std::vector<std::string> intrinsic_tokens;
      std::vector<std::string> distortion_tokens;
      boost::split(intrinsic_tokens, vm["camera_intrinsics"].as<std::string>(), boost::is_any_of(","));
      boost::split(distortion_tokens, vm["camera_distortion_coeffs"].as<std::string>(), boost::is_any_of(","));

      intrinsics.resize(intrinsic_tokens.size());
      distortion_coeffs.resize(distortion_tokens.size());
      std::transform(intrinsic_tokens.begin(), intrinsic_tokens.end(), intrinsics.begin(), [](const auto& token) { return std::stod(token); });
      std::transform(distortion_tokens.begin(), distortion_tokens.end(), distortion_coeffs.begin(), [](const auto& token) { return std::stod(token); });
    }

    return {camera_model, image_size, intrinsics, distortion_coeffs};
  }

  std::cout << "try to get the camera model automatically" << std::endl;
  auto [distortion_model, intrinsics, distortion_coeffs] = get_camera_info(bag_filename, camera_info_topic);
  return {distortion_model, image_size, intrinsics, distortion_coeffs};
}

std::pair<cv::Mat, Frame::ConstPtr> Preprocess::get_image_and_points(
  const boost::program_options::variables_map& vm,
  const std::string& bag_filename,
  const std::string& image_topic,
  const std::string& points_topic,
  const std::string& intensity_channel,
  const int num_threads) {
  //
  cv::Mat image = get_image(bag_filename, image_topic);
  if (!image.data) {
    std::cerr << vlcal::console::bold_red << "error: failed to obtain an image (image_topic=" << image_topic << ")" << vlcal::console::reset << std::endl;
    abort();
  }
  cv::equalizeHist(image.clone(), image);

  // integrate points
  TimeKeeper time_keeper;
  std::unique_ptr<vlcal::PointCloudIntegrator> points_integrator;

  if (vm.count("dynamic_lidar_integration")) {
    vlcal::DynamicPointCloudIntegratorParams params;
    params.visualize = vm.count("visualize");
    params.verbose = vm.count("verbose");
    params.k_neighbors = vm["k_neighbors"].as<int>();
    params.voxel_resolution = vm["voxel_resolution"].as<double>();
    params.min_distance = vm["min_distance"].as<double>();
    params.num_threads = num_threads;
    points_integrator.reset(new vlcal::DynamicPointCloudIntegrator(params));
  } else {
    vlcal::StaticPointCloudIntegratorParams params;
    params.visualize = vm.count("visualize");
    params.voxel_resolution = vm["voxel_resolution"].as<double>();
    params.min_distance = vm["min_distance"].as<double>();
    points_integrator.reset(new vlcal::StaticPointCloudIntegrator(params));
  }

  auto points_reader = get_point_cloud_reader(bag_filename, points_topic, intensity_channel);
  while (true) {
    auto raw_points = points_reader->read_next();
    if (!raw_points) {
      break;
    }

    if (!time_keeper.process(raw_points)) {
      std::cerr << vlcal::console::yellow << "warning: skip frame with an invalid timestamp!!" << vlcal::console::reset << std::endl;
      continue;
    }

    auto points = std::make_shared<FrameCPU>(raw_points->points);
    points->add_times(raw_points->times);
    points->add_intensities(raw_points->intensities);
    points = filter(points, [](const Eigen::Vector4d& p) { return p.array().isFinite().all(); });

    points_integrator->insert_points(points);
  }

  auto points = points_integrator->get_points();

  // histrogram equalization
  std::vector<int> indices(points->size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return points->intensities[lhs] < points->intensities[rhs]; });

  const int bins = 256;
  for (int i = 0; i < indices.size(); i++) {
    const double value = std::floor(bins * static_cast<double>(i) / indices.size()) / bins;
    points->intensities[indices[i]] = value;
  }

  return {image, points};
}

}  // namespace vlcal
