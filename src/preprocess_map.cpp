#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <nlohmann/json.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/common/frame_cpu.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/preprocess/generate_lidar_image.hpp>

#include <glk/io/ply_io.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

class PreprocessMap {
public:
  PreprocessMap() {}

  int run(int argc, char** argv) {
    using namespace boost::program_options;
    options_description description("preprocess_map");

    // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("map_path", value<std::string>(), "path to input point cloud map (PCD or PLY)")
    ("image_path", value<std::string>(), "path to input image (PNG or JPG)")
    ("dst_path", value<std::string>(), "directory to save preprocessed data")
    ("camera_model", value<std::string>(), "atan, plumb_bob, fisheye, omnidir, or equirectangular")
    ("camera_intrinsics", value<std::string>(), "camera intrinsic parameters [fx,fy,cx,cy(,xi)] (don't put spaces between values!!)")
    ("camera_distortion_coeffs", value<std::string>(), "camera distortion parameters [k1,k2,p1,p2,k3] (don't put spaces between values!!)")
    ("voxel_resolution", value<double>()->default_value(0.002), "voxel grid resolution")
    ("min_distance", value<double>()->default_value(1.0), "minimum point distance. Points closer than this value will be discarded")
    ("visualize,v", "if true, show extracted images and points")
  ;
    // clang-format on
    variables_map vm;
    store(command_line_parser(argc, argv).options(description).run(), vm);
    notify(vm);

    if (
      vm.count("help") || !vm.count("map_path") || !vm.count("image_path") || !vm.count("dst_path") || !vm.count("camera_model") || !vm.count("camera_intrinsics") ||
      !vm.count("camera_distortion_coeffs")) {
      std::cout << description << std::endl;
      return 0;
    }

    const std::string map_path = vm["map_path"].as<std::string>();
    const std::string image_path = vm["image_path"].as<std::string>();
    const std::string dst_path = vm["dst_path"].as<std::string>();
    boost::filesystem::create_directories(vm["dst_path"].as<std::string>());

    // Load image
    cv::Mat image = cv::imread(image_path, 0);
    if (!image.data) {
      std::cerr << "error: failed to load image " << image_path << std::endl;
      return 1;
    }
    cv::equalizeHist(image.clone(), image);
    cv::imwrite(dst_path + "/000000.png", image);

    // Camera params
    const std::string camera_model = vm["camera_model"].as<std::string>();
    std::vector<double> intrinsics;
    std::vector<double> distortion_coeffs;

    std::vector<std::string> intrinsic_tokens;
    std::vector<std::string> distortion_tokens;
    boost::split(intrinsic_tokens, vm["camera_intrinsics"].as<std::string>(), boost::is_any_of(","));
    boost::split(distortion_tokens, vm["camera_distortion_coeffs"].as<std::string>(), boost::is_any_of(","));
    intrinsics.resize(intrinsic_tokens.size());
    distortion_coeffs.resize(distortion_tokens.size());
    std::transform(intrinsic_tokens.begin(), intrinsic_tokens.end(), intrinsics.begin(), [](const auto& token) { return std::stod(token); });
    std::transform(distortion_tokens.begin(), distortion_tokens.end(), distortion_coeffs.begin(), [](const auto& token) { return std::stod(token); });

    // Load map points
    auto lidar_points = load_lidar_points(map_path, vm["voxel_resolution"].as<double>(), vm["min_distance"].as<double>());
    if (!lidar_points || !save_lidar_data(dst_path, lidar_points)) {
      return 1;
    }

    nlohmann::json config;
    config["meta"]["data_path"] = map_path;
    config["meta"]["camera_info_topic"] = "N/A";
    config["meta"]["image_topic"] = "N/A";
    config["meta"]["points_topic"] = "N/A";
    config["meta"]["intensity_channel"] = "N/A";
    config["meta"]["bag_names"] = {"000000"};
    config["camera"]["camera_model"] = camera_model;
    config["camera"]["intrinsics"] = intrinsics;
    config["camera"]["distortion_coeffs"] = distortion_coeffs;

    std::ofstream ofs(dst_path + "/calib.json");
    ofs << config.dump(2) << std::endl;

    if (vm.count("visualize")) {
      auto viewer = guik::LightViewer::instance();

      cv::Mat bgr;
      cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
      viewer->update_image("image", glk::create_texture(bgr));

      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(lidar_points->points, lidar_points->size());
      cloud_buffer->add_intensity(glk::COLORMAP::TURBO, lidar_points->intensities, lidar_points->size());
      viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
      viewer->spin();
    }

    return 0;
  }

  vlcal::Frame::ConstPtr load_lidar_points(const std::string& path, double voxel_resolution, double min_distance) {
    auto map_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (pcl::io::load(path, *map_points)) {
      std::cerr << "error: failed to load " << path << std::endl;
      return nullptr;
    }

    if (map_points->empty()) {
      std::cerr << "error: no map points in " << path << std::endl;
      return nullptr;
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
    voxelgrid.setInputCloud(map_points);
    auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    voxelgrid.filter(*filtered);

    std::cout << "map_points=" << map_points->size() << " filtered=" << filtered->size() << std::endl;
    map_points = filtered;

    std::vector<Eigen::Vector4d> points(map_points->size());
    std::vector<double> intensities(map_points->size());
    for (int i = 0; i < map_points->size(); i++) {
      points[i] = map_points->at(i).getVector4fMap().cast<double>();
      intensities[i] = map_points->at(i).intensity;
    }

    auto frame = std::make_shared<FrameCPU>(points);
    frame->add_intensities(intensities);

    // histrogram equalization
    std::vector<int> indices(frame->size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return frame->intensities[lhs] < frame->intensities[rhs]; });

    const int bins = 256;
    for (int i = 0; i < indices.size(); i++) {
      const double value = std::floor(bins * static_cast<double>(i) / indices.size()) / bins;
      frame->intensities[indices[i]] = value;
    }

    return frame;
  }

  bool save_lidar_data(const std::string& dst_path, const Frame::ConstPtr& lidar_points) {
    glk::PLYData ply;
    ply.vertices.resize(lidar_points->size());
    ply.intensities.resize(lidar_points->size());
    for (int i = 0; i < lidar_points->size(); i++) {
      ply.vertices[i] = lidar_points->points[i].cast<float>().head<3>();
      ply.intensities[i] = lidar_points->intensities[i];
    }
    glk::save_ply_binary(dst_path + "/000000.ply", ply);

    // Generate LiDAR images
    const double lidar_fov = vlcal::estimate_lidar_fov(lidar_points);
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
    auto [intensities, indices] = vlcal::generate_lidar_image(lidar_proj, lidar_image_size, T_lidar_camera.inverse(), lidar_points);

    cv::Mat indices_8uc4(indices.rows, indices.cols, CV_8UC4, reinterpret_cast<cv::Vec4b*>(indices.data));

    intensities.clone().convertTo(intensities, CV_8UC1, 255.0);
    cv::imwrite(dst_path + "/000000" + "_lidar_intensities.png", intensities);
    cv::imwrite(dst_path + "/000000" + "_lidar_indices.png", indices_8uc4);

    return true;
  }

private:
};

}  // namespace vlcal

int main(int argc, char** argv) {
  vlcal::PreprocessMap preprocess_map;
  return preprocess_map.run(argc, argv);
}