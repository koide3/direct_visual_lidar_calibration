#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <camera/create_camera.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/calib/cost_calculator_nid.hpp>

#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

camera::GenericCameraBase::ConstPtr create_pinhole_camera(const Eigen::Vector2i& image_size, const double fov) {
  const double fx = image_size.x() / (2.0 * std::tan(fov / 2.0));
  return camera::create_camera("plumb_bob", {fx, fx, image_size.x() / 2.0, image_size.y() / 2.0}, {});
}

cv::Mat generate_lidar_image(
  const camera::GenericCameraBase::ConstPtr& proj,
  const Eigen::Vector2i& image_size,
  const gtsam_ext::Frame::ConstPtr& points,
  const Eigen::Isometry3d& T_camera_lidar) {
  //
  const double camera_fov = estimate_camera_fov(proj, image_size);
  const double min_z = std::cos(camera_fov);

  cv::Mat sq_dist_image(image_size[1], image_size[0], CV_64FC1, cv::Scalar::all(std::numeric_limits<double>::max()));
  cv::Mat intensity_image(image_size[1], image_size[0], CV_64FC1, cv::Scalar::all(0));

  for (int i = 0; i < points->size(); i++) {
    const auto& pt_lidar = points->points[i];
    const Eigen::Vector4d pt_camera = T_camera_lidar * pt_lidar;

    if (pt_camera.head<3>().normalized().z() < min_z) {
      continue;
    }

    const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= image_size.array()).any()) {
      continue;
    }

    const double sq_dist = pt_camera.head<3>().squaredNorm();
    if (sq_dist_image.at<double>(pt_2d.y(), pt_2d.x()) < sq_dist) {
      continue;
    }

    sq_dist_image.at<double>(pt_2d.y(), pt_2d.x()) = sq_dist;
    intensity_image.at<double>(pt_2d.y(), pt_2d.x()) = points->intensities[i];
  }

  return intensity_image;
}

}  // namespace vlcal

int main(int argc, char** argv) {
  // const std::string image_path = "/home/koide/datasets/lidar_camera/ouster_pinhole/2022-09-05-15-36-26.bag.png";
  // const cv::Mat image = cv::imread(image_path, 0);

  // const std::string ply_path = "/home/koide/datasets/lidar_camera/ouster_pinhole/2022-09-05-15-36-26.bag.ply";
  const std::string ply_path = "/home/koide/datasets/lidar_camera/flatwall_calib/2022-06-29-21-27-57.bag.ply";
  auto ply = glk::load_ply(ply_path);

  auto points = std::make_shared<gtsam_ext::FrameCPU>(ply->vertices);
  points->add_intensities(ply->intensities);

  Eigen::Isometry3d T_lidar_camera = Eigen::Isometry3d::Identity();
  T_lidar_camera.linear() = (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  // T_lidar_camera.linear() = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const Eigen::Isometry3d T_camera_lidar = T_lidar_camera.inverse();

  // auto proj = camera::create_camera("equirectangular", {1920.0, 960.0}, {});

  const double lidar_fov = vlcal::estimate_lidar_fov(points);
  auto proj = vlcal::create_pinhole_camera({512, 512}, lidar_fov);

  // auto proj = camera::create_camera("plumb_bob", {700.0, 700.0, 960.0, 480.0}, {});

  auto lidar_image = vlcal::generate_lidar_image(proj, {512, 512}, points, T_camera_lidar);

  cv::imshow("image", lidar_image);
  cv::imwrite("/home/koide/test.png", lidar_image * 255.0);
  cv::waitKey();

  /*
  vlcal::estimate_lidar_fov(points);

  return 0;

  Eigen::Isometry3d T_lidar_camera = Eigen::Isometry3d::Identity();
  T_lidar_camera.linear() = (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const Eigen::Isometry3d T_camera_lidar = T_lidar_camera.inverse();

  auto viewer = guik::LightViewer::instance();
  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->points, points->size());
  cloud_buffer->add_intensity(glk::COLORMAP::TURBO, points->intensities, points->size());
  viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
  viewer->spin_until_click();

  points = polar_downsample(points, T_camera_lidar);

  cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->points, points->size());
  cloud_buffer->add_intensity(glk::COLORMAP::TURBO, points->intensities, points->size());
  viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
  viewer->spin_until_click();

  vlcal::VisualLiDARData::Ptr data(new vlcal::VisualLiDARData);
  data->image = image;
  data->points = points;

  vlcal::CostCalculatorNID nid_cost(proj, data);

  auto t1 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 10; i++) {
    std::cout << "nid:" << nid_cost.calculate(T_camera_lidar) << std::endl;
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  std::cout << "time:" << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6 << "[msec]" << std::endl;
  */

  return 0;
}