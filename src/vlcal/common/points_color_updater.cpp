#include <vlcal/common/points_color_updater.hpp>
#include <vlcal/common/estimate_fov.hpp>

#include <glk/primitives/icosahedron.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

PointsColorUpdater::PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image)
: proj(proj),
  min_nz(std::cos(estimate_camera_fov(proj, {image.cols, image.rows}) + 0.5 * M_PI / 180.0)),
  image(image) {
  glk::Icosahedron icosahedron;
  for (int i = 0; i < 6; i++) {
    icosahedron.subdivide();
  }
  icosahedron.spherize();

  points = std::make_shared<FrameCPU>(icosahedron.vertices);
  intensity_colors.resize(points->size(), Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));

  cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->points, points->size());
}

PointsColorUpdater::PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image, const FrameCPU::ConstPtr& points)
: proj(proj),
  min_nz(std::cos(estimate_camera_fov(proj, {image.cols, image.rows}) + 0.5 * M_PI / 180.0)),
  image(image),
  points(points) {
  cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points->points, points->size());
  intensity_colors.resize(points->size());
  for (int i = 0; i < points->size(); i++) {
    intensity_colors[i] = glk::colormapf(glk::COLORMAP::TURBO, points->intensities[i]);
  }
}

void PointsColorUpdater::update(const Eigen::Isometry3d& T_camera_liar, const double blend_weight) {
  std::shared_ptr<std::vector<Eigen::Vector4f>> colors(new std::vector<Eigen::Vector4f>(points->size(), Eigen::Vector4f::Zero()));

  for (int i = 0; i < points->size(); i++) {
    const Eigen::Vector4d pt_camera = T_camera_liar * points->points[i];

    if (pt_camera.head<3>().normalized().z() < min_nz) {
      // Out of FoV
      continue;
    }

    const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= Eigen::Array2i(image.cols, image.rows)).any()) {
      // Out of Image
      continue;
    }

    const unsigned char pix = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x());
    const Eigen::Vector4f color(pix / 255.0f, pix / 255.0f, pix / 255.0f, 1.0f);

    colors->at(i) = color * blend_weight + intensity_colors[i] * (1.0 - blend_weight);
  }

  guik::LightViewer::instance()->invoke([cloud_buffer = cloud_buffer, colors = colors] { cloud_buffer->add_color(*colors); });
}

}  // namespace vlcal
