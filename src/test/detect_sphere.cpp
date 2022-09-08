#include <test/detect_sphere.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

Eigen::Vector4d detect_sphere_ransac(const gtsam_ext::Frame::ConstPtr& points) {
  if(points->size() == 0) {
    std::cerr << "empty points!!" << std::endl;
    return Eigen::Vector4d(0.0, 0.0, 0.0, 0.2);
  }

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->resize(points->size());
  std::transform(points->points, points->points + points->size(), cloud->begin(), [](const auto& p) { return pcl::PointXYZ(p.x(), p.y(), p.z()); });

  auto model_sphere = pcl::make_shared<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>(cloud);
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();

  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);

  return coeffs.cast<double>();
}