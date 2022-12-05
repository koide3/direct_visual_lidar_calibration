#include <vlcal/common/estimate_pose.hpp>

#include <random>

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <sophus/se3.hpp>
#include <sophus/ceres_manifold.hpp>

#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/costs/reprojection_cost.hpp>

namespace vlcal {

PoseEstimation::PoseEstimation(const PoseEstimationParams& params) {}

PoseEstimation::~PoseEstimation() {}

Eigen::Isometry3d PoseEstimation::estimate(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  std::vector<bool>* inliers) {
  // RANSAC
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
  T_camera_lidar.linear() = estimate_rotation_ransac(proj, correspondences, inliers);

  std::cout << "--- T_camera_lidar (RANSAC) ---" << std::endl;
  std::cout << T_camera_lidar.matrix() << std::endl;

  // Reprojection error minimization
  T_camera_lidar = estimate_pose_lsq(proj, correspondences, T_camera_lidar);

  std::cout << "--- T_camera_lidar (LSQ) ---" << std::endl;
  std::cout << T_camera_lidar.matrix() << std::endl;

  return T_camera_lidar;
}

Eigen::Matrix3d PoseEstimation::estimate_rotation_ransac(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  std::vector<bool>* inliers) {
  std::cout << "estimating bearing vectors" << std::endl;
  // Compute bearing vectors
  std::vector<Eigen::Vector4d> directions_camera(correspondences.size());
  std::vector<Eigen::Vector4d> directions_lidar(correspondences.size());
  for (int i = 0; i < correspondences.size(); i++) {
    directions_camera[i] << estimate_direction(proj, correspondences[i].first), 0.0;
    directions_lidar[i] << correspondences[i].second.head<3>().normalized(), 0.0;
  }

  // LSQ rotation estimation
  // https://web.stanford.edu/class/cs273/refs/umeyama.pdf
  const auto find_rotation = [&](const std::vector<int>& indices) {
    Eigen::Matrix<double, 3, -1> A(3, indices.size());
    Eigen::Matrix<double, 3, -1> B(3, indices.size());

    for (int i = 0; i < indices.size(); i++) {
      const int index = indices[i];
      const auto& d_c = directions_camera[index];
      const auto& d_l = directions_lidar[index];

      A.col(i) = d_c.head<3>();
      B.col(i) = d_l.head<3>();
    }

    const Eigen::Matrix3d AB = A * B.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(AB, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    const Eigen::Matrix3d D = svd.singularValues().asDiagonal();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();

    double det = U.determinant() * V.determinant();
    if (det < 0.0) {
      S(2, 2) = -1.0;
    }

    const Eigen::Matrix3d R_camera_lidar = U * S * V.transpose();
    return R_camera_lidar;
  };

  const double error_thresh_sq = std::pow(params.ransac_error_thresh, 2);

  int best_num_inliers = 0;
  Eigen::Matrix4d best_R_camera_lidar;

  std::mt19937 mt;
  std::vector<std::mt19937> mts(omp_get_max_threads());
  for (int i = 0; i < mts.size(); i++) {
    mts[i] = std::mt19937(mt() + 8192 * i);
  }

  const int num_samples = 2;

  std::cout << "estimating rotation using RANSAC" << std::endl;
#pragma omp parallel for
  for (int i = 0; i < params.ransac_iterations; i++) {
    const int thread_id = omp_get_thread_num();

    // Sample correspondences
    std::vector<int> indices(num_samples);
    std::uniform_int_distribution<> udist(0, correspondences.size() - 1);
    for (int i = 0; i < num_samples; i++) {
      indices[i] = udist(mts[thread_id]);
    }

    // Estimate rotation
    Eigen::Matrix4d R_camera_lidar = Eigen::Matrix4d::Zero();
    R_camera_lidar.topLeftCorner<3, 3>() = find_rotation(indices);

    // Count num of inliers
    int num_inliers = 0;
    for (int j = 0; j < correspondences.size(); j++) {
      const Eigen::Vector4d direction_cam = R_camera_lidar * directions_lidar[j];
      const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());

      if ((correspondences[j].first - pt_2d).squaredNorm() < error_thresh_sq) {
        num_inliers++;
      }
    }

#pragma omp critical
    if (num_inliers > best_num_inliers) {
      // Update the best rotation
      best_num_inliers = num_inliers;
      best_R_camera_lidar = R_camera_lidar;
    }
  }

  std::cout << "num_inliers: " << best_num_inliers << " / " << correspondences.size() << std::endl;

  if (inliers) {
    inliers->resize(correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
      const Eigen::Vector4d direction_cam = best_R_camera_lidar * directions_lidar[i];
      const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());
      (*inliers)[i] = (correspondences[i].first - pt_2d).squaredNorm() < error_thresh_sq;
    }
  }

  return best_R_camera_lidar.topLeftCorner<3, 3>(0, 0);
}


Eigen::Isometry3d PoseEstimation::estimate_pose_lsq(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  const Eigen::Isometry3d& init_T_camera_lidar) {
  // 
  Sophus::SE3d T_camera_lidar = Sophus::SE3d(init_T_camera_lidar.matrix());
  
  ceres::Problem problem;
  problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());
  // The default ceres (2.0.0) on Ubuntu 20.04 does not have manifold.hpp yet
  // problem.AddParameterBlock(T_camera_lidar.data(), Sophus::SE3d::num_parameters, new Sophus::Manifold<Sophus::SE3>());

  // Create reprojection error costs
  for (const auto& [pt_2d, pt_3d] : correspondences) {
    auto reproj_error = new ReprojectionCost(proj, pt_3d.head<3>(), pt_2d);
    auto ad_cost = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, Sophus::SE3d::num_parameters>(reproj_error);
    auto loss = new ceres::CauchyLoss(params.robust_kernel_width);
    problem.AddResidualBlock(ad_cost, loss, T_camera_lidar.data());
  }

  // Solve!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  return Eigen::Isometry3d(T_camera_lidar.matrix());
}

}  // namespace vlcal
