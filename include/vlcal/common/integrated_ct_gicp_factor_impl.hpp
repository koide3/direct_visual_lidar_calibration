#include <vlcal/common/integrated_ct_gicp_factor.hpp>

#include <gtsam/linear/HessianFactor.h>
#include <vlcal/common/nearest_neighbor_search.hpp>

namespace vlcal {

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>::IntegratedCT_GICPFactor_(
  gtsam::Key source_t0_key,
  gtsam::Key source_t1_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source,
  const std::shared_ptr<NearestNeighborSearch>& target_tree)
: IntegratedCT_ICPFactor_<TargetFrame, SourceFrame>(source_t0_key, source_t1_key, target, source, target_tree) {
  //
  if (!frame::has_points(*target) || !frame::has_covs(*target)) {
    std::cerr << "error: target frame doesn't have required attributes for ct_gicp" << std::endl;
    abort();
  }

  if (!frame::has_points(*source) || !frame::has_covs(*source)) {
    std::cerr << "error: source frame doesn't have required attributes for ct_gicp" << std::endl;
    abort();
  }
}

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>::IntegratedCT_GICPFactor_(
  gtsam::Key source_t0_key,
  gtsam::Key source_t1_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source)
: IntegratedCT_GICPFactor_(source_t0_key, source_t1_key, target, source, nullptr) {}

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>::~IntegratedCT_GICPFactor_() {}

template <typename TargetFrame, typename SourceFrame>
double IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>::error(const gtsam::Values& values) const {
  this->update_poses(values);
  if (this->correspondences.size() != frame::size(*this->source)) {
    this->update_correspondences();
  }

  double sum_errors = 0.0;
#pragma omp parallel for reduction(+ : sum_errors) schedule(guided, 8) num_threads(this->num_threads)
  for (int i = 0; i < frame::size(*this->source); i++) {
    const long target_index = this->correspondences[i];
    if (target_index < 0) {
      continue;
    }

    const int time_index = this->time_indices[i];
    const Eigen::Isometry3d pose(this->source_poses[time_index].matrix());

    const auto& source_pt = frame::point(*this->source, i);
    const auto& target_pt = frame::point(*this->target, target_index);

    Eigen::Vector4d transed_source_pt = pose * source_pt;
    Eigen::Vector4d error = transed_source_pt - target_pt;

    sum_errors += 0.5 * error.transpose() * mahalanobis[i] * error;
  }

  return sum_errors;
}

template <typename TargetFrame, typename SourceFrame>
boost::shared_ptr<gtsam::GaussianFactor> IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>::linearize(const gtsam::Values& values) const {
  this->update_poses(values);
  this->update_correspondences();

  double sum_errors = 0.0;
  std::vector<gtsam::Matrix6, Eigen::aligned_allocator<gtsam::Matrix6>> Hs_00(this->num_threads, gtsam::Matrix6::Zero());
  std::vector<gtsam::Matrix6, Eigen::aligned_allocator<gtsam::Matrix6>> Hs_01(this->num_threads, gtsam::Matrix6::Zero());
  std::vector<gtsam::Matrix6, Eigen::aligned_allocator<gtsam::Matrix6>> Hs_11(this->num_threads, gtsam::Matrix6::Zero());
  std::vector<gtsam::Vector6, Eigen::aligned_allocator<gtsam::Vector6>> bs_0(this->num_threads, gtsam::Vector6::Zero());
  std::vector<gtsam::Vector6, Eigen::aligned_allocator<gtsam::Vector6>> bs_1(this->num_threads, gtsam::Vector6::Zero());

  gtsam::Matrix6 H_00 = gtsam::Matrix6::Zero();
  gtsam::Matrix6 H_01 = gtsam::Matrix6::Zero();
  gtsam::Matrix6 H_11 = gtsam::Matrix6::Zero();
  gtsam::Vector6 b_0 = gtsam::Vector6::Zero();
  gtsam::Vector6 b_1 = gtsam::Vector6::Zero();

#pragma omp parallel for reduction(+ : sum_errors) schedule(guided, 8) num_threads(this->num_threads)
  for (int i = 0; i < frame::size(*this->source); i++) {
    const long target_index = this->correspondences[i];
    if (target_index < 0) {
      continue;
    }

    const int time_index = this->time_indices[i];

    const Eigen::Isometry3d pose(this->source_poses[time_index].matrix());
    const auto& H_pose_0 = this->pose_derivatives_t0[time_index];
    const auto& H_pose_1 = this->pose_derivatives_t1[time_index];

    const auto& source_pt = frame::point(*this->source, i);
    const auto& target_pt = frame::point(*this->target, target_index);

    gtsam::Matrix46 H_transed_pose = gtsam::Matrix46::Zero();
    H_transed_pose.block<3, 3>(0, 0) = pose.linear() * -gtsam::SO3::Hat(source_pt.template head<3>());
    H_transed_pose.block<3, 3>(0, 3) = pose.linear();
    const Eigen::Vector4d transed_source_pt = pose * source_pt;

    const auto& H_error_pose = H_transed_pose;
    const Eigen::Vector4d error = transed_source_pt - target_pt;

    const gtsam::Matrix46 H_0 = H_error_pose * H_pose_0;
    const gtsam::Matrix46 H_1 = H_error_pose * H_pose_1;

    int thread_num = 0;
#ifdef _OPENMP
    thread_num = omp_get_thread_num();
#endif

    const gtsam::Vector4 mahalanobis_error = mahalanobis[i] * error;
    const gtsam::Matrix64 H_0_mahalanobis = H_0.transpose() * mahalanobis[i];
    const gtsam::Matrix64 H_1_mahalanobis = H_1.transpose() * mahalanobis[i];

    sum_errors += 0.5 * error.transpose() * mahalanobis_error;
    Hs_00[thread_num] += H_0_mahalanobis * H_0;
    Hs_11[thread_num] += H_1_mahalanobis * H_1;
    Hs_01[thread_num] += H_0_mahalanobis * H_1;
    bs_0[thread_num] += H_0.transpose() * mahalanobis_error;
    bs_1[thread_num] += H_1.transpose() * mahalanobis_error;
  }

  for (int i = 1; i < Hs_00.size(); i++) {
    Hs_00[0] += Hs_00[i];
    Hs_11[0] += Hs_11[i];
    Hs_01[0] += Hs_01[i];
    bs_0[0] += bs_0[i];
    bs_1[0] += bs_1[i];
  }

  auto factor = gtsam::HessianFactor::shared_ptr(
    new gtsam::HessianFactor(this->keys_[0], this->keys_[1], Hs_00[0], Hs_01[0], -bs_0[0], Hs_11[0], -bs_1[0], sum_errors));
  return factor;
}

template <typename TargetFrame, typename SourceFrame>
void IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>::update_correspondences() const {
  this->correspondences.resize(frame::size(*this->source));
  this->mahalanobis.resize(frame::size(*this->source));

#pragma omp parallel for schedule(guided, 8) num_threads(this->num_threads)
  for (int i = 0; i < frame::size(*this->source); i++) {
    const int time_index = this->time_indices[i];
    const Eigen::Matrix4d pose = this->source_poses[time_index].matrix();

    const auto& pt = frame::point(*this->source, i);
    const Eigen::Vector4d transed_pt = pose * pt;

    size_t k_index = -1;
    double k_sq_dist = std::numeric_limits<double>::max();
    size_t num_found = this->target_tree->knn_search(transed_pt.data(), 1, &k_index, &k_sq_dist);

    if (num_found == 0 || k_sq_dist > this->max_correspondence_distance_sq) {
      this->correspondences[i] = -1;
      this->mahalanobis[i].setZero();
    } else {
      this->correspondences[i] = k_index;

      const long target_index = this->correspondences[i];
      const auto& cov_A = frame::cov(*this->source, i);
      const auto& cov_B = frame::cov(*this->target, target_index);
      Eigen::Matrix4d RCR = (cov_B + pose * cov_A * pose.transpose());
      RCR(3, 3) = 1.0;

      mahalanobis[i] = RCR.inverse();
      mahalanobis[i](3, 3) = 0.0;
    }
  }
}
}  // namespace vlcal