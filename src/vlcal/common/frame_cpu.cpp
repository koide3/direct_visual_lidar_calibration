#include <vlcal/common/frame_cpu.hpp>

#include <regex>
#include <numeric>
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/iterator/counting_iterator.hpp>

// #include <gtsam_ext/ann/kdtree.hpp>
#include <vlcal/common/vector3i_hash.hpp>

namespace vlcal {

// constructors & deconstructor
template <typename T, int D>
FrameCPU::FrameCPU(const Eigen::Matrix<T, D, 1>* points, int num_points) {
  add_points(points, num_points);
}

template FrameCPU::FrameCPU(const Eigen::Matrix<float, 3, 1>* points, int num_points);
template FrameCPU::FrameCPU(const Eigen::Matrix<float, 4, 1>* points, int num_points);
template FrameCPU::FrameCPU(const Eigen::Matrix<double, 3, 1>* points, int num_points);
template FrameCPU::FrameCPU(const Eigen::Matrix<double, 4, 1>* points, int num_points);

FrameCPU::FrameCPU(const Frame& frame) {
  if (frame.points) {
    add_points(frame.points, frame.size());
  }

  if (frame.times) {
    add_times(frame.times, frame.size());
  }

  if (frame.normals) {
    add_normals(frame.normals, frame.size());
  }

  if (frame.covs) {
    add_covs(frame.covs, frame.size());
  }

  if (frame.intensities) {
    add_intensities(frame.intensities, frame.size());
  }

  for (const auto& attrib : frame.aux_attributes) {
    const auto& name = attrib.first;
    const size_t elem_size = attrib.second.first;
    const unsigned char* data_ptr = static_cast<const unsigned char*>(attrib.second.second);

    auto storage = std::make_shared<std::vector<unsigned char>>(frame.size() * elem_size);
    memcpy(storage->data(), data_ptr, elem_size * frame.size());

    aux_attributes_storage[name] = storage;
    aux_attributes[name] = std::make_pair(elem_size, storage->data());
  }
}

FrameCPU::FrameCPU() {}

FrameCPU::~FrameCPU() {}

// add_times
template <typename T>
void FrameCPU::add_times(const T* times, int num_points) {
  assert(num_points == size());
  times_storage.resize(num_points);
  std::copy(times, times + num_points, times_storage.begin());
  this->times = this->times_storage.data();
}

template void FrameCPU::add_times(const float* times, int num_points);
template void FrameCPU::add_times(const double* times, int num_points);

// add_points
template <typename T, int D>
void FrameCPU::add_points(const Eigen::Matrix<T, D, 1>* points, int num_points) {
  points_storage.resize(num_points, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
  for (int i = 0; i < num_points; i++) {
    points_storage[i].head<D>() = points[i].template head<D>().template cast<double>();
  }
  this->points = points_storage.data();
  this->num_points = num_points;
}

template void FrameCPU::add_points(const Eigen::Matrix<float, 3, 1>* points, int num_points);
template void FrameCPU::add_points(const Eigen::Matrix<float, 4, 1>* points, int num_points);
template void FrameCPU::add_points(const Eigen::Matrix<double, 3, 1>* points, int num_points);
template void FrameCPU::add_points(const Eigen::Matrix<double, 4, 1>* points, int num_points);

// add_normals
template <typename T, int D>
void FrameCPU::add_normals(const Eigen::Matrix<T, D, 1>* normals, int num_points) {
  assert(num_points == size());
  normals_storage.resize(num_points, Eigen::Vector4d::Zero());
  for (int i = 0; i < num_points; i++) {
    normals_storage[i].head<D>() = normals[i].template head<D>().template cast<double>();
  }
  this->normals = normals_storage.data();
}

template void FrameCPU::add_normals(const Eigen::Matrix<float, 3, 1>* normals, int num_points);
template void FrameCPU::add_normals(const Eigen::Matrix<float, 4, 1>* normals, int num_points);
template void FrameCPU::add_normals(const Eigen::Matrix<double, 3, 1>* normals, int num_points);
template void FrameCPU::add_normals(const Eigen::Matrix<double, 4, 1>* normals, int num_points);

// add_covs
template <typename T, int D>
void FrameCPU::add_covs(const Eigen::Matrix<T, D, D>* covs, int num_points) {
  assert(num_points == size());
  covs_storage.resize(num_points, Eigen::Matrix4d::Zero());
  for (int i = 0; i < num_points; i++) {
    covs_storage[i].block<D, D>(0, 0) = covs[i].template block<D, D>(0, 0).template cast<double>();
  }
  this->covs = covs_storage.data();
}

template void FrameCPU::add_covs(const Eigen::Matrix<float, 3, 3>* covs, int num_points);
template void FrameCPU::add_covs(const Eigen::Matrix<float, 4, 4>* covs, int num_points);
template void FrameCPU::add_covs(const Eigen::Matrix<double, 3, 3>* covs, int num_points);
template void FrameCPU::add_covs(const Eigen::Matrix<double, 4, 4>* covs, int num_points);

// add_intensities
template <typename T>
void FrameCPU::add_intensities(const T* intensities, int num_points) {
  assert(num_points == size());
  intensities_storage.resize(num_points);
  std::copy(intensities, intensities + num_points, intensities_storage.begin());
  this->intensities = this->intensities_storage.data();
}

template void FrameCPU::add_intensities(const float* intensities, int num_points);
template void FrameCPU::add_intensities(const double* intensities, int num_points);

// FrameCPU::load
FrameCPU::Ptr FrameCPU::load(const std::string& path) {
  FrameCPU::Ptr frame(new FrameCPU);

  if (boost::filesystem::exists(path + "/points.bin")) {
    std::ifstream ifs(path + "/points.bin", std::ios::binary | std::ios::ate);
    std::streamsize points_bytes = ifs.tellg();
    size_t num_points = points_bytes / (sizeof(Eigen::Vector4d));

    frame->num_points = num_points;
    frame->points_storage.resize(num_points);
    frame->points = frame->points_storage.data();

    ifs.seekg(0, std::ios::beg);
    ifs.read(reinterpret_cast<char*>(frame->points), sizeof(Eigen::Vector4d) * frame->size());

    if (boost::filesystem::exists(path + "/times.bin")) {
      frame->times_storage.resize(frame->size());
      frame->times = frame->times_storage.data();
      std::ifstream ifs(path + "/times.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(frame->times), sizeof(double) * frame->size());
    }

    if (boost::filesystem::exists(path + "/normals.bin")) {
      frame->normals_storage.resize(frame->size());
      frame->normals = frame->normals_storage.data();
      std::ifstream ifs(path + "/normals.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(frame->normals), sizeof(Eigen::Vector4d) * frame->size());
    }

    if (boost::filesystem::exists(path + "/covs.bin")) {
      frame->covs_storage.resize(frame->size());
      frame->covs = frame->covs_storage.data();
      std::ifstream ifs(path + "/covs.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(frame->covs), sizeof(Eigen::Matrix4d) * frame->size());
    }

    if (boost::filesystem::exists(path + "/intensities.bin")) {
      frame->intensities_storage.resize(frame->size());
      frame->intensities = frame->intensities_storage.data();
      std::ifstream ifs(path + "/intensities.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(frame->intensities), sizeof(double) * frame->size());
    }
  } else if (boost::filesystem::exists(path + "/points_compact.bin")) {
    std::ifstream ifs(path + "/points_compact.bin", std::ios::binary | std::ios::ate);
    std::streamsize points_bytes = ifs.tellg();
    size_t num_points = points_bytes / (sizeof(Eigen::Vector3f));

    frame->num_points = num_points;
    frame->points_storage.resize(num_points);
    frame->points = frame->points_storage.data();
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points_f(num_points);

    ifs.seekg(0, std::ios::beg);
    ifs.read(reinterpret_cast<char*>(points_f.data()), sizeof(Eigen::Vector3f) * frame->size());
    std::transform(points_f.begin(), points_f.end(), frame->points, [](const Eigen::Vector3f& p) { return Eigen::Vector4d(p[0], p[1], p[2], 1.0); });

    if (boost::filesystem::exists(path + "/times_compact.bin")) {
      frame->times_storage.resize(frame->size());
      frame->times = frame->times_storage.data();
      std::vector<float> times_f(frame->size());

      std::ifstream ifs(path + "/times_compact.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(times_f.data()), sizeof(float) * frame->size());
      std::copy(times_f.begin(), times_f.end(), frame->times);
    }

    if (boost::filesystem::exists(path + "/normals_compact.bin")) {
      frame->normals_storage.resize(frame->size());
      frame->normals = frame->normals_storage.data();
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals_f(frame->size());

      std::ifstream ifs(path + "/normals_compact.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(normals_f.data()), sizeof(Eigen::Vector3f) * frame->size());
      std::transform(normals_f.begin(), normals_f.end(), frame->normals, [](const Eigen::Vector3f& p) {
        return Eigen::Vector4d(p[0], p[1], p[2], 0.0);
      });
    }

    if (boost::filesystem::exists(path + "/covs_compact.bin")) {
      frame->covs_storage.resize(frame->size());
      frame->covs = frame->covs_storage.data();
      std::vector<Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1>>> covs_f(frame->size());

      std::ifstream ifs(path + "/covs_compact.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(covs_f.data()), sizeof(Eigen::Matrix<float, 6, 1>) * frame->size());
      std::transform(covs_f.begin(), covs_f.end(), frame->covs, [](const Eigen::Matrix<float, 6, 1>& c) {
        Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
        cov(0, 0) = c[0];
        cov(0, 1) = cov(1, 0) = c[1];
        cov(0, 2) = cov(2, 0) = c[2];
        cov(1, 1) = c[3];
        cov(1, 2) = cov(2, 1) = c[4];
        cov(2, 2) = c[5];
        return cov;
      });
    }

    if (boost::filesystem::exists(path + "/intensities_compact.bin")) {
      frame->intensities_storage.resize(frame->size());
      frame->intensities = frame->intensities_storage.data();
      std::vector<float> intensities_f(frame->size());

      std::ifstream ifs(path + "/intensities_compact.bin", std::ios::binary);
      ifs.read(reinterpret_cast<char*>(intensities_f.data()), sizeof(Eigen::Vector4f) * frame->size());
      std::copy(intensities_f.begin(), intensities_f.end(), frame->intensities);
    }

  } else {
    std::cerr << "error: " << path << " does not constain points(_compact)?.bin" << std::endl;
    return nullptr;
  }

  boost::filesystem::directory_iterator itr(path);
  boost::filesystem::directory_iterator end;
  const std::regex aux_name_regex("/aux_([^_]+).bin");
  for (; itr != end; itr++) {
    std::smatch matched;
    if (!std::regex_search(itr->path().string(), matched, aux_name_regex)) {
      continue;
    }
    const std::string name = matched.str(1);

    std::ifstream ifs(itr->path().string(), std::ios::ate | std::ios::binary);
    const size_t bytes = ifs.tellg();
    ifs.seekg(0);

    const int elem_size = bytes / frame->size();
    if (elem_size * frame->size() != bytes) {
      std::cerr << "warning: elem_size=" << elem_size << " num_points=" << frame->size() << " bytes=" << bytes << std::endl;
      std::cerr << "       : bytes != elem_size * num_points" << std::endl;
    }

    auto storage = std::make_shared<std::vector<char>>(bytes);
    ifs.read(storage->data(), bytes);

    frame->aux_attributes_storage[name] = storage;
    frame->aux_attributes[name] = std::make_pair(elem_size, storage->data());
  }

  return frame;
}

// sample
FrameCPU::Ptr sample(const Frame::ConstPtr& frame, const std::vector<int>& indices) {
  FrameCPU::Ptr sampled(new FrameCPU);
  sampled->num_points = indices.size();
  sampled->points_storage.resize(indices.size());
  sampled->points = sampled->points_storage.data();
  std::transform(indices.begin(), indices.end(), sampled->points, [&](const int i) { return frame->points[i]; });

  if (frame->times) {
    sampled->times_storage.resize(indices.size());
    sampled->times = sampled->times_storage.data();
    std::transform(indices.begin(), indices.end(), sampled->times, [&](const int i) { return frame->times[i]; });
  }

  if (frame->normals) {
    sampled->normals_storage.resize(indices.size());
    sampled->normals = sampled->normals_storage.data();
    std::transform(indices.begin(), indices.end(), sampled->normals, [&](const int i) { return frame->normals[i]; });
  }

  if (frame->covs) {
    sampled->covs_storage.resize(indices.size());
    sampled->covs = sampled->covs_storage.data();
    std::transform(indices.begin(), indices.end(), sampled->covs, [&](const int i) { return frame->covs[i]; });
  }

  if (frame->intensities) {
    sampled->intensities_storage.resize(indices.size());
    sampled->intensities = sampled->intensities_storage.data();
    std::transform(indices.begin(), indices.end(), sampled->intensities, [&](const int i) { return frame->intensities[i]; });
  }

  for (const auto& attrib : frame->aux_attributes) {
    const auto& name = attrib.first;
    const size_t elem_size = attrib.second.first;
    const unsigned char* data_ptr = static_cast<const unsigned char*>(attrib.second.second);

    auto storage = std::make_shared<std::vector<unsigned char>>(indices.size() * elem_size);
    for (int i = 0; i < indices.size(); i++) {
      const auto src = data_ptr + elem_size * indices[i];
      auto dst = storage->data() + elem_size * i;
      memcpy(dst, src, elem_size);
    }

    sampled->aux_attributes_storage[name] = storage;
    sampled->aux_attributes[name] = std::make_pair(elem_size, storage->data());
  }

  return sampled;
}

// random_sampling
FrameCPU::Ptr random_sampling(const Frame::ConstPtr& frame, const double sampling_rate, std::mt19937& mt) {
  if (sampling_rate >= 0.99) {
    // No need to do sampling
    return FrameCPU::Ptr(new FrameCPU(*frame));
  }

  const int num_samples = frame->size() * sampling_rate;

  std::vector<int> sample_indices(num_samples);
  std::iota(sample_indices.begin(), sample_indices.end(), 0);
  std::sample(boost::counting_iterator<int>(0), boost::counting_iterator<int>(frame->size()), sample_indices.begin(), num_samples, mt);
  std::sort(sample_indices.begin(), sample_indices.end());

  return sample(frame, sample_indices);
}

// voxelgrid_sampling
FrameCPU::Ptr voxelgrid_sampling(const Frame::ConstPtr& frame, const double voxel_resolution) {
  using Indices = std::shared_ptr<std::vector<int>>;
  using VoxelMap = std::unordered_map<
    Eigen::Vector3i,
    Indices,
    Vector3iHash,
    std::equal_to<Eigen::Vector3i>,
    Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, Indices>>>;

  VoxelMap voxelmap;

  // Insert point indices to corresponding voxels
  for (int i = 0; i < frame->size(); i++) {
    const Eigen::Vector3i coord = (frame->points[i].array() / voxel_resolution).floor().cast<int>().head<3>();
    auto found = voxelmap.find(coord);
    if (found == voxelmap.end()) {
      found = voxelmap.insert(found, std::make_pair(coord, std::make_shared<std::vector<int>>()));
      found->second->reserve(32);
    }
    found->second->push_back(i);
  }

  std::vector<Indices> voxels(voxelmap.size());
  std::transform(voxelmap.begin(), voxelmap.end(), voxels.begin(), [](const std::pair<Eigen::Vector3i, Indices>& x) { return x.second; });

  // Take the average of point attributes of each voxel
  FrameCPU::Ptr downsampled(new FrameCPU);
  downsampled->num_points = voxels.size();
  downsampled->points_storage.resize(voxels.size());
  downsampled->points = downsampled->points_storage.data();
  std::transform(voxels.begin(), voxels.end(), downsampled->points, [&](const Indices& indices) {
    Eigen::Vector4d sum = Eigen::Vector4d::Zero();
    for (const auto i : *indices) {
      sum += frame->points[i];
    }
    return sum / indices->size();
  });

  if (frame->times) {
    downsampled->times_storage.resize(voxels.size());
    downsampled->times = downsampled->times_storage.data();
    std::transform(voxels.begin(), voxels.end(), downsampled->times, [&](const Indices& indices) {
      double sum = 0.0;
      for (const auto i : *indices) {
        sum += frame->times[i];
      }
      return sum / indices->size();
    });
  }

  if (frame->normals) {
    downsampled->normals_storage.resize(voxels.size());
    downsampled->normals = downsampled->normals_storage.data();
    std::transform(voxels.begin(), voxels.end(), downsampled->normals, [&](const Indices& indices) {
      Eigen::Vector4d sum = Eigen::Vector4d::Zero();
      for (const auto i : *indices) {
        sum += frame->normals[i];
      }
      return sum / indices->size();
    });
  }

  if (frame->covs) {
    downsampled->covs_storage.resize(voxels.size());
    downsampled->covs = downsampled->covs_storage.data();
    std::transform(voxels.begin(), voxels.end(), downsampled->covs, [&](const Indices& indices) {
      Eigen::Matrix4d sum = Eigen::Matrix4d::Zero();
      for (const auto i : *indices) {
        sum += frame->covs[i];
      }
      return sum / indices->size();
    });
  }

  if (frame->intensities) {
    downsampled->intensities_storage.resize(voxels.size());
    downsampled->intensities = downsampled->intensities_storage.data();
    std::transform(voxels.begin(), voxels.end(), downsampled->intensities, [&](const Indices& indices) {
      double sum = 0.0;
      for (const auto i : *indices) {
        sum += frame->intensities[i];
      }
      return sum / indices->size();
    });
  }

  if (!frame->aux_attributes.empty()) {
    std::cout << "warning: voxelgrid_sampling does not support aux attributes!!" << std::endl;
  }

  return downsampled;
}

// randomgrid_sampling
FrameCPU::Ptr randomgrid_sampling(const Frame::ConstPtr& frame, const double voxel_resolution, const double sampling_rate, std::mt19937& mt) {
  if (sampling_rate >= 0.99) {
    // No need to do sampling
    return FrameCPU::Ptr(new FrameCPU(*frame));
  }

  using Indices = std::shared_ptr<std::vector<int>>;
  using VoxelMap = std::unordered_map<
    Eigen::Vector3i,
    Indices,
    Vector3iHash,
    std::equal_to<Eigen::Vector3i>,
    Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, Indices>>>;
  VoxelMap voxelmap;
  voxelmap.rehash(frame->size() * sampling_rate);

  // Insert point indices to corresponding voxels
  for (int i = 0; i < frame->size(); i++) {
    const Eigen::Vector3i coord = (frame->points[i].array() / voxel_resolution).floor().cast<int>().head<3>();
    auto found = voxelmap.find(coord);
    if (found == voxelmap.end()) {
      // found = voxelmap.insert(found, std::make_pair(coord, std::make_shared<std::vector<int>>()));
      found = voxelmap.emplace_hint(found, coord, std::make_shared<std::vector<int>>());
      found->second->reserve(8);
    }
    found->second->push_back(i);
  }

  const int points_per_voxel = std::ceil((sampling_rate * frame->size()) / voxelmap.size());
  const int max_num_points = frame->size() * sampling_rate * 1.2;

  // Sample points from voxels
  std::vector<int> indices;
  indices.reserve(max_num_points);

  for (const auto& voxel : voxelmap) {
    const auto& voxel_indices = *voxel.second;
    if (voxel_indices.size() <= points_per_voxel) {
      indices.insert(indices.end(), voxel_indices.begin(), voxel_indices.end());
    } else {
      std::sample(voxel_indices.begin(), voxel_indices.end(), std::back_insert_iterator(indices), points_per_voxel, mt);
    }
  }

  if (indices.size() > max_num_points) {
    std::vector<int> sub_indices(max_num_points);
    std::sample(indices.begin(), indices.end(), sub_indices.begin(), max_num_points, mt);
    indices = std::move(sub_indices);
  }

  // Sort indices to keep points ordered (and for better memory accessing)
  std::sort(indices.begin(), indices.end());

  // Sample points and return it
  return sample(frame, indices);
}

// transform
FrameCPU::Ptr sort_by_time(const Frame::ConstPtr& frame) {
  if (!frame->has_times()) {
    std::cerr << "warning: frame does not have per-point times" << std::endl;
  }

  return sort(frame, [&](const int lhs, const int rhs) { return frame->times[lhs] < frame->times[rhs]; });
}

// transform
template <>
FrameCPU::Ptr transform(const Frame::ConstPtr& frame, const Eigen::Transform<double, 3, Eigen::Isometry>& transformation) {
  auto transformed = std::make_shared<FrameCPU>(*frame);
  for (int i = 0; i < frame->size(); i++) {
    transformed->points[i] = transformation * frame->points[i];
  }

  if (frame->normals) {
    for (int i = 0; i < frame->size(); i++) {
      transformed->normals[i] = transformation.matrix() * frame->normals[i];
    }
  }

  if (frame->covs) {
    for (int i = 0; i < frame->size(); i++) {
      transformed->covs[i] = transformation.matrix() * frame->covs[i] * transformation.matrix().transpose();
    }
  }

  return transformed;
}

template <>
FrameCPU::Ptr transform(const Frame::ConstPtr& frame, const Eigen::Transform<float, 3, Eigen::Isometry>& transformation) {
  return transform<double, Eigen::Isometry>(frame, transformation.cast<double>());
}

template <>
FrameCPU::Ptr transform(const Frame::ConstPtr& frame, const Eigen::Transform<double, 3, Eigen::Affine>& transformation) {
  auto transformed = std::make_shared<FrameCPU>(*frame);
  for (int i = 0; i < frame->size(); i++) {
    transformed->points[i] = transformation * frame->points[i];
  }

  if (frame->normals) {
    Eigen::Matrix4d normal_matrix = Eigen::Matrix4d::Zero();
    normal_matrix.block<3, 3>(0, 0) = transformation.linear().inverse().transpose();
    for (int i = 0; i < frame->size(); i++) {
      transformed->normals[i] = normal_matrix * frame->normals[i];
    }
  }

  if (frame->covs) {
    for (int i = 0; i < frame->size(); i++) {
      transformed->covs[i] = transformation.matrix() * frame->covs[i] * transformation.matrix().transpose();
    }
  }

  return transformed;
}

template <>
FrameCPU::Ptr transform(const Frame::ConstPtr& frame, const Eigen::Transform<float, 3, Eigen::Affine>& transformation) {
  return transform<double, Eigen::Affine>(frame, transformation.cast<double>());
}

// transform_inplace
template <>
void transform_inplace(Frame::Ptr& frame, const Eigen::Transform<double, 3, Eigen::Isometry>& transformation) {
  for (int i = 0; i < frame->size(); i++) {
    frame->points[i] = transformation * frame->points[i];
  }

  if (frame->normals) {
    for (int i = 0; i < frame->size(); i++) {
      frame->normals[i] = transformation.matrix() * frame->normals[i];
    }
  }

  if (frame->covs) {
    for (int i = 0; i < frame->size(); i++) {
      frame->covs[i] = transformation.matrix() * frame->covs[i] * transformation.matrix().transpose();
    }
  }
}

template <>
void transform_inplace(Frame::Ptr& frame, const Eigen::Transform<float, 3, Eigen::Isometry>& transformation) {
  transform_inplace<float, Eigen::Isometry>(frame, transformation);
}

template <>
void transform_inplace(Frame::Ptr& frame, const Eigen::Transform<double, 3, Eigen::Affine>& transformation) {
  for (int i = 0; i < frame->size(); i++) {
    frame->points[i] = transformation * frame->points[i];
  }

  if (frame->normals) {
    for (int i = 0; i < frame->size(); i++) {
      Eigen::Matrix4d normal_matrix = Eigen::Matrix4d::Zero();
      normal_matrix.block<3, 3>(0, 0) = transformation.linear().inverse().transpose();
      frame->normals[i] = normal_matrix * frame->normals[i];
    }
  }

  if (frame->covs) {
    for (int i = 0; i < frame->size(); i++) {
      frame->covs[i] = transformation.matrix() * frame->covs[i] * transformation.matrix().transpose();
    }
  }
}

template <>
void transform_inplace(Frame::Ptr& frame, const Eigen::Transform<float, 3, Eigen::Affine>& transformation) {
  return transform_inplace<double, Eigen::Affine>(frame, transformation.cast<double>());
}

// statistical outlier removal
std::vector<int> find_inlier_points(const Frame::ConstPtr& frame, const std::vector<int>& neighbors, const int k, const double std_thresh) {
  std::vector<double> dists(frame->size());

  for (int i = 0; i < frame->size(); i++) {
    const auto& pt = frame->points[i];

    double sum_dist = 0.0;
    for (int j = 0; j < k; j++) {
      const int index = neighbors[i * k + j];
      sum_dist += (frame->points[index] - pt).norm();
    }

    dists[i] = sum_dist / k;
  }

  double sum_dists = 0.0;
  double sum_sq_dists = 0.0;
  for (int i = 0; i < dists.size(); i++) {
    sum_dists += dists[i];
    sum_sq_dists += dists[i] * dists[i];
  }

  const double mean = sum_dists / frame->size();
  const double var = sum_sq_dists / frame->size() - mean * mean;
  const double dist_thresh = mean + std::sqrt(var) * std_thresh;

  std::vector<int> inliers;
  inliers.reserve(frame->size());

  for (int i = 0; i < frame->size(); i++) {
    if (dists[i] < dist_thresh) {
      inliers.emplace_back(i);
    }
  }

  return inliers;
}

}  // namespace gtsam_ext
