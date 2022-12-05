// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <Eigen/Core>
#include <boost/functional/hash/hash.hpp>

namespace vlcal {

/**
 * @brief Spatial hashing function using boost::hash_combine
 */
class Vector3iHash {
public:
  size_t operator()(const Eigen::Vector3i& x) const {
    size_t seed = 0;
    boost::hash_combine(seed, x[0]);
    boost::hash_combine(seed, x[1]);
    boost::hash_combine(seed, x[2]);
    return seed;
  }
};

/**
 * @brief Spatial hashing function
 *        Teschner et al., "Optimized Spatial Hashing for Collision Detection of Deformable Objects", VMV2003
 */
class XORVector3iHash {
public:
  size_t operator()(const Eigen::Vector3i& x) const {
    const size_t p1 = 73856093;
    const size_t p2 = 19349669;  // 19349663 was not a prime number
    const size_t p3 = 83492791;
    return static_cast<size_t>((x[0] * p1) ^ (x[1] * p2) ^ (x[2] * p3));
  }
};

}  // namespace gtsam_ext