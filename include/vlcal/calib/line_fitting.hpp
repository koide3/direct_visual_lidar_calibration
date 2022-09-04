#pragma once

#include <vector>
#include <Eigen/Core>

namespace vlcal {

/**
 * @brief Least squares line fitting
 *
 * @param points Input points
 * @return       A point on line and normal
 */
std::pair<Eigen::Vector2d, Eigen::Vector2d> fit_line(const std::vector<Eigen::Vector2i>& points);

}  // namespace vlcal
