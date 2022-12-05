#pragma once

#include <iostream>
// #include <type_traits>
#include <boost/poly_collection/detail/is_invocable.hpp>

#include <Eigen/Core>

namespace vlcal {

namespace frame {

template <typename T>
struct traits {};

// int size(const T& t)
template <typename T, typename = void>
struct size_defined : std::false_type {};

template <typename T>
struct size_defined<T, std::enable_if_t<boost::poly_collection::detail::is_invocable<decltype(&traits<T>::size), const T&>::value>> : std::true_type {
};

template <typename T>
std::enable_if_t<size_defined<T>::value, int> size(const T& t) {
  return traits<T>::size(t);
}

template <typename T>
std::enable_if_t<!size_defined<T>::value, int> size(const T& t) {
  std::cerr << "warning: calling frame::size() for unsupported class" << std::endl;
  return 0;
}

// bool has_times(const T& t)
template <typename T, typename = void>
struct has_times_defined : std::false_type {};

template <typename T>
struct has_times_defined<T, std::enable_if_t<boost::poly_collection::detail::is_invocable<decltype(&traits<T>::has_times), const T&>::value>> : std::true_type {};

template <typename T>
std::enable_if_t<has_times_defined<T>::value, bool> has_times(const T& t) {
  return traits<T>::has_times(t);
}

template <typename T>
std::enable_if_t<!has_times_defined<T>::value, bool> has_times(const T& t) {
  return false;
}

// bool has_points(const T& t)
template <typename T, typename = void>
struct has_points_defined : std::false_type {};

template <typename T>
struct has_points_defined<T, std::enable_if_t<boost::poly_collection::detail::is_invocable<decltype(&traits<T>::has_points), const T&>::value>>
: std::true_type {};

template <typename T>
std::enable_if_t<has_points_defined<T>::value, bool> has_points(const T& t) {
  return traits<T>::has_points(t);
}

template <typename T>
std::enable_if_t<!has_points_defined<T>::value, bool> has_points(const T& t) {
  return false;
}

// bool has_normals(const T& t)
template <typename T, typename = void>
struct has_normals_defined : std::false_type {};

template <typename T>
struct has_normals_defined<T, std::enable_if_t<boost::poly_collection::detail::is_invocable<decltype(&traits<T>::has_normals), const T&>::value>>
: std::true_type {};

template <typename T>
std::enable_if_t<has_normals_defined<T>::value, bool> has_normals(const T& t) {
  return traits<T>::has_normals(t);
}

template <typename T>
std::enable_if_t<!has_normals_defined<T>::value, bool> has_normals(const T& t) {
  return false;
}

// bool has_covs(const T& t)
template <typename T, typename = void>
struct has_covs_defined : std::false_type {};

template <typename T>
struct has_covs_defined<T, std::enable_if_t<boost::poly_collection::detail::is_invocable<decltype(&traits<T>::has_covs), const T&>::value>>
: std::true_type {};

template <typename T>
std::enable_if_t<has_covs_defined<T>::value, bool> has_covs(const T& t) {
  return traits<T>::has_covs(t);
}

template <typename T>
std::enable_if_t<!has_covs_defined<T>::value, bool> has_covs(const T& t) {
  return false;
}

// bool has_intensities(const T& t)
template <typename T, typename = void>
struct has_intensities_defined : std::false_type {};

template <typename T>
struct has_intensities_defined<
  T,
  std::enable_if_t<boost::poly_collection::detail::is_invocable<decltype(&traits<T>::has_intensities), const T&>::value>> : std::true_type {};

template <typename T>
std::enable_if_t<has_intensities_defined<T>::value, bool> has_intensities(const T& t) {
  return traits<T>::has_intensities(t);
}

template <typename T>
std::enable_if_t<!has_intensities_defined<T>::value, bool> has_intensities(const T& t) {
  return false;
}

// Point accessors
template <typename T>
double time(const T& t, size_t i) {
  return traits<T>::time(t, i);
}

template <typename T>
auto point(const T& t, size_t i) {
  return traits<T>::point(t, i);
}

// Normal
template <typename T, typename = void>
struct normal_defined : std::false_type {};

template <typename T>
struct normal_defined<T, std::enable_if_t<boost::poly_collection::detail::is_invocable<decltype(&traits<T>::normal), const T&, size_t>::value>>
: std::true_type {};

template <typename T, typename std::enable_if_t<normal_defined<T>::value>* = nullptr>
auto normal(const T& t, size_t i) {
  return traits<T>::normal(t, i);
}

template <typename T, typename std::enable_if_t<!normal_defined<T>::value>* = nullptr>
Eigen::Vector4d normal(const T& t, size_t i) {
  std::cerr << "error: undefined point attribute access (normal)!!" << std::endl;
  abort();
  return Eigen::Vector4d(0, 0, 0, 0);
}

template <typename T>
auto cov(const T& t, size_t i) {
  return traits<T>::cov(t, i);
}

template <typename T>
auto intensity(const T& t, size_t i) {
  return traits<T>::intensity(t, i);
}

template <typename T>
auto intensity_gradient(const T& t, size_t i) {
  return traits<T>::intensity_gradient(t, i);
}

template <typename T>
auto time_gpu(const T& t, size_t i) {
  return traits<T>::time_gpu(t, i);
}

template <typename T>
auto point_gpu(const T& t, size_t i) {
  return traits<T>::point_gpu(t, i);
}

template <typename T>
auto normal_gpu(const T& t, size_t i) {
  return traits<T>::normal_gpu(t, i);
}

template <typename T>
auto cov_gpu(const T& t, size_t i) {
  return traits<T>::cov_gpu(t, i);
}

template <typename T>
auto intensity_gpu(const T& t, size_t i) {
  return traits<T>::intensity_gpu(t, i);
}

}  // namespace frame

}  // namespace vlcal