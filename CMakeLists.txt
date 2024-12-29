cmake_minimum_required(VERSION 3.16)
project(direct_visual_lidar_calibration VERSION 0.0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(ENABLE_ROS OFF)

if(ENABLE_ROS)
    if($ENV{ROS_VERSION} EQUAL 1)
      # ROS1
      find_package(catkin REQUIRED COMPONENTS rosbag sensor_msgs cv_bridge)
      catkin_package()
    else()
      # ROS2
      find_package(ament_cmake_auto REQUIRED)
      find_package(ament_cmake_python REQUIRED)
      ament_auto_find_build_dependencies()
    endif()
endif()

find_package(PCL REQUIRED COMPONENTS io surface sample_consensus filters)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
# Bug in OpenCV (at least some versions): Because it does not find VTK correctly for
# some module or another, we get a seemingly unrelated error when finding PCL.
# Therefore, only find the actually needed modules
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(Iridescence REQUIRED)
find_package(fmt REQUIRED)

if(ENABLE_ROS)
set(OLD_DISTRO "humble;galactic;foxy")
    if($ENV{ROS_DISTRO} IN_LIST OLD_DISTRO)
      add_definitions(-DCV_BRIDGE_INCLUDE_H)
    else()
      add_definitions(-DCV_BRIDGE_INCLUDE_HPP)
    endif()
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# direct_visual_lidar_calibration
add_library(direct_visual_lidar_calibration SHARED
  src/camera/create_camera.cpp
  src/vlcal/common/frame.cpp
  src/vlcal/common/frame_cpu.cpp
  src/vlcal/common/ivox.cpp
  src/vlcal/common/cloud_covariance_estimation.cpp
  src/vlcal/common/time_keeper.cpp
  src/vlcal/common/estimate_fov.cpp
  src/vlcal/common/estimate_pose.cpp
  src/vlcal/common/points_color_updater.cpp
  src/vlcal/common/visual_lidar_data.cpp
  src/vlcal/common/visual_lidar_visualizer.cpp
  src/vlcal/preprocess/preprocess.cpp
  src/vlcal/preprocess/generate_lidar_image.cpp
  src/vlcal/preprocess/static_point_cloud_integrator.cpp
  src/vlcal/preprocess/dynamic_point_cloud_integrator.cpp
  src/vlcal/calib/cost_calculator_nid.cpp
  src/vlcal/calib/view_culling.cpp
  src/vlcal/calib/visual_camera_calibration.cpp
)
target_include_directories(direct_visual_lidar_calibration PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/json/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/Bonxai/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/nanoflann/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/Sophus>

    $<INSTALL_INTERFACE:include>
    # $<INSTALL_INTERFACE:thirdparty/json/include>
    # $<INSTALL_INTERFACE:thirdparty/Bonxai/include>
    # $<INSTALL_INTERFACE:thirdparty/nanoflann/include>
    # $<INSTALL_INTERFACE:thirdparty/Sophus>

    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_sources(direct_visual_lidar_calibration PUBLIC
    FILE_SET public_headers
    TYPE HEADERS
    BASE_DIRS ${PROJECT_SOURCE_DIR}/include/
    FILES
   ${PROJECT_SOURCE_DIR}/include/dfo/optimizer.hpp
   ${PROJECT_SOURCE_DIR}/include/dfo/directional_direct_search.hpp
   ${PROJECT_SOURCE_DIR}/include/dfo/nelder_mead.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/generic_camera.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/atan.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/traits.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/fisheye.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/pinhole.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/omnidir.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/equirectangular.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/generic_camera_base.hpp
   ${PROJECT_SOURCE_DIR}/include/camera/create_camera.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/calib/visual_camera_calibration.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/calib/cost_calculator.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/calib/view_culling.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/calib/cost_calculator_nid.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/costs/reprojection_cost.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/costs/nid_cost.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/preprocess/preprocess.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/preprocess/generate_lidar_image.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/preprocess/dynamic_point_cloud_integrator.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/preprocess/point_cloud_integrator.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/preprocess/static_point_cloud_integrator.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/console_colors.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/integrated_ct_gicp_factor.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/ros_cloud_converter.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/raw_points.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/frame_cpu.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/kdtree2.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/points_color_updater.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/integrated_ct_icp_factor_impl.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/statistical_median_filter.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/estimate_fov.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/nearest_neighbor_search.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/time_keeper.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/vector3i_hash.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/visual_lidar_data.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/concurrent_queue.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/cloud_covariance_estimation.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/visual_lidar_visualizer.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/integrated_ct_icp_factor.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/frame.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/ivox.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/integrated_ct_gicp_factor_impl.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/frame_traits.hpp
   ${PROJECT_SOURCE_DIR}/include/vlcal/common/estimate_pose.hpp
   ${PROJECT_SOURCE_DIR}/include/nanoflann.hpp
    )


target_link_libraries(direct_visual_lidar_calibration
    PRIVATE
    ${CERES_LIBRARIES}
    fmt::fmt
)

target_link_libraries(direct_visual_lidar_calibration
    PUBLIC
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GTSAM_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Iridescence::Iridescence
)


if(ENABLE_ROS)
    if($ENV{ROS_VERSION} EQUAL 1)
      # ROS1
      add_executable(preprocess
        src/preprocess_ros1.cpp
      )
      target_link_libraries(preprocess
        direct_visual_lidar_calibration
      )
      target_include_directories(preprocess PUBLIC
        ${catkin_INCLUDE_DIRS}
      )
      target_link_libraries(preprocess
        ${catkin_LIBRARIES}
      )
    else()
      # ROS2
      ament_auto_add_executable(preprocess
        src/preprocess_ros2.cpp
      )
      target_link_libraries(preprocess
        direct_visual_lidar_calibration
      )
    endif()
endif()

# preprocess_map
add_executable(preprocess_map
  src/preprocess_map.cpp
)
if (ENABLE_ROS)
target_include_directories(preprocess_map PRIVATE
  ${catkin_INCLUDE_DIRS}
)
endif()
target_link_libraries(preprocess_map
  direct_visual_lidar_calibration
)

# initial_guess_manual
add_executable(initial_guess_manual
  src/initial_guess_manual.cpp
)
if (ENABLE_ROS)
target_include_directories(initial_guess_manual PRIVATE
  ${catkin_INCLUDE_DIRS}
)
endif()
target_link_libraries(initial_guess_manual
  direct_visual_lidar_calibration
)

# initial_guess_auto
add_executable(initial_guess_auto
  src/initial_guess_auto.cpp
)
if (ENABLE_ROS)
target_include_directories(initial_guess_auto PRIVATE
  ${catkin_INCLUDE_DIRS}
)
endif()
target_link_libraries(initial_guess_auto
  direct_visual_lidar_calibration
)

# calibrate
add_executable(calibrate
  src/calibrate.cpp
)
if (ENABLE_ROS)
target_include_directories(calibrate PRIVATE
  ${catkin_INCLUDE_DIRS}
)
endif()
target_link_libraries(calibrate
  direct_visual_lidar_calibration
)

# viewer
add_executable(viewer
  src/viewer.cpp
)

if (ENABLE_ROS)
target_include_directories(viewer PRIVATE
  ${catkin_INCLUDE_DIRS}
)
endif()

target_link_libraries(viewer
  direct_visual_lidar_calibration
)

include(GNUInstallDirs)

install(
  TARGETS
  direct_visual_lidar_calibration
  EXPORT ${PROJECT_NAME}_Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  FILE_SET public_headers DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration/
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/sophus/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration/sophus
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/json/include/nlohmann/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration/nlohmann/
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/nanoflann/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration/
)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}_Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${INSTALL_CONFIGDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

# install(
#   TARGETS
#   preprocess_map
#   initial_guess_manual
#   initial_guess_auto
#   calibrate
#   viewer
#   DESTINATION
#   lib/${PROJECT_NAME}
# )

if(ENABLE_ROS)
    if($ENV{ROS_VERSION} EQUAL 1)
      # ROS1
    else()
      # ROS2
      ament_python_install_package(${PROJECT_NAME})
      install(PROGRAMS scripts/find_matches_superglue.py DESTINATION lib/${PROJECT_NAME})
      ament_export_libraries(direct_visual_lidar_calibration)
      ament_auto_package()
    endif()
endif()
