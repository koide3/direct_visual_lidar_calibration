cmake_minimum_required(VERSION 3.5.2)
project(direct_visual_lidar_calibration)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

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

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(Iridescence REQUIRED)

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
  include
  thirdparty/json/include
  thirdparty/Bonxai/include
  thirdparty/nanoflann/include
  thirdparty/Sophus
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
)
target_link_libraries(direct_visual_lidar_calibration
  fmt
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Iridescence_LIBRARIES}
)


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

# preprocess_map
add_executable(preprocess_map
  src/preprocess_map.cpp
)
target_include_directories(preprocess_map PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(preprocess_map
  direct_visual_lidar_calibration
)

# initial_guess_manual
add_executable(initial_guess_manual
  src/initial_guess_manual.cpp
)
target_include_directories(initial_guess_manual PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(initial_guess_manual
  direct_visual_lidar_calibration
)

# initial_guess_auto
add_executable(initial_guess_auto
  src/initial_guess_auto.cpp
)
target_include_directories(initial_guess_auto PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(initial_guess_auto
  direct_visual_lidar_calibration
)

# calibrate
add_executable(calibrate
  src/calibrate.cpp
)
target_include_directories(calibrate PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(calibrate
  direct_visual_lidar_calibration
)

# viewer
add_executable(viewer
  src/viewer.cpp
)
target_include_directories(viewer PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(viewer
  direct_visual_lidar_calibration
)

install(
  TARGETS
  direct_visual_lidar_calibration
)

install(
  TARGETS
  preprocess_map
  initial_guess_manual
  initial_guess_auto
  calibrate
  viewer
  DESTINATION
  lib/${PROJECT_NAME}
)

if($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
else()
  # ROS2
  ament_python_install_package(${PROJECT_NAME})
  install(PROGRAMS scripts/find_matches_superglue.py DESTINATION lib/${PROJECT_NAME})
  ament_export_libraries(direct_visual_lidar_calibration)
  ament_auto_package()
endif()