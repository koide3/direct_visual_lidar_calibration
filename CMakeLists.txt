cmake_minimum_required(VERSION 3.0.2)
project(direct_visual_lidar_calibration)

add_compile_options(-std=c++20)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(catkin REQUIRED COMPONENTS rosbag sensor_msgs cv_bridge)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(glim REQUIRED)
find_package(Iridescence REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES direct_visual_lidar_calibration
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# direct_visual_lidar_calibration
add_library(direct_visual_lidar_calibration SHARED
  src/camera/create_camera.cpp
  src/vlcal/common/estimate_fov.cpp
  src/vlcal/common/estimate_pose.cpp
  src/vlcal/common/points_color_updater.cpp
  src/vlcal/common/visual_lidar_data.cpp
  src/vlcal/common/visual_lidar_visualizer.cpp
  src/vlcal/preprocess/generate_lidar_image.cpp
  src/vlcal/preprocess/static_point_cloud_integrator.cpp
  src/vlcal/preprocess/dynamic_point_cloud_integrator.cpp
  src/vlcal/calib/cost_calculator_nid.cpp
  src/vlcal/calib/cost_calculator_edge.cpp
  src/vlcal/calib/edge_extraction.cpp
  src/vlcal/calib/nearest_neighbor_search.cpp
  src/vlcal/calib/line_fitting.cpp
  src/vlcal/calib/view_culling.cpp
  src/vlcal/calib/visual_camera_calibration.cpp
)
target_include_directories(direct_visual_lidar_calibration PUBLIC
  include
  thirdparty/json/include
  thirdparty/Bonxai/include
  thirdparty/nanoflann/include
  thirdparty/Sophus
  ${PCL_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
)
target_link_libraries(direct_visual_lidar_calibration
  fmt
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${glim_LIBRARIES}
  ${Iridescence_LIBRARIES}
)

# preprocess
add_executable(preprocess
  src/preprocess.cpp
)
target_include_directories(preprocess PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(preprocess
  direct_visual_lidar_calibration
  ${catkin_LIBRARIES}
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
  ${catkin_LIBRARIES}
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
  ${catkin_LIBRARIES}
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
  ${catkin_LIBRARIES}
)



################ test code ################
add_executable(reference
  src/test/reference.cpp
  src/test/detect_sphere.cpp
)
target_include_directories(reference PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(reference
  direct_visual_lidar_calibration
  ${catkin_LIBRARIES}
)
