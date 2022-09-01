cmake_minimum_required(VERSION 3.0.2)
project(direct_visual_lidar_calibration)

add_compile_options(-std=c++20)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(catkin REQUIRED COMPONENTS rosbag sensor_msgs cv_bridge)

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


add_library(direct_visual_lidar_calibration SHARED
  src/vlcal/preprocess/static_point_cloud_integrator.cpp
  src/vlcal/preprocess/dynamic_point_cloud_integrator.cpp
)
target_include_directories(direct_visual_lidar_calibration PUBLIC
  include
  thirdparty/json/include
  thirdparty/Bonxai/include
  ${glim_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
)
target_link_libraries(direct_visual_lidar_calibration
  ${glim_LIBRARIES}
  ${Iridescence_LIBRARIES}
)

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

# add_executable(calib
#   src/calib.cpp
# )
# target_include_directories(calib PRIVATE
#   ${catkin_INCLUDE_DIRS}
# )
# target_link_libraries(calib
#   direct_visual_lidar_calibration
#   ${catkin_LIBRARIES}
# )