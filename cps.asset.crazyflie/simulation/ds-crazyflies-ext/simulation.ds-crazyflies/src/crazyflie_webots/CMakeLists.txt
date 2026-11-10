cmake_minimum_required(VERSION 3.8)
project(crazyflie_webots)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(NOT DEFINED ENV{WEBOTS_HOME})
  message(FATAL_ERROR "Environment variable WEBOTS_HOME is not set. 
  Please set it to the Webots installation directory.
  Typically: export WEBOTS_HOME=/usr/local/webots")
endif()
set(WEBOTS_HOME $ENV{WEBOTS_HOME})

## Include official Webots C/C++ controller API
include_directories(
  ${WEBOTS_HOME}/include/controller/c
  ${WEBOTS_HOME}/include/controller/cpp
)
link_directories(${WEBOTS_HOME}/lib/controller)

# Automatically embed Webots controller library path in all targets
set(CMAKE_INSTALL_RPATH "${WEBOTS_HOME}/lib/controller")
set(CMAKE_BUILD_RPATH   "${WEBOTS_HOME}/lib/controller")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)


## Build the Custom Webots Driver Library
set(WEBOTS_DRIVER_SRC
  src/webots_driver/webots_robot_driver.cpp
  src/webots_driver/webots_crazyflie_driver.cpp
  src/webots_driver/webots_wand_driver.cpp
)
add_library(webots_driver_lib ${WEBOTS_DRIVER_SRC})
set_target_properties(webots_driver_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(webots_driver_lib PUBLIC cxx_std_17) 
target_include_directories(webots_driver_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)

target_link_libraries(webots_driver_lib
  Controller
  CppController
  Eigen3::Eigen
)


## Build the CRTP Driver Library
set(CRTP_DRIVER_SRC
  src/crtp_driver/console.cpp
  src/crtp_driver/generic_commander.cpp
  src/crtp_driver/hl_commander.cpp
  src/crtp_driver/localization.cpp
  src/crtp_driver/logging.cpp
  src/crtp_driver/logblock.cpp
  src/crtp_driver/parameters.cpp)
add_library(crtp_driver_lib ${CRTP_DRIVER_SRC})
set_target_properties(crtp_driver_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(crtp_driver_lib PUBLIC cxx_std_17)
target_include_directories(crtp_driver_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)

target_link_libraries(crtp_driver_lib
  webots_driver_lib  
  Eigen3::Eigen)

ament_target_dependencies(crtp_driver_lib
  rclcpp crazyflie_interfaces nav_msgs sensor_msgs rosgraph_msgs std_msgs std_srvs)

# Build the crazyflie node executable
add_executable(crazyflie src/crazyflie.cpp)

target_link_libraries(crazyflie
  webots_driver_lib
  crtp_driver_lib
  Controller
  CppController
)

ament_target_dependencies(crazyflie 
  rclcpp_lifecycle
  rclcpp
  crazyflie_interfaces)

target_include_directories(crazyflie PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(crazyflie PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

## Build the wand node executable
add_executable(wand src/wand.cpp)
target_link_libraries(wand
  webots_driver_lib
  Controller
  CppController
)

ament_target_dependencies(wand 
  rclcpp_lifecycle
  rclcpp
  tf2
  tf2_ros
  geometry_msgs)

target_include_directories(wand PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(wand PUBLIC c_std_99 cxx_std_17)  # Require

## Finally build all nodes
install(TARGETS crazyflie wand
  DESTINATION lib/${PROJECT_NAME})

ament_package()

# export WEBOTS_HOME=/usr/local/webots
# export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH