cmake_minimum_required(VERSION 3.8)
project(crazyflie_webots_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(gateway src/gateway.cpp)
target_include_directories(gateway PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gateway PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(gateway
  rclcpp
  crazyflie_interfaces rosgraph_msgs)


set(GATEWAY_DEPS
  src/crazyflie_lifecycle_client.cpp)
add_library(gateway_deps ${GATEWAY_DEPS})
target_compile_features(gateway_deps PUBLIC cxx_std_17)
target_include_directories(gateway_deps PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)
ament_target_dependencies(gateway_deps
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs)


target_link_libraries(gateway gateway_deps)

install(TARGETS gateway
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
