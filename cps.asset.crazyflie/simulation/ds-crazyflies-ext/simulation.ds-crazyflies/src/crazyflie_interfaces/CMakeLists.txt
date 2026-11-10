cmake_minimum_required(VERSION 3.10)
project(crazyflie_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConnectionStatistics.msg"
  "msg/ConnectionStatisticsArray.msg"
  "msg/CrazyflieLinkQuality.msg"
  "msg/CrazyflieLinkQualityArray.msg"
  "msg/FullState.msg"
  "msg/LogDataGeneric.msg"
  "msg/Hover.msg"
  "msg/LogBlock.msg"
  "msg/Position.msg"
  "msg/Status.msg"
  "msg/TrajectoryPolynomialPiece.msg"
  "msg/VelocityWorld.msg"
  "srv/GoTo.srv"
  "srv/Land.srv"
  "srv/NotifySetpointsStop.srv"
  "srv/SetGroupMask.srv"
  "srv/StartTrajectory.srv"
  "srv/Stop.srv"
  "srv/Takeoff.srv"
  "srv/UpdateParams.srv"
  "srv/UploadTrajectory.srv"
  "srv/RemoveLogging.srv"
  "srv/AddLogging.srv"
  "srv/Arm.srv"
  "srv/AddCrazyflie.srv"
  "srv/RemoveCrazyflie.srv"
  "msg/PoseNamed.msg"
  "msg/PoseNamedArray.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
