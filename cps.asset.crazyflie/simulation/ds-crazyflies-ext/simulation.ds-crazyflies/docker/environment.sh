#!/usr/bin/env bash
set -e
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source /ds/ds-crazyflies/install/setup.bash
export ROS_DOMAIN_ID=${FEDT_ROS_DOMAIN:-41}
export WEBOTS_HOME=/usr/local/webots RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH="/usr/local/webots/lib/controller:${LD_LIBRARY_PATH:-}"
if [ -e /dev/dxg ] && [ -d /usr/lib/wsl/lib ]; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH"
fi
export PYTHONPATH="/workspace/controller/cf.PyControl/src:${PYTHONPATH:-}"
exec "$@"
