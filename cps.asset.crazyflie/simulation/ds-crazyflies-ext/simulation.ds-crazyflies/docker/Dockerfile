FROM osrf/ros:humble-desktop@sha256:fb07245b32187d74350be25323d8ad2f8ca5c25c325759911a1eff2267a49c1e
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive WEBOTS_HOME=/usr/local/webots \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONDONTWRITEBYTECODE=1 \
    CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates nano graphviz xvfb libxcb-cursor0 libxtst6 \
    libeigen3-dev python3-pip python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rosbridge-server \
    && curl -fL --retry 3 https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb -o /tmp/webots.deb \
    && echo '6253d58c9b625a83ed7b62cd85a640fd0542d441c48d633a60932208b40b0657  /tmp/webots.deb' | sha256sum -c - \
    && apt-get install -y --no-install-recommends /tmp/webots.deb \
    && rm -f /tmp/webots.deb && rm -rf /var/lib/apt/lists/*
COPY controller/cf.PyControl/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt
COPY simulation/ds-crazyflies-ext/simulation.ds-crazyflies/src /ds/ds-crazyflies/src
WORKDIR /ds/ds-crazyflies
RUN source /opt/ros/humble/setup.bash && colcon build --executor sequential \
    --packages-up-to crazyflie_webots crazyflie_webots_gateway \
    --cmake-args -DBUILD_TESTING=OFF
COPY simulation/ds-crazyflies-ext/simulation.ds-crazyflies/resources /ds/crazywebotsworld
COPY simulation/ds-crazyflies-ext/crazyflie.wbt /ds/crazywebotsworld/worlds/crazyflie.wbt
COPY simulation/ds-crazyflies-ext/simulation.ds-crazyflies/tools /ds/ds-crazyflies/tools
COPY simulation/ds-crazyflies-ext/simulation.ds-crazyflies/config.json /ds/ds-crazyflies/config.json
COPY simulation/ds-crazyflies-ext/simulation.ds-crazyflies/LICENSE.webots /ds/ds-crazyflies/LICENSE.webots
COPY simulation/ds-crazyflies-ext/simulation.ds-crazyflies/UPSTREAM.md /ds/ds-crazyflies/UPSTREAM.md
COPY controller /workspace/controller
COPY simulation/ds-crazyflies-ext/simulation.ds-crazyflies/docker/environment.sh /usr/local/bin/ds-env
RUN apt-get update && apt-get install -y --no-install-recommends mesa-utils && rm -rf /var/lib/apt/lists/*
RUN chmod +x /usr/local/bin/ds-env \
    && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /ds/ds-crazyflies/install/setup.bash' >> /root/.bashrc \
    && echo 'export ROS_DOMAIN_ID=41 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp WEBOTS_HOME=/usr/local/webots' >> /root/.bashrc
LABEL io.fedt.stack="fedt-bigrid-uas.webots"
ENTRYPOINT ["/usr/local/bin/ds-env"]
CMD ["python3", "tools/runtime.py"]
