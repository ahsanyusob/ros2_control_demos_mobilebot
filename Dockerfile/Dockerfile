ARG ROS_DISTRO=rolling

FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get upgrade -y \
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

COPY . /home/ros2_ws/src/ros2_control_demos

RUN cd /home/ros2_ws \
    # && vcs import src < src/ros2_control_demos/ros2_control_demos.$ROS_DISTRO.repos \  # uncomment for building dependencies from source
    && rosdep update --rosdistro ${ROS_DISTRO}  \
    # uncomment this line if you want to use the latest version of ros2_control
    # && vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.rolling-on-$ROS_DISTRO.repos \
    && apt-get update \
    && rosdep install --from-paths src/ -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys ros-${ROS_DISTRO}-joint-state-publisher-gui --skip-keys ros-${ROS_DISTRO}-rviz2\
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

RUN cd /home/ros2_ws/ \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --merge-install --symlink-install

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

COPY Dockerfile/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch ros2_control_demo_bicycledrivebot_carlike carlikebot.launch.py remap_odometry_tf:=true
