# Gurdy 3.0 Phase 9e: MoveIt 2 + pick_ik ON TOP of the existing ROS image (decided 2026-11-27).
# gurdy2/ros:jazzy is never modified or retagged; this image only adds packages. Used only for arm:=true work
# (compose service `sim_moveit`; moveit-ros-visualization = RViz MotionPlanning panel); everything else keeps running on gurdy2/ros:jazzy.
#   docker compose build sim_moveit         -> gurdy3/ros:jazzy-moveit
FROM gurdy2/ros:jazzy

USER root
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-moveit-ros-move-group \
      ros-jazzy-moveit-planners-ompl \
      ros-jazzy-moveit-kinematics \
      ros-jazzy-moveit-simple-controller-manager \
      ros-jazzy-moveit-configs-utils \
      ros-jazzy-moveit-msgs \
      ros-jazzy-pick-ik \
      ros-jazzy-moveit-ros-visualization \
    && rm -rf /var/lib/apt/lists/*
USER ubuntu
