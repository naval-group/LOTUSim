ROS_DISTRO=jazzy

build:
	docker pull ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full
	docker build -t lotusim --build-arg=ROS_DISTRO=${ROS_DISTRO} .
