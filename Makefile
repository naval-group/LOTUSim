ROS_DISTRO=jazzy

build:
	docker pull ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full
	docker build --no-cache -t lotusim --build-arg=ROS_DISTRO=${ROS_DISTRO} .

it:
	# docker run -it lotusim
	docker run -d --name my_lotusim --entrypoint /bin/bash lotusim -c "sleep infinity"
	docker exec my_lotusim lotusim --help
	docker exec -d my_lotusim xdyn-for-cs /lotusim_ws/src/LOTUSim/assets/models/lrauv/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346