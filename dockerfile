ARG ROS_DISTRO
FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full

SHELL ["/bin/bash", "-c"]

ENV LOTUSIM_WS=/lotusim_ws \
    LOTUSIM_PATH=/lotusim_ws/src/LOTUSim \
    PATH=/lotusim_ws/src/LOTUSim/physics:/lotusim_ws/src/LOTUSim/launch:$PATH \
    LD_LIBRARY_PATH=/lotusim_ws/src/LOTUSim/physics:$LD_LIBRARY_PATH \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4     

WORKDIR ${LOTUSIM_WS}

COPY . src/LOTUSim

RUN \
  source /opt/ros/${ROS_DISTRO}/setup.bash && \
  chmod +x $LOTUSIM_PATH/launch/lotusim && \
  lotusim install
