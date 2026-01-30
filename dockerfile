ARG ROS_DISTRO
FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full

SHELL ["/bin/bash", "-c"]

ENV LOTUSIM_WS=/lotusim_ws
ENV LOTUSIM_PATH=/lotusim_ws/src/LOTUSim

WORKDIR ${LOTUSIM_WS}

COPY . src/LOTUSim

RUN \
  source /opt/ros/${ROS_DISTRO}/setup.bash && \
  export PATH=$LOTUSIM_PATH/physics:$LOTUSIM_PATH/launch:$PATH && \
  chmod +x $LOTUSIM_PATH/launch/lotusim && \
  lotusim install
