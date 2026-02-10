ARG ROS_DISTRO
FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full

SHELL ["/bin/bash", "-c"]

ENV LOTUSIM_WS=/lotusim_ws \
    LOTUSIM_PATH=/lotusim_ws/src/LOTUSim \
    PATH=/lotusim_ws/src/LOTUSim/physics:/lotusim_ws/src/LOTUSim/launch:$PATH \
    LOTUSIM_MODELS_PATH=/lotusim_ws/src/LOTUSim/assets/models/ \
    LD_LIBRARY_PATH=/lotusim_ws/src/LOTUSim/physics:$LD_LIBRARY_PATH \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4     

WORKDIR ${LOTUSIM_WS}

COPY . src/LOTUSim

RUN \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${LOTUSIM_PATH}/launch/bash_completion.sh" >> ~/.bashrc && \
    echo "source $LOTUSIM_WS/install/setup.bash" >> ~/.bashrc && \
    chmod +x $LOTUSIM_PATH/launch/lotusim && \
    lotusim install
