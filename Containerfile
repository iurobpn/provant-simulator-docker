FROM osrf/ros:jazzy-desktop-full

ARG USER=ubuntu
ARG UID=1000

RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install --yes \
    gpg \
    wget \
    curl \
    git \
    cmake \
    build-essential \
    mesa-utils \
    python3-pip \
    xterm \
    qtchooser \
    libmsgsl-dev \
    software-properties-common \
    vim \
    tmux \
    sudo \
    manpages-dev \
    gfortran \
    patch \
    pkg-config \
    coinor-libipopt-dev \
    liblapack-dev \
    libmetis-dev \
    libopenblas-dev \
    file \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    python3-opencv \
    python3-dev \
    libeigen3-dev \
    python3-aiohttp \
    python3-numpy \
    python3-pip \
    python3-av \
    python3-fastapi \
    python3-pytest \
    python3-scipy \
    libcurl4-openssl-dev \
    python3-uvicorn \
    python3-pyproj \
    python3-pydantic \
    zlib1g-dev \
    python3-aiohttp-cors \
    python3-flask-socketio \
    python3-yaml \
    python3-requests \
    libarmadillo-dev \
    python3-aiortc \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages --upgrade conan ninja cmake

COPY ./install-*.sh /tmp
RUN /tmp/install-gcc.sh \
    && /tmp/install-casadi.sh \
    && /tmp/install-boost.sh \
    && sudo rm -f /tmp/install-*.sh

# add current user, delete standard ubuntu user
RUN bash -c "[[ \"${USER}\" != \"ubuntu\" ]] && userdel ubuntu || true" \
    && bash -c "! id ${USER} > /dev/null 2> /dev/null && useradd -u ${UID} -m ${USER} && echo \"${USER} ALL=(ALL) NOPASSWD:ALL\" | sudo tee /etc/sudoers.d/90-${USER}" \
    && usermod -a -G video || true \
    && usermod -a -G render || true

USER ${USER}
RUN sudo chmod 440 /etc/sudoers

# WORKDIR /home/${USER}
# RUN --mount=type=bind,source=./shared/catkin_ws,target=/mnt/shared/catkin_ws,rw \
#     [ -d "/mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/" ] \
#     && cd /mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/ \
#     && sudo rm -f /usr/local/bin/provant_gui \
#     && sudo cp /mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/source/build/GUI /usr/local/bin/provant_gui

RUN ln -s /mnt/shared/.bash_history /home/${USER}/.bash_history \
    && ln -s /mnt/shared/.zsh_history /home/${USER}/.zsh_history

COPY ./shared/.prov /home/${USER}/

RUN bash -c "source /opt/ros/jazzy/setup.bash && rosdep update"

ENTRYPOINT []
WORKDIR /mnt/shared/ros2_ws/

