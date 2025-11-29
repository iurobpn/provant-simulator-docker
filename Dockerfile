FROM osrf/ros:noetic-desktop-full

RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install --yes \
    gpg \
    wget \
    curl \
    git \
    cmake \
    build-essential \
    qtcreator \
    qt5-default \
    qtchooser \
    libqt5serialport5-dev \
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
    && rm -rf /var/lib/apt/lists/*


RUN pip3 install --upgrade conan

# ADD https://cmake.org/files/v3.23/cmake-3.23.5.tar.gz /tmp/
# WORKDIR /tmp
# RUN tar -xvf cmake-3.23.5.tar.gz \
#     && cd cmake-3.23.5 \
#     && ./bootstrap --system-curl \
#     && make -j$(nproc) \
#     && make install

# RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
#     && apt-get update && apt-get install --yes gcc-13 g++-13 \
#     && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 110 \
#     && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 110 \
#     && rm -rf /var/lib/apt/lists/*

RUN pip install ninja
COPY ./install-*.sh /tmp
RUN /tmp/install-cmake.sh \
    && /tmp/install-gcc.sh \
    && /tmp/install-casadi.sh \
    && /tmp/install-boost.sh \
    && sudo rm -f /tmp/install-*.sh

ARG USER=ubuntu
RUN useradd -m ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/90-${USER}
USER ${USER}
RUN mkdir -p /home/${USER}/catkin_ws/src/ \
    && echo "source /opt/ros/noetic/setup.bash" >> /home/${USER}/.bashrc \
    && echo '[ -f "/home/${USER}/catkin_ws/devel/setup.bash" ] && source /home/${USER}/catkin_ws/devel/setup.bash' >> /home/${USER}/.bashrc \
    && sudo chmod 440 /etc/sudoers


WORKDIR /home/${USER}
RUN --mount=type=bind,source=./shared/catkin_ws,target=/mnt/shared/catkin_ws,rw \
    [ -d "/mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/" ] \
    && cd /mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/ \
    && sudo bash -c 'source /opt/ros/noetic/setup.bash && ./install.sh' \
    && sudo rm -f /usr/local/bin/provant_gui \
    && sudo cp /mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/source/build/GUI /usr/local/bin/provant_gui

RUN ln -s /mnt/shared/.bash_history /home/${USER}/.bash_history \
    && ln -s /mnt/shared/.bash_config /home/${USER}/.bash_config \
    && ln -s /mnt/shared /home/${USER}/shared \
    && ln -s /mnt/shared/catkin_ws /home/${USER}/catkin_ws \
    && echo 'export BOOST_ROOT=/usr/local' >> /home/${USER}/.bashrc \
    && echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH'  >> /home/${USER}/.bashrc \
    && echo 'export CPLUS_INCLUDE_PATH=/usr/local/include:$CPLUS_INCLUDE_PATH'  >> /home/${USER}/.bashrc

COPY ./shared/.prov /home/${USER}/

WORKDIR /mnt/shared

