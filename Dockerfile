FROM osrf/ros:noetic-desktop-full

RUN apt update && apt dist-upgrade -y \
        && apt install --yes gpg wget curl git vim tmux cmake build-essential \
        qtcreator qt5-default qtchooser libqt5serialport5-dev mesa-utils \
        python3-pip xterm qtchooser libmsgsl-dev software-properties-common \
        vim tmux sudo manpages-dev \
        && rm -rf /var/lib/apt/lists/*


RUN pip3 install --upgrade conan

ADD https://cmake.org/files/v3.23/cmake-3.23.5.tar.gz /tmp/
WORKDIR /tmp
RUN tar -xvf cmake-3.23.5.tar.gz \
    && cd cmake-3.23.5 \
    && ./bootstrap --system-curl \
    && make -j$(nproc) \
    && make install

# Compile Ipopt
# Install dependencies
RUN apt-get update \
    && apt-get install \
    gfortran \
    patch \
    pkg-config \
    coinor-libipopt-dev \
    liblapack-dev \
    libmetis-dev \
    libopenblas-dev \
    file --install-recommends --yes \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get update && apt-get install --yes gcc-13 g++-13 \
    && rm -rf /var/lib/apt/lists/*
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 110
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 110

RUN pip install ninja
COPY ./install-casadi.sh /tmp
RUN /tmp/install-casadi.sh

RUN useradd -m ubuntu
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/90-ubuntu
USER ubuntu
RUN mkdir -p /home/ubuntu/catkin_ws/src/
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/ubuntu/.bashrc
RUN echo '[ -f "/home/ubuntu/catkin_ws/devel/setup.bash" ] && source /home/ubuntu/catkin_ws/devel/setup.bash' >> /home/ubuntu/.bashrc
RUN sudo chmod 440 /etc/sudoers

COPY install-boost.sh /tmp/
RUN sudo /tmp/install-boost.sh \
    && sudo rm -f /tmp/install-boost.sh

WORKDIR /home/ubuntu
RUN --mount=type=bind,source=./shared/catkin_ws,target=/home/ubuntu/catkin_ws,rw \
    [ -d "/home/ubuntu/catkin_ws/src/ProVANT-Simulator_Developer/" ] \
    && cd /home/ubuntu/catkin_ws/src/ProVANT-Simulator_Developer/ \
    && sudo bash -c "awk '!/conan/ { print }' install.sh > install_no_conan.sh" \
    && sudo chmod +x install_no_conan.sh \
    && sudo bash -c 'source /opt/ros/noetic/setup.bash && ./install_no_conan.sh'

RUN ln -s /mnt/shared/.bash_history /home/ubuntu/.bash_history \
    && ln -s /mnt/shared/.bash_config /home/ubuntu/.bash_config \
    && ln -s /mnt/shared /home/ubuntu/shared

WORKDIR /home/ubuntu/catkin_ws

