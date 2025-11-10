FROM osrf/ros:noetic-desktop-full

RUN apt update && apt dist-upgrade -y \
        && apt install --yes gpg wget curl git vim tmux cmake build-essential \
        qtcreator qt5-default qtchooser libqt5serialport5-dev mesa-utils \
        python3-pip xterm qtchooser libmsgsl-dev software-properties-common \
        vim tmux sudo manpages-dev \
        && rm -rf /var/lib/apt/lists/*


RUN pip3 install --upgrade conan==1.63.0

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

# compile and install HSL libraries
# COPY ./coinhsl-2024.05.15.tar.gz /root/ipopt
# COPY ./install-hsl.sh /root/ipopt
# RUN /root/ipopt/install-hsl.sh
#
# COPY ./install-mumps.sh /root/ipopt
# RUN /root/ipopt/install-mumps.sh
#
# COPY ./install-ipopt.sh /root/ipopt
# RUN /root/ipopt/install-ipopt.sh
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
RUN mkdir -p /home/ubuntu/catkin_ws/src/ProVANT-Simulator_Developer
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/ubuntu/.bashrc
RUN echo "source /home/ubuntu/catkin_ws/devel/setup.bash" >> /home/ubuntu/.bashrc
RUN sudo chmod 440 /etc/sudoers

# RUN --mount=type=bind,source=./shared/catkin_ws,target=/home/ubuntu/catkin_ws \
#     cd /home/ubuntu/catkin_ws/src/ProVANT_Simulator_Developer/ \
#     && ./install.sh
RUN sudo ln -s /home/ubuntu/catkin_ws/src/ProVANT_Simulator/source/build/GUI /usr/local/bin/provant_gui

# ProVANT Simulator Environment Variables
RUN echo 'export TILT_PROJECT=$HOME/catkin_ws/src/ProVANT_Simulator/' >> /home/ubuntu/.bashrc
RUN echo 'export PROVANT_ROS=$HOME/catkin_ws/src/' >> /home/ubuntu/.bashrc
RUN echo 'export DIR_ROS=$HOME/catkin_ws/' >> /home/ubuntu/.bashrc
RUN echo 'export TILT_STRATEGIES=$HOME/catkin_ws/devel/lib/' >> /home/ubuntu/.bashrc
RUN echo 'export TILT_MATLAB=$HOME/catkin_ws/src/ProVANT_Simulator/source/Structure/Matlab/' >> /home/ubuntu/.bashrc
RUN echo 'export PROVANT_DATABASE=$HOME/catkin_ws/src/ProVANT_Simulator/source/Database/' >> /home/ubuntu/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/ProVANT_Simulator/source/Database/models/' >> /home/ubuntu/.bashrc
RUN echo 'export CONTROL_STRATEGIES_SOURCE=$HOME/catkin_ws/src/ProVANT_Simulator/source/Structure/control_strategies' >> /home/ubuntu/.bashrc

USER root
WORKDIR /tmp
COPY ./install-boost.sh /tmp
RUN /tmp/install-boost.sh \
    && rm -rf /tmp/install-boost.sh

USER ubuntu
WORKDIR /home/ubuntu/catkin_ws

