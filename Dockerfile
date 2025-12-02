FROM osrf/ros:noetic-desktop-full
ARG USER=ubuntu
ARG UID=1000
ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/home/${USER}
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

RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get update && apt-get install --yes gcc-13 g++-13 \
    && rm -rf /var/lib/apt/lists/*
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 110
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 110

RUN pip install ninja
COPY ./install-casadi.sh /tmp
RUN /tmp/install-casadi.sh

RUN useradd -u ${UID} -m ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/90-${USER}
RUN chown -R ${USER}:${USER} ${HOME}
USER ${USER}

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USER}/.bashrc
RUN echo "source /home/${USER}/catkin_ws/devel/setup.bash" >> /home/${USER}/.bashrc
RUN sudo chmod 440 /etc/sudoers

RUN sudo ln -s /home/${USER}/catkin_ws/src/ProVANT_Simulator/source/build/GUI /usr/local/bin/provant_gui

# ProVANT Simulator Environment Variables
RUN echo 'export TILT_PROJECT=$HOME/catkin_ws/src/ProVANT_Simulator/' >> /home/${USER}/.bashrc
RUN echo 'export PROVANT_ROS=$HOME/catkin_ws/src/' >> /home/${USER}/.bashrc
RUN echo 'export DIR_ROS=$HOME/catkin_ws/' >> /home/${USER}/.bashrc
RUN echo 'export TILT_STRATEGIES=$HOME/catkin_ws/devel/lib/' >> /home/${USER}/.bashrc
RUN echo 'export TILT_MATLAB=$HOME/catkin_ws/src/ProVANT_Simulator/source/Structure/Matlab/' >> /home/${USER}/.bashrc
RUN echo 'export PROVANT_DATABASE=$HOME/catkin_ws/src/ProVANT_Simulator/source/Database/' >> /home/${USER}/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/ProVANT_Simulator/source/Database/models/' >> /home/${USER}/.bashrc
RUN echo 'export CONTROL_STRATEGIES_SOURCE=$HOME/catkin_ws/src/ProVANT_Simulator/source/Structure/control_strategies' >> /home/${USER}/.bashrc

WORKDIR /home/${USER}/catkin_ws

