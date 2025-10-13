FROM osrf/ros:noetic-desktop-full

RUN apt update && apt dist-upgrade -y \
        && apt install --yes gpg wget curl git vim tmux cmake build-essential \
        qtcreator qt5-default qtchooser libqt5serialport5-dev mesa-utils \
        python3-pip xterm qtchooser libmsgsl-dev software-properties-common \
        vim tmux sudo \
        && rm -rf /var/lib/apt/lists/*


RUN pip3 install --upgrade conan==1.63.0

ADD https://cmake.org/files/v3.23/cmake-3.23.5.tar.gz /tmp/
WORKDIR /tmp
RUN tar -xvf cmake-3.23.5.tar.gz \
    && cd cmake-3.23.5 \
    && ./bootstrap --system-curl \
    && make -j$(nproc) \
    && make install

RUN useradd -m ubuntu
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/90-ubuntu
USER ubuntu
RUN mkdir -p /home/ubuntu/catkin_ws/src
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/ubuntu/.bashrc
RUN echo "source /home/ubuntu/catkin_ws/devel/setup.bash" >> /home/ubuntu/.bashrc
RUN sudo chmod 440 /etc/sudoers


WORKDIR /home/ubuntu/catkin_ws

