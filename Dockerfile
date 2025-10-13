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
RUN mkdir -p /home/ubuntu/catkin_ws/src/roVANT-Simulator_Developer
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

WORKDIR /home/ubuntu/catkin_ws

