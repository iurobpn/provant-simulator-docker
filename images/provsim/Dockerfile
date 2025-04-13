FROM osrf/ros:noetic-desktop-full AS ros-noetic
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install --yes \
        git \
        wget \
        gpg \
        build-essential \
        qtcreator \
        qt5-default \
        qtchooser \
        libqt5serialport5-dev \
        curl \
        ca-certificates \
        # cmake \
        build-essential \
        libmsgsl-dev \
        python3 \
        python3-pip \
        software-properties-common 

## conan
ADD https://github.com/conan-io/conan/releases/download/1.66.0/conan-ubuntu-64.deb /tmp/
RUN dpkg -i /tmp/conan-ubuntu-64.deb 

## cmake
# RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg > /dev/null
# RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list > /dev/null

# RUN apt-get update && apt-get install --yes kitware-archive-keyring
# RUN apt install --yes cmake

## QT - in the apt install command above
## ROS
# ros is already installed in this image

RUN apt install --yes python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool

RUN python3 -m pip install cmake==3.31.6

RUN rosdep init 2> /dev/null || echo "rosdep already initialized"
RUN rosdep update

######### workspace configuration ###########
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/
# RUN catkin_make let's do it iterativelly!
# RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "source /root/config/provant.env" >> /root/.bashrc

########### ros environment config ###########
RUN echo ". /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "[ -f /root/catkin_ws/devel/setup.bash ] && . /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Install graphical libraries for gazebo
RUN apt-get update \
    && apt-get install -y \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl1-mesa \
    libgbm1

# RUN make
RUN ln -sf /root/catkin_ws/src/ProVANT-Software_Developer/source/build/GUI /usr/local/bin/provant_gui

VOLUME [ "/root/catkin_ws/", "/root/.ros/", "/root/.gazebo/"]
VOLUME "/root/.bash_history"
WORKDIR /root/catkin_ws

