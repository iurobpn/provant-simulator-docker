FROM osrf/ros:noetic-desktop-full

RUN apt update && apt dist-upgrade -y
RUN  apt install --yes gpg wget git
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN apt update
RUN apt install --yes kitware-archive-keyring
RUN apt install --yes cmake
RUN apt install --yes build-essential qtcreator qt5-default qtchooser \
		libqt5serialport5-dev mesa-utils python3-pip xterm qtchooser \
		libmsgsl-dev libpolyclipping-dev fzf software-properties-common

RUN pip3 install --upgrade conan

# Compile Ipopt
# Install dependencies
RUN apt install gfortran patch pkg-config liblapack-dev libmetis-dev libopenblas-dev file --install-recommends --yes
RUN mkdir -p /root/ipopt

ADD https://cmake.org/files/v3.23/cmake-3.23.5.tar.gz /tmp/

WORKDIR /tmp
RUN tar -xvf cmake-3.23.5.tar.gz \
    && cd cmake-3.23.5 \
    && ./bootstrap --system-curl \
    && make -j$(nproc) \
    && make install

COPY install-cmake.sh /root

WORKDIR /root/catkin_ws

