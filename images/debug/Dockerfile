FROM osrf/ros:noetic-desktop-full

RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install --yes \
        git \
        wget \
        curl \
        ca-certificates \
        gpg \
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
        libpolyclipping-dev \
        software-properties-common 

RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt dist-upgrade --yes \
    && apt-get install --yes python3.12

ADD https://bootstrap.pypa.io/get-pip.py /tmp/
RUN python3.12 /tmp/get-pip.py

RUN python3.12 -m pip install pipx
RUN python3.12 -m pipx ensurepath --global

RUN python3.12 -m pip install virtualenv
# RUN pip install conan==1.66.0

ADD https://github.com/conan-io/conan/releases/download/1.66.0/conan-ubuntu-64.deb /tmp/
RUN dpkg -i /tmp/conan-ubuntu-64.deb 

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg > /dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list > /dev/null


####### README installation ############
RUN ! [ -f  /etc/ros/rosdep/sources.list.d/20-default.list ] && rosdep init 2> /dev/null || echo "rosdep already initialized"
RUN rosdep update

# QT5 installation is executed via ubuntu repositories (above)
# ADD https://d13lb3tujbc8s0.cloudfront.net/onlineinstallers/qt-online-installer-linux-x64-4.9.0.run /tmp
# RUN chmod +x /tmp/qt-unified-linux* \
    # && /tmp/qt-unified-linux*

######### workspace configuration ###########
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/
# RUN catkin_make let's do it iterativelly!
# RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

########### bash config ###########
RUN echo ". /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "[ -f /root/catkin_ws/devel/setup.bash ] && . /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
# gpg cmake are already installed


############## DEBUG TOOLS ##############
# install tmux
RUN apt-get install --yes byacc bison
ENV TMUX_VERSION=3.5a
ADD https://github.com/tmux/tmux/releases/download/3.5a/tmux-${TMUX_VERSION}.tar.gz /tmp/
RUN tar -xzf /tmp/tmux-${TMUX_VERSION}.tar.gz -C /tmp \
    && cd /tmp/tmux-* \
    && ./configure \
    && make \
    && make install

# install neovim configuration
WORKDIR /root
ADD https://raw.githubusercontent.com/mrzool/bash-sensible/refs/heads/master/sensible.bash /root/

#######        vim 9 install           ##################
RUN add-apt-repository ppa:jonathonf/vim
RUN apt-get update && apt-get install --yes vim

#######       fzf install           #####################
RUN git clone --depth 1 https://github.com/junegunn/fzf.git /root/.fzf
RUN /root/.fzf/install --all

####### bash config  - ajuste as suas necessidades ######
RUN echo ". /root/config/.bashrc_base" >> /root/.bashrc

WORKDIR /root
RUN ln -s /root/config/lscolors.sh \
    && ln -s /root/config/.bash_aliases \
    && ln -s /root/config/.bashrc_base \
    && ln -s /root/config/.gitprompt.sh \
    && ln -s /root/config/.vim \
    && ln -s /root/config/.tmux.conf \
    && ln -s /root/config/.vimrc
    # && ln -s /root/config/.luapaths \

#### NODE installation #####
ENV NODE_VERSION=22
ENV NVM_VERSION=v0.40.2
RUN apt install -y curl
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/${NVM_VERSION}/install.sh | bash
ENV NVM_DIR=/root/.nvm
RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm use current
RUN echo "export NVM_DIR=\"$NVM_DIR\"" >> /root/.bashrc
RUN echo "[ -f \"$NVM_DIR/nvm.sh\" ] && . \"$NVM_DIR/nvm.sh\"" >> /root/.bashrc

WORKDIR /root/catkin_ws

