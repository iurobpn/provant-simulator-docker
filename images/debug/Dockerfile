ARG BASE=provsim
FROM ${BASE} AS provdebug

RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install --yes \
        git \
        wget \
        gpg \
        curl \
        ca-certificates \
        build-essential \
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


############## DEBUG TOOLS ##############
## install tmux
RUN apt-get install --yes byacc bison
ENV TMUX_VERSION=3.5a
ADD https://github.com/tmux/tmux/releases/download/3.5a/tmux-${TMUX_VERSION}.tar.gz /tmp/
RUN tar -xzf /tmp/tmux-${TMUX_VERSION}.tar.gz -C /tmp \
    && cd /tmp/tmux-* \
    && ./configure \
    && make \
    && make install

## bash configurations
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

## rust and  ago packages
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN /root/.cargo/bin/cargo install ripgrep
RUN /root/.cargo/bin/cargo install fd-find

RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"

WORKDIR /root/catkin_ws
