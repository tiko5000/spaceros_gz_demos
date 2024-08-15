FROM osrf/space-ros:latest

# https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu
RUN sudo apt-get update
RUN sudo apt-get install lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN yes | sudo apt-get install gz-harmonic

RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        alsa-utils \
        apache2-utils \
        bash \
        bash-completion \
        build-essential \
        ca-certificates \
        clang-14 \
        clang-tidy-14 \
        cmake \
        cmake-curses-gui \
        curl \
        dbus-x11 \
        ffmpeg \
        git \
        gnupg \
        gpg-agent \
        gpgconf \
        htop \
        iproute2 \
        iputils-ping \
        libboost-dev \
        libdrm-dev \
        libgl1-mesa-dev \
        libgl1-mesa-dri \
        libgl1-mesa-glx \
        libglfw3-dev \
        libglu1-mesa-dev \
        libgl1-mesa-dri \
        libglx-mesa0 \
        libgtk-3-dev \
        libopenblas-dev \
        libopenmpi3 \
        libssl-dev \
        libusb-1.0-0-dev \
        mesa-utils \
        net-tools \
        nginx \
        patch \
        pkg-config \
        python-is-python3 \
        python3-pip \
        qtcreator \
        software-properties-common \
        sudo \
        supervisor \
        tmux \
        udev \
        unzip \
        vim \
        wget \
        x11-utils \
        xz-utils \
        zenity \
        zip \
    && sudo apt-get autoremove -y 

RUN yes | sudo apt-get install ros-humble-rviz2

# https://gazebosim.org/docs/latest/ros_installation/
RUN yes | sudo apt-get install ros-humble-ros-gzharmonic

# Copy the entrypoint script into the container
COPY ./entrypoint.sh /home/spaceros-user/spaceros/entrypoint.sh

# Ensure the entrypoint script is executable
RUN sudo chmod +x /home/spaceros-user/spaceros/entrypoint.sh

ENTRYPOINT ["/home/spaceros-user/spaceros/entrypoint.sh"]
