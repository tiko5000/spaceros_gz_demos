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
        bash \
        bash-completion \
        build-essential \
        cmake \
        curl \
        git \
        htop \
        iputils-ping \
        mesa-utils \
        net-tools \
        python-is-python3 \
        python3-pip \
        unzip \
        vim \
        wget \
    && sudo apt-get autoremove -y 

RUN yes | sudo apt-get install ros-humble-rviz2

# https://gazebosim.org/docs/latest/ros_installation/
RUN yes | sudo apt-get install ros-humble-ros-gzharmonic

# # Copy the entrypoint script into the container
# COPY ./entrypoint.sh /home/spaceros-user/spaceros/entrypoint.sh

# # Ensure the entrypoint script is executable
# RUN sudo chmod +x /home/spaceros-user/spaceros/entrypoint.sh

# ENTRYPOINT ["/home/spaceros-user/spaceros/entrypoint.sh"]
