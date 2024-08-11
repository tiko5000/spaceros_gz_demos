FROM osrf/space-ros:latest

# https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu
RUN sudo apt-get update
RUN sudo apt-get install lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN yes | sudo apt-get install gz-harmonic

# https://gazebosim.org/docs/latest/ros_installation/
RUN yes | sudo apt-get install ros-humble-ros-gzharmonic
