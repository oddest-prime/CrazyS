FROM ubuntu:bionic

RUN apt-get update && \
    apt-get install -y gnupg build-essential curl

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update
#    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \

ENV TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    apt-get update && apt install -y tzdata

RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-octomap-mapping \
    python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox \
    python-rosinstall python-rosinstall-generator build-essential \
    ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-mav-msgs ros-melodic-rotors-control \
    ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation ros-melodic-rotors-joy-interface \
    ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs ros-melodic-rotors-description ros-melodic-rotors-hil-interface \
    ffmpeg screen xvfb imagemagick gnuplot

RUN apt-get update && apt-get -y remove ros-melodic-gazebo* gazebo* && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get -y install sudo gazebo9 gazebo9-* ros-melodic-gazebo-* && \
    apt-get -y upgrade

RUN useradd -m flie && \
    echo "flie:flie" | chpasswd && \
    adduser flie sudo && \
    mkdir /crazyflie_ws && \
    chown flie:flie /crazyflie_ws && \
    mkdir -p /home/flie/.ssh && \
    chown flie:flie /home/flie/.ssh

COPY deploy-key.pub /home/flie/.ssh/id_rsa.pub
COPY deploy-key /home/flie/.ssh/id_rsa

RUN chown flie:flie /home/flie/.ssh/* && \
    chmod 0600 /home/flie/.ssh/id_rsa

USER flie

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

WORKDIR /crazyflie_ws

RUN mkdir -p /crazyflie_ws/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd /crazyflie_ws/src; catkin_init_workspace; cd /crazyflie_ws; catkin init"

RUN ssh-keyscan github.com >> ~/.ssh/known_hosts
# RUN cd /crazyflie_ws/src && git clone -b dev/ros-melodic https://github.com/gsilano/CrazyS.git
RUN cd /crazyflie_ws/src && git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
RUN cd /crazyflie_ws/src && git clone -b dockerized git@github.com:oddest-prime/crazys.git

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd /crazyflie_ws; catkin build"

RUN echo "source /crazyflie_ws/devel/setup.bash" >> ~/.bashrc

CMD /bin/bash
