FROM ubuntu:focal

RUN apt-get update && \
    apt-get install -y gnupg build-essential curl

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    DEBIAN_FRONTEND=noninteractive apt-get update

ENV TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    apt-get update && apt install -y tzdata

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink ros-noetic-octomap-mapping \
    ros-noetic-control-toolbox \
    ros-noetic-jackal-description ros-noetic-jackal-control ros-noetic-jackal-gazebo ros-noetic-jackal-simulator \
    python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev \
    python3-rosdep python3-wstool ros-noetic-ros \
    ffmpeg screen xvfb imagemagick gnuplot sqlite3 jq

RUN groupadd -g 1001 flie && \
    useradd -u 1001 -g flie -m -s /bin/bash flie && \
    echo "flie:flie" | chpasswd && \
    adduser flie sudo && \
    adduser flie plugdev && \
    mkdir /crazyflie_ws && \
    chown flie:flie /crazyflie_ws && \
    mkdir -p /home/flie/.ssh && \
    chown flie:flie /home/flie/.ssh

USER flie

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "alias crazys_init='catkin clean --yes; catkin build; source /crazyflie_ws/devel/setup.bash'" >> ~/.bashrc && \
    echo "test -e /crazyflie_ws/devel/setup.bash && source /crazyflie_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "echo -e \"you are in a screen session, use CTRL+A,C to create new tab, CTRL+A,space to change to next tab\n\nalias for initial build:\n  crazys_init\nthen you might try:\n  roslaunch rotors_gazebo crazyflie2_hovering_example.launch\n  roslaunch rotors_gazebo crazyflie2_swarm4.launch\n\n\"" >> ~/.bashrc && \
    echo "shell \"/bin/bash\"" >> ~/.screenrc

WORKDIR /crazyflie_ws

RUN mkdir -p /crazyflie_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /crazyflie_ws/src; catkin_init_workspace; cd /crazyflie_ws; catkin init"

RUN ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN cd /crazyflie_ws/src && git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git

CMD /bin/screen /bin/bash
