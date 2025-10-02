ARG ROS=humble
FROM gai313/ros2:${ROS}
ARG ROS


# user setting
ARG USERNAME=erasers
ARG GROUPNAME=erasers
ARG UID=1000
ARG GID=1000
ARG PASSWORD

RUN groupadd -g $GID $GROUPNAME &&\
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME &&\
    echo $USERNAME:$PASSWORD | chpasswd
    #echo "$USERNAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


# Build workspace
WORKDIR /home/${USERNAME}/colcon_ws/src
COPY ./erasers_kachaka ./erasers_kachaka
COPY ./setup.repos ./setup.repos
RUN . /opt/ros/${ROS}/setup.bash &&\
    vcs import . < ./setup.repos &&\
    apt-get update && rosdep update &&\
    rosdep install -y -i --from-path . \
        --skip-keys=ros2_aruco_interfaces \
        --skip-keys=ros2_aruco

    
USER $USERNAME
WORKDIR /home/${USERNAME}/colcon_ws
RUN pip install kachaka-api \
                "scipy>=1.13.0" \
                transform3d \
                matplotlib \
                numpy==1.22.4

RUN . /opt/ros/${ROS}/setup.bash &&\
    colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
