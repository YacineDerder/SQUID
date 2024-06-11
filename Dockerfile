FROM ros:noetic-perception

SHELL ["/bin/bash", "-c"]

# apt update
RUN apt update

# openvins prerequisites
RUN apt install -y git python3-catkin-tools python3-osrf-pycommon libeigen3-dev libboost-all-dev libceres-dev v4l-utils ros-noetic-cv-camera ros-noetic-usb-cam nano
RUN mkdir -p ~/workspace/src/
RUN cd ~/workspace/src/

# Source ros in .bashrc
RUN echo 'source opt/ros/noetic/setup.bash' >> ~/.bashrc
RUN source ~/.bashrc

# openvins install
RUN source /opt/ros/noetic/setup.bash && \
    cd ~/workspace/src/ && \ 
    git clone https://github.com/rpng/open_vins/ && \
    cd .. && \
    catkin build -j3

# Mavros install
RUN apt install -y ffmpeg git python3-pip wget ros-noetic-mavros ros-noetic-mavros-extras
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod a+x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh
RUN rm install_geographiclib_datasets.sh

# Robot localization
RUN apt install -y ros-noetic-robot-localization ros-noetic-imu-tools

# squid package install
RUN apt -y install screen
COPY squid.launch /
RUN source /opt/ros/noetic/setup.bash && \
    cd ~/workspace/src/ && \ 
    catkin_create_pkg squid_package mavros ov_msckf usb_cam && \
    mkdir -p ~/workspace/src/squid_package/launch && \
    cp /squid.launch ~/workspace/src/squid_package/launch/squid.launch && \
    cd .. && \
    catkin build squid_package -j3

RUN echo "source ~/workspace/devel/setup.bash" >> ~/.bashrc

# inspection tools
RUN apt install -y ros-noetic-rqt-image-view xauth
RUN touch ~/.Xauthority

# Add test scripts
COPY test.py /

# Make shared folder
RUN mkdir shared_folder

# Roslaunch squid.launch on container creation
CMD [ "/bin/bash", "-c", "xauth add $DISPLAY . 39fd949eee6479efc1cbc9b937d81e5d; roslaunch squid.launch" ]