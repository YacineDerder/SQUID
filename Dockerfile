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

# Dronekit
RUN pip3 install dronekit pyserial

# Robot localization and Dronekit
RUN apt install -y ros-noetic-robot-localization ros-noetic-imu-tools 

# inspection tools
RUN apt install -y ros-noetic-rqt-image-view xauth iproute2
RUN touch ~/.Xauthority

# squid package install
RUN apt -y install screen python-is-python3 dos2unix
RUN source /opt/ros/noetic/setup.bash && \
    cd ~/workspace/src/ && \ 
    catkin_create_pkg squid_package mavros ov_msckf usb_cam && \
    mkdir -p ~/workspace/src/squid_package/launch

COPY squid.launch /
COPY scripts /scripts
COPY src /tmp/src

RUN find /scripts -type f -exec dos2unix {} \;
RUN find /scripts -type f -exec chmod +x {} \;

RUN mv /scripts ~/workspace/src/squid_package/ && \
    cp /squid.launch ~/workspace/src/squid_package/launch/squid.launch && \
    mv /tmp/src ~/workspace/src/squid_package/

# Update CMakeLists.txt
RUN echo 'add_executable(setRates src/setRates.cpp)' >> ~/workspace/src/squid_package/CMakeLists.txt && \
    echo 'target_link_libraries(setRates ${catkin_LIBRARIES})' >> ~/workspace/src/squid_package/CMakeLists.txt

RUN echo "source ~/workspace/devel/setup.bash" >> ~/.bashrc

# Build the package
RUN source /opt/ros/noetic/setup.bash && \
    cd ~/workspace/ && \
    catkin build squid_package

# Make shared folder
RUN mkdir shared_folder

CMD [ "/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/workspace/devel/setup.bash && roslaunch squid_package squid.launch" ]