FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-lc"]

# 1) 基本 + カメラ + rqt + mux + GL(ソフトウェア描画) を入れる
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl nano vim x11-apps \
    python3-pip \
    ros-noetic-usb-cam \
    ros-noetic-openni2-camera ros-noetic-openni2-launch \
    ros-noetic-rgbd-launch \
    ros-noetic-realsense2-camera \
    ros-noetic-topic-tools \
    ros-noetic-rqt-image-view \
    ros-noetic-image-view \
    libgl1-mesa-dri libgl1-mesa-glx mesa-utils \
    ros-noetic-web-video-server  \
    && rm -rf /var/lib/apt/lists/*

# 2) catkin workspace
WORKDIR /catkin_ws
RUN mkdir -p /catkin_ws/src

# 3) camera_launch を「パッケージ丸ごと」コピー
#    （launch/ だけのコピーにすると scripts/ や package.xml が漏れて事故りやすい）
COPY camera_launch/ /catkin_ws/src/camera_launch/

# 4) entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 5) build
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

