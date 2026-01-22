FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-lc"]

# 基本ツール
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl nano vim \
    python3-pip \
    ros-noetic-usb-cam \
    ros-noetic-image-transport \
    ros-noetic-image-transport-plugins \
    ros-noetic-camera-info-manager \
    ros-noetic-diagnostic-updater \
    ros-noetic-openni2-camera ros-noetic-openni2-launch \
    && rm -rf /var/lib/apt/lists/*

# (任意) RealSense を使うなら: ホスト側USBアクセスに加えてrealsense2_cameraを追加
# NOTE: ros-noetic-realsense2-camera が apt にある環境前提。なければソースビルドへ。
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-realsense2-camera \
    && rm -rf /var/lib/apt/lists/* || true

WORKDIR /catkin_ws
RUN mkdir -p /catkin_ws/src

# launch を配置する場所
COPY launch/ /catkin_ws/src/camera_launch/launch/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 最小のcatkin workspace生成（launchだけなら実ビルド不要だが、devel/setup.bashを作っておくと便利）
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
