#
ros-noetic-cameras

## 目的
- ROS Noetic 上で 複数種類のカメラ入力（Webカメラ / Xtion / RealSense）を Docker Compose で運用する
- camera_info（キャリブレーションyaml）を ホストに永続化
- 下流（YOLO等）はカメラ種類に依存せず、統一トピックだけ購読すれば良いようにする（mux）

## 対応カメラ
- Webカメラ（UVC: /dev/video*）
- ASUS Xtion（OpenNI2）
- Intel RealSense（任意：導入済みの場合）

## ディレクトリ構成
```
ros-noetic-cameras/
  compose.yaml
  Dockerfile
  entrypoint.sh
  launch/
    webcam.launch
    xtion.launch
    realsense.launch          # 任意
    camera_mux.launch         # 統一トピックmux（default_source対応）
  persist/
    camera_info/
      webcam/
      xtion/
      realsense/
```

