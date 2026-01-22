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

## 1. 事前準備
```
git clone
cd 
```

### 1-1. ホスト側ディレクトリ作成（camera_info永続化先）
```
$ mkdir -p persist/camera_info/webcam
$ mkdir -p persist/camera_info/xtion
$ mkdir -p persist/camera_info/realsense
```

### 1-2. デバイス確認
#### Webカメラ
```
$ ls -l /dev/video*
crw-rw----+ 1 root video 81, 0  1月 22 11:14 /dev/video0
crw-rw----+ 1 root video 81, 1  1月 22 11:14 /dev/video1
crw-rw----+ 1 root video 81, 2  1月 22 11:14 /dev/video2
crw-rw----+ 1 root video 81, 3  1月 22 11:14 /dev/video3
```

#### Xtion / RealSense
USBデバイスが見えることを確認（Linux）
```
$ lsusb
...
Bus 003 Device 005: ID 1d27:0601 ASUS Xtion
...
```

## 2. 起動方法
### 2-1. Webカメラ起動
```
docker compose --profile webcam up --build
```

### 2-2. Xtion起動
```
docker compose --profile xtion up --build
```

### 2-3. RealSense起動（任意）
```
docker compose --profile realsense up --build
```

### 停止：
```
docker compose down
```

## 3. 出力トピック（統一トピック）
下流ノード（YOLO等）は以下だけ購読すればOKです。
### 3-1. RGB
```
/camera/rgb/image_raw
/camera/rgb/camera_info
```
### 3-2. Depth（3Dカメラのみ）
```
/camera/depth/image_raw
/camera/depth/camera_info
/camera/depth/points
```

確認：
```
rostopic list | egrep "^/camera/"

```

## 4. mux（統一トピック）設定
camera_mux.launch は default_source 引数で、起動時にどの入力を選択するか制御します。
### 4-1. 起動時のデフォルト入力を指定する
compose.yaml の camera_mux の command を変更します。

#### 例：Xtionをデフォルト（推奨）
```
command: ["roslaunch", "camera_launch", "camera_mux.launch", "default_source:=xtion"]
```

#### 例：Webカメラをデフォルト
```
command: ["roslaunch", "camera_launch", "camera_mux.launch", "default_source:=webcam"]
```

#### 例：RealSenseをデフォルト
```
command: ["roslaunch", "camera_launch", "camera_mux.launch", "default_source:=realsense"]
```

## 5. 実行中に入力を切り替える（selectサービス）
topic_tools/mux は .../select サービスで入力トピックを切替できます。

### 5-1. RGBをXtionに切り替える
```
rosservice call /mux_rgb_image/select "/xtion/rgb/image_raw"
rosservice call /mux_rgb_info/select  "/xtion/rgb/camera_info"
```

### 5-2. RGBをWebカメラに切り替える
```
rosservice call /mux_rgb_image/select "/webcam/image_raw"
rosservice call /mux_rgb_info/select  "/webcam/camera_info"
```

### 5-3. DepthをRealSenseに切り替える（例）
```
rosservice call /mux_depth_image/select "/realsense/depth/image_rect_raw"
rosservice call /mux_depth_info/select  "/realsense/depth/camera_info"
rosservice call /mux_points/select      "/realsense/depth/points"
```

### 利用可能サービス一覧：
```
rosservice list | grep mux
```

## 6. camera_info（キャリブレーション）の永続化
### 6-1. 永続化の仕組み
各カメラコンテナは以下をホストにマウントしています。
- コンテナ：/root/.ros/camera_info
- ホスト：./persist/camera_info/<camera>/

これによりキャリブレーションyamlがホストに残り、再起動しても保持されます。

### 6-2. Webカメラの camera_info を読み込む
Webカメラは webcam.launch で camera_info_url を指定し、yamlを読み込みます。

#### 例（compose.yaml内）：
```
"camera_info_url:=file:///root/.ros/camera_info/webcam.yaml"
```

#### ホスト側の実体：
```
./persist/camera_info/webcam/webcam.yaml
```

## 7. カメラパラメータの変更方法
### 基本方針：

- launch/*.launch に <arg> を用意
- compose.yaml の command で arg:=value を渡す

### 7-1. Webカメラの変更例
compose.yaml の camera_webcam の command を変更します。

#### 例：解像度とFPS変更
```
"width:=640", "height:=480", "fps:=15"
```

#### 例：デバイス変更
```
"device:=/dev/video2"
```

### 7-2. Xtionの変更例
camera_xtion の command を変更します。

#### 例：IRを有効にする
```
"ir_processing:=true"
```

## 8. よくあるトラブルと対処
### 8-1. トピックが出ない
- roscoreが起動しているか
- 対象profileを指定しているか
- デバイスが見えているか（/dev/video0, /dev/bus/usb）

#### 確認：
```
rostopic list
```

### 8-2. RealSenseのトピック名が違う
RealSenseは環境でトピック名が変わることがあります。

以下で実際の名前を確認し、camera_mux.launch の rs_* 引数を合わせてください。
```
rostopic list | grep realsense
```

## 9. 推奨運用（下流のYOLO等）
- 下流は /camera/rgb/image_raw（必要ならDepth系も）だけ購読
- カメラの切替は muxで吸収
- camera_infoは persist で固定化し、再現性を確保

