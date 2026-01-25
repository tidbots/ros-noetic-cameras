# カメラ入力
ros-noetic-cameras

## 目的
- ROS Noetic 上で 複数種類のカメラ入力（Webカメラ / Xtion / RealSense）を Docker Compose で運用する
- camera_info（キャリブレーションyaml）を ホストに永続化

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
    realsense.launch          
  persist/
    camera_info/
      webcam/
      xtion/
      realsense/
```

## 1. 事前準備
```
git clone https://github.com/tidbots/ros-noetic-cameras.git
cd ros-noetic-cameras
docker compose build
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
docker compose --profile webcam up
```
起動時のWebカメラのパラメータを変更する
```
CAM_DEV=/dev/video2 CAM_W=640 CAM_H=480 CAM_FPS=15 docker compose --profile webcam up
```

### 2-2. Xtion起動
```
docker compose --profile xtion up
```

### 2-3. RealSense起動
```
docker compose --profile realsense up
```

### 停止：
```
docker compose down
```

## 5. デバッグ
### rqt_image_viewで確認
```
docker compose --profile webcam --profile debug up
```

###  ブラウザで確認
```
docker compose --profile webcam --profile debug-web up
```

同じマシンなら：
```
http://localhost:8080/
```
```
直リンク：http://localhost:8080/stream?topic=/webcam/image_raw
```

別PCから見るなら localhost をホストネームに





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

