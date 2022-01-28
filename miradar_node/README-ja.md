# miradar_node
miradar12eのROSノード。
センサパラメータはdynamic_reconfigureを用いて設定できる。


## Launch
```bash
roslaunch miradar_node miradar_rviz.launch
```

## ROS Node
### miradar_node
miradar12eをROSに変換するノード。
```bash
rosrun miradar_node miradar_node _devicename=/dev/ttyUSB0
```

### ppi_visualizer.py
PPIデータのRVIZ可視化ノード。
```bash
rosrun miradar_node ppi_visualizer.py
```


## 非ROS環境でのビルド
### ビルド
src/maincore.cppをビルドする。
```bash
g++ -std=c++11 maincore.cpp -o miradar
```
### 実行コマンド
```bash
./miradar [デバイスファイル] [センサモード]
```
センサモード
0 : 停止（省電力モード）
1 : PPIモード
2 : 地図モード
例
```bash
./miradar /dev/ttyUSB0 1
```


## Topic
## /miradar/ppidata : miradar_node/PPIData
障害物情報(PPI)データを取得できるROS Topic。
センサモードがPPIモードのみトピックが送信される。

## /miradar/image_raw : sensor_msgs/Image
地図情報を取得できるROS Topic。　　
センサモードがMapのときのみトピックが送信される。

## /miradar/markers : visualization_msgs/MarkerArray
PPIデータをRVizのマーカーとして表示するトピック。
赤色に近いほど信号が強く、青色に近いほど信号が弱い。

## /miradar/scan : sensor_msgs/LaserScan
レーダーデバイスのマップをLaserScanに変換したトピック。

## /miradar/points : sensor_msgs/PointCloud2
レーダーデバイスのマップをPointCloud2に変換したトピック。



## ROSパラメータ
### devicename : str
デバイス名をデバイスファイル名で指定する。
例:
Linuxの場合
```bash
rosrun miradar_node miradar_node _devicename:=/dev/ttyUSB0
```
WSL1等の仮想Linuxマシンの場合
```bash
rosrun miradar_node miradar_node _devicename:=/dev/ttyS7
```

## Dynamic Reconfigure Param
### sensor_mode : Halt, PPI, Map
センサモードを停止(0)、PPI（障害物情報、1）、地図表示モード(2)の3つに切り替える。

### max_distance : int
センサのデータ受信最大距離を決める。

### min_distance : int
センサのデータ受信最小距離を決める。

### alarm_distance : int
センサのアラーム範囲距離を決める。

### max_angle : int
センサの最大受信角度を決める。

### max_dB : int
センサが地図表示する場合の最大dBを決める。最大デシベルの場合画素が255, マーカーが赤になる。

### min_dB : int
センサが受信可能な最小dBを決める。

### tx_power : int
センサの送信電波の強さを決める。標準の値が推奨。

### hpf_gain : 0db, 12db, 18db
HPFゲインを設定する。

### pga_gain : 0db, 6db, 12db, 18db, 24db
PGAゲインを設定する。

### duration : int
センサの応答時間をmsで決める。
