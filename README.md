# Detect Text Mover
カメラ画像から文字を認識して、ロボットを動かすパッケージ

## Directory structure
* img
    * back.jpg
        * 「BACK」の文字が書かれている画像
    * fast.jpg
        * 「FAST」の文字が書かれている画像
    * forward.jpg
        * 「FORWARD」の文字が書かれている画像
    * left.jpg
        * 「LEFT」の文字が書かれている画像
    * right.jpg
        * 「RIGHT」の文字が書かれている画像
    * slow.jpg
        * 「SLOW」の文字が書かれている画像
    * stop.jpg
        * 「STOP」の文字が書かれている画像
* launch
    * detect_text_mover_sim.launch
        * 文字認識をして、Turtlesimを動かすローンチファイル
    * detect_text_mover.launch
        * 文字認識をして、モーターを動かすローンチファイル
    * detect_text_node_dev.launch
        * 文字認識の動作確認用のローンチファイル
    * move_node_dev.launch
        * 速度指令出力の動作確認用のローンチファイル
* scripts
    * detect_text_node.py
        * 文字認識をするプログラム
    * aws_detect_text.py
        * 文字認識をするライブラリ
    * move_node.py
        * 速度指令を出すプログラム
* CMakeLists.txt
    * ROSパッケージの設定ファイル
* package.xml
    * ROSパッケージの設定ファイル
* LICENCE
    * ROSパッケージのライセンスファイル
* README.md
    * パッケージの説明書

## Requirements
* [uvc_camera](http://wiki.ros.org/uvc_camera)

## Installation
```sh
cd ~/catkin_ws
rosdep install -r -y -i --from-paths src
```

## Usage
* 速度指令出力の動作確認
```sh
roslaunch detect_text_mover move_node_dev.launch
```
* 文字認識の動作確認
```sh
roslaunch detect_text_mover detect_text_node_dev.launch
```
* 文字認識をして、Turtlesimを動かす
```sh
roslaunch detect_text_mover detect_text_mover_sim.launch
```
* 文字認識をして、モーターを動かす
```sh
roslaunch detect_text_mover detect_text_mover.launch
```