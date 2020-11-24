#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ライブラリをインポート
import rospy
import roslib.packages
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import boto3

# 自作のライブラリをインポート
import aws_detect_text

# 画像中の文字を認識するクラス
class DetectText():
    def __init__(self):
        # サブスクライバーを定義
        rospy.Subscriber("/image_raw", Image, self.callback_img)
        # パブリッシャーを定義
        self.text_pub = rospy.Publisher("/text", String, queue_size=1)
        # 文字列を保持する変数
        self.text = ""
        # 画像を保持する変数
        self.image = None
        # 画像の保存先
        self.image_path = roslib.packages.get_pkg_dir("detect_text_mover") + "/img/" + "text_image.jpg"
        # フォルダがない場合は新しく作成
        if not os.path.isdir(roslib.packages.get_pkg_dir("detect_text_mover") + "/img/"):
            os.mkdir(roslib.packages.get_pkg_dir("detect_text_mover") + "/img/")
    
    def callback_img(self, data):
        """
        受け取った画像を保存する関数
        """

        # 画像をROSのデータ形式からOpenCV形式に変換して変数に保存
        try:
            self.image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def save_image(self):
        """
        画像をファイルとして保存する関数
        """
        
        # 画像を指定した場所に保存
        cv2.imwrite(self.image_path, self.image)
        rospy.loginfo("カメラ画像を保存しました。")

    def detect_text(self):
        """
        画像中から文字を認識する関数
        """

        # 画像を渡して文字認識の結果を受け取る
        self.text = aws_detect_text.detect_text(self.image_path)
        print("--------------------------------------------------")
        # 文字列をパブリッシュ
        self.text_pub.publish(self.text)

    def run(self):
        """
        一連の処理を行う関数
        """

        if self.image is not None:
            self.save_image()
            self.detect_text()
        else:
            rospy.loginfo("画像が未取得です。")

if __name__ == '__main__':
    # ノードを宣言
    rospy.init_node('detect_text')
    # クラスのインスタンスを作成
    dt = DetectText()
    # 一定周期で処理を実行するための準備
    rate = rospy.Rate(1)
    # ループ処理開始
    while not rospy.is_shutdown():
        # 処理を実行
        dt.run()
        rate.sleep()