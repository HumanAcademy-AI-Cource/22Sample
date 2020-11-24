#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ライブラリをインポート
import rospy
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# ロボットを動かすクラス
class Mover():
    def __init__(self):
        # サブスクライバーを定義
        rospy.Subscriber("/text", String, self.callback_text)
        # パブリッシャーを定義
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # 速度を保持する変数
        self.cmd_vel = Twist()
        # 認識結果の文字列を保持する変数
        self.text = None
        # 定速の値を定義
        self.const_vel = 0.2
        # ロボットの状態を保持する変数
        self.robot_status = 0
    
    def callback_text(self, data):
        """
        受け取った文字列を変数に代入する関数
        """

        self.text = data.data

    def robot_motion(self):
        """
        ロボットに速度指令を出す関数
        """

        # 停止
        if self.text == "STOP":
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.robot_status = 0
        # 前進
        elif self.text == "FORWARD":
            self.cmd_vel.linear.x = self.const_vel
            self.cmd_vel.angular.z = 0.0
            self.robot_status = 1
        # 後進
        elif self.text == "BACK":
            self.cmd_vel.linear.x = -1.0 * self.const_vel
            self.cmd_vel.angular.z = 0.0
            self.robot_status = 2
        # 右旋回
        elif self.text == "RIGHT":
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = -1.0 * self.const_vel
            self.robot_status = 3
        # 左旋回
        elif self.text == "LEFT":
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.const_vel
            self.robot_status = 4
        
        # 速度を早く
        if self.text == "FAST":
            if self.robot_status == 1:
                self.cmd_vel.linear.x = self.const_vel * 1.5
            elif self.robot_status == 2:
                self.cmd_vel.linear.x = self.const_vel * -1.5
            elif self.robot_status == 3:
                self.cmd_vel.angular.z = self.const_vel * -1.5
            elif self.robot_status == 4:
                self.cmd_vel.angular.z = self.const_vel * 1.5
        # 速度を遅く
        elif self.text == "SLOW":
            if self.robot_status == 1:
                self.cmd_vel.linear.x = self.const_vel * 0.5
            elif self.robot_status == 2:
                self.cmd_vel.linear.x = self.const_vel * -0.5
            elif self.robot_status == 3:
                self.cmd_vel.angular.z = self.const_vel * -0.5
            elif self.robot_status == 4:
                self.cmd_vel.angular.z = self.const_vel * 0.5

        # 速度をパブリッシュ
        self.cmd_pub.publish(self.cmd_vel)
        
    def run(self):
        """
        一連の処理を行う関数
        """

        if self.text is not None:
            self.robot_motion()

if __name__ == '__main__':
    # ノードを宣言
    rospy.init_node('move')
    # クラスのインスタンスを作成
    m = Mover()
    # 一定周期で処理を実行するための準備
    rate = rospy.Rate(1)
    # ループ処理開始
    while not rospy.is_shutdown():
        # 処理を実行
        m.run()
        rate.sleep()