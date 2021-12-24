#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーをゲームパッドで動かすためのノードです。

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

speed = Twist()

def call_back(data):
        global speed

        #ゲームパッドの入力値からライトローバーの直進・旋回速度を算出
        #ゲームパッドによってボタン、アナログスティック等の配列要素は違うことに注意
        speed.linear.x = data.axes[1]*0.1
        speed.angular.z = data.axes[2]*2.0

def rover_gamepad():
        global speed

        rospy.init_node('rover_gamepad', anonymous=True)

        pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

        rate = rospy.Rate(20)

        rospy.Subscriber('joy',Joy,call_back)

        while not rospy.is_shutdown():
                pub.publish(speed)
                rate.sleep()

if __name__ == '__main__':
        rover_gamepad()
