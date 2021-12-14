#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーを速度制御するためのノードです。

import rospy
import sys
from lightrover_ros.srv import *
import time
import math
import vs_wrc201_motor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

write_msg = rospy.ServiceProxy('wrc201_i2c',Wrc201Msg)
MU8_TRIG = 0x10
MS16_FB_PG0 = 0x20
MS16_FB_PG1 = 0x22
MS16_T_OUT0 = 0x50
MS16_T_OUT1 = 0x52

liner_x = 0.0
angular_z = 0.0

current_v = [0.0, 0.0]
target_rover_v = [0.0, 0.0]

ROVER_D = 0.143/2.0

pid_controller = vs_wrc201_motor.VsWrc201Motor()

def get_rover_v(data):
        global linear_x,angular_z,current_v,ROVER_D,target_rover_v
        linear_x = data.twist.twist.linear.x
        angular_z = data.twist.twist.angular.z

        #現在の直進・旋回速度から左右モータの現在の回転速度を算出
        current_v[1] = (linear_x + ROVER_D * angular_z)
        current_v[0] = -1.0 * (linear_x - ROVER_D * angular_z)

        #現在のモータ回転速度と目標のモータ回転速度をPIDコントローラに入力
        #左右モータへの出力値を算出
        output = pid_controller.pid_controll(current_v, target_rover_v)
        drive_motor(output[0],output[1])

def set_target_v(data):
        global ROVER_D,target_rover_v
        #目標直進・旋回速度から左右モータの目標回転速度を算出
        target_rover_v[1] = (data.linear.x + ROVER_D * data.angular.z)
        target_rover_v[0] = -1.0 * (data.linear.x - ROVER_D * data.angular.z)

def drive_motor(r_speed, l_speed):
        write_msg(MS16_T_OUT0,r_speed,2,'w')
        write_msg(MS16_T_OUT1,l_speed,2,'w')
        write_msg(0,0,0,'s')

def lightrover_pid_controller():
        rospy.init_node('listenr', anonymous=True)

        rospy.loginfo('Start PID')

        write_msg(MU8_TRIG,0x03,1,'w')          #エンコーダリセット
        write_msg(MS16_FB_PG0,0,2,'w')          #モータ0位置補償Pゲイン設定
        write_msg(MS16_FB_PG1,0,2,'w')          #モータ1位置補償Pゲイン設定

        rospy.Subscriber('odom',Odometry,get_rover_v)
        rospy.Subscriber('rover_drive',Twist,set_target_v)

        rospy.spin()

if __name__ == '__main__':
        lightrover_pid_controller()
