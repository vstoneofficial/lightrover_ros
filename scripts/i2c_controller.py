#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、VS-WRC201を制御するためのノードです。

import rospy
from std_msgs.msg import String
import vs_wrc201_i2c
import time
from lightrover_ros.srv import *

i2c = vs_wrc201_i2c.VsWrc201I2c(0x10)

def handle_wrc201_i2c(req):
        if(req.cmd=="w"):
                #マイコンのメモリマップの特定アドレスを上書き
                try:
                        if(req.length==4):#4byte
                                i2c.write_4_byte(req.addr,req.data)
                        elif(req.length==2):#2byte
                                i2c.write_2_byte(req.addr,req.data)
                        elif(req.length==1):#1byte
                                i2c.write_1_byte(req.addr,req.data)
                except IOError as e:
                        return None

                return Wrc201MsgResponse(1)

        elif(req.cmd=="s"):
                #マイコンのメモリマップをすべて上書き
                try:
                        i2c.send_write_map()
                except IOError as e:
                        return None

                return Wrc201MsgResponse(1)

        elif(req.cmd=='rm'):
                #マイコンのメモリマップをすべて読み込み
                try:
                        i2c.read_all()
                except IOError as e:
                        return None

                return Wrc201MsgResponse(1)

        elif(req.cmd=="r"):
                #マイコンのメモリマップの特定アドレスを読み込み
                try:
                        i2c.read_memmap(req.addr,req.length)
                        if(req.length==4):#4byte
                                read=i2c.read_s32map(req.addr)
                        elif(req.length==2):#2byte
                                read=i2c.read_s16map(req.addr)
                        elif(req.length==1):#1byte
                                read=i2c.read_s8map(req.addr)
                        else:
                                read=0
                except IOError as e:
                        return None

                return Wrc201MsgResponse(read)

def wrc201_i2c_server():
        rospy.init_node('wrc201_i2c_server')

        p_dev_addr = rospy.get_param('dev_addr', 0x10)

        i2c.set_dev_addr(p_dev_addr)
        i2c.read_all()
        i2c.init_memmap(2.0)
        i2c.send_write_map()
        s = rospy.Service('wrc201_i2c',Wrc201Msg,handle_wrc201_i2c)
        rospy.loginfo('Ready to VS-WRC201 i2c communication.')
        rospy.spin()

if __name__ == "__main__":
        wrc201_i2c_server()
