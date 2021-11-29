#!/usr/bin/env python
# -*- coding: utf-8 -*-

#このライブラリは、VS-WRC201とI2C通信を行うためのものです。

import smbus
import time

class VsWrc201I2c:
        #メモリマップアドレス
        DEV_ADDR = 0x10
        MAP_SIZE = 0x100

        MU16_SYSNAME = 0x00
        MU16_FIRMREC = 0x02
        MU32_TRIPTIME = 0x04
        MU8_MODE = 0x0d
        MU16_POWOFF_T = 0x0e
        MU8_O_EN = 0x10
        MU8_TRIG = 0x11
        MU16_SD_VI = 0x12
        MU16_OD_DI = 0x14
        MU16_SPD_T0 = 0x16
        MU16_MOVE_T0 = 0x18
        MU16_FB_PG0 = 0x20
        MU16_FB_PG1 = 0x22
        MU16_FB_ALIM0 = 0x24
        MU16_FB_ALIM1 = 0x26
        MU16_FB_DLIM0 = 0x28
        MU16_FB_DLIM1 = 0x2a
        MU16_FB_OLIM0 = 0x2c
        MU16_FB_OLIM1 = 0x2e
        MU16_FB_PCH0 = 0x30
        MU16_FB_PCH1 = 0x32
        MS32_T_POS0 = 0x40
        MS32_T_POS1 = 0x44
        MS32_A_POS0 = 0x48
        MS32_A_POS1 = 0x4c
        MS16_T_OUT0 = 0x50
        MS16_T_OUT1 = 0x52
        MS16_T_OUT2 = 0x54
        MS32_M_POS0 = 0x60
        MS32_M_POS1 = 0x64
        MS16_M_SPD0 = 0x68
        MS16_M_SPD1 = 0x6a
        MS16_M_OUT0 = 0x6c
        MS16_M_OUT1 = 0x6e
        MU16_M_DI = 0x7e
        MS32_WP_PX = 0x80
        MS32_WP_PY = 0x84
        MS16_WP_TH = 0x88
        MU16_M_VI = 0x90
        MS32_P_DIS = 0xa0
        MS16_P_RAD = 0xa4
        MS16_P_SPD = 0xa8
        MU8_P_STTS = 0xaa
        MS16_S_XS = 0xac
        MS16_S_ZS = 0xae
        MS32_P_PX0 = 0xb0
        MS32_P_PX1 = 0xb4
        MS32_P_PX2 = 0xb8
        MS32_P_PX3 = 0xbc
        MS32_P_PY0 = 0xc0
        MS32_P_PY1 = 0xc4
        MS32_P_PY2 = 0xc8
        MS32_P_PY3 = 0xcc
        MS16_P_TH0 = 0xd0
        MS16_P_TH1 = 0xd2
        MS16_P_TH2 = 0xd4
        MS16_P_TH3 = 0xd6
        MS32_P_PXIN = 0xd8
        MS32_P_PYIN = 0xdc
        MS16_P_THIN = 0xe0
        MU8_P_TOP = 0xe2
        MU8_P_BTM = 0xe3
        MU8_P_NOW = 0xe4
        MU8_A_EN = 0xf0
        MU8_PWN_SW = 0xf1
        MU16_A_PCTR = 0xf2
        MU8_TRIG2 = 0xf4

        memmap = [0x00] * MAP_SIZE
        write_flag = [0x00] * MAP_SIZE

        initialMemmap = [0x00, 0x00, 0xed, 0x05, 0xff, 0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x10, 0x00, 0x10,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        def __init__(self, dev_addr):
                self.__i2c = smbus.SMBus(1)
                self.DEV_ADDR = dev_addr
                time.sleep(0.1)

        def set_dev_addr(self, dev_addr):
                self.DEV_ADDR = dev_addr

                return 1

        #メモリマップ初期化
        def init_memmap(self,cut_off_level):
                cut_off_hex = int((cut_off_level/3.3)*0xfff)
                self.write_memmap(self.MU8_O_EN,self.initialMemmap,0xF0)
                self.read_all()
                self.write_s16map(self.MU16_SD_VI,cut_off_hex)

        #VS-WRC201上のマイコンのメモリマップの特定アドレスを上書き（1byte）
        def write_1_byte(self,addr,data):
                self.__i2c.write_byte_data(self.DEV_ADDR,addr,data)

                return 1

        #VS-WRC201上のマイコンのメモリマップの特定アドレスを上書き（2byte）
        def write_2_byte(self,addr,data):
                for i in range(2):
                        write_byte = ((data >> (i*8)) & 0xff)
                        self.write_1_byte(addr+i,write_byte)

                return 1

        #VS-WRC201上のマイコンのメモリマップの特定アドレスを上書き（4byte）
        def write_4_byte(self,addr,data):
                for i in range(4):
                        write_byte = ((data >> (i*8)) & 0xff)
                        self.write_1_byte(addr+i,write_byte)

                return 1

        #VS-WRC201上のマイコンのメモリマップの特定アドレスから指定バイト分上書き
        def write_memmap(self,addr,data_array,length):
                if length<=0:
                        return -1

                for i in range(length):
                        self.write_1_byte(addr+i,data_array[i])

                return 1

        #ラズパイのメモリマップをすべてクリア
        def memmap_clean(self):
                for i in range(self.MAP_SIZE):
                        self.memmap[i] = 0x00

        #VS-WRC201上のマイコンのメモリマップの特定アドレスから指定バイト分読み込み
        #ラズパイのメモリマップへ反映
        def read_memmap(self,addr,length):
                for i in range(length+1):
                        self.memmap[addr+i] = self.__i2c.read_byte_data(self.DEV_ADDR, addr+i)

                return i

        #VS-WRC201上のマイコンのメモリマップをすべて読み込み
        #ラズパイのメモリマップへ反映
        def read_all(self):
                self.read_memmap(0x00,64)
                self.read_memmap(0x40,64)
                self.read_memmap(0x80,20)

                return 1

        #ラズパイ上のメモリマップをVS-WRC201のマイコンのメモリマップへ反映
        def send_write_map(self):
                for i in range(0x12,0x90):
                        head_addr = i
                        length = 0
                        while self.write_flag[i]:
                                self.write_flag[i]=0x00
                                length+=1
                                i+=1
                                if i >= self.MAP_SIZE:
                                        break
                        write_map = self.memmap[head_addr:head_addr+length]
                        self.write_memmap(head_addr,write_map,length)
                for i in range(0x0e,0x12):
                        head_addr = i
                        length = 0
                        while self.write_flag[i]:
                                self.write_flag[i]=0x00
                                length+=1
                                i+=1
                                if i >= self.MAP_SIZE:
                                        break
                        write_map = self.memmap[head_addr:head_addr+length]
                        self.write_memmap(head_addr,write_map,length)

        #ラズパイのメモリマップの読み込み（1byte）
        def read_s8map(self,addr):
                return self.memmap[addr]

        #ラズパイのメモリマップの上書き（1byte）
        def write_s8map(self,addr,data):
                self.memmap[addr] = data
                self.write_flag[addr] = 0x01

                return self.memmap[addr]

        #ラズパイのメモリマップの読み込み（2byte）
        def read_s16map(self,addr):
                return ((self.memmap[addr+1] << 8) | self.memmap[addr])

        #ラズパイのメモリマップの上書き（2byte）
        def write_s16map(self,addr,data):
                self.memmap[addr] = (0xff & data)
                self.memmap[addr+1] = ((0xff00 & data) >> 8)
                self.write_flag[addr] = 0x0101
                return ((self.memmap[addr+1] << 8) | self.memmap[addr])

        #ラズパイのメモリマップの読み込み（4byte）
        def read_s32map(self,addr):
                return ((self.memmap[addr+3] << 24) | (self.memmap[addr+2] << 16) | (self.memmap[addr+1] << 8) | self.memmap[addr])

        #ラズパイのメモリマップの上書き（4byte）
        def write_s32map(self,addr,data):
                self.memmap[addr] = (0xff & data)
                self.memmap[addr+1] = ((0xff00 & data) >> 8)
                self.memmap[addr+2] = ((0xff0000 & data) >> 16)
                self.memmap[addr+3] = ((0xff000000 & data) >> 24)
                self.write_flag[addr] = 0x01010101
                return ((self.memmap[addr+3] << 24) | (self.memmap[addr+2] << 16) | (self.memmap[addr+1] << 8) | self.memmap[addr])

        #エンコーダ値の読み込み
        def read_enc(self):
                encL = self.read_s32map(self.MS32_M_POS0)
                encR = self.read_s32map(self.MS32_M_POS1)

                return [encL,encR]

        #エンコーダ値をクリア
        def clear_enc(self):
                self.write_s8map(self.MU8_TRIG,self.read_s8map(self.MU8_TRIG) | 0x0C)

        #メモリマップへ書き込みを行ったかを確認
        def check_write_flag(self,addr):
                return self.write_flag[addr]

        #電源電圧の取得
        def get_vin(self):
                self.read_memmap(self.MU16_M_VI, 0x02)
                memmapV = self.read_s16map(self.MU16_M_VI)

                vin = (float(memmapV)/float(0x0fff))*3.3

                return vin
