#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このライブラリは、VS-WRC201のモータを制御するためのものです。

import time
import math
import numpy as np

class VsWrc201Motor:
        ROVER_D = 0.0717767
        TIRE_CIRCUMFERENCE = 60*math.pi/1000
        ENC_COUNTS_PER_TURN = 1188.024
        ENC_PER_M = ENC_COUNTS_PER_TURN/TIRE_CIRCUMFERENCE

        std_motor_param = [0.5, math.pi, 520.0, 3.6, 2.0, 3.6, 30.0, 127.0, 1.5]
        motor_param = [0.5, math.pi, 520.0, 3.6, 2.0, 3.6, 30.0, 127.0, 1.5]

        ctl_v_com = [0.0, 0.0]
        v_com = [0.0, 0.0]
        m_com = [0.0, 0.0]

        enc = [0, 0]
        old_enc = [0, 0]

        sum_v = [[0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0]]
        avr_v = [0.0, 0.0]
        old_micros = 0

        MIN_OUTPUT = 2500

        v_enc = [0.0, 0.0]
        v_diff = [0.0, 0.0]
        prev_v_diff = [0.0, 0.0]
        prev2_v_diff = [0.0, 0.0]
        prev_m = [0, 0]

        buf_enc_com = [0.0, 0.0]

        KP_NORMAL = 0x08000800
        KP_ZERO = 0x00000000

        M_L = 1
        M_R = 0

        MOTOR_LIST_F_L = 0
        MOTOR_LIST_F_R = 1
        MOTOR_LIST_R_L = 2
        MOTOR_LIST_R_R = 3

        INDEX_MAX_SPEED = 0
        INDEX_MAX_RAD = 1
        INDEX_K_V2MP = 2
        INDEX_K_P = 3
        INDEX_K_I = 4
        INDEX_K_D = 5
        INDEX_PAD_DEAD = 6
        INDEX_PAD_MAX = 7
        INDEX_MAX_ACC = 8

        pre_t_calc_2_v = -1.0
        old_micros = 0.0
        pre_t_pos_ctl = -1.0

        pid_time = 0.0

        def calc_2_v(self,x_speed,z_speed):
                mem_com_x = float(x_speed)
                mem_com_z = float(z_speed)

                self.set_motor_param_2_std()

                self.ctl_v_com[M_L] = (mem_com_x - self.ROVER_D*mem_com_z)
                self.ctl_v_com[M_R] = -1.0 * (mem_com_x + self.ROVER_D*mem_com_z)

                if(math.fabs(self.ctl_v_com[M_L]) > self.motor_param[self.MAX_SPEED] or
                math.fabs(self.ctl_v_com[M_R]) > self.motor_param[self.MAX_SPEED]):

                        ctl_v_com_max = math.fabs(self.ctl_v_com[0])
                        for i in range(2):
                                if(ctl_v_com_max < math.fabs(self.ctl_v_com[i])):
                                        ctl_v_com_max = math.fabs(self.ctl_v_com[i])

                        for i in range(2):
                                self.ctl_v_com[i] /= (ctl_v_com_max/self.motor_param[self.MAX_SPEED])

                return self.ctl_v_com

        def ctl_2_v_com(self):
                v_diff = [0.0, 0.0]
                for i in range(2):
                        v_diff[i] = self.ctl_v_com[i] - self.v_com[i]

                max_v_diff = math.fabs(v_diff[self.M_L])

                for i in range(2):
                        if(math.fabs(v_diff[i]) > max_v_diff):
                                max_v_diff = math.fabs(v_diff[i])

                if(self.pre_t_calc_2_v < 0):
                        self.pre_t_calc_2_v = time.time()
                new_micros = time.time()
                elapsed_time = new_micros - self.pre_t_calc_2_v

                add_v = 0.0
                if(max_v_diff != 0.0):
                        for i in range(2):
                                add_v = ((self.motor_param[self.INDEX_MAX_ACC] * (v_diff[i]/max_v_diff)*elapsed_time))
                                if(math.fabs(v_diff[i]) >= math.fabs(add_v)):
                                        self.v_com[i] = self.v_com[i] + add_v
                                else:
                                        self.v_com[i] = self.ctl_v_com[i]

                for i in range(2):
                        if(self.ctl_v_com[i] == 0.0 and math.fabs(self.v_com[i]) <= 0.02):
                                self.v_com[i] = 0.0

                self.pre_t_calc_2_v = new_micros

        def set_motor_param_2_std(self):
                for i in range(7):
                        self.motor_param[i] = self.std_motor_param[i]

        def pid_controll(self, rover_v, target_v):
                if(time.time()-self.pid_time < 10.0/1000000):
                        return
                self.pid_time = time.time()

                self.v_enc = rover_v
                self.ctl_v_com = target_v

                self.ctl_2_v_com()

                for i in range(2):
                        if(math.fabs(self.v_enc[i]) > 2 * self.motor_param[self.INDEX_MAX_SPEED]):
                                self.v_enc[i] = self.sum_v[i][0]

                        for j in range(4,0,-1):
                                self.sum_v[i][4] += self.sum_v[i][j-1]
                                self.sum_v[i][j] = self.sum_v[i][j-1]

                        self.sum_v[i][4] += self.v_enc[i]
                        self.avr_v[i] = self.sum_v[i][4]/5.0
                        self.sum_v[i][0] = self.v_enc[i]

                        self.v_diff[i] = self.v_com[i] - self.avr_v[i]
                        self.m_com[i] = self.prev_m[i] + int((self.v_diff[i]*self.motor_param[self.INDEX_K_I]
                                        + (self.v_diff[i]-self.prev_v_diff[i])*self.motor_param[self.INDEX_K_P]
                                        + ((self.v_diff[i]-self.prev_v_diff[i])-(self.prev_v_diff[i]-self.prev2_v_diff[i]))*self.motor_param[self.INDEX_K_D])
                                        * self.motor_param[self.INDEX_K_V2MP])

                        if((math.fabs(target_v[i]) >= 0.05) and (math.fabs(self.m_com[i]) <= self.MIN_OUTPUT)):
                                if(target_v[i] < 0):
                                        self.m_com[i] = -1.0 * self.MIN_OUTPUT
                                else:
                                        self.m_com[i] = self.MIN_OUTPUT

                        if(self.m_com[i] > 5000):
                                self.m_com[i] = 5000
                        elif(self.m_com[i] < -5000):
                                self.m_com[i] = -5000

                        self.prev2_v_diff[i] = self.prev_v_diff[i]
                        self.prev_v_diff[i] = self.v_diff[i]
                        self.prev_m[i] = self.m_com[i]

                if(self.v_com[self.M_L] == 0.0 and self.v_com[self.M_R] == 0.0
                        and math.fabs(self.avr_v[self.M_L]) < 0.05 and math.fabs(self.avr_v[self.M_R]) < 0.05):
                        self.m_com[self.M_L] = 0
                        self.m_com[self.M_R] = 0
                        self.prev_m[self.M_L] = 0
                        self.prev_m[self.M_R] = 0

                return self.m_com

        def pos_controll(self, rover_v, target_v):

                self.v_enc = rover_v
                self.ctl_v_com = target_v

                self.ctl_2_v_com()

                e_v_com = [0.0, 0.0] #速度指令値[ecnt/sec]
                e_v_com[self.M_L] = self.v_com[self.M_L]*self.ENC_PER_M
                e_v_com[self.M_R] = self.v_com[self.M_R]*self.ENC_PER_M

                if(self.pre_t_pos_ctl < 0):
                        self.pre_t_pos_ctl = time.time()
                new_micros = time.time()
                elapsed_time = new_micros - self.pre_t_pos_ctl
                self.pre_t_pos_ctl = new_micros

                self.buf_enc_com[self.M_L] += (e_v_com[self.M_L]*elapsed_time)
                self.buf_enc_com[self.M_R] += (e_v_com[self.M_R]*elapsed_time)

                enc_com = [0, 0]
                enc_com[self.M_L] = int(self.buf_enc_com[self.M_L])
                enc_com[self.M_R] = int(self.buf_enc_com[self.M_R])

                self.buf_enc_com[self.M_L] -= enc_com[self.M_L]
                self.buf_enc_com[self.M_R] -= enc_com[self.M_R]

                return enc_com
