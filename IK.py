# -*- coding: utf-8 -*-

import numpy as np
from math import *

# body parameters
leg_l1 = 90.0
leg_l2 = 90.0
sole_r = 20.0
body_l = 100.0
body_w = 50.0

# calc parameters
min_rxz = 10.0
max_z = -68.0

def regionCut(p_target):
    if(p_target[2] < max_z):
        range_out = False
    else:
        range_out = True

    return range_out

def calcIK(p_target):
    [x, y, z] = p_target
    r2 = x**2 + y**2 + z**2
    if(r2 > (leg_l1+leg_l2)**2):
        theta_out = [0.0, pi/2, 0.0]
        is_range_out = True
    
        return theta_out, is_range_out
    rxz2 = x**2 + z**2
    rxz = rxz2**0.5
    if(rxz < min_rxz):
        theta_out = [0.0, 0.0, 0.0]
        is_range_out = True

        return theta_out, is_range_out
    theta3 = acos(-(leg_l1**2+leg_l2**2-r2)/(2*leg_l1*leg_l2))
    lx = leg_l2*sin(theta3)
    al = atan2(-z, x)
    lxrxz = lx/rxz
    if(lx > rxz): # 丸め誤差などで起こりうる
        lxrxz = 1.0
    be = acos(lxrxz)
    theta1 = be - al
    zd = -x*sin(theta1) + z*cos(theta1)
    theta2 = atan2(y, -zd)
    is_range_out = False

    # θ1、θ3は図形的に軸が反転しているので符号反転
    return [-theta1, theta2, -theta3], is_range_out

# レンジ外が入力された場合、前回位置に向かう
def legIK(p_target, theta_pre, leg_num):
    # 左右対称のため、y軸反転
    if(leg_num == 2 or leg_num == 3):
        p_target_new = p_target*np.array([1, -1, 1])
    else:
        p_target_new = p_target
    if(regionCut(p_target_new)):
        theta_out = theta_pre
        is_pre_theta = True

        return theta_out, is_pre_theta
    else:
        theta_out, is_range_out = calcIK(p_target_new)
        if(is_range_out):
            theta_out = theta_pre
            is_pre_theta = True
      
            return theta_out, is_pre_theta
        else:
            is_pre_theta = False
      
            return theta_out, is_pre_theta

def legFK(theta):
    R_theta1 = np.array([
        [cos(theta[0]), 0, sin(theta[0])],
        [0, 1, 0],
        [-sin(theta[0]), 0, cos(theta[0])]
    ])
    R_theta2 = np.array([
        [1, 0, 0],
        [0, cos(theta[1]), -sin(theta[1])],
        [0, sin(theta[1]), cos(theta[1])]
    ])
    R_theta3 = np.array([
        [cos(theta[2]), 0, sin(theta[2])],
        [0, 1, 0],
        [-sin(theta[2]), 0, cos(theta[2])]
    ])
    J12 = np.array([0, 0, 0])
    J3 = J12 + R_theta2@R_theta1@np.array([0, 0, -leg_l1])
    J4 = J3 + R_theta3@np.array([0, 0, -leg_l2])

    return np.array([J12, J3, J4])