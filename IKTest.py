# -*- coding: utf-8 -*-

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

from math import *
import time

MIN_RXZ = 10

l1 = 90
l2 = 90

def IK(x, y, z):
    r2 = x**2 + y**2 + z**2
    if(r2 > (l1+l2)**2):
        #print("遠すぎィ！！")
        theta1 = 0.0
        theta2 = pi/2
        theta3 = 0.0

        return theta1, theta2, theta3

    rxz2 = x**2 + z**2
    rxz = rxz2**0.5
    if(rxz < MIN_RXZ):
        #print("allzero")
        theta1 = 0.0
        theta2 = 0.0
        theta3 = 0.0
        
    else:
        theta3 = acos(-(l1**2+l2**2-r2)/(2*l1*l2))
        lx = l2*sin(theta3)
        al = atan2(-z, x)
        lxrxz = lx/rxz
        if(lx > rxz): # 丸め誤差などで起こりうる
            lxrxz = 1.0
        be = acos(lxrxz)
        theta1 = be - al
        zd = -x*sin(theta1) + z*cos(theta1)
        theta2 = atan2(y, -zd)

    return theta1, theta2, theta3

def FK(theta1, theta2, theta3):
    P03 = [-l1*sin(theta1)*cos(theta2),
           l1*sin(theta2),
           -l1*cos(theta1)*cos(theta2)]
    P04 = [-l2*sin(theta3)*cos(theta1) + (-l2*cos(theta3) - l2)*sin(theta1)*cos(theta2),
           -(-l2*cos(theta3) - l2)*sin(theta2),
           l2*sin(theta1)*sin(theta3) + (-l2*cos(theta3) - l2)*cos(theta1)*cos(theta2)]

    return P03, P04

def calcErr(theta1, theta2, theta3): # 入力の単位はdeg
    J3, P = FK(-theta1/180*pi, theta2/180*pi, -theta3/180*pi)
    x = P[0]
    y = P[1]
    z = P[2]
    theta1d, theta2d, theta3d = IK(x, y, z)

    # エラーの単位もdeg
    err1 = theta1d/pi*180 - theta1
    err2 = theta2d/pi*180 - theta2
    err3 = theta3d/pi*180 - theta3

def main():
    start_time = time.perf_counter()
    for i1 in range(0, 36):
        for i2 in range(0, 36):
            for i3 in range(0, 36):
                calcErr(i1*10, i2*10, i3*10)
    end_time = time.perf_counter()
    # 経過時間を出力(秒)
    elapsed_time = end_time - start_time
    print(elapsed_time)
    
if __name__ == '__main__':
    main()
