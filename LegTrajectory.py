# -*- coding: utf-8 -*-

import numpy as np

def calcQuinticFactor(x, v, t):
    Cmat = np.array([
        [t[0]**5, t[0]**4, t[0]**3, t[0]**2, t[0], 1],
        [t[1]**5, t[1]**4, t[1]**3, t[1]**2, t[1], 1],
        [t[2]**5, t[2]**4, t[2]**3, t[2]**2, t[2], 1],
        [5*t[3]**4, 4*t[3]**3, 3*t[3]**2, 2*t[3], 1, 0],
        [5*t[4]**4, 4*t[4]**3, 3*t[4]**2, 2*t[4], 1, 0],
        [5*t[5]**4, 4*t[5]**3, 3*t[5]**2, 2*t[5], 1, 0]
    ])

    Xvec = np.array([
        x[0], x[1], x[2], v[0], v[1], v[2] 
    ])
    
    Avec = np.linalg.pinv(Cmat)@Xvec
    
    return Avec

# 目標位置と前回位置、現在位置から脚軌道を計算
def calcQuinticFactorXYZ(p_pre, p_cur, p_obj, v_pre, v_cur, v_obj, t):
    x = [p_pre[0], p_cur[0], p_obj[0]]
    y = [p_pre[1], p_cur[1], p_obj[1]]
    z = [p_pre[2], p_cur[2], p_obj[2]]
    vx = [v_pre[0], v_cur[0], v_obj[0]]
    vy = [v_pre[1], v_cur[1], v_obj[1]]
    vz = [v_pre[2], v_cur[2], v_obj[2]]
    fact_x = calcQuinticFactor(x, vx, t)
    fact_y = calcQuinticFactor(y, vy, t)
    fact_z = calcQuinticFactor(z, vz, t)
    
    return [fact_x, fact_y, fact_z]

def calcPoint(fact_xyz, t):
    x = fact[0][0]*t**5 + fact[0][1]*t**4 + fact[0][2]*t**3 + fact[0][3]*t**2 + fact[0][4]*t + fact[0][5]
    y = fact[1][0]*t**5 + fact[1][1]*t**4 + fact[1][2]*t**3 + fact[1][3]*t**2 + fact[1][4]*t + fact[1][5]
    z = fact[2][0]*t**5 + fact[2][1]*t**4 + fact[2][2]*t**3 + fact[2][3]*t**2 + fact[2][4]*t + fact[2][5]

    return [x, y, z]

def calcVelocity(fact_xyz, t):
    vx = 5*fact[0][0]*t**4 + 4*fact[0][1]*t**3 + 3*fact[0][2]*t**2 + 2*fact[0][3]*t + fact[0][4]
    vy = 5*fact[1][0]*t**4 + 4*fact[1][1]*t**3 + 3*fact[1][2]*t**2 + 2*fact[1][3]*t + fact[1][4]
    vz = 5*fact[2][0]*t**4 + 4*fact[2][1]*t**3 + 3*fact[2][2]*t**2 + 2*fact[2][3]*t + fact[2][4]
    
    return [vx, vy, vz]

# 遊脚前半、次回目標位置計算
def calcNextPV(p_str, p_cur, p_obj, v_str, v_cur, v_obj, t_str, t_cur, t_obj, dt):
    t = [t_str, t_cur, t_obj, t_str, t_cur, t_obj]
    fact_xyz = calcQuinticFactorXYZ(p_str, p_cur, p_obj, v_str, v_cur, v_obj, t)
    p_next = calcPoint(fact_xyz, t+dt)
    v_next = calcVelocity(fact_xyz, t+dt)

    return p_next, v_next

def timeDiv(t_start, t_goal, div_num):
    

# 目標位置までを分割数で割ったベクトルを返す
def linearDiv(p_start, p_goal, div_num):
    div_vec = (p_goal - p_start)/div_num

    return div_vec

# 分割ベクトルをゴールまで足していく
def vecIntegral(p_start, p_goal, p_now, div_vec, div_num, iter_now):
    # iter_nowは0スタートなので(div_num-1)でゴール位置に到達
    if(iter_now < div_num):
        p_next = p_now + div_vec

        return p_next
    else:
        p_next = p_goal

        return p_next
