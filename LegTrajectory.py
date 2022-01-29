# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

g = 9800 # 単位は [mm/s^2]

# 5次スプラインを求めるための行列は時間軸を正規化するので固定可能
S_RATE = 1.0
Cmat = np.array([
    [0, 0, 0, 0, 0, 1],
    [S_RATE**5, S_RATE**4, S_RATE**3, S_RATE**2, S_RATE, 1],
    [0, 0, 0, 0, 1, 0],
    [5*S_RATE**4, 4*S_RATE**3, 3*S_RATE**2, 2*S_RATE, 1, 0],
    [0, 0, 0, 2, 0, 0],
    [20*S_RATE**3, 12*S_RATE**2, 6*S_RATE, 2, 0, 0]
])
Cmat_inv = np.linalg.inv(Cmat)

# 正規化5次関数の係数を計算
def calcQuinticFactor(X, V, A):
    Xvec = np.array([
        X[0], X[1], V[0], V[1], A[0], A[1] 
    ])
    Avec = Cmat_inv@Xvec
    
    return Avec

# 目標位置と前回位置、現在位置から脚軌道を計算
def calcQuinticFactorXYZ(p0, p1, v0, v1, a0, a1, t0, t1):
    eps = t1 - t0
    X = [p0[0]*S_RATE/eps, p1[0]*S_RATE/eps]
    Y = [p0[1]*S_RATE/eps, p1[1]*S_RATE/eps]
    Z = [p0[2]*S_RATE/eps, p1[2]*S_RATE/eps]
    VX = [v0[0], v1[0]]
    VY = [v0[1], v1[1]]
    VZ = [v0[2], v1[2]]
    AX = [a0[0]*eps/S_RATE, a1[0]*eps/S_RATE]
    AY = [a0[1]*eps/S_RATE, a1[1]*eps/S_RATE]
    AZ = [a0[2]*eps/S_RATE, a1[2]*eps/S_RATE]
    fact_x = calcQuinticFactor(X, VX, AX)
    fact_y = calcQuinticFactor(Y, VY, AY)
    fact_z = calcQuinticFactor(Z, VZ, AZ)

    return [fact_x, fact_y, fact_z]

def calcPoint(fact_xyz, t0, t1, t):
    eps = t1 - t0
    S = (t - t0)*S_RATE/eps
    X = fact_xyz[0][0]*S**5 + fact_xyz[0][1]*S**4 + fact_xyz[0][2]*S**3 + fact_xyz[0][3]*S**2 + fact_xyz[0][4]*S + fact_xyz[0][5]
    Y = fact_xyz[1][0]*S**5 + fact_xyz[1][1]*S**4 + fact_xyz[1][2]*S**3 + fact_xyz[1][3]*S**2 + fact_xyz[1][4]*S + fact_xyz[1][5]
    Z = fact_xyz[2][0]*S**5 + fact_xyz[2][1]*S**4 + fact_xyz[2][2]*S**3 + fact_xyz[2][3]*S**2 + fact_xyz[2][4]*S + fact_xyz[2][5]
    x = X*eps/S_RATE
    y = Y*eps/S_RATE
    z = Z*eps/S_RATE

    return [x, y, z]

def calcVelocity(fact_xyz, t0, t1, t):
    eps = t1 - t0
    S = (t - t0)*S_RATE/eps
    # vは正規化前と同じなので普通に計算
    vx = 5*fact_xyz[0][0]*S**4 + 4*fact_xyz[0][1]*S**3 + 3*fact_xyz[0][2]*S**2 + 2*fact_xyz[0][3]*S + fact_xyz[0][4]
    vy = 5*fact_xyz[1][0]*S**4 + 4*fact_xyz[1][1]*S**3 + 3*fact_xyz[1][2]*S**2 + 2*fact_xyz[1][3]*S + fact_xyz[1][4]
    vz = 5*fact_xyz[2][0]*S**4 + 4*fact_xyz[2][1]*S**3 + 3*fact_xyz[2][2]*S**2 + 2*fact_xyz[2][3]*S + fact_xyz[2][4]
    
    return [vx, vy, vz]

def calcAccele(fact_xyz, t0, t1, t):
    eps = t1 - t0
    S = (t - t0)*S_RATE/eps
    AX = 20*fact_xyz[0][0]*S**3 + 12*fact_xyz[0][1]*S**2 + 6*fact_xyz[0][2]*S + 2*fact_xyz[0][3]
    AY = 20*fact_xyz[1][0]*S**3 + 12*fact_xyz[1][1]*S**2 + 6*fact_xyz[1][2]*S + 2*fact_xyz[1][3]
    AZ = 20*fact_xyz[2][0]*S**3 + 12*fact_xyz[2][1]*S**2 + 6*fact_xyz[2][2]*S + 2*fact_xyz[2][3]
    ax = AX*S_RATE/eps
    ay = AY*S_RATE/eps
    az = AZ*S_RATE/eps

    return [ax, ay, az]

# 次回目標位置と理論上の速度を計算
def calcNextPVA(p, v, a, t, dt):
    fact_xyz = calcQuinticFactorXYZ(p[0], p[1], v[0], v[1], a[0], a[1], t[0], t[1])
    p_next = calcPoint(fact_xyz, t[0], t[1], t[0]+dt)
    v_next = calcVelocity(fact_xyz, t[0], t[1], t[0]+dt)
    a_next = calcAccele(fact_xyz, t[0], t[1], t[0]+dt)

    return p_next, v_next, a_next

# ボディ速度を見て目標位置を決定(ジャイロから計算された擬似的な値を想定しているのでヨー軸の制御は無し)
# state = [p_xyz, v_xyz, a_xyz, t]
# h_offsetは立脚期間中でボディ重量により脚が沈み込む分の補正値
def calcBalancePVA(state_cur, v_body_tar, v_body_obs, leg_center, body_h, leg_up_h, h_offset, T, dt):
    T_list = [0, T/4, T/2, 3*T/4, T]
    _v_body_tar = np.array(v_body_tar)
    _v_body_obs = np.array(v_body_obs)
    _leg_center = np.array(leg_center)
    _v_leg_stance = -_v_body_tar - 4/T*(_v_body_obs - _v_body_tar)*(body_h/g)**0.5
    alpha = 1.0
    
    if(state_cur[3]>=T_list[0] and state_cur[3]<T_list[1]):
        p_bottom = _leg_center - np.array([0, 0, h_offset])
        a_bottom = [0, 0, 0]
        p = [state_cur[0], p_bottom]
        v = [state_cur[1], _v_leg_stance]
        a = [state_cur[2], a_bottom]
        t = [state_cur[3], T_list[1]]
        # print("0 ~ T/4")
        p_next, v_next, a_next = calcNextPVA(p, v, a, t, dt)
        # 立脚期のxy方向速度は一定なので別で計算して上書き
        # p_next[0] = state_cur[0][0] + _v_leg_stance[0]*dt
        # p_next[1] = state_cur[0][1] + _v_leg_stance[1]*dt

    elif(state_cur[3]>=T_list[1] and state_cur[3]<T_list[2]):
        p_up = _leg_center - T/4*_v_body_tar - (_v_body_obs*alpha - _v_body_tar)*(body_h/g)**0.5
        a_up = [0, 0, 0]
        p = [state_cur[0], p_up]
        v = [state_cur[1], _v_leg_stance]
        a = [state_cur[2], a_up]
        t = [state_cur[3], T_list[2]]
        # print("T/4 ~ T/2")
        p_next, v_next, a_next = calcNextPVA(p, v, a, t, dt)
        # 立脚期のxy方向速度は一定なので別で計算して上書き
        # p_next[0] = state_cur[0][0] + _v_leg_stance[0]*dt
        # p_next[1] = state_cur[0][1] + _v_leg_stance[1]*dt

    elif(state_cur[3]>=T_list[2] and state_cur[3]<T_list[3]):
        p_top = _leg_center + np.array([0, 0, leg_up_h])
        a_top = [0, 0, 0]
        p = [state_cur[0], p_top]
        v = [state_cur[1], -2*_v_leg_stance]
        a = [state_cur[2], a_top]
        t = [state_cur[3], T_list[3]]
        # print("T/2 ~ 3T/4")
        p_next, v_next, a_next = calcNextPVA(p, v, a, t, dt)

    elif(state_cur[3]>=T_list[3] and state_cur[3]<T_list[4]):
        p_touch = _leg_center + T/4*_v_body_tar + (_v_body_obs*alpha - _v_body_tar)*(body_h/g)**0.5
        a_touch = [0, 0, 0]
        p = [state_cur[0], p_touch]
        v = [state_cur[1], _v_leg_stance]
        a = [state_cur[2], a_touch]
        t = [state_cur[3], T_list[4]]
        # print("3T/4 ~ T")
        p_next, v_next, a_next = calcNextPVA(p, v, a, t, dt)

    else:
        print("Error : Current t = {}".format(state_cur[3]))

    return p_next, v_next, a_next

def makeTrajectoryList(state_cur, v_body_tar, v_body_obs, leg_center, body_h, leg_up_h, h_offset, T, dt):
    N = int(5*T/dt)
    px_list, py_list, pz_list = [], [], []
    vx_list, vy_list, vz_list = [], [], []
    ax_list, ay_list, az_list = [], [], []
    t_list = []
    for i in range(N):
        p, v, a = calcBalancePVA(state_cur, v_body_tar, v_body_obs, leg_center, body_h, leg_up_h, h_offset, T, dt)
        px_list.append(p[0])
        py_list.append(p[1])
        pz_list.append(p[2])
        vx_list.append(v[0])
        vy_list.append(v[1])
        vz_list.append(v[2])
        ax_list.append(a[0])
        ay_list.append(a[1])
        az_list.append(a[2])
        t_list.append(state_cur[3])
        if(state_cur[3]+dt > T):
            state_cur = [p, v, a, 0]
        else:
            state_cur = [p, v, a, state_cur[3]+dt]
    
    return [[px_list, py_list, pz_list], [vx_list, vy_list, vz_list], [ax_list, ay_list, az_list], t_list]

def main():
    state_cur = [[0, 0, -95], [0, 0, 0], [0, 0, 0], 0]
    v_body_tar = [200, 200, 0]
    v_body_obs = [90, 10, 0]
    leg_center = [0, 0, -90]
    body_h = 95
    leg_up_h = 40
    h_offset = 3
    T = 0.3
    dt = 0.05 # 最小：0.001
    trj = makeTrajectoryList(state_cur, v_body_tar, v_body_obs, leg_center, body_h, leg_up_h, h_offset, T, dt)

    limit = 2000
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.plot(trj[0][0], trj[0][1], trj[0][2], marker='o')
    #plt.plot(trj[3], trj[0][2])
    #plt.plot(trj[3], trj[1][2])
    #plt.plot(trj[3], trj[2][2])
    plt.show()
    
if __name__ == '__main__':
    main()
