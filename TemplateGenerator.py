import numpy as np
import csv
import pandas as pd

# from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import IK

def rad2XYZ(rad_array):
    pos_xyz = np.array(list(map(IK.legFK, rad_array)))
    print("-----------")
    print(pos_xyz)
    
    return pos_xyz[:, 2] # 脚先以外の座標もはいっているので抽出
            
def main():
    template_file = './simple_walk.csv'
    data_df = pd.read_csv(template_file)
    
    # 一周分の歩行データをタイムスタンプの列を無視して取得
    n = 80 # データ個数
    data_array = data_df.values[0:n, 1:13]
    data_len = len(data_array)
    
    # データを整形
    data_array = np.reshape(data_array, (data_len, 4, 3))
    
    # 各脚先座標のarrayを作成
    XYZ_array = np.array(list(map(rad2XYZ, data_array)))
    
    # データを2次元配列にしてCSVに保存
    XYZ_mat = np.reshape(XYZ_array, (data_len, 12))
    print(XYZ_mat[0])
    XYZ_template_file = './test_simple_walk_xyz.csv'
    with open(XYZ_template_file, 'w') as f:
        writer = csv.writer(f, lineterminator='\n')
        item_name = [
            'leg 0 x', 'leg 0 y', 'leg 0 z',
            'leg 1 x', 'leg 1 y', 'leg 1 z',
            'leg 2 x', 'leg 2 y', 'leg 2 z',
            'leg 3 x', 'leg 3 y', 'leg 3 z'
        ]
        writer.writerow(item_name)
        writer.writerows(XYZ_mat)

    # 3Dプロット
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    
    plot_leg = 0

    ax.scatter(XYZ_array[:, plot_leg, 0], XYZ_array[:, plot_leg, 1], XYZ_array[:, plot_leg, 2])
    plt.show()

    plt.plot(XYZ_array[:, plot_leg, 0])
    plt.show()

    plt.plot(XYZ_array[:, plot_leg, 1])
    plt.show()

    plt.plot(XYZ_array[:, plot_leg, 2])
    plt.show()
        
if __name__ == '__main__':
    main()
