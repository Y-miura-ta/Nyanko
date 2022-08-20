import numpy as np
import csv
import pandas as pd

# from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import IK

def rad2XYZ(rad_array):
    leg_list = [0, 1, 2, 3]
    pos_xyz = np.array(list(map(IK.legSmartFK, rad_array, leg_list)))
    
    return pos_xyz
            
def main():
    template_file = './simple_walk.csv'
    data_df = pd.read_csv(template_file)
    
    # 1??????????????????????????
    n = 80 # ?????
    data_array = data_df.values[0:n, 1:13]
    data_len = len(data_array)
    
    # ??????
    data_array = np.reshape(data_array, (data_len, 4, 3))

    # ??????array???
    rad2XYZ(data_array[0])
    XYZ_array = np.array(list(map(rad2XYZ, data_array)))
    print(XYZ_array)

    # ????2???????CSV???
    XYZ_mat = np.reshape(XYZ_array, (data_len, 12))
    print(XYZ_mat)
    XYZ_template_file = './simple_walk_xyz.csv'
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

    # 3D????
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect((1,1,1))

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
