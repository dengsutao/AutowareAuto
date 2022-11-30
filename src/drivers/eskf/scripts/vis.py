import os
from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt
def vis_enu_pose(data):
    enu_poses = np.array([x['enu_pose'] for x in data])
    plt.plot(enu_poses[:,0], enu_poses[:,1])
    plt.savefig('enu_pose.png')
    plt.close()

def vis_gps_enu_pose(data):
    gps_enu_poses = np.array([x['gps_enu_pose'] for x in data])
    plt.plot(gps_enu_poses[:,0], gps_enu_poses[:,1])
    plt.savefig('gps_enu_pose.png')
    plt.close()

def vis_odom_enu_pose(data):
    odom_enu_poses = np.array([x['odom_enu_pose'] for x in data])
    plt.plot(odom_enu_poses[:,0], odom_enu_poses[:,1])
    plt.savefig('odom_enu_pose.png')
    plt.close()

def parse_data(data):
    out = []
    for i,data_i in enumerate(data):
        row = {}
        for j,pair_j in enumerate(data_i):
            idx = pair_j.index(":")
            key = pair_j[:idx]
            values = list(map(float, pair_j[idx+1:].split(',')))
            row[key] = np.array(values)
        out.append(row)
    return out



if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--file_name", type=str)
    args = parser.parse_args()
    saved_file_dir = "../records"
    saved_file_name = args.file_name
    saved_file_path = os.path.join(saved_file_dir, saved_file_name)
    data = [[y[:-1] for y in x.split(' ')] for x in open(saved_file_path)]
    print(data[0])
    parsed_data = parse_data(data)
    vis_enu_pose(parsed_data)
    vis_gps_enu_pose(parsed_data)
    vis_odom_enu_pose(parsed_data)


