import time
import numpy as np
import open3d as o3d
import util as util
import ctypes
import os
from multiprocessing import Pool
from tqdm import tqdm
import json

class OnlineToolKit:
    def __init__(self) -> None:
        # S1 parameters
        self.PC_MAX_RANGE = 60
        self.PC_NUM_RING = 60
        self.PC_NUM_SECTOR = 60
        self.PC_MIN_Z = -2.3
        self.PC_MAX_Z = 0.7

        # S2 parameters
        self.PF_MIN_RANGE = -1
        self.PF_MAX_RANGE = 60
        self.PF_ALPHA = 1.5
        self.PF_K = 1.0
        self.PF_D = 1E5
        self.PF_H = 0.5

        lib_path = './../pcl_file/build/libsus_vis_lib.so'
        lib = ctypes.CDLL(lib_path)
        lib.callsByPython.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
        lib.callsByPython.restype = ctypes.POINTER(ctypes.c_float)
        self.LIB = lib

        # S3 parameters

    def _cal_s1_score(self, pc):
        # 1. make scan and bev desc
        scan_desc = np.zeros((self.PC_NUM_RING, self.PC_NUM_SECTOR))
        bev_max = np.zeros((self.PC_MAX_RANGE, self.PC_MAX_RANGE))
        bev_min = np.zeros((self.PC_MAX_RANGE, self.PC_MAX_RANGE))
        pt_range = self.PC_MAX_RANGE / 2
        # 2. select near points and build desc
        pt_x = pc[:, 0]
        pt_y = pc[:, 1]
        pt_z = pc[:, 2]
        valid_indices = np.where((pt_z < self.PC_MAX_Z) & (pt_z > self.PC_MIN_Z))     
        pt_x = pt_x[valid_indices]
        pt_y = pt_y[valid_indices]
        pt_z = pt_z[valid_indices]
        # 2.1 build scan desc
        azim_range = np.sqrt(pt_x ** 2+ pt_y ** 2)
        azim_angle = np.rad2deg(np.arctan2(pt_y, pt_x))
        azim_angle[azim_angle < 0] += 360

        valid_indices = np.where(azim_range < self.PC_MAX_RANGE) 
        azim_sector = np.floor(azim_angle[valid_indices] / (360 / self.PC_NUM_SECTOR)).astype(np.int32)
        azim_ring = np.floor(azim_range[valid_indices] / (self.PC_MAX_RANGE / self.PC_NUM_RING)).astype(np.int32)

        np.add.at(scan_desc, (azim_ring, azim_sector), 1)

        valid_indices = np.where((pt_x < pt_range) & (pt_x > -pt_range) & (pt_y < pt_range) & (pt_y > -pt_range))
        pt_x_valid = pt_x[valid_indices] + pt_range
        pt_y_valid = pt_y[valid_indices] + pt_range
        pt_z_valid = pt_z[valid_indices]

        # 2.2. build bev scan
        bev_max_indices = (pt_x_valid.astype(int), pt_y_valid.astype(int))
        np.maximum.at(bev_max, bev_max_indices, pt_z_valid)

        bev_min_indices = (pt_x_valid.astype(int), pt_y_valid.astype(int))
        np.minimum.at(bev_min, bev_min_indices, pt_z_valid)

        bev_scan = np.subtract(bev_max, bev_min)

        # 3. calculate entropy
        scan_entropy = util.cal_scene_entropy(scan_desc, pc[:, :3])
        bev_entropy = util.cal_scene_entropy(bev_scan, pc[:, :3])
        
        return scan_entropy, bev_entropy
    
    def _cal_s2_score(self, root_path , pc_idx):
        s2_array = self.LIB.callsByPython(str(root_path).encode('utf-8'), str(pc_idx).encode('utf-8'), b'none')
        s2_array = [s2_array[i] for i in range(1,int(s2_array[0])+1)]
        s2_array = [s2_array[i:i+4] for i in range(0, len(s2_array), 4)]

        # s2:array: idx, distace, mesh_count, bbox_volume
        s2_array = [s2 for s2 in s2_array if s2[1] < self.PF_MAX_RANGE]
        distance = [s2[1] for s2 in s2_array]
        mesh_count = [s2[2] for s2 in s2_array]
        bbox_volume = [s2[3] for s2 in s2_array]
        distance = np.array(distance)
        l = distance / self.PF_MAX_RANGE
        s2_score = (distance**2 * mesh_count / (bbox_volume * np.sqrt(bbox_volume))) / (self.PF_D * (l * np.log(l) + self.PF_H))
        # s2_number = np.sum(s2_score)
        # s2_vector = np.arctan2(np.sum(s2_score * np.cos(degree)), np.sum(s2_score * np.sin(degree)))
        return s2_score

def process_pc(args):
    root_path, pc_idx = args
    pc_path = os.path.join(root_path, 'lidar', f"{pc_idx}.pcd")
    pc = o3d.io.read_point_cloud(pc_path)
    online_toolkit = OnlineToolKit()
    s1_score = online_toolkit._cal_s1_score(np.asarray(pc.points))
    s2_score = online_toolkit._cal_s2_score(root_path, pc_idx)

    s1_gt, s1_metric = 0, 60
    value_types = {'Car', 'Truck', 'Bus', 'Van'}
    with open(os.path.join(root_path, 'label', f"{pc_idx}.json"), 'r') as f:
        json_file = json.load(f)
    objs = json_file['objs']
    for obj in objs:
        if obj['obj_type'] in value_types:
            dist = np.sqrt(obj['psr']['position']['x']**2 + obj['psr']['position']['y']**2 + obj['psr']['position']['z']**2)
            if dist < s1_metric:
                s1_gt += 1

    return pc_idx, s1_gt, s1_score, s2_score

if __name__ == "__main__":
    origin_path = '/home/ghosnp/mirror/mmdet_sandbox/home/dataset/'
    save_path = '/home/ghosnp/mirror/mmdet_sandbox/home/dataset/'
    for scene_id in os.listdir(origin_path):
        print(f"Processing scene: {scene_id}")
        root_path = os.path.join(origin_path, scene_id)
        pc_path = os.path.join(root_path, 'lidar')
        # online_toolkit = OnlineToolKit()
        pc_indices = [pc_idx[:-4] for pc_idx in os.listdir(pc_path) if pc_idx.endswith('.pcd')]
        args = [(root_path, pc_idx) for pc_idx in pc_indices]
        total = len(args)

        with Pool() as pool:
            results = list(tqdm(pool.imap(process_pc, args), total=total, unit='files'))

        # for s1_score, s2_score in results:
        #     print(f"s1_score: {s1_score}, s2_score: {s2_score}")    

        with open(save_path+scene_id+'_online.txt', 'w') as f:
            for pc_idx, s1_gt, s1_score, s2_score in results:
                f.write(f"{pc_idx} {s1_gt} {s1_score[0]} {s1_score[1]} {np.sum(s2_score)}\n")

