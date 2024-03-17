import time
import numpy as np
import util as util
import math
import ctypes
import os

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
        lib.callsByPython.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        lib.callsByPython.restype = ctypes.POINTER(ctypes.c_float)
        self.LIB = lib

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
    
    def _cal_s2_score(self, pc_idx):
        s2_array = self.LIB.lib.callsByPython(str(pc_idx).encode('utf-8'), b'none')
        s2_array = [s2_array[i] for i in range(1,int(s2_array[0])+1)]
        s2_array = [s2_array[i:i+4] for i in range(0, len(s2_array), 4)]

        # s2:array: idx, distace, mesh_count, bbox_volume
        s2_array = [s2 for s2 in s2_array if s2[1] < self.PF_MAX_RANGE]
        distance = [s2[1] for s2 in s2_array]
        mesh_count = [s2[2] for s2 in s2_array]
        bbox_volume = [s2[3] for s2 in s2_array]
        l = distance / self.PF_MAX_RANGE

        s2_score = (distance**2 * mesh_count / (bbox_volume * np.sqrt(bbox_volume))) / (self.PF_D * (l * np.log(l) + self.PF_H))
        # s2_number = np.sum(s2_score)
        # s2_vector = np.arctan2(np.sum(s2_score * np.cos(degree)), np.sum(s2_score * np.sin(degree)))
        return s2_score

    