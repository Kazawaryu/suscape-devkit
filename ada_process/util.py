import numpy as np
import math
from enum import Enum

    
def cal_scene_entropy(desc, pcd):
    max_pcd = np.max(pcd,axis=0)
    min_pcd = np.min(pcd,axis=0)
    vt = len(pcd) / (max_pcd[0]-min_pcd[0])*(max_pcd[1]-min_pcd[1])*(max_pcd[2]-min_pcd[2])
    nonzero_indices = np.nonzero(desc)
    vi = desc[nonzero_indices]
    entropy = -np.sum((vi /vt) * np.log(vi /vt))

    return entropy

class LShapeFitting():  
    class Criteria(Enum):
        AREA = 1
        CLOSENESS = 2
        VARIANCE = 3

    def __init__(self):
        # Parameters
        self.criteria = self.Criteria.VARIANCE
        self.min_dist_of_closeness_crit = 0.01  # [m]
        self.dtheta_deg_for_serarch = 5.0  # [deg]
        self.R0 = 1.0  # [m] range segmentation param
        self.Rd = 0.01  # [m] range segmentation param

    def _calc_area_criterion(self, c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        alpha = -(c1_max - c1_min) * (c2_max - c2_min)

        return alpha

    def _calc_closeness_criterion(self, c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        D1 = [min([np.linalg.norm(c1_max - ic1),
                    np.linalg.norm(ic1 - c1_min)]) for ic1 in c1]
        D2 = [min([np.linalg.norm(c2_max - ic2),
                    np.linalg.norm(ic2 - c2_min)]) for ic2 in c2]

        beta = 0
        for i, _ in enumerate(D1):
            d = max(min([D1[i], D2[i]]), self.min_dist_of_closeness_crit)
            beta += (1.0 / d)

        return beta

    def _calc_variance_criterion(self, c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        D1 = [min([np.linalg.norm(c1_max - ic1),
                    np.linalg.norm(ic1 - c1_min)]) for ic1 in c1]
        D2 = [min([np.linalg.norm(c2_max - ic2),
                    np.linalg.norm(ic2 - c2_min)]) for ic2 in c2]

        E1, E2 = [], []
        for (d1, d2) in zip(D1, D2):
            if d1 < d2:
                E1.append(d1)
            else:
                E2.append(d2)

        V1 = 0.0
        if E1:
            V1 = - np.var(E1)

        V2 = 0.0
        if E2:
            V2 = - np.var(E2)

        gamma = V1 + V2

        return gamma


    def get_rectangle(self, X,type):
        dtheta = np.deg2rad(self.dtheta_deg_for_serarch)
        minp = (-float('inf'), None)
        cnt = 0
        for theta in np.arange(0.0, np.pi / 2.0 - dtheta, dtheta):
            
            cnt += 1
            if type != 14. and cnt > 1:
                break

            e1 = np.array([np.cos(theta), np.sin(theta)])
            e2 = np.array([-np.sin(theta), np.cos(theta)])

            c1 = X @ e1.T
            c2 = X @ e2.T

            # Select criteria
            if self.criteria == self.Criteria.AREA:
                cost = self._calc_area_criterion(c1, c2)
            elif self.criteria == self.Criteria.CLOSENESS:
                cost = self._calc_closeness_criterion(c1, c2)
            elif self.criteria == self.Criteria.VARIANCE:
                cost = self._calc_variance_criterion(c1, c2)

            if minp[0] < cost:
                minp = (cost, theta)
                worse = 0
            else:
                worse+=1
                if worse == 13:
                    break
        # print(type,cnt,worse)

        # calculate best rectangle
        sin_s = np.sin(minp[1])
        cos_s = np.cos(minp[1])

        c1_s = X @ np.array([cos_s, sin_s]).T
        c2_s = X @ np.array([-sin_s, cos_s]).T

        rect = RectangleData()
        rect.a[0] = cos_s
        rect.b[0] = sin_s
        rect.c[0] = min(c1_s)
        rect.a[1] = -sin_s
        rect.b[1] = cos_s
        rect.c[1] = min(c2_s)
        rect.a[2] = cos_s
        rect.b[2] = sin_s
        rect.c[2] = max(c1_s)
        rect.a[3] = -sin_s
        rect.b[3] = cos_s
        rect.c[3] = max(c2_s)

        return rect, theta
    

    def get_rectangle_given_theta(self, X, theta):
        minp = (-float('inf'), None)
        theta = -np.float64(theta)
        e1 = np.array([np.cos(theta), np.sin(theta)])
        e2 = np.array([-np.sin(theta), np.cos(theta)])

        c1 = X @ e1.T
        c2 = X @ e2.T

        # Select criteria
        if self.criteria == self.Criteria.AREA:
            cost = self._calc_area_criterion(c1, c2)
        elif self.criteria == self.Criteria.CLOSENESS:
            cost = self._calc_closeness_criterion(c1, c2)
        elif self.criteria == self.Criteria.VARIANCE:
            cost = self._calc_variance_criterion(c1, c2)

        if minp[0] < cost:
            minp = (cost, theta)

        sin_s = np.sin(minp[1])
        cos_s = np.cos(minp[1])
        c1_s = X @ np.array([cos_s, sin_s]).T
        c2_s = X @ np.array([-sin_s, cos_s]).T

        rect = RectangleData()
        rect.a[0] = cos_s
        rect.b[0] = sin_s
        rect.c[0] = min(c1_s)
        rect.a[1] = -sin_s
        rect.b[1] = cos_s
        rect.c[1] = min(c2_s)
        rect.a[2] = cos_s
        rect.b[2] = sin_s
        rect.c[2] = max(c1_s)
        rect.a[3] = -sin_s
        rect.b[3] = cos_s
        rect.c[3] = max(c2_s)

        return rect


class RectangleData():
    def __init__(self):
        self.a = [None] * 4
        self.b = [None] * 4
        self.c = [None] * 4
        self.rect_c_x = [None] * 5
        self.rect_c_y = [None] * 5

    def calc_rect_contour(self):
        self.rect_c_x[0], self.rect_c_y[0] = self.calc_cross_point(self.a[0:2], self.b[0:2], self.c[0:2])
        self.rect_c_x[1], self.rect_c_y[1] = self.calc_cross_point(self.a[1:3], self.b[1:3], self.c[1:3])
        self.rect_c_x[2], self.rect_c_y[2] = self.calc_cross_point(self.a[2:4], self.b[2:4], self.c[2:4])
        self.rect_c_x[3], self.rect_c_y[3] = self.calc_cross_point([self.a[3], self.a[0]], [self.b[3], self.b[0]], [self.c[3], self.c[0]])
        self.rect_c_x[4], self.rect_c_y[4] = self.rect_c_x[0], self.rect_c_y[0]
        return self.rect_c_x,self.rect_c_y

    def calc_cross_point(self, a, b, c):
        x = (b[0] * -c[1] - b[1] * -c[0]) / (a[0] * b[1] - a[1] * b[0])
        y = (a[1] * -c[0] - a[0] * -c[1]) / (a[0] * b[1] - a[1] * b[0])
        return x, y