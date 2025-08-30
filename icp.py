#%% Classes Integreator and Odometry

import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy import linalg
from math import pi, cos, sin, atan2, sqrt
from scipy.spatial import KDTree

class Integrator:
    def __init__(self, x0, T):
        self.val_prev = x0
        self.T = T
        self.integral = x0

    def update(self, val):
        self.integral += (val + self.val_prev) * self.T / 2
        self.val_prev = val
        return self.integral
    
class Odometry:
    def __init__(self, r, B, T):
        self.x_integrator = Integrator(0, T)
        self.y_integrator = Integrator(0, T)
        self.theta_integrator = Integrator(0, T)
        self.r = r
        self.B = B
        self.theta = self.theta_integrator.integral

    def get_speed(self, wr: float, wl: float) -> tuple:
        v = (wr + wl) * self.r / 2
        dx = v * cos(self.theta)
        dy = v * sin(self.theta)
        dth = (wr - wl)  * self.r / self.B
        return (dx, dy, dth)
    
    def update(self, wr: float, wl: float):
        dx, dy, dth = self.get_speed(wr, wl)
        x, y, self.theta = self.x_integrator.update(dx), self.y_integrator.update(dy), self.theta_integrator.update(dth)
        return (x, y, self.theta)


wheel_radius = 0.0275
base = 0.185
T = 0.08


#%% Odometry

odometry = Odometry(wheel_radius, base, T)
odom = []

with open('data_odom.pkl', 'rb') as f:
    while True:
        try:
            timestamp, wr, wl = pickle.load(f)
            x, y, theta = odometry.update(wr*(pi/180), wl*(pi/180))
            odom.append([timestamp, x, y, theta])
        except EOFError:
            break

odom = np.array(odom)

#%% LaserScan from lidar

allScans = []
singleScan = []
time = 0

try:
    with open('data_laser.pkl', 'rb') as f:

        while True:
            timestamp, rho, ang = pickle.load(f, errors="")
            if not time == timestamp:
                allScans.append((time, np.array(singleScan)))
                singleScan = []
                time = timestamp
            singleScan.append([rho, ang])

except EOFError:
    pass


#%% LaserScan in cartesian coordinates with odometry

scans = []

for i in range (1, len(allScans)):

    time1 = allScans[i][0]
    scan1 = allScans[i][1]
    r_1 = scan1[:, 0]
    ang_1 = scan1[:, 1]

    limit = (r_1 >= 0.1) & (r_1 <= 4)
    r = r_1[limit]
    ang = ang_1[limit]
    

    idx = np.argmin(odom[:, 0] - time1)
    _, tx, ty, th = odom[idx, :]

    x = r * np.cos(ang)
    y = r * np.sin(ang)

    x_base = x * np.cos(th) - y * np.sin(th) + tx
    y_base = x * np.sin(th) + y * np.cos(th) + ty
    dob = np.array(list(zip(x_base, y_base)), dtype=float)
    scans.append(dob)

#%% ICP

def night (scans):
    ICP = np.array(scans[0])

    for i in range (1, len(scans)-1, 2):

        Q = ICP.copy()
        P = scans[i].copy()
        
        kd_tree = KDTree(Q)
        R = np.eye(2)
        t = np.array([0, 0])
        print(i)

        for k in range(100):

            distances, nearest_indices = kd_tree.query(P)
            
            matches = []
            Q_filter = []
            P_filter = []

            for idx in range(len(distances)):
                dist = distances[idx]
                
                if k > 10 and dist > 0.05: 
                    continue
                elif k > 2 and dist > 0.5: 
                    continue
                
                matches.append(idx)
                Q_filter.append(Q[nearest_indices[idx]])
                P_filter.append(P[idx])

            Q_new_1 = np.array(Q_filter)
            P_new_1 = np.array(P_filter)

            q2 = np.mean(Q_new_1, axis = 0)
            p2 = np.mean(P_new_1, axis = 0)

            Q_1 = np.array(Q_new_1 - q2)
            P_1 = np.array(P_new_1 - p2)
            cov = P_1.T @ Q_1

            u, _, vh = linalg.svd(cov)
            Ri = vh.T @ u.T
            ti =  q2 - (Ri @ p2)

            R = Ri @ R
            t = (Ri @ t) + ti


            P = (Ri @ P.T).T + ti
        
        P_final = (R @ (scans[i]).T).T + t
        ICP = np.vstack((ICP, P_final))

        for j in range (i+2, len(scans)-1, 2):
            scans[j] = (R @ (scans[j]).T).T + t

    return ICP

scans1 = scans.copy()
result = night(scans1)


#%% Map output

fig = plt.figure(1)
ax = fig.add_subplot(111)
x_plt = result[:, 0]
y_plt = result[:, 1] 
ax.scatter(x_plt, y_plt, s=1)
plt.grid(True)
plt.show()
