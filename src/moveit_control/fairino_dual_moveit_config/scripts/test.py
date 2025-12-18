# Python示例：旋转矩阵转RPY
import numpy as np
from scipy.spatial.transform import Rotation as R

# 您的旋转矩阵 (3x3)
rot_matrix = np.array([
[4.03197354e-03, 2.36963431e-03, 9.99989064e-01, 0.143270888],

[ 2.88564975e-02,  9.99580476e-01, -2.48501600e-03, -0.122350629],

[-9.99575433e-01,  2.88662015e-02,  3.96190268e-03, -0.107061451],

[0., 0., 0., 1.]
]) # small arm

Rot = rot_matrix[:3,:3]
rotation = R.from_matrix(np.linalg.inv(Rot))
rpy = rotation.as_euler('xyz', degrees=False)
print(f"rpy: {rpy}",-np.linalg.inv(Rot)@rot_matrix[:3,3]) 

rot_matrix = np.array([
[-9.55803915e-02,  9.89934543e-01,  1.04374277e-01, -0.261118910],

[ -0.03916202,  -0.10851276,  0.99332337, -0.01493063542],

[ 9.94651057e-01,  9.08547288e-02,  4.91395305e-02, -0.168859204],

[0., 0., 0., 1.]
]) # big arm

# 转换为RPY
Rot = rot_matrix[:3,:3]
rotation = R.from_matrix(np.linalg.inv(Rot))
rpy = rotation.as_euler('xyz', degrees=False)
print(f"rpy: {rpy}",-np.linalg.inv(Rot)@rot_matrix[:3,3]) 


rot_matrix = np.array([
[1.06923335e-01, 9.92820419e-01, 5.36191704e-02, 0.195328810],

[ 9.86374228e-01, -1.12701865e-01,  1.19850622e-01,  0.154248881],

[ 1.25033125e-01,  4.00737396e-02, -9.91342934e-01, -0.241643032],

[0., 0., 0., 1.]
]) # big arm

# 转换为RPY
Rot = rot_matrix[:3,:3]
rotation = R.from_matrix(Rot)
rpy = rotation.as_euler('xyz', degrees=False)
print(f"rpy: {rpy}",rot_matrix[:3,3]) 

# print(T1)
# print(T2)