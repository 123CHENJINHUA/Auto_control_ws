import numpy as np
from scipy.spatial.transform import Rotation as R

def rotation_matrix_to_quaternion_scipy(rotation_matrix):
    """
    使用scipy将旋转矩阵转换为四元数
    
    Args:
        rotation_matrix: 3x3旋转矩阵
    
    Returns:
        list: 四元数 [w, x, y, z] 格式
    """
    # 确保输入是numpy数组
    rot_matrix = np.array(rotation_matrix)
    
    # 创建旋转对象并转换为四元数
    rotation = R.from_matrix(rot_matrix)
    
    # 注意：scipy返回的是 [x, y, z, w] 顺序
    quaternion_xyzw = rotation.as_quat()
    
    # 转换为常见的 [w, x, y, z] 顺序
    quaternion_wxyz = [
        
        quaternion_xyzw[0],  # x
        quaternion_xyzw[1],  # y
        quaternion_xyzw[2],  # z
        quaternion_xyzw[3],  # w
    ]
    
    return quaternion_wxyz

# 示例：单位旋转矩阵（无旋转）
identity_matrix = np.eye(3)
quat = rotation_matrix_to_quaternion_scipy(identity_matrix)
print(f"单位矩阵转四元数: {quat}")

# 示例：绕X轴旋转90度的旋转矩阵
import math
theta = math.radians(90)
rot_x = np.array([
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 0]
])
quat_x = rotation_matrix_to_quaternion_scipy(rot_x)
print(f"绕X轴旋转90度矩阵转四元数: {quat_x}")