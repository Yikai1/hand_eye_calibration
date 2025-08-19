import numpy as np
from scipy.spatial.transform import Rotation as R

def matrix_to_pose(T_pose2base: np.ndarray):
    """
    将 pose2base 的 4x4 齐次矩阵转换为 base2pose 的 [x, y, z, rx, ry, rz] 欧拉角形式（单位为度）
    
    参数:
        T_pose2base: 4x4 numpy 矩阵，表示 pose 在 base 坐标系下的变换（即 pose2base）

    返回:
        [x, y, z, rx, ry, rz]，其中位置单位为米，角度为度
    """
    assert T_pose2base.shape == (4, 4), "输入矩阵必须是 4x4 齐次变换矩阵"

    # 取逆，得到 base2pose
    T_base2pose = np.linalg.inv(T_pose2base)

    # 提取平移向量
    x, y, z = T_base2pose[:3, 3]

    # 提取旋转矩阵并转为欧拉角（xyz顺序）
    R_mat = T_base2pose[:3, :3]
    r = R.from_matrix(R_mat)
    rx, ry, rz = r.as_euler('xyz', degrees=True)

    return [x, y, z, rx, ry, rz]


print(matrix_to_pose(np.array([[0.692360897603158,-0.11358701783385752,-0.7125548237502572,0.9835989014743565],
[-0.21963953869317018,-0.9738449623935682,-0.05817613138840443,-0.21767193591355433],
[-0.6873098722649054,0.19678409132925465,-0.6992003724876861,0.03236249977739507],
[0.0,0.0,0.0,1.0]])))
