import numpy as np
import cv2
import glob
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
from mpl_toolkits.mplot3d import Axes3D

# 棋盘格参数
checkerboard_size = (5, 5)  # (XX, YY)
square_size = 0.03  # 每格边长，单位：米

# 相机内参
fx = 0.6545703125
fy = 0.6545703125
cx = 0.6469695434570312
cy = 0.36922967529296875
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# 读取poses.txt
def load_poses(file_path):
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split(',')))
            poses.append(values)
    return np.unique(np.array(poses), axis=0)

# 位姿转变换矩阵
def pose_to_matrix(pose):
    t = pose[:3]  # 平移向量 (x, y, z)
    r = Rotation.from_euler('xyz', pose[3:], degrees=False).as_matrix()  # 欧拉角转旋转矩阵
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = t
    return T

# 加载位姿
poses = load_poses('poses.txt')
gripper_to_base = [pose_to_matrix(pose) for pose in poses]

# 棋盘格3D点
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2) * square_size

# 提取相机到棋盘格的位姿
cam_to_board = []
image_files = sorted(glob.glob('*.jpg'))
for img_path in image_files:
    img = cv2.imread(img_path)
    if img is None:
        print(f"Failed to load image {img_path}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
    if ret:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        ret, rvec, tvec = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)
        if ret:
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()
            cam_to_board.append(T)
        else:
            print(f"Failed to solvePnP for {img_path}")
    else:
        print(f"Chessboard not found in {img_path}")

# 确保数量匹配
if len(cam_to_board) != len(gripper_to_base):
    print(f"警告：位姿数量 ({len(gripper_to_base)}) 与有效图像数量 ({len(cam_to_board)}) 不匹配！")
    min_len = min(len(cam_to_board), len(gripper_to_base))
    cam_to_board = cam_to_board[:min_len]
    gripper_to_base = gripper_to_base[:min_len]

# 准备手眼标定输入
R_gripper_to_base = [T[:3, :3] for T in gripper_to_base]
t_gripper_to_base = [T[:3, 3] for T in gripper_to_base]
R_cam_to_board = [T[:3, :3] for T in cam_to_board]
t_cam_to_board = [T[:3, 3] for T in cam_to_board]

# 手眼标定
R_cam_to_base, t_cam_to_base = cv2.calibrateHandEye(
    R_gripper_to_base, t_gripper_to_base,
    R_cam_to_board, t_cam_to_board,
    method=cv2.CALIB_HAND_EYE_TSAI
)

# 构建变换矩阵
T_cam_to_base = np.eye(4)
T_cam_to_base[:3, :3] = R_cam_to_base
T_cam_to_base[:3, 3] = t_cam_to_base.flatten()

# 输出结果
print("相机到机械臂基座的变换矩阵 T_cam_to_base:")
print(T_cam_to_base)

# 验证旋转角度
euler_angles = Rotation.from_matrix(R_cam_to_base).as_euler('xyz', degrees=True)
print("相机到基座的旋转（欧拉角，度）：", euler_angles)

# 绘制坐标轴的函数
def plot_coordinate_frame(ax, T, label, colors, length=0.1):
    """
    绘制坐标系
    T: 4x4 变换矩阵
    label: 坐标系名称
    colors: 列表，包含 X, Y, Z 轴和原点的颜色，例如 ['r', 'g', 'b', 'k']
    length: 坐标轴长度（单位：米）
    """
    origin = T[:3, 3]
    R = T[:3, :3]
    axes = np.eye(3) * length
    axes_transformed = np.dot(R, axes)
    ax.quiver(origin[0], origin[1], origin[2],
              axes_transformed[0, 0], axes_transformed[1, 0], axes_transformed[2, 0],
              color=colors[0], label=f'{label} X')
    ax.quiver(origin[0], origin[1], origin[2],
              axes_transformed[0, 1], axes_transformed[1, 1], axes_transformed[2, 1],
              color=colors[1], label=f'{label} Y')
    ax.quiver(origin[0], origin[1], origin[2],
              axes_transformed[0, 2], axes_transformed[1, 2], axes_transformed[2, 2],
              color=colors[2], label=f'{label} Z')
    ax.scatter([origin[0]], [origin[1]], [origin[2]], color=colors[3], s=50, label=f'{label} Origin')

# 设置 3D 绘图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 基坐标系（红色 X，绿色 Y，蓝色 Z，黑色原点）
T_base = np.eye(4)
plot_coordinate_frame(ax, T_base, 'Base', colors=['red', 'green', 'blue', 'black'], length=0.1)

# 相机坐标系（紫色 X，橙色 Y，青色 Z，灰色原点）
plot_coordinate_frame(ax, T_cam_to_base, 'Camera', colors=['purple', 'orange', 'cyan', 'gray'], length=0.1)

# 设置图形属性
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Base and Camera Coordinate Frames')

# 设置视角
ax.view_init(elev=20, azim=45)

# 设置坐标轴范围（根据 T_cam_to_base 的平移部分调整）
t = T_cam_to_base[:3, 3]
ax.set_xlim([min(-0.2, t[0] - 0.2), max(0.2, t[0] + 0.2)])
ax.set_ylim([min(-0.2, t[1] - 0.2), max(0.2, t[1] + 0.2)])
ax.set_zlim([min(-0.2, t[2] - 0.2), max(0.2, t[2] + 0.2)])

# 显示图例
ax.legend()

# 显示图形
plt.show()