import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

# === 1. 棋盘格参数 ===
CHECKERBOARD = (5, 5)  # 内角点数量
SQUARE_SIZE = 0.03  # 单位：米

# 相机内参（根据你自己的标定修改！）
K = np.array([[600, 0, 320],
              [0, 600, 240],
              [0,   0,   1]], dtype=np.float64)
D = np.zeros(5)  # 无畸变参数，或者用实际的畸变系数

# === 2. 构造棋盘格世界坐标 ===
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# === 3. 读取 poses.txt ===
def load_poses(pose_file):
    poses = []
    with open(pose_file, 'r') as f:
        for line in f:
            if line.strip() == "":
                continue
            vals = list(map(float, line.strip().split(',')))
            t = np.array(vals[:3])
            rpy = np.array(vals[3:])
            R_mat = R.from_euler('xyz', rpy).as_matrix()
            poses.append((R_mat, t))
    return poses

# === 4. 遍历图像和位姿，构建输入数据 ===
def extract_data(image_dir, pose_file):
    poses = load_poses(pose_file)
    image_files = sorted([f for f in os.listdir(image_dir) if f.endswith('.jpg')],
                         key=lambda x: int(os.path.splitext(x)[0]))

    R_gripper2base_list, t_gripper2base_list = [], []
    R_target2cam_list, t_target2cam_list = [], []

    for i, img_file in enumerate(image_files):
        if i >= len(poses):
            print(f"图像 {img_file} 超出 poses 范围，跳过")
            continue

        img_path = os.path.join(image_dir, img_file)
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        # if ret:
        #     # 显示角点检测结果
        #     vis_img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        #     cv2.imshow(f"" + str(i), vis_img)
        #     cv2.waitKey(1000)  # 300 毫秒后自动下一张

        if not ret:
            print(f"图像 {img_file} 未检测到角点，跳过")
            continue

        # 提高角点精度
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        # solvePnP 估计相机坐标系下的棋盘位姿（target2cam）
        ret, rvec, tvec = cv2.solvePnP(objp, corners2, K, D)
        R_target2cam, _ = cv2.Rodrigues(rvec)
        t_target2cam = tvec.flatten()

        # # ✅ 正确方式：求逆
        # R_target2cam_raw, _ = cv2.Rodrigues(rvec)
        # t_target2cam_raw = tvec.reshape(3, 1)
        #
        # R_cam2target = R_target2cam_raw.T
        # t_cam2target = -R_cam2target @ t_target2cam_raw
        #
        # R_target2cam_list.append(R_cam2target)
        # t_target2cam_list.append(t_cam2target.flatten())

        # base2gripper → 求逆 → gripper2base
        R_bg, t_bg = poses[i]
        R_gb = R_bg.T
        t_gb = -R_gb @ t_bg

        # 收集数据
        R_gripper2base_list.append(R_gb)
        t_gripper2base_list.append(t_gb)
        R_target2cam_list.append(R_target2cam)
        t_target2cam_list.append(t_target2cam)

    return R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list

# === 5. 标定主函数 ===
def calibrate_eye_to_hand():
    image_dir = '.'  # 当前目录
    pose_file = 'poses.txt'

    print("开始提取数据...")
    Rg2b, tg2b, Rt2c, tt2c = extract_data(image_dir, pose_file)
    print(f"有效数据对数量: {len(Rg2b)}")

    if len(Rg2b) < 5:
        print("样本太少，无法进行手眼标定！")
        return

    print("开始手眼标定...")
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        Rg2b, tg2b,
        Rt2c, tt2c,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    T_cam2base = np.eye(4)
    T_cam2base[:3, :3] = R_cam2base
    T_cam2base[:3, 3] = t_cam2base.flatten()

    print("\n=== 相机到机械臂基座的变换矩阵 T_cam2base ===")
    print(T_cam2base)

    # 可选：保存为文件
    np.savetxt("T_cam2base.txt", T_cam2base, fmt="%.6f")
    # 示例 T_cam2base，请替换为你标定输出的矩阵
    visualize_camera_and_base(T_cam2base)


import open3d as o3d
import numpy as np


def create_frame(T=np.eye(4), size=0.1):
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    frame.transform(T)
    return frame


def visualize_camera_and_base(T_cam2base: np.ndarray):
    """
    使用Open3D可视化base坐标系和camera坐标系。
    :param T_cam2base: shape=(4,4)，表示从相机到base的变换矩阵
    """
    frame_base = create_frame(np.eye(4), size=0.1)
    frame_camera = create_frame(T_cam2base, size=0.07)

    print("变换矩阵 T_cam2base:\n", T_cam2base)
    o3d.visualization.draw_geometries([frame_base, frame_camera],
                                      zoom=0.6,
                                      front=[0, 0, -1],
                                      lookat=[0, 0, 0],
                                      up=[0, -1, 0])



if __name__ == "__main__":
    calibrate_eye_to_hand()
