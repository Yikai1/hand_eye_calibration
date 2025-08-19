# coding=utf-8

"""
眼在手外 用采集到的图片信息和机械臂位姿信息计算 相机坐标系相对于机械臂基坐标系的 旋转矩阵和平移向量

"""

import os
import logging

import yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from libs.auxiliary import find_latest_data_folder
from libs.log_setting import CommonLog
import open3d as o3d
from save_poses2 import poses2_main

np.set_printoptions(precision=8,suppress=True)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


current_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"eye_hand_data")

# images_path = os.path.join("eye_hand_data",find_latest_data_folder(current_path))
images_path = os.path.join("./eye_hand_data",'data2025080101')
file_path = os.path.join(images_path,"poses.txt")  #采集标定板图片时对应的机械臂末端的位姿 从 第一行到最后一行 需要和采集的标定板的图片顺序进行对应


with open("config.yaml", 'r', encoding='utf-8') as file:
    data = yaml.safe_load(file)

XX = data.get("checkerboard_args").get("XX") #标定板的中长度对应的角点的个数
YY = data.get("checkerboard_args").get("YY") #标定板的中宽度对应的角点的个数
L = data.get("checkerboard_args").get("L")   #标定板一格的长度  单位为米

# 手动设置内参矩阵和畸变系数
def get_camera_intrinsics():
    """
    返回手动设置的相机内参
    请根据您的相机实际参数进行修改
    """
    # 654.5703125, 654.5703125, 646.9695434570312, 369.22967529296875
    # 内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    mtx = np.array([[654.5703125, 0.0, 646.9695434570312],     # fx, cx
                    [0.0, 654.5703125, 369.22967529296875],     # fy, cy  
                    [0.0, 0.0, 1.0]], dtype=np.float32)
    
    # 畸变系数 [k1, k2, p1, p2, k3]
    dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    
    return mtx, dist

def func():

    path = os.path.dirname(__file__)

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 100, 0.0001)  # 提高精度

    # 获取标定板角点的位置
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
    objp = L * objp

    obj_points = []
    img_points = []

    # 对图片文件名进行排序，确保顺序一致
    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]
    images_num.sort(key=lambda x: int(x.split('.')[0]))

    valid_images = 0  # 记录有效图片数量

    for i in range(1, len(images_num) + 1):
        image_file = os.path.join(images_path, f"{i}.jpg")

        if os.path.exists(image_file):
            logger_.info(f'读 {image_file}')

            img = cv2.imread(image_file)
            if img is None:
                logger_.info(f'无法读取图片: {image_file}')
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            size = gray.shape[::-1]

            # 使用更严格的角点检测标志
            flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + 
                    cv2.CALIB_CB_NORMALIZE_IMAGE + 
                    cv2.CALIB_CB_FILTER_QUADS)
            
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), flags)

            if ret:
                # 使用更大的亚像素搜索窗口
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # 验证角点质量
                if corners2 is not None and len(corners2) == XX * YY:
                    # 计算角点的重投影误差来评估质量
                    corners_reshaped = corners2.reshape(-1, 2)
                    
                    # 简单的角点质量检查：检查是否有异常值
                    mean_x = np.mean(corners_reshaped[:, 0])
                    mean_y = np.mean(corners_reshaped[:, 1])
                    std_x = np.std(corners_reshaped[:, 0])
                    std_y = np.std(corners_reshaped[:, 1])
                    
                    # 排除离群点过多的图片
                    outliers = np.sum((np.abs(corners_reshaped[:, 0] - mean_x) > 3 * std_x) |
                                    (np.abs(corners_reshaped[:, 1] - mean_y) > 3 * std_y))
                    
                    if outliers <= XX * YY * 0.1:  # 允许最多10%的离群点
                        obj_points.append(objp)
                        img_points.append(corners2)
                        valid_images += 1
                        
                        # 绘制并显示角点
                        cv2.drawChessboardCorners(img, (XX, YY), corners2, ret)
                        cv2.imshow('img', img)
                        cv2.waitKey(500)  # 减少等待时间
                        
                        logger_.info(f'图片 {i} 角点检测成功')
                    else:
                        logger_.info(f'图片 {i} 角点质量不佳，跳过')
                else:
                    logger_.info(f'图片 {i} 亚像素角点优化失败')
            else:
                logger_.info(f'图片 {i} 未检测到角点')

    cv2.destroyAllWindows()

    if valid_images < 10:
        logger_.info(f'有效图片数量过少: {valid_images}，建议至少10张')
        return None, None

    logger_.info(f'总共使用 {valid_images} 张有效图片进行标定')

    # 使用手动设置的内参
    mtx, dist = get_camera_intrinsics()
    logger_.info(f"使用手动设置的内参矩阵:\n{mtx}")
    logger_.info(f"使用手动设置的畸变系数:\n{dist}")

    # 使用已知内参计算每张图片的位姿
    rvecs = []
    tvecs = []
    for i in range(len(obj_points)):
        ret, rvec, tvec = cv2.solvePnP(obj_points[i], img_points[i], mtx, dist)
        if ret:
            rvecs.append(rvec)
            tvecs.append(tvec)
        else:
            logger_.info(f"第{i+1}张图片位姿求解失败")

    # 计算重投影误差
    total_error = 0
    for i in range(len(obj_points)):
        if i < len(rvecs):
            img_points_proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(img_points[i], img_points_proj, cv2.NORM_L2) / len(img_points_proj)
            total_error += error

    mean_error = total_error / len(rvecs) if len(rvecs) > 0 else float('inf')
    logger_.info(f"使用手动内参的平均重投影误差: {mean_error}")

    print("-----------------------------------------------------")

    # 转换旋转向量为旋转矩阵
    R_cam = []
    t_cam = []
    for i in range(len(rvecs)):
        R_matrix, _ = cv2.Rodrigues(rvecs[i])
        R_cam.append(R_matrix)
        t_cam.append(tvecs[i].reshape(3, 1))

    poses2_main(file_path)
    
    # 机器人末端在基座标系下的位姿
    csv_file = os.path.join(path, "RobotToolPose.csv")
    tool_pose = np.loadtxt(csv_file, delimiter=',')

    R_tool = []
    t_tool = []

    # 确保只使用有效图片对应的机器人位姿
    for i in range(len(R_cam)):  # 使用实际计算出位姿的图片数量
        R_tool.append(tool_pose[0:3, 4*i:4*i+3])
        t_tool.append(tool_pose[0:3, 4*i+3])

    # 尝试不同的手眼标定算法
    methods = [
        cv2.CALIB_HAND_EYE_TSAI,
        cv2.CALIB_HAND_EYE_PARK,
        cv2.CALIB_HAND_EYE_HORAUD,
        cv2.CALIB_HAND_EYE_ANDREFF,
        cv2.CALIB_HAND_EYE_DANIILIDIS
    ]
    
    method_names = ['TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS']
    
    best_R = None
    best_t = None
    best_error = float('inf')
    best_method = ""
    
    for method, name in zip(methods, method_names):
        try:
            R, t = cv2.calibrateHandEye(R_tool, t_tool, R_cam, t_cam, method)
            
            # 更好的评估标准：计算所有位姿的一致性
            error = 0
            for i in range(len(R_cam)):
                # 计算 T_base_cam = T_base_tool * T_tool_cam
                T_tool_cam = np.eye(4)
                T_tool_cam[:3, :3] = R
                T_tool_cam[:3, 3] = t.flatten()
                
                T_base_tool = np.eye(4)
                T_base_tool[:3, :3] = R_tool[i]
                T_base_tool[:3, 3] = t_tool[i].flatten()
                
                T_base_cam_calc = T_base_tool @ T_tool_cam
                
                # 与第一次计算的结果比较
                if i == 0:
                    T_base_cam_ref = T_base_cam_calc
                else:
                    # 计算旋转和平移的差异
                    R_diff = np.linalg.norm(T_base_cam_ref[:3, :3] - T_base_cam_calc[:3, :3])
                    t_diff = np.linalg.norm(T_base_cam_ref[:3, 3] - T_base_cam_calc[:3, 3])
                    error += R_diff + t_diff
            
            logger_.info(f"{name} 方法结果 - 一致性误差: {error:.6f}")
            
            if error < best_error:
                best_error = error
                best_R = R
                best_t = t
                best_method = name
                
        except Exception as e:
            logger_.info(f"{name} 方法失败: {e}")
    
    if best_R is not None:
        logger_.info(f"选择最佳结果: {best_method}，一致性误差: {best_error:.6f}")
        return best_R, best_t
    else:
        logger_.info("所有手眼标定方法都失败，返回第一个相机位姿")
        return R_cam[0], t_cam[0]  # 返回第一个相机位姿作为备选

if __name__ == '__main__':

    # 旋转矩阵
    rotation_matrix, translation_vector = func()

    # 将旋转矩阵转换为四元数
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()
    euler = rotation.as_euler('XYZ',degrees=True)
    x, y, z = translation_vector.flatten()

    logger_.info(f"旋转矩阵是:\n {            rotation_matrix}")

    logger_.info(f"平移向量是:\n {            translation_vector}")

    logger_.info(f"四元数是：\n {             quaternion}")

    logger_.info(f'欧拉角是：\n{euler}')

    import matplotlib.pyplot as plt

    def draw_coordinate(ax, origin, R, label, length=0.05):
        """
        在ax中绘制一个以 origin 为起点的坐标系，方向由旋转矩阵 R 给出
        """
        # 三个方向单位向量
        x_axis = R @ np.array([1, 0, 0])
        y_axis = R @ np.array([0, 1, 0])
        z_axis = R @ np.array([0, 0, 1])

        ax.quiver(*origin, *x_axis, color='r', length=length, normalize=True)
        ax.quiver(*origin, *y_axis, color='g', length=length, normalize=True)
        ax.quiver(*origin, *z_axis, color='b', length=length, normalize=True)

        ax.text(*origin, label, fontsize=10, color='k')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1,1,1])

    # 绘制原始坐标系（通常设为单位阵）
    draw_coordinate(ax, np.zeros(3), np.eye(3), 'Original')

    # 绘制变换后的坐标系
    draw_coordinate(ax, translation_vector.flatten(), rotation_matrix, 'Transformed')

    # 设置范围
    ax.set_xlim([-1.0, 1.0])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([-1.0, 1.0])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('Coordinate Frame Visualization')
    plt.show()
