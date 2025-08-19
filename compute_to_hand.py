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
images_path = os.path.join("./eye_hand_data",'data2025072201')
file_path = os.path.join(images_path,"poses.txt")  #采集标定板图片时对应的机械臂末端的位姿 从 第一行到最后一行 需要和采集的标定板的图片顺序进行对应


with open("config.yaml", 'r', encoding='utf-8') as file:
    data = yaml.safe_load(file)

XX = data.get("checkerboard_args").get("XX") #标定板的中长度对应的角点的个数
YY = data.get("checkerboard_args").get("YY") #标定板的中宽度对应的角点的个数
L = data.get("checkerboard_args").get("L")   #标定板一格的长度  单位为米

def func():

    path = os.path.dirname(__file__)
    # print(path)

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 获取标定板角点的位置
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp = L*objp

    obj_points = []     # 存储3D点
    img_points = []     # 存储2D点

    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]

    for i in range(1, len(images_num) + 1):   #标定好的图片在images_path路径下，从0.jpg到x.jpg

        image_file = os.path.join(images_path,f"{i}.jpg")

        if os.path.exists(image_file):

            logger_.info(f'读 {image_file}')

            img = cv2.imread(image_file)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

            if ret:

                # 绘制并显示角点
                cv2.drawChessboardCorners(img, (XX, YY), corners, ret)
                cv2.imshow('img', img)
                cv2.waitKey(1000)
                obj_points.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
                if [corners2]:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)
    cv2.destroyAllWindows()

    N = len(img_points)

    # 标定,得到图案在相机坐标系下的位姿
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

    logger_.info(f"内参矩阵:\n:{mtx}" ) # 内参数矩阵
    logger_.info(f"畸变系数:\n:{dist}")  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)

    print("-----------------------------------------------------")
    
    # board_posse = np.loadtxt('./extrinsics.csv',delimiter=',')
    # import csv
    # import numpy as np

    # # 存储rvecs和tvecs
    # rvecs = []
    # tvecs = []

    # # 读取所有的图像文件名
    # image_filenames = [
    #     '1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg', '6.jpg', '7.jpg', '8.jpg', '9.jpg', '10.jpg',
    #     '11.jpg', '12.jpg', '13.jpg', '14.jpg', '15.jpg', '16.jpg', '17.jpg', '18.jpg', '19.jpg'
    # ]

    # # 打开CSV文件
    # with open('/home/wql/kyk/hand_eye_calibration/extrinsics.csv', 'r') as file:
    #     reader = csv.reader(file)
    #     # 跳过头部
    #     next(reader)

    #     # 将每行数据存储到字典中，以图像名称为键
    #     data_dict = {}
    #     for row in reader:
    #         filename = row[0]
    #         rvec = np.array([float(row[1]), float(row[2]), float(row[3])])
    #         tvec = np.array([float(row[4]), float(row[5]), float(row[6])]) / 1000  # 除以1000
    #         data_dict[filename] = (rvec, tvec)

    # # # 按图像顺序读取并提取rvecs和tvecs
    # # for filename in sorted(image_filenames, key=lambda x: int(x.split('.')[0])):
    # #     rvec, tvec = data_dict[filename]
    # #     rvecs.append(rvec)
    # #     tvecs.append(tvec)

    # # 转换为NumPy数组
    # rvecs = np.array(rvecs)
    # tvecs = np.array(tvecs)
    poses2_main(file_path)
    # 机器人末端在基座标系下的位姿

    csv_file = os.path.join(path,"RobotToolPose.csv")
    tool_pose = np.loadtxt(csv_file,delimiter=',')

    R_tool = []
    t_tool = []

    for i in range(int(N)):

        R_tool.append(tool_pose[0:3,4*i:4*i+3])
        t_tool.append(tool_pose[0:3,4*i+3])

    R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_DANIILIDIS)
    """One of [CALIB_HAND_EYE_TSAI, CALIB_HAND_EYE_PARK, CALIB_HAND_EYE_HORAUD, CALIB_HAND_EYE_ANDREFF, CALIB_HAND_EYE_DANIILIDIS]"""
    return R,t

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
