import cv2
import numpy as np
from cv2 import aruco
import pyrealsense2 as rs
import time
from scipy.spatial.transform import Rotation as R
import cv2
import time
from fairino import Robot_311 as Robot


def get_euler_from_matrix(matrix, axes='sxyz'):
    """
    从齐次变换矩阵提取欧拉角
    :param matrix: 4x4 齐次变换矩阵
    :param axes: 欧拉角的旋转顺序（默认为'sxyz'，表示按照XYZ顺序旋转）
    :return: (rx, ry, rz) 对应的欧拉角
    """
    r = R.from_matrix(matrix)
    euler_angles = r.as_euler('xyz', degrees=True)
    return euler_angles


def arucodetection():

    camera_matrix = np.array([[654.5703125, 0.0, 646.9695434570312],  # fx, cx
                    [0.0, 654.5703125, 369.22967529296875],  # fy, cy
                    [0.0, 0.0, 1.0]], dtype=np.float32)
    dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # 定义ArUco标定板参数
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    aruco_params = cv2.aruco.DetectorParameters_create()
    marker_size = 0.06

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
    # device = pipeline.start(config).get_device()
    # sensor = device.query_sensors()[1]

    # # 关闭自动曝光
    # sensor.set_option(rs.option.enable_auto_exposure, 0)

    # sensor.set_option(rs.option.exposure, 600)  # 设置曝光时间（可以尝试不同值来调整亮度）
    # sensor.set_option(rs.option.gain, 64)

    coordinate = []
    matrix = []
    k = 0
    while True:
        time.sleep(0.5)

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)

        # 检测ArUco标定板角点
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        # 如果检测到了标定板
        if ids is not None:
            k += 1
            # 绘制标定板检测结果
            cv2.aruco.drawDetectedMarkers(color_frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

            # 绘制每个检测到的标定板的坐标轴
            for i in range(len(ids)):
                # 绘制坐标轴，轴的长度为0.05米
                cv2.aruco.drawAxis(color_frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

                # 获取当前标定板的旋转向量和位移向量
                rvec = rvecs[i]
                tvec = tvecs[i]

                rotation_matrix, _ = cv2.Rodrigues(rvec)
                # 构造4x4齐次变换矩阵
                transformation_matrix = np.eye(4)  # 初始化为单位矩阵
                transformation_matrix[:3, :3] = rotation_matrix  # 设置旋转矩阵
                transformation_matrix[:3, 3] = tvec.flatten() * 1000  # 设置平移向量

                coordinate.append(tvec.flatten())
                matrix.append(transformation_matrix)
                # print(f"ArUco ID: {ids[i]}")
                # print(f"坐标: {tvec}")
                # print(f"旋转向量 (Rotation Vector): {rvec}")

        cv2.imshow('Aruco Detection', color_frame)
        # 按下 'esc' 键退出循环
        if cv2.waitKey(1) & 0xFF == 27:
            break
        if k == 10:
            break
    position = np.mean(coordinate, axis=0)
    matrix = np.mean(matrix, axis=0)
    print("--------------------------------")
    print(f"坐标：{position * 1000}")

    pipeline.stop()
    cv2.destroyAllWindows()
    return position * 1000, matrix


if __name__ == '__main__':
    
    # robot = Robot.RPC("192.168.58.2")
    # # get current pose, transform it and move robot to new pose
    # ret = robot.GetActualTCPPose()
    # pose = ret[1]
    # hand = str(pose[0] / 1000) + ", " + str(pose[1] / 1000) + ", " + str(pose[2] / 1000) + ", " + str(
    #     pose[3]) + ", " + str(pose[4]) + ", " + str(pose[5]) + ", "
    # print("hand: " + hand + "\n")

    e_p, matrix = arucodetection()
    print(matrix)
    euler_angles = get_euler_from_matrix(matrix=matrix[:3, :3])
    x = matrix[0, 3] / 1000
    y = matrix[1, 3] / 1000
    z = matrix[2, 3] / 1000
    rx = euler_angles[0]
    ry = euler_angles[1]
    rz = euler_angles[2]
    camera = str(x) + ", " + str(y) + ", " + str(z) + ", " + str(rx) + ", " + str(ry) + ", " + str(rz) + ", "
    print("camera: " + camera + "\n")

    # with open("/home/wql/kyk/camera_calibrate/0704_hand.txt", "a") as file:
    #     file.write(hand + "\n")
    #
    # with open("/home/wql/kyk/camera_calibrate/0704_camera.txt", "a") as file:
    #     file.write(camera + "\n")
    p_A = np.hstack([e_p / 1000, 1])

    T_cw = np.array([
        [-0.25990729, -0.9310292,  0.25618905, -1.21273251],
        [-0.96510642, 0.25922178, -0.03706305, -0.31406583],
        [-0.031903,   -0.25688265, -0.96591589, 0.86762693],
        [0.0,0.0,0.0,1.0]])

    print(p_A)
    print(T_cw.dot(p_A))

