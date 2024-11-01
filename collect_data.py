# coding=utf-8

import logging,os

import yaml
import numpy as np
import cv2
import pyrealsense2 as rs

from libs.log_setting import CommonLog
from libs.auxiliary import create_folder_with_date, get_ip, popup_message

from robotic_arm_package.robotic_arm import *


cam0_origin_path = create_folder_with_date() # 提前建立好的存储照片文件的目录


logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)

def callback(frame):

    scaling_factor = 2.0
    global count

    cv_img = cv2.resize(frame, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
    cv2.imshow("Capture_Video", cv_img)  # 窗口显示，显示名为 Capture_Video

    k = cv2.waitKey(30) & 0xFF  # 每帧数据延时 1ms，延时不能为 0，否则读取的结果会是静态帧

    if k == ord('s'):  # 若检测到按键 ‘s’，打印字符串

        error_code, joints, curr_pose, arm_err_ptr, sys_err_ptr = robot.Get_Current_Arm_State()  # 获取当前机械臂状态

        logger_.info(f"ret:{error_code}")

        state,pose = error_code,curr_pose
        logger_.info(f'获取状态：{"成功" if state == 0 else "失败"}，{f"当前位姿为{pose}" if state == 0 else None}')
        if state == 0:

            filename = os.path.join(cam0_origin_path,"poses.txt")

            with open(filename, 'a+') as f:
                # 将列表中的元素用空格连接成一行
                pose_ = [str(i) for i in pose]
                new_line = f'{",".join(pose_)}\n'
                # 将新行附加到文件的末尾
                f.write(new_line)

            image_path = os.path.join(cam0_origin_path,f"{str(count)}.jpg")
            cv2.imwrite(image_path , cv_img)
            logger_.info(f"===采集第{count}次数据！")

        count += 1

    else:
        pass

def displayD435():

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
    except Exception as e:
        logger_.error_(f"相机连接异常：{e}")
        popup_message("提醒", "相机连接异常")

        sys.exit(1)

    global count
    count = 1

    logger_.info(f"开始手眼标定程序，当前程序版号V1.0.0")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            callback(color_image)

    finally:

        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':

    robot_ip = get_ip()


    logger_.info(f'robot_ip:{robot_ip}')

    if robot_ip:

        with open("config.yaml", 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)

        ROBOT_TYPE = data.get("ROBOT_TYPE")

        robot = Arm(ROBOT_TYPE, robot_ip)

        robot.Change_Work_Frame()

        # API版本信息
        print(robot.API_Version())

    else:

        popup_message("提醒", "机械臂ip没有ping通")
        sys.exit(1)

    displayD435()
