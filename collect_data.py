# coding=utf-8
import json
import logging,os
import socket
import time
import sys
import numpy as np
import cv2
import pyrealsense2 as rs
import math
from libs.log_setting import CommonLog
from libs.auxiliary import create_folder_with_date, get_ip, popup_message
from fairino import Robot
cam0_origin_path = create_folder_with_date() # 提前建立好的存储照片文件的目录
# cam0_origin_path = '/home/wql/kyk/hand_eye_calibration/eye_hand_data/data2025071806'

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)

def callback(frame):

    scaling_factor = 1.0
    global count

    cv_img = cv2.resize(frame, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
    cv2.imshow("Capture_Video", cv_img)  # 窗口显示，显示名为 Capture_Video

    k = cv2.waitKey(30) & 0xFF  # 每帧数据延时 1ms，延时不能为 0，否则读取的结果会是静态帧

    if k == ord('s'):  # 若检测到按键 ‘s’，打印字符串

        # socket_command = '{"command": "get_current_arm_state"}'
        # state,pose = send_cmd(client,socket_command)
        ret = robot.GetActualTCPPose()
        pose = ret[1]
        pose_converted = [
            pose[0] / 1000,  # x: mm → m
            pose[1] / 1000,  # y: mm → m
            pose[2] / 1000,  # z: mm → m
            math.radians(pose[3]),   # rx: deg → rad
            math.radians(pose[4]),   # ry: deg → rad
            math.radians(pose[5])    # rz: deg → rad
        ]
        logger_.info({f"当前位姿为{pose_converted}"})
        if pose:

            filename = os.path.join(cam0_origin_path,"poses.txt")

            with open(filename, 'a+') as f:
                # 将列表中的元素用空格连接成一行
                pose_ = [str(i) for i in pose_converted]
                new_line = f'{",".join(pose_)}\n'
                # 将新行附加到文件的末尾
                f.write(new_line)

            image_path = os.path.join(cam0_origin_path,f"{str(count)}.jpg")
            cv2.imwrite(image_path , cv_img)
            logger_.info(f"===采集第{count}次数据！")

        count += 1

    else:
        pass


def send_cmd(client, cmd, get_pose=True):
    """
    发送命令到机械臂并可选择性地获取姿态(pose)数据

    参数:
    client: socket客户端连接
    cmd: 要发送的命令字符串或JSON字符串
    get_pose: 是否需要获取pose数据

    返回:
    如果get_pose为True，返回tuple (状态, pose或错误信息)
    如果get_pose为False，返回布尔值表示命令是否成功发送
    """
    client.send(cmd.encode('utf-8'))

    if not get_pose:
        response = client.recv(1024).decode('utf-8')
        logger_.info(f"response:{response}")
        return True

    time.sleep(0.1)
    response = client.recv(4096).decode('utf-8')  # 增大接收缓冲区
    logger_.info(f'response:{response}')

    try:
        decoder = json.JSONDecoder()
        data_list = []
        index = 0
        # 分割并解析所有可能的JSON对象
        while index < len(response):
            try:
                # 跳过空白字符
                while index < len(response) and response[index].isspace():
                    index += 1
                if index >= len(response):
                    break
                obj, idx = decoder.raw_decode(response[index:])
                data_list.append(obj)
                index += idx
            except json.JSONDecodeError as e:
                logger_.error(f"JSON解析错误：{str(e)}")
                break

        # 寻找最后一个包含目标状态的响应
        target_data = None
        for data in reversed(data_list):
            if data.get("state") == "current_arm_state":
                target_data = data
                break

        if not target_data:
            return False, "未找到有效的机械臂状态响应"

        # 检查错误码
        if target_data["arm_state"]["err"] != [0]:
            return False, f"机械臂报错: {target_data['arm_state']['err']}"

        # 转换单位
        pose_raw = target_data["arm_state"]["pose"]
        pose_converted = [
            pose_raw[0] / 1000000,  # x: 0.001mm → m
            pose_raw[1] / 1000000,  # y: 0.001mm → m
            pose_raw[2] / 1000000,  # z: 0.001mm → m
            pose_raw[3] / 1000,    # rx: 0.001rad → rad
            pose_raw[4] / 1000,    # ry: 0.001rad → rad
            pose_raw[5] / 1000     # rz: 0.001rad → rad
        ]

        return True, pose_converted

    except json.JSONDecodeError:
        return False, "JSON解析错误"
    except KeyError as e:
        return False, f"响应缺少关键字段: {str(e)}"
    except Exception as e:
        return False, f"处理响应时发生错误: {str(e)}"
#
def displayD435():

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

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

    # robot_ip = get_ip()



    # logger_.info(f'robot_ip:{robot_ip}')

    # if robot_ip:

    #     client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     client.connect((robot_ip, 8080))
    #     socket_command = '{"command":"set_change_work_frame","frame_name":"Base"}'
    #     send_cmd(client,socket_command,get_pose = False
    # else:

    #     popup_message("提醒", "机械臂ip没有ping通")
    #     sys.exit(1)

    robot = Robot.RPC("192.168.1.200")
    # get current pose, transform it and move robot to new pose
    ret = robot.GetActualTCPPose()
    pose = ret[1]
    print(pose)
    displayD435()
