import xmlrpc.client
import os
import socket
import hashlib
import time
from datetime import datetime
import logging
from functools import wraps
from logging.handlers import RotatingFileHandler
from queue import Queue
import threading
import sys


class BufferedFileHandler(RotatingFileHandler):
    def __init__(self, filename, mode='a', maxBytes=0, backupCount=0, encoding=None, delay=False):
        super().__init__(filename, mode, maxBytes, backupCount, encoding, delay)
        self.buffer = []

    def emit(self, record):
        # log_entry = self.format(record)  # 格式化日志记录
        # print(log_entry)  # 打印日志条目
        if RPC.log_output_model == 2:
            RPC.queue.put(record)
        else:
            self.buffer.append(record)
            if len(self.buffer) >= 50:
                for r in self.buffer:
                    super().emit(r)
                self.buffer = []


class LogWriterThread(threading.Thread):
    def __init__(self, queue, log_handler):
        super().__init__()
        self.queue = queue
        self.log_handler = log_handler
        self.daemon = True

    def run(self):
        while True:
            record = self.queue.get()
            if record is None:
                break
            log_entry = self.log_handler.format(record)
            self.log_handler.stream.write(log_entry + self.log_handler.terminator)
            self.log_handler.flush()


def calculate_file_md5(file_path):
    if not os.path.exists(file_path):
        raise ValueError(f"{file_path} 不存在")
    md5 = hashlib.md5()
    with open(file_path, 'rb') as file:
        while chunk := file.read(8192):  # Read in 8KB chunks
            md5.update(chunk)
    return md5.hexdigest()

def xmlrpc_timeout(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if RPC.is_conect==False:
            return -4
        else:
            result = func(self, *args, **kwargs)
            return result
    return wrapper

class RobotError:
    ERR_SUCCESS = 0
    ERR_POINTTABLE_NOTFOUND = -7  # 上传文件不存在
    ERR_SAVE_FILE_PATH_NOT_FOUND = -6  # 保存文件路径不存在
    ERR_NOT_FOUND_LUA_FILE = -5  # lua文件不存在
    ERR_RPC_ERROR = -4
    ERR_SOCKET_COM_FAILED = -2
    ERR_OTHER = -1
    
class RPC():
    ip_address = "192.168.58.2"
    logger = None
    log_output_model = -1
    queue = Queue(maxsize=10000 * 1024)
    logging_thread = None
    is_conect = True

    def __init__(self, ip="192.168.58.2"):
        self.ip_address = ip
        link = 'http://' + self.ip_address + ":20003"
        self.robot = xmlrpc.client.ServerProxy(link)
        print(self.robot)
        
        try:
            # 调用 XML-RPC 方法
            socket.setdefaulttimeout(1)
            self.robot.GetControllerIP()
        except socket.timeout:
            print("XML-RPC connection timed out.")
            RPC.is_conect = False

        except socket.error as e:
            print("可能是网络故障，请检查网络连接。")
            RPC.is_conect = False
        except Exception as e:
            print("An error occurred during XML-RPC call:", e)
            RPC.is_conect = False
        finally:
            # 恢复默认超时时间
            self.robot = None
            socket.setdefaulttimeout(None)
            self.robot = xmlrpc.client.ServerProxy(link)


    def setup_logging(self, output_model=1, file_path="", file_num=5):
        self.logger = logging.getLogger("RPCLogger")
        log_level = logging.DEBUG
        log_handler = None

        if not file_path:
            current_datetime = datetime.now()
            formatted_date = current_datetime.strftime("%Y%m%d")
            file_name = "fairino_" + formatted_date + ".log"
            file_path = os.path.join(os.getcwd(), file_name)  # 使用当前工作目录，如果没有提供路径的话
        else:
            file_path = os.path.abspath(file_path)  # 获取绝对路径

        # 检查目录是否存在
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            # print(f"Error: The directory '{directory}' does not exist. Logging setup aborted.")
            return -1  # 如果目录不存在，则返回错误码

        if output_model == 0:
            RPC.log_output_model = 0
            log_handler = RotatingFileHandler(file_path, maxBytes=50 * 1024, backupCount=file_num)
        elif output_model == 1:
            RPC.log_output_model = 1
            log_handler = BufferedFileHandler(file_path, mode='a', maxBytes=50 * 1024, backupCount=file_num)
        elif output_model == 2:
            RPC.log_output_model = 2
            log_handler = BufferedFileHandler(file_path, mode='a', maxBytes=50 * 1024, backupCount=file_num)
            self.start_logging_thread(log_handler)

        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s pid:%(process)d]  %(message)s')
        if log_handler:
            log_handler.setFormatter(formatter)
            self.logger.addHandler(log_handler)
        else:
            print("Error: Log handler not created. Logging setup aborted.")

        return 0  # 如果日志记录设置成功，则返回成功码

    def start_logging_thread(self, log_handler):
        logging_thread = LogWriterThread(RPC.queue, log_handler)
        RPC.logging_thread = logging_thread  # 存储日志线程的引用
        logging_thread.start()

    def join_logging_thread(self):
        if RPC.logging_thread is not None:
            RPC.queue.put(None)  # 通知日志线程停止
            RPC.logging_thread.join()  # 等待日志线程完成

    def __del__(self):
        self.join_logging_thread()

    def set_log_level(self, lvl):
        levels = {1: logging.ERROR, 2: logging.WARNING, 3: logging.INFO, 4: logging.DEBUG}
        log_level = levels.get(lvl, logging.DEBUG)
        self.logger.setLevel(log_level)
        return log_level

    def log_call(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            args_str = ', '.join(map(repr, args))
            kwargs_str = ', '.join([f"{key}={value}" for key, value in kwargs.items()])
            if (kwargs_str) == "":
                call_message = f"Calling {func.__name__}" + f"({args_str}" + ")."
            else:
                call_message = f"Calling {func.__name__}" + f"({args_str}" + "," + f"{kwargs_str})."

            self.log_info(call_message)
            result = func(self, *args, **kwargs)
            if isinstance(result, (list, tuple)) and len(result) > 0:
                if result[0] == 0:
                    self.log_debug(f"{func.__name__} returned: {result}.")
                else:
                    self.log_error(f"{func.__name__} Error occurred. returned: {result}")
            else:
                if result == 0:
                    self.log_debug(f"{func.__name__} returned: {result}.")
                else:
                    self.log_error(f"{func.__name__} Error occurred. returned: {result}")

            return result

        return wrapper

    def log_debug(self, message):
        if self.logger:
            self.logger.debug(message)

    def log_info(self, message):
        if self.logger:
            self.logger.info(message)

    def log_warning(self, message):
        if self.logger:
            self.logger.warning(message)

    def log_error(self, message):
        if self.logger:
            self.logger.error(message)

    """   
    ***************************************************************************机器人基础********************************************************************************************
    """

    """   
    @brief  查询 SDK 版本号
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） version SDK版本号
    """

    @log_call
    @xmlrpc_timeout
    def GetSDKVersion(self):
        error = 0
        sdk = ["SDK:V2.0.3", "Robot:V3.7.2"]
        return error, sdk

    """   
    @brief  查询控制器 IP
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回） ip  控制器IP
    """

    @log_call
    @xmlrpc_timeout
    def GetControllerIP(self):
        _error = self.robot.GetControllerIP()
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  控制机器人手自动模式切换
    @param  [in] 必选参数 state：0-自动模式 1-手动模式
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def Mode(self, state):
        state = int(state)
        error = self.robot.Mode(state)
        return error

    """   
    @brief  控制机器人进入或退出拖动示教模式
    @param  [in] 必选参数 state：0-退出拖动示教模式, 1-进入拖动示教模式
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def DragTeachSwitch(self, state):
        state = int(state)  # 强制转换为int型
        error = self.robot.DragTeachSwitch(state)
        return error

    """   
    @brief  查询机器人是否处于拖动示教模式
    @param  [in] NULL
    @return 错误码 成功-0，失败-错误码
    @return 返回值（调用成功返回） state 0-非拖动示教模式，1-拖动示教模式
    """

    @log_call
    @xmlrpc_timeout
    def IsInDragTeach(self):
        _error = self.robot.IsInDragTeach()
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  控制机器人上使能或下使能
    @param  [in] 必选参数 state：0-下使能, 1-上使能
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def RobotEnable(self, state):
        state = int(state)  # 强制转换为int型
        error = self.robot.RobotEnable(state)
        return error

    """   
    ***************************************************************************机器人运动********************************************************************************************
    """

    """   
    @brief  jog点动
    @param  [in] 必选参数 ref：0-关节点动,2-基坐标系点动,4-工具坐标系点动,8-工件坐标系点动
    @param  [in] 必选参数 nb：1-关节1(或x轴)，2-关节2(或y轴)，3-关节3(或z轴)，4-关节4(或绕x轴旋转)，5-关节5(或绕y轴旋转)，6-关节6(或绕z轴旋转)
    @param  [in] 必选参数 dir：0-负方向，1-正方向
    @param  [in] 必认参数 max_dis：单次点动最大角度/距离，单位 ° 或 mm
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 默认100
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StartJOG(self, ref, nb, dir, max_dis, vel=20.0, acc=100.0):
        ref = int(ref)  # 强制转换为int型
        nb = int(nb)  # 强制转换为int型
        dir = int(dir)  # 强制转换为int型
        max_dis = float(max_dis)  # 强制转换为float型
        vel = float(vel)  # 强制转换为float型
        acc = float(acc)  # 强制转换为float型
        error = self.robot.StartJOG(ref, nb, dir, vel, acc, max_dis)
        return error

    """   
    @brief  jog 点动减速停止
    @param  [in] 必选参数：1-关节点动停止,3-基坐标系点动停止,5-工具坐标系点动停止,9-工件坐标系点动停止
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StopJOG(self, ref):
        ref = int(ref)  # 强制转换为int型
        error = self.robot.StopJOG(ref)
        return error

    """   
    @brief  jog 点动立即停止
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ImmStopJOG(self):
        error = self.robot.ImmStopJOG()
        return error

    """   
    @brief  关节空间运动
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用正运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放,默认0.0 
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 blendT:[-1.0]-运动到位 (阻塞)，[0~500.0]-平滑时间 (非阻塞)，单位 [ms] 默认-1.0
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveJ(self, joint_pos, tool, user, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
              exaxis_pos=[0.0, 0.0, 0.0, 0.0], blendT=-1.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        exaxis_pos = list(map(float, exaxis_pos))
        blendT = float(blendT)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        if (desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0) and (
                desc_pos[4] == 0.0) and (desc_pos[5] == 0.0):  # 若未输入参数则调用正运动学求解
            ret = self.robot.GetForwardKin(joint_pos)  # 正运动学求解
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, exaxis_pos, blendT, offset_flag,
                                 offset_pos)
        return error

    """   
    @brief  笛卡尔空间直线运动
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 search:[0]-不焊丝寻位，[1]-焊丝寻位
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveL(self, desc_pos, tool, user, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
              blendR=-1.0,
              exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, -1)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.MoveL(joint_pos, desc_pos, tool, user, vel, acc, ovl, blendR, exaxis_pos, search,
                                 offset_flag, offset_pos)
        return error

    """   
    @brief  笛卡尔空间圆弧运动
    @param  [in] 必选参数 desc_pos_p: 路径点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_p: 路径点工具号，[0~14]
    @param  [in] 必选参数 user_p: 路径点工件号，[0~14]
    @param  [in] 必选参数 desc_pos_t: 目标点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_t: 工具号，[0~14]
    @param  [in] 必选参数 user_t: 工件号，[0~14]
    @param  [in] 默认参数 joint_pos_p: 路径点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 joint_pos_t: 目标点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel_p: 路径点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_p: 路径点加速度百分比，[0~100] 暂不开放,默认0.0
    @param  [in] 默认参数 exaxis_pos_p: 路径点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 offset_flag_p: 路径点是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos_p: 路径点位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 vel_t: 目标点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_t: 目标点加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 exaxis_pos_t: 目标点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 offset_flag_t: 目标点是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos_t: 目标点位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveC(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              vel_p=20.0, acc_p=100.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], offset_flag_p=0,
              offset_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              vel_t=20.0, acc_t=100.0, exaxis_pos_t=[0.0, 0.0, 0.0, 0.0], offset_flag_t=0,
              offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ovl=100.0, blendR=-1.0):
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = float(int(tool_p))
        user_p = float(int(user_p))
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))
        offset_flag_p = int(offset_flag_p)
        offset_pos_p = list(map(float, offset_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = float(int(tool_t))
        user_t = float(int(user_t))
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))
        offset_flag_t = int(offset_flag_t)
        offset_pos_t = list(map(float, offset_pos_t))

        ovl = float(ovl)
        blendR = float(blendR)

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, -1)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, -1)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error
        error = self.robot.MoveC(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p, offset_flag_p,
                                 offset_pos_p, joint_pos_t, desc_pos_t, [tool_t, user_t, vel_t, acc_t], exaxis_pos_t,
                                 offset_flag_t, offset_pos_t, ovl, blendR)
        return error

    """   
    @brief  笛卡尔空间整圆运动
    @param  [in] 必选参数 desc_pos_p: 路径点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_p: 路径点工具号，[0~14]
    @param  [in] 必选参数 user_p: 路径点工件号，[0~14]
    @param  [in] 必选参数 desc_pos_t: 目标点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_t: 工具号，[0~14]
    @param  [in] 必选参数 user_t: 工件号，[0~14]
    @param  [in] 默认参数 joint_pos_p: 路径点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 joint_pos_t: 目标点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel_p: 路径点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_p: 路径点加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 exaxis_pos_p: 路径点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 vel_t: 目标点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_t: 目标点加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 exaxis_pos_t: 目标点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 offset_flag: 是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               vel_p=20.0, acc_p=0.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=0.0,
               exaxis_pos_t=[0.0, 0.0, 0.0, 0.0],
               ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = float(int(tool_p))
        user_p = float(int(user_p))
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = float(int(tool_t))
        user_t = float(int(user_t))
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))

        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, -1)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, -1)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error

        error = self.robot.Circle(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p, joint_pos_t,
                                  desc_pos_t,
                                  [tool_t, user_t, vel_t, acc_t], exaxis_pos_t, ovl, offset_flag, offset_pos)
        return error

    """   
    @brief  笛卡尔空间螺旋线运动
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 param:[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction]circle_num: 螺旋圈数，circle_angle: 螺旋倾角，
    rad_init: 螺旋初始半径，rad_add: 半径增量，rotaxis_add: 转轴方向增量，rot_direction: 旋转方向，0-顺时针，1-逆时针
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 默认100.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSpiral(self, desc_pos, tool, user, param, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0,
                  exaxis_pos=[0.0, 0.0, 0.0, 0.0],
                  ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        param[0] = float(param[0])
        param[1] = float(param[1])
        param[2] = float(param[2])
        param[3] = float(param[3])
        param[4] = float(param[4])
        param[5] = float(param[5])
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        exaxis_pos = list(map(float, exaxis_pos))
        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, -1)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.NewSpiral(joint_pos, desc_pos, tool, user, vel, acc, exaxis_pos, ovl, offset_flag,
                                     offset_pos, param)
        return error

    """   
    @brief  伺服运动开始，配合ServoJ、ServoCart指令使用
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveStart(self):
        error = self.robot.ServoMoveStart()
        return error

    """   
    @brief  伺服运动结束，配合ServoJ、ServoCart指令使用
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveEnd(self):
        error = self.robot.ServoMoveEnd()
        return error

    """   
    @brief  关节空间伺服模式运动
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 cmdT: 指令下发周期，单位s，建议范围[0.001~0.0016], 默认为0.008
    @param  [in] 默认参数 filterT: 滤波时间，单位 [s]，暂不开放， 默认为0.0
    @param  [in] 默认参数 gain: 目标位置的比例放大器，暂不开放， 默认为0.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJ(self, joint_pos, acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0):
        joint_pos = list(map(float, joint_pos))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        error = self.robot.ServoJ(joint_pos, acc, vel, cmdT, filterT, gain)
        return error

    """   
    @brief  笛卡尔空间伺服模式运动
    @param  [in] 必选参数 mode:[0]-绝对运动 (基坐标系)，[1]-增量运动 (基坐标系)，[2]-增量运动(工具坐标系)
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位置/目标笛卡尔位置增量
    @param  [in] 默认参数 pos_gain: 位姿增量比例系数，仅在增量运动下生效，范围 [0~1], 默认为 [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 cmdT:指令下发周期，单位s，建议范围[0.001~0.0016], 默认为0.008
    @param  [in] 默认参数 filterT: 滤波时间，单位 [s]，暂不开放， 默认为0.0
    @param  [in] 默认参数 gain: 目标位置的比例放大器，暂不开放， 默认为0.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoCart(self, mode, desc_pos, pos_gain=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], acc=0.0, vel=0.0, cmdT=0.008,
                  filterT=0.0, gain=0.0):
        mode = int(mode)
        desc_pos = list(map(float, desc_pos))
        pos_gain = list(map(float, pos_gain))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        error = self.robot.ServoCart(mode, desc_pos, pos_gain, acc, vel, cmdT, filterT, gain)
        return error

    """   
    @brief  关节扭矩控制开始
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJTStart(self):
        error = self.robot.ServoJTStart()
        return error

    """   
    @brief  关节扭矩控制
    @param  [in] 必选参数 torque j1~j6关节扭矩，单位Nm
    @param  [in] 必选参数 interval 指令周期，单位s，范围[0.001~0.008]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJT(self, torque, interval):
        torque = list(map(float, torque))
        interval = float(interval)
        error = self.robot.ServoJT(torque, interval)
        return error

    """   
    @brief  关节扭矩控制结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJTEnd(self):
        error = self.robot.ServoJTEnd()
        return error

    """   
    @brief  笛卡尔空间点到点运动
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位置/目标笛卡尔位置增量
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，默认为 20.0
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放,默认为 0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100，默认为 100.0
    @param  [in] 默认参数 blendT:[-1.0]-运动到位 (阻塞)，[0~500]-平滑时间 (非阻塞)，单位 [ms] 默认为 -1.0
    @param  [in] 默认参数 config: 关节配置，[-1]-参考当前关节位置求解，[0~7]-依据关节配置求解 默认为 -1
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveCart(self, desc_pos, tool, user, vel=20.0, acc=0.0, ovl=100.0, blendT=-1.0, config=-1):
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendT = float(blendT)
        config = int(config)
        error = self.robot.MoveCart(desc_pos, tool, user, vel, acc, ovl, blendT, config)
        return error

    """   
    @brief  样条运动开始
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SplineStart(self):
        error = self.robot.SplineStart()
        return error

    """   
    @brief  样条运动 PTP
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用正运动学求解返回值
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，默认为 20.0
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，默认为 100.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100，默认为 100.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SplinePTP(self, joint_pos, tool, user, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=100.0, ovl=100.0):
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        if ((desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0)
                and (desc_pos[4] == 0.0) and (desc_pos[5] == 0.0)):  # 若未输入参数则调用正运动学求解
            ret = self.robot.GetForwardKin(joint_pos)  # 正运动学求解
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.SplinePTP(joint_pos, desc_pos, tool, user, vel, acc, ovl)
        return error

    """   
    @brief  样条运动结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SplineEnd(self):
        error = self.robot.SplineEnd()
        return error

    """   
    @brief  新样条运动开始
    @param  [in] 必选参数 type:0-圆弧过渡，1-给定点位路径点
    @param  [in] 默认参数 averageTime: 全局平均衔接时间（ms）默认为 2000
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSplineStart(self, type, averageTime=2000):
        type = int(type)
        averageTime = int(averageTime)
        error = self.robot.NewSplineStart(type, averageTime)
        return error

    """   
    @brief  新样条指令点
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 lastFlag: 是否为最后一个点，0-否，1-是
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认为 100.0
    @param  [in] 默认参数 blendR: [0~1000]-平滑半径，单位 [mm] 默认0.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSplinePoint(self, desc_pos, tool, user, lastFlag, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.0,
                       acc=0.0, ovl=100.0, blendR=0.0):
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        lastFlag = int(lastFlag)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, -1)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.NewSplinePoint(joint_pos, desc_pos, tool, user, vel, acc, ovl, blendR, lastFlag)
        return error

    """   
    @brief  新样条运动结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSplineEnd(self):
        error = self.robot.NewSplineEnd()
        return error

    """   
    @brief  终止运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StopMotion(self):
        error = self.robot.StopMotion()
        return error

    """   
    @brief  暂停运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PauseMotion(self):
        error = self.robot.PauseMotion()
        return error

    """   
    @brief  恢复运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ResumeMotion(self):
        error = self.robot.ResumeMotion()
        return error

    """   
    @brief  点位整体偏移开始
    @param  [in] 必选参数 flag:0-基坐标或工件坐标系下偏移，2-工具坐标系下偏移
    @param  [in] 必选参数 offset_pos: 偏移量，单位 [mm][°]。
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointsOffsetEnable(self, flag, offset_pos):
        flag = int(flag)
        offset_pos = list(map(float, offset_pos))
        error = self.robot.PointsOffsetEnable(flag, offset_pos)
        return error

    """   
    @brief  点位整体偏移结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointsOffsetDisable(self):
        error = self.robot.PointsOffsetDisable()
        return error

    """   
    ***************************************************************************机器人IO********************************************************************************************
    """

    """   
    @brief  设置控制箱数字量输出
    @param  [in] 必选参数 id:io 编号，范围 [0~15]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 默认参数 smooth:0-不平滑，1-平滑 默认0
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetDO(self, id, status, smooth=0, block=0):
        id = int(id)
        status = int(status)
        smooth = int(smooth)
        block = int(block)
        error = self.robot.SetDO(id, status, smooth, block)
        return error

    """   
    @brief  设置工具数字量输出
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 默认参数 smooth:0-不平滑，1-平滑 默认0
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolDO(self, id, status, smooth=0, block=0):
        id = int(id)
        status = int(status)
        smooth = int(smooth)
        block = int(block)
        error = self.robot.SetToolDO(id, status, smooth, block)
        return error

    """   
    @brief  设置控制箱模拟量输出
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 value: 电流或电压值百分比，范围 [0~100%] 对应电流值 [0~20mA] 或电压 [0~10V]；
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAO(self, id, value, block=0):
        id = int(id)
        value = float(value)
        block = int(block)
        error = self.robot.SetAO(id, value * 40.95, block)
        return error

    """   
    @brief  设置工具模拟量输出
    @param  [in] 必选参数 id:io 编号，范围 [0]
    @param  [in] 必选参数 value: 电流或电压值百分比，范围 [0~100%] 对应电流值 [0~20mA] 或电压 [0~10V]；
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolAO(self, id, value, block=0):
        id = int(id)
        value = float(value)
        block = int(block)
        error = self.robot.SetToolAO(id, value * 40.95, block)
        return error

    """   
    @brief  获取控制箱数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0-15]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回）di: 0-低电平，1-高电平
    """

    @log_call
    @xmlrpc_timeout
    def GetDI(self, id, block=0):
        id = int(id)
        block = int(block)
        _error = self.robot.GetDI(id, block)
        error = _error[0]
        if _error[0] == 0:
            di = _error[1]
            return error, di
        else:
            return error

    """   
    @brief  获取工具数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）di: 0-低电平，1-高电平
    """

    @log_call
    @xmlrpc_timeout
    def GetToolDI(self, id, block=0):
        id = int(id)
        block = int(block)
        _error = self.robot.GetToolDI(id, block)
        error = _error[0]
        if _error[0] == 0:
            di = _error[1]
            return error, di
        else:
            return error

    """   
    @brief  等待控制箱数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~15]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitDI(self, id, status, maxtime, opt):
        id = int(id)
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        error = self.robot.WaitDI(id, status, maxtime, opt)
        return error

    """   
    @brief  等待控制箱多路数字量输入
    @param  [in] 必选参数 mode 0-多路与，1-多路或
    @param  [in] 必选参数 id  io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitMultiDI(self, mode, id, status, maxtime, opt):
        mode = int(mode)
        id = int(id)
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        error = self.robot.WaitMultiDI(mode, id, status, maxtime, opt)
        return error

    """   
    @brief  等待工具数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitToolDI(self, id, status, maxtime, opt):
        id = int(id)
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        error = self.robot.WaitToolDI(id, status, maxtime, opt)
        return error

    """   
    @brief  获取控制箱模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    """

    @log_call
    @xmlrpc_timeout
    def GetAI(self, id, block=0):
        id = int(id)
        block = int(block)
        _error = self.robot.GetAI(id, block)
        error = _error[0]
        if _error[0] == 0:
            value = _error[1]
            return error, value
        else:
            return error

    """   
    @brief  获取工具模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    """

    @log_call
    @xmlrpc_timeout
    def GetToolAI(self, id, block=0):
        id = int(id)
        block = int(block)
        _error = self.robot.GetToolAI(id, block)
        error = _error[0]
        if _error[0] == 0:
            value = _error[1]
            return error, value
        else:
            return error

    """   
    @brief  获取机器人末端点记录按钮状态
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）按钮状态，0-按下，1-松开
    """

    @log_call
    @xmlrpc_timeout
    def GetAxlePointRecordBtnState(self):
        _error = self.robot.GetAxlePointRecordBtnState()
        error = _error[0]
        if _error[0] == 0:
            value = _error[1]
            return error, value
        else:
            return error

    """   
    @brief  获取机器人末端DO输出状态
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）do_state DO输出状态，do0~do1对应bit1~bit2,从bit0开始
    """

    @log_call
    @xmlrpc_timeout
    def GetToolDO(self):
        _error = self.robot.GetToolDO()
        error = _error[0]
        if _error[0] == 0:
            value = _error[1]
            return error, value
        else:
            return error

    """   
    @brief  获取机器人控制器DO输出状态
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）do_state_h DO输出状态，co0~co7对应bit0~bit7 do_state_l DO输出状态，do0~do7对应bit0~bit7
    """

    @log_call
    @xmlrpc_timeout
    def GetDO(self):
        _error = self.robot.GetDO()
        error = _error[0]
        if _error[0] == 0:
            do_state_h = _error[1]
            do_state_l = _error[2]
            return error, [do_state_h, do_state_l]
        else:
            return error

    """   
    @brief  等待控制箱模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 sign:0-大于，1-小于
    @param  [in] 必选参数 value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitAI(self, id, sign, value, maxtime, opt):
        id = int(id)
        sign = int(sign)
        value = float(value)
        maxtime = int(maxtime)
        opt = int(opt)
        error = self.robot.WaitAI(id, sign, value, maxtime, opt)
        return error

    """   
    @brief  等待工具模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0]
    @param  [in] 必选参数 sign:0-大于，1-小于
    @param  [in] 必选参数 value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitToolAI(self, id, sign, value, maxtime, opt):
        id = int(id)
        sign = int(sign)
        value = float(value)
        maxtime = int(maxtime)
        opt = int(opt)
        error = self.robot.WaitToolAI(id, sign, value, maxtime, opt)
        return error

    """   
    ***************************************************************************机器人常用设置********************************************************************************************
    """

    """   
    @brief  设置全局速度
    @param  [in] 必选参数 vel  速度百分比，范围[0~100]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetSpeed(self, vel):
        vel = int(vel)
        error = self.robot.SetSpeed(vel)
        return error

    """   
    @brief  设置系统变量
    @param  [in] 必选参数 id：变量编号，范围 [1~20]
    @param  [in] 必选参数 value：变量值
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetSysVarValue(self, id, value):
        id = int(id)
        value = float(value)
        error = self.robot.SetSysVarValue(id, value)
        return error

    """   
    @brief  设置工具参考点-六点法
    @param  [in] 必选参数 point_num 点编号,范围[1~6] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolPoint(self, point_num):
        point_num = int(point_num)
        error = self.robot.SetToolPoint(point_num)
        return error

    """   
    @brief  计算工具坐标系-六点法
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）tcp_pose [x,y,z,rx,ry,rz] 工具坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeTool(self):
        _error = self.robot.ComputeTool()
        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  设置工具参考点-四点法
    @param  [in] 必选参数 point_num 点编号,范围[1~4] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTcp4RefPoint(self, point_num):
        point_num = int(point_num)
        error = self.robot.SetTcp4RefPoint(point_num)
        return error

    """   
    @brief  计算工具坐标系-四点法
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）tcp_pose [x,y,z,rx,ry,rz]  工具坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeTcp4(self):
        _error = self.robot.ComputeTcp4()
        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  设置工具坐标系
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 t_coord:[x,y,z,rx,ry,rz]  工具中心点相对末端法兰中心位姿，单位 [mm][°]
    @param  [in] 必选参数 type:0-工具坐标系，1-传感器坐标系
    @param  [in] 必选参数 install: 安装位置，0-机器人末端，1-机器人外部
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolCoord(self, id, t_coord, type, install):
        id = int(id)
        t_coord = list(map(float, t_coord))
        type = int(type)
        install = int(install)
        error = self.robot.SetToolCoord(id, t_coord, type, install)
        return error

    """   
    @brief  设置工具坐标系列表
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 t_coord:[x,y,z,rx,ry,rz]  工具中心点相对末端法兰中心位姿，单位 [mm][°]
    @param  [in] 必选参数 type:0-工具坐标系，1-传感器坐标系
    @param  [in] 必选参数 install: 安装位置，0-机器人末端，1-机器人外部
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolList(self, id, t_coord, type, install):
        id = int(id)
        t_coord = list(map(float, t_coord))
        type = int(type)
        install = int(install)
        error = self.robot.SetToolList(id, t_coord, type, install)
        return error

    """   
    @brief  设置外部工具参考点-三点法
    @param  [in] 必选参数 point_num 点编号,范围[1~3] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExTCPPoint(self, point_num):
        point_num = int(point_num)
        error = self.robot.SetExTCPPoint(point_num)
        return error

    """   
    @brief  计算外部工具坐标系-三点法
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）tcp_pose [x,y,z,rx,ry,rz] 外部工具坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeExTCF(self):
        _error = self.robot.ComputeExTCF()
        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  设置外部工具坐标系
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 etcp: [x,y,z,rx,ry,rz] 外部工具坐标系，单位 [mm][°]
    @param  [in] 必选参数 etool: [x,y,z,rx,ry,rz] 末端工具坐标系，单位 [mm][°]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExToolCoord(self, id, etcp, etool):
        id = int(id)
        etcp = list(map(float, etcp))
        etool = list(map(float, etool))
        error = self.robot.SetExToolCoord(id, etcp, etool)
        return error

    """   
    @brief  设置外部工具坐标系列表
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 etcp: [x,y,z,rx,ry,rz] 外部工具坐标系，单位 [mm][°]
    @param  [in] 必选参数 etool: [x,y,z,rx,ry,rz] 末端工具坐标系，单位 [mm][°]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExToolList(self, id, etcp, etool):
        id = int(id)
        etcp = list(map(float, etcp))
        etool = list(map(float, etool))
        error = self.robot.SetExToolList(id, etcp, etool)
        return error

    """   
    @brief  设置工件参考点-三点法
    @param  [in] 必选参数 point_num 点编号,范围[1~3] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjCoordPoint(self, point_num):
        point_num = int(point_num)
        error = self.robot.SetWObjCoordPoint(point_num)
        return error

    """   
    @brief  计算工件坐标系
    @param  [in] method 计算方式 0：原点-x轴-z轴  1：原点-x轴-xy平面
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）wobj_pose [x,y,z,rx,ry,rz] 工件坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeWObjCoord(self, method):
        method = int(method)
        _error = self.robot.ComputeWObjCoord(method)
        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  设置工件坐标系
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 w_coord: 坐标系相对位姿，单位 [mm][°]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjCoord(self, id, w_coord):
        id = int(id)
        w_coord = list(map(float, w_coord))
        error = self.robot.SetWObjCoord(id, w_coord)
        return error

    """   
    @brief  设置工件坐标系列表
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 w_coord: 坐标系相对位姿，单位 [mm][°]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjList(self, id, w_coord):
        id = int(id)
        w_coord = list(map(float, w_coord))
        error = self.robot.SetWObjList(id, w_coord)
        return error

    """   
    @brief  设置末端负载重量
    @param  [in] 必选参数 weight: 单位 [kg]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoadWeight(self, weight):
        weight = float(weight)
        error = self.robot.SetLoadWeight(weight)
        return error

    """   
    @brief  设置机器人安装方式-固定安装
    @param  [in] 必选参数 method:0-正装，1-侧装，2-挂装
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotInstallPos(self, method):
        method = int(method)
        error = self.robot.SetRobotInstallPos(method)
        return error

    """   
    @brief  设置机器人安装角度
    @param  [in] 必选参数 yangle：倾斜角
    @param  [in] 必选参数 zangle：旋转角
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotInstallAngle(self, yangle, zangle):
        yangle = float(yangle)
        zangle = float(zangle)
        error = self.robot.SetRobotInstallAngle(yangle, zangle)
        return error

    """   
    @brief  设置末端负载质心坐标
    @param  [in] 必选参数 x: 质心坐标，单位 [mm]
    @param  [in] 必选参数 y: 质心坐标，单位 [mm]
    @param  [in] 必选参数 z: 质心坐标，单位 [mm]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoadCoord(self, x, y, z):
        x = float(x)
        y = float(y)
        z = float(z)
        error = self.robot.SetLoadCoord(x, y, z)
        return error

    """   
    @brief  等待指定时间
    @param  [in] 必选参数 t_ms: 单位 [ms]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitMs(self, t_ms):
        t_ms = int(t_ms)
        error = self.robot.WaitMs(t_ms)
        return error

    """   
    ***************************************************************************机器人安全设置********************************************************************************************
    """

    """   
    @brief  设置碰撞等级
    @param  [in] 必选参数 mode:0-等级，1-百分比
    @param  [in] 必选参数 level=[j1,j2,j3,j4,j5,j6]: 碰撞阈值 mode=0时，范围：1-10 对应mode=1时，范围0-100%
    @param  [in] 必选参数 config:0-不更新配置文件，1-更新配置文件
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAnticollision(self, mode, level, config):
        mode = int(mode)
        level = list(map(float, level))
        config = int(config)
        error = self.robot.SetAnticollision(mode, level, config)
        return error

    """   
    @brief  设置碰撞后策略
    @param  [in] 必选参数 strategy：0-报错暂停，1-继续运行
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetCollisionStrategy(self, strategy):
        strategy = int(strategy)
        error = self.robot.SetCollisionStrategy(strategy)
        return error

    """   
    @brief  设置正限位
    @param  [in] 必选参数 p_limit=[j1,j2,j3,j4,j5,j6]：六个关节位置
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLimitPositive(self, p_limit):
        p_limit = list(map(float, p_limit))
        error = self.robot.SetLimitPositive(p_limit)
        return error

    """   
    @brief  设置负限位
    @param  [in] 必选参数 n_limit=[j1,j2,j3,j4,j5,j6]：六个关节位置
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLimitNegative(self, n_limit):
        n_limit = list(map(float, n_limit))
        error = self.robot.SetLimitNegative(n_limit)
        return error

    """   
    @brief  错误状态清除，只能清除可复位的错误
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ResetAllError(self):
        error = self.robot.ResetAllError()
        return error

    """   
    @brief  关节摩擦力补偿开关
    @param  [in] 必选参数 state：0-关，1-开
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FrictionCompensationOnOff(self, state):
        state = int(state)
        error = self.robot.FrictionCompensationOnOff(state)
        return error

    """   
    @brief  设置关节摩擦力补偿系数-固定安装-正装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_level(self, coeff):
        coeff = list(map(float, coeff))
        error = self.robot.SetFrictionValue_level(coeff)
        return error

    """   
    @brief  设置关节摩擦力补偿系数-固定安装-侧装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_wall(self, coeff):
        coeff = list(map(float, coeff))
        error = self.robot.SetFrictionValue_wall(coeff)
        return error

    """   
    @brief  设置关节摩擦力补偿系数-固定安装-倒装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_ceiling(self, coeff):
        coeff = list(map(float, coeff))
        error = self.robot.SetFrictionValue_ceiling(coeff)
        return error

    """   
    @brief  设置关节摩擦力补偿系数-自由安装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_freedom(self, coeff):
        coeff = list(map(float, coeff))
        error = self.robot.SetFrictionValue_freedom(coeff)
        return error

    """   
    ***************************************************************************机器人状态查询********************************************************************************************
    """

    """   
    @brief  获取机器人安装角度
    @param  [in] NULL
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回）[yangle,zangle] yangle-倾斜角,zangle-旋转角
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotInstallAngle(self):
        _error = self.robot.GetRobotInstallAngle()
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error

    """   
    @brief  获取系统变量值
    @param  [in] id：系统变量编号，范围 [1~20]
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） var_value：系统变量值
    """

    @log_call
    @xmlrpc_timeout
    def GetSysVarValue(self, id):
        id = int(id)
        _error = self.robot.GetSysVarValue(id)
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取当前关节位置 (角度)
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointPosDegree(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualJointPosDegree(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取关节当前位置 (弧度)
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointPosRadian(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualJointPosRadian(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取关节反馈速度-deg/s
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） speed=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointSpeedsDegree(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualJointSpeedsDegree(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取关节反馈加速度-deg/s^2
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） acc=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointAccDegree(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualJointAccDegree(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取TCP指令合速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回）[tcp_speed,ori_speed] tcp_speed 线性合速度 ori_speed 姿态合速度 
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetTCPCompositeSpeed(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetTargetTCPCompositeSpeed(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error

    """   
    @brief  获取TCP反馈合速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回）[tcp_speed,ori_speed] tcp_speed 线性合速度 ori_speed 姿态合速度 
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPCompositeSpeed(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualTCPCompositeSpeed(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error

    """   
    @brief  获取TCP指令速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞  默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） speed [x,y,z,rx,ry,rz]速度
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetTCPSpeed(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetTargetTCPSpeed(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取TCP反馈速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞  默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） speed [x,y,z,rx,ry,rz]速度
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPSpeed(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualTCPSpeed(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取当前工具位姿
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） tcp_pose=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPPose(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualTCPPose(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取当前工具坐标系编号
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） tool_id:工具坐标系编号
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPNum(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualTCPNum(flag)
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取当前工件坐标系编号 
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） wobj_id:工件坐标系编号
    """

    @log_call
    @xmlrpc_timeout
    def GetActualWObjNum(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualWObjNum(flag)
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取当前末端法兰位姿
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） flange_pose=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualToolFlangePose(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetActualToolFlangePose(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  逆运动学，笛卡尔位姿求解关节位置
    @param  [in] 必选参数 type:0-绝对位姿 (基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    @param  [in] 必选参数 desc_pose:[x,y,z,rx,ry,rz], 工具位姿，单位 [mm][°]
    @param  [in] 默认参数 config: 关节配置，[-1]-参考当前关节位置求解，[0~7]-依据关节配置求解 默认-1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKin(self, type, desc_pos, config=-1):
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        config = int(config)
        _error = self.robot.GetInverseKin(type, desc_pos, config)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  逆运动学，工具位姿求解关节位置，参考指定关节位置求解
    @param  [in] 必选参数 type:0-绝对位姿 (基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    @param  [in] 必选参数 desc_pose:[x,y,z,rx,ry,rz], 工具位姿，单位 [mm][°]
    @param  [in] 必选参数 joint_pos_ref：[j1,j2,j3,j4,j5,j6]，关节参考位置，单位 [°]
    @return 错误码 成功- 0,joint_pos=[j1,j2,j3,j4,j5,j6] 失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKinRef(self, type, desc_pos, joint_pos_ref):
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        joint_pos_ref = list(map(float, joint_pos_ref))
        _error = self.robot.GetInverseKinRef(type, desc_pos, joint_pos_ref)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  逆运动学，工具位姿求解关节位置是否有解
    @param  [in] 必选参数 type:0-绝对位姿 (基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    @param  [in] 必选参数 desc_pose:[x,y,z,rx,ry,rz], 工具位姿，单位 [mm][°]
    @param  [in] 必选参数 joint_pos_ref：[j1,j2,j3,j4,j5,j6]，关节参考位置，单位 [°]
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） result:“True”-有解，“False”-无解
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKinHasSolution(self, type, desc_pos, joint_pos_ref):
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        joint_pos_ref = list(map(float, joint_pos_ref))
        _error = self.robot.GetInverseKinHasSolution(type, desc_pos, joint_pos_ref)
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  正运动学，关节位置求解工具位姿
    @param  [in] 必选参数 joint_pos:[j1,j2,j3,j4,j5,j6]: 关节位置，单位 [°]
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） desc_pos=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetForwardKin(self, joint_pos):
        joint_pos = list(map(float, joint_pos))
        _error = self.robot.GetForwardKin(joint_pos)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取当前关节转矩
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） torques=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetJointTorques(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetJointTorques(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取当前负载的质量
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weight  单位 [kg]
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayload(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetTargetPayload(flag)
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取当前负载的质心
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）cog=[x,y,z]: 质心坐标，单位 [mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayloadCog(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetTargetPayloadCog(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]]
        else:
            return error

    """   
    @brief  获取当前工具坐标系
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）tcp_offset=[x,y,z,rx,ry,rz]: 相对位姿，单位 [mm][°]
    """

    @log_call
    @xmlrpc_timeout
    def GetTCPOffset(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetTCPOffset(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取当前工件坐标系
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）wobj_offset=[x,y,z,rx,ry,rz]: 相对位姿，单位 [mm][°]
    """

    @log_call
    @xmlrpc_timeout
    def GetWObjOffset(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetWObjOffset(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取关节软限位角度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[j1min,j1max,j2min,j2max,j3min,j3max,j4min,j4max,j5min,j5max,j6min,j6max]: 轴 1~ 轴 6 关节负限位与正限位，单位 [mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetJointSoftLimitDeg(self, flag=1):
        flag = int(flag)
        _error = self.robot.GetJointSoftLimitDeg(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8],
                           _error[9], _error[10], _error[11], _error[12]]
        else:
            return error

    """   
    @brief  获取系统时间
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）t_ms: 单位 [ms]
    """

    @log_call
    @xmlrpc_timeout
    def GetSystemClock(self):
        _error = self.robot.GetSystemClock()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取机器人当前关节配置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）config: 范围 [0~7]
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotCurJointsConfig(self):
        _error = self.robot.GetRobotCurJointsConfig()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取默认速度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）vel: 单位 [mm/s]
    """

    @log_call
    @xmlrpc_timeout
    def GetDefaultTransVel(self):
        _error = self.robot.GetDefaultTransVel()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  查询机器人运动是否完成
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state:0-未完成，1-完成
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotMotionDone(self):
        _error = self.robot.GetRobotMotionDone()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  查询机器人错误码
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[maincode subcode] maincode 主错误码 subcode 子错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotErrorCode(self):
        _error = self.robot.GetRobotErrorCode()
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error

    """   
    @brief  查询机器人示教管理点位数据
    @param  [in] 必选参数 name  点位名
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data 点位数据[x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool, wobj,speed,acc,e1,e2,e3,e4]
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotTeachingPoint(self, name):
        name = str(name)
        _error = self.robot.GetRobotTeachingPoint(name)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8],
                           _error[9], _error[10],
                           _error[11], _error[12], _error[13], _error[14], _error[15], _error[16], _error[17],
                           _error[18], _error[19]]
        else:
            return error

    """   
    @brief  查询机器人运动队列缓存长度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）len  缓存长度
    """

    @log_call
    @xmlrpc_timeout
    def GetMotionQueueLength(self):
        _error = self.robot.GetMotionQueueLength()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取机器人急停状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state 急停状态，0-非急停，1-急停
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotEmergencyStopState(self):
        _error = self.robot.GetRobotEmergencyStopState()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取安全停止信号
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[si0_state,si1_state] si0_state 安全停止信号SI0，0-无效，1-有效 si1_state 安全停止信号SI1，0-无效，1-有效
    """

    @log_call
    @xmlrpc_timeout
    def GetSafetyStopState(self):
        _error = self.robot.GetSafetyStopState()
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error

    """   
    @brief  获取SDK与机器人的通讯状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state 通讯状态，0-通讯正常，1-通讯异常
    """

    @log_call
    @xmlrpc_timeout
    def GetSDKComState(self):
        _error = self.robot.GetSDKComState()
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error

    """   
       @brief  获取SSH公钥
       @param  [in] NULL
       @return 错误码 成功-0，失败-错误码
       @return 返回值（调用成功返回） keygen 公钥
       """

    @log_call
    @xmlrpc_timeout
    def GetSSHKeygen(self):
        _error = self.robot.GetSSHKeygen()
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  下发SCP指令
    @param  [in] 必选参数 mode 0-上传（上位机->控制器），1-下载（控制器->上位机）
    @param  [in] 必选参数 sshname 上位机用户名
    @param  [in] 必选参数 sship 上位机ip地址
    @param  [in] 必选参数 usr_file_url 上位机文件路径
    @param  [in] 必选参数 robot_file_url 机器人控制器文件路径
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetSSHScpCmd(self, mode, sshname, sship, usr_file_url, robot_file_url):
        mode = int(mode)
        sshname = str(sshname)
        sship = str(sship)
        usr_file_url = str(usr_file_url)
        robot_file_url = str(robot_file_url)
        error = self.robot.SetSSHScpCmd(mode, sshname, sship, usr_file_url, robot_file_url)
        return error

    """   
    @brief  计算指定路径下文件的MD5值
    @param  [in] 必选参数 file_path 文件路径包含文件名，默认Traj文件夹路径为:"/fruser/traj/",如"/fruser/traj/trajHelix_aima_1.txt"
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回）md5 文件MD5值
    """

    @log_call
    @xmlrpc_timeout
    def ComputeFileMD5(self, file_path):
        file_path = str(file_path)
        _error = self.robot.ComputeFileMD5(file_path)
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取机器人版本信息
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） robotModel 机器人模型
    @return 返回值（调用成功返回） webVersion web版本
    @return 返回值（调用成功返回） controllerVersion 控制器版本
    """

    @log_call
    @xmlrpc_timeout
    def GetSoftwareVersion(self):
        _error = self.robot.GetSoftwareVersion()
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3]
        else:
            return error

    """   
    @brief  获取机器人硬件版本信息
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） ctrlBoxBoardVersion 控制箱版本
    @return 返回值（调用成功返回） driver1Version 
    @return 返回值（调用成功返回） driver2Version 
    @return 返回值（调用成功返回） driver3Version
    @return 返回值（调用成功返回） driver4Version
    @return 返回值（调用成功返回） driver5Version
    @return 返回值（调用成功返回） driver6Version
    @return 返回值（调用成功返回） endBoardVersion
    """

    @log_call
    @xmlrpc_timeout
    def GetSlaveHardVersion(self):
        _error = self.robot.GetSlaveHardVersion()
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error

    """   
    @brief  获取机器人固件版本信息
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） ctrlBoxBoardVersion 控制箱版本
    @return 返回值（调用成功返回） driver1Version 
    @return 返回值（调用成功返回） driver2Version 
    @return 返回值（调用成功返回） driver3Version
    @return 返回值（调用成功返回） driver4Version
    @return 返回值（调用成功返回） driver5Version
    @return 返回值（调用成功返回） driver6Version
    @return 返回值（调用成功返回） endBoardVersion
    """

    @log_call
    @xmlrpc_timeout
    def GetSlaveFirmVersion(self):
        _error = self.robot.GetSlaveFirmVersion()
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error

    """   
    @brief  获取DH补偿参数
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） dhCompensation 机器人DH参数补偿值(mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
    """

    @log_call
    @xmlrpc_timeout
    def GetDHCompensation(self):
        _error = self.robot.GetDHCompensation()
        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    ***************************************************************************机器人轨迹复现********************************************************************************************
    """

    """   
    @brief  设置轨迹记录参数
    @param  [in] 必选参数 name：轨迹名
    @param  [in] 必选参数 period_ms：采样周期，固定值，2ms 或 4ms 或 8ms
    @param  [in] 默认参数 type：数据类型，1-关节位置 默认1
    @param  [in] 默认参数 di_choose：DI 选择,bit0~bit7 对应控制箱 DI0~DI7，bit8~bit9 对应末端DI0~DI1，0-不选择，1-选择 默认0
    @param  [in] 默认参数 do_choose：DO 选择,bit0~bit7 对应控制箱 DO0~DO7，bit8~bit9 对应末端 DO0~DO1，0-不选择，1-选择 默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDParam(self, name, period_ms, type=1, di_choose=0, do_choose=0):
        name = str(name)
        period_ms = int(period_ms)
        type = int(type)
        di_choose = int(di_choose)
        do_choose = int(do_choose)
        error = self.robot.SetTPDParam(type, name, period_ms, di_choose, do_choose)
        return error

    """   
    @brief  开始轨迹记录
    @param  [in] 必选参数 name：轨迹名
    @param  [in] 必选参数 period_ms：采样周期，固定值，2ms 或 4ms 或 8ms
    @param  [in] 默认参数 type：数据类型，1-关节位置 默认1
    @param  [in] 默认参数 di_choose：DI 选择,bit0~bit7 对应控制箱 DI0~DI7，bit8~bit9 对应末端DI0~DI1，0-不选择，1-选择 默认0
    @param  [in] 默认参数 do_choose：DO 选择,bit0~bit7 对应控制箱 DO0~DO7，bit8~bit9 对应末端 DO0~DO1，0-不选择，1-选择 默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDStart(self, name, period_ms, type=1, di_choose=0, do_choose=0):
        name = str(name)
        period_ms = int(period_ms)
        type = int(type)
        di_choose = int(di_choose)
        do_choose = int(do_choose)
        error = self.robot.SetTPDStart(type, name, period_ms, di_choose, do_choose)
        return error

    """   
    @brief  停止轨迹记录
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWebTPDStop(self):
        error = self.robot.SetWebTPDStop()
        return error

    """   
    @brief  删除轨迹记录
    @param  [in] 必选参数 name：轨迹名
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDDelete(self, name):
        name = str(name)
        error = self.robot.SetTPDDelete(name)
        return error

    """   
    @brief  轨迹预加载
    @param  [in] 必选参数 name：轨迹名
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadTPD(self, name):
        name = str(name)
        error = self.robot.LoadTPD(name)
        return error

    """   
    @brief  获取轨迹起始位姿
    @param  [in] name 轨迹文件名,不需要文件后缀
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）desc_pose [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetTPDStartPose(self, name):
        name = str(name)
        _error = self.robot.GetTPDStartPose(name)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  轨迹复现
    @param  [in] 必选参数 name：轨迹名
    @param  [in] 必选参数 blend：是否平滑，0-不平滑，1-平滑
    @param  [in] 必选参数 ovl：速度缩放因子，范围 [0~100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveTPD(self, name, blend, ovl):
        name = str(name)
        blend = int(blend)
        ovl = float(ovl)
        error = self.robot.MoveTPD(name, blend, ovl)
        return error

    """   
    @brief  轨迹预处理
    @param  [in] 必选参数 name：轨迹名 如/fruser/traj/trajHelix_aima_1.txt
    @param  [in] 必选参数 ovl 速度缩放百分比，范围[0~100]
    @param  [in] 默认参数 opt 1-控制点，默认为1
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadTrajectoryJ(self, name, ovl, opt=1):
        name = str(name)
        ovl = float(ovl)
        opt = int(opt)
        error = self.robot.LoadTrajectoryJ(name, ovl, opt)
        return error

    """   
    @brief  轨迹复现
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveTrajectoryJ(self):
        error = self.robot.MoveTrajectoryJ()
        return error

    """   
    @brief  获取轨迹起始位姿
    @param  [in] 必选参数 name：轨迹名
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）desc_pose [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetTrajectoryStartPose(self, name):
        name = str(name)
        _error = self.robot.GetTrajectoryStartPose(name)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取轨迹点编号
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）pnum
    """

    @log_call
    @xmlrpc_timeout
    def GetTrajectoryPointNum(self):
        _error = self.robot.GetTrajectoryPointNum()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  设置轨迹运行中的速度
    @param  [in] 必选参数 ovl 速度缩放百分比，范围[0~100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJSpeed(self, ovl):
        ovl = float(ovl)
        error = self.robot.SetTrajectoryJSpeed(ovl)
        return error

    """   
    @brief  设置轨迹运行中的力和扭矩
    @param  [in] 必选参数 ft [fx,fy,fz,tx,ty,tz]，单位N和Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceTorque(self, ft):
        ft = list(map(float, ft))
        error = self.robot.SetTrajectoryJForceTorque(ft)
        return error

    """   
    @brief  设置轨迹运行中的沿x方向的力
    @param  [in] 必选参数 fx 沿x方向的力，单位N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFx(self, fx):
        fx = float(fx)
        error = self.robot.SetTrajectoryJForceFx(fx)
        return error

    """   
    @brief  设置轨迹运行中的沿y方向的力
    @param  [in] 必选参数 fy 沿y方向的力，单位N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFy(self, fy):
        fy = float(fy)
        error = self.robot.SetTrajectoryJForceFy(fy)
        return error

    """   
    @brief  设置轨迹运行中的沿z方向的力
    @param  [in] 必选参数 fz 沿z方向的力，单位N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFz(self, fz):
        fz = float(fz)
        error = self.robot.SetTrajectoryJForceFy(fz)
        return error

    """   
    @brief  设置轨迹运行中的绕x轴的扭矩
    @param  [in] 必选参数 tx 绕x轴的扭矩，单位Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTx(self, tx):
        tx = float(tx)
        error = self.robot.SetTrajectoryJTorqueTx(tx)
        return error

    """   
    @brief  设置轨迹运行中的绕y轴的扭矩
    @param  [in] 必选参数 ty 绕y轴的扭矩，单位Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTy(self, ty):
        ty = float(ty)
        error = self.robot.SetTrajectoryJTorqueTx(ty)
        return error

    """   
    @brief  设置轨迹运行中的绕z轴的扭矩
    @param  [in] 必选参数 tz 绕z轴的扭矩，单位Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTz(self, tz):
        tz = float(tz)
        error = self.robot.SetTrajectoryJTorqueTx(tz)
        return error

    """   
    ***************************************************************************机器人WebAPP程序使用********************************************************************************************
    """

    """   
    @brief  设置开机自动加载默认的作业程序
    @param  [in] 必选参数 flag：0-开机不自动加载默认程序，1-开机自动加载默认程序
    @param  [in] 必选参数 program_name：作业程序名及路径，如“/fruser/movej.lua”，其中“/fruser/”为固定路径
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadDefaultProgConfig(self, flag, program_name):
        flag = int(flag)
        program_name = str(program_name)
        error = self.robot.LoadDefaultProgConfig(flag, program_name)
        return error

    """   
    @brief  加载指定的作业程序
    @param  [in] 必选参数 program_name：作业程序名及路径，如“/fruser/movej.lua”，其中“/fruser/”为固定路径
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramLoad(self, program_name):
        program_name = str(program_name)
        error = self.robot.ProgramLoad(program_name)
        return error

    """   
    @brief  获取当前机器人作业程序的执行行号
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）line_num
    """

    @log_call
    @xmlrpc_timeout
    def GetCurrentLine(self):
        _error = self.robot.GetCurrentLine()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  运行当前加载的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramRun(self):
        error = self.robot.ProgramRun()
        return error

    """   
    @brief  暂停当前运行的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramPause(self):
        error = self.robot.ProgramPause()
        return error

    """   
    @brief  恢复当前暂停的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramResume(self):
        error = self.robot.ProgramResume()
        return error

    """   
    @brief  终止当前运行的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramStop(self):
        error = self.robot.ProgramStop()
        return error

    """   
    @brief  获取机器人作业程序执行状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state:1-程序停止或无程序运行，2-程序运行中，3-程序暂停
    """

    @log_call
    @xmlrpc_timeout
    def GetProgramState(self):
        _error = self.robot.GetProgramState()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  获取已加载的作业程序名
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）program_name
    """

    @log_call
    @xmlrpc_timeout
    def GetLoadedProgram(self):
        _error = self.robot.GetLoadedProgram()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    ***************************************************************************机器人外设********************************************************************************************
    """

    """   
    @brief  获取夹爪配置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[number,company,device,softversion] 
            number 夹爪编号
            company夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行 
            device  设备号，Robotiq(0-2F-85系列)，慧灵(0-NK系列,1-Z-EFG-100)，天机(0-TEG-110)，大寰(0-PGI-140)，知行(0-CTPM2F20)
            softvesion  软件版本号，暂不使用，默认为0
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperConfig(self):
        _error = self.robot.GetGripperConfig()
        error = _error[0]
        if error == 0:
            return error, [_error[1] + 1, _error[2] + 1, _error[3], _error[4]]
        else:
            return error

    """   
    @brief  激活夹爪
    @param  [in] 必选参数 index: 夹爪编号
    @param  [in] 必选参数 action:0-复位，1-激活
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ActGripper(self, index, action):
        index = int(index)
        action = int(action)
        error = self.robot.ActGripper(index, action)
        return error

    """   
    @brief  控制夹爪
    @param  [in] 必选参数 index: 夹爪编号
    @param  [in] 必选参数 pos: 位置百分比，范围 [0~100]
    @param  [in] 必选参数 speed: 速度百分比，范围 [0~100]
    @param  [in] 必选参数 force: 力矩百分比，范围 [0~100]
    @param  [in] 必选参数 maxtime: 最大等待时间，范围 [0~30000]，单位 [ms]
    @param  [in] 必选参数 block:0-阻塞，1-非阻塞
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveGripper(self, index, pos, speed, force, maxtime, block):
        index = int(index)
        pos = int(pos)
        speed = int(speed)
        force = int(force)
        maxtime = int(maxtime)
        block = int(block)
        error = self.robot.MoveGripper(index, pos, speed, force, maxtime, block)
        return error

    """   
    @brief  获取夹爪运动状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）status:0-运动未完成，1-运动完成     
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperMotionDone(self):
        _error = self.robot.GetGripperMotionDone()
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error

    """   
    @brief  配置夹爪
    @param  [in] 必选参数 company：夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行
    @param  [in] 必选参数 device：设备号，Robotiq(0-2F-85 系列)，慧灵 (0-NK 系列,1-Z-EFG-100)，天机 (0-TEG-110)，大寰 (0-PGI-140)，知行 (0-CTPM2F20)
    @param  [in] 默认参数 softversion：软件版本号，暂不使用，默认为 0
    @param  [in] 默认参数 bus：设备挂载末端总线位置，暂不使用，默认为 0；
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetGripperConfig(self, company, device, softversion=0, bus=0):
        company = int(company)
        device = int(device)
        softversion = int(softversion)
        bus = int(bus)
        error = self.robot.SetGripperConfig(company, device, softversion, bus)
        return error

    """   
    @brief  计算预抓取点-视觉
    @param  [in] 必选参数 desc_pos  抓取点笛卡尔位姿
    @param  [in] 必选参数 zlength   z轴偏移量
    @param  [in] 必选参数 zangle    绕z轴旋转偏移量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） pre_pos  预抓取点笛卡尔位姿
    """

    @log_call
    @xmlrpc_timeout
    def ComputePrePick(self, desc_pos, zlength, zangle):
        desc_pos = list(map(float, desc_pos))
        zlength = float(zlength)
        zangle = float(zangle)
        _error = self.robot.ComputePrePick(desc_pos, zlength, zangle)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  计算撤退点-视觉
    @param  [in] 必选参数 desc_pos  抓取点笛卡尔位姿
    @param  [in] 必选参数 zlength   z轴偏移量
    @param  [in] 必选参数 zangle    绕z轴旋转偏移量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） post_pos  撤退点笛卡尔位姿
    """

    @log_call
    @xmlrpc_timeout
    def ComputePostPick(self, desc_pos, zlength, zangle):
        desc_pos = list(map(float, desc_pos))
        zlength = float(zlength)
        zangle = float(zangle)
        _error = self.robot.ComputePostPick(desc_pos, zlength, zangle)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    ***************************************************************************机器人力控********************************************************************************************
    """

    """   
    @brief  获取力传感器配置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[number,company,device,softversion,bus]
            number 传感器编号
            company  力传感器厂商，17-坤维科技，19-航天十一院，20-ATI 传感器，21-中科米点，22-伟航敏芯
            device  设备号，坤维 (0-KWR75B)，航天十一院 (0-MCS6A-200-4)，ATI(0-AXIA80-M8)，中科米点 (0-MST2010)，伟航敏芯 (0-WHC6L-YB10A)
            softvesion  软件版本号，暂不使用，默认为0    
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetConfig(self):
        _error = self.robot.FT_GetConfig()
        error = _error[0]
        if error == 0:
            return error, [_error[1] + 1, _error[2] + 1, _error[3], _error[4]]
        else:
            return error

    """   
    @brief  力传感器配置
    @param  [in] 必选参数 company：传感器厂商，17-坤维科技，19-航天十一院，20-ATI 传感器，21-中科米点，22-伟航敏芯；
    @param  [in] 必选参数 device：设备号，坤维 (0-KWR75B)，航天十一院 (0-MCS6A-200-4)，ATI(0-AXIA80-M8)，中科米点 (0-MST2010)，伟航敏芯 (0-WHC6L-YB10A)；
    @param  [in] 默认参数 softversion：软件版本号，暂不使用，默认为 0
    @param  [in] 默认参数 bus：设备挂载末端总线位置，暂不使用，默认为 0；
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetConfig(self, company, device, softversion=0, bus=0):
        company = int(company)
        device = int(device)
        softversion = int(softversion)
        bus = int(bus)
        error = self.robot.FT_SetConfig(company, device, softversion, bus)
        return error

    """   
    @brief  力传感器激活
    @param  [in] 必选参数 state：0-复位，1-激活
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_Activate(self, state):
        state = int(state)
        error = self.robot.FT_Activate(state)
        return error

    """   
    @brief  力传感器校零
    @param  [in] 必选参数 state：0-去除零点，1-零点矫正
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetZero(self, state):
        state = int(state)
        error = self.robot.FT_SetZero(state)
        return error

    """   
    @brief  设置力传感器参考坐标系
    @param  [in] 必选参数 ref：0-工具坐标系，1-基坐标系
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetRCS(self, ref):
        ref = int(ref)
        error = self.robot.FT_SetRCS(ref)
        return error

    """   
    @brief  负载重量辨识计算
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weight-负载重量，单位 kg
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdIdenCompute(self):
        _error = self.robot.FT_PdIdenCompute()
        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error

    """   
    @brief  负载重量辨识记录
    @param  [in] 必选参数 tool_id：传感器坐标系编号，范围 [1~14]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdIdenRecord(self, tool_id):
        tool_id = int(tool_id)
        error = self.robot.FT_PdIdenRecord(tool_id)
        return error

    """   
    @brief  负载质心辨识计算
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）cog=[cogx,cogy,cogz] ，负载质心，单位 mm
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdCogIdenCompute(self):
        _error = self.robot.FT_PdCogIdenCompute()
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]]
        else:
            return error

    """   
    @brief  负载质心辨识记录
    @param  [in] 必选参数 tool_id：传感器坐标系编号，范围 [0~14]
    @param  [in] 必选参数 index 点编号，范围[1~3]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdCogIdenRecord(self, tool_id, index):
        tool_id = int(tool_id)
        index = int(index)
        error = self.robot.FT_PdCogIdenRecord(tool_id, index)
        return error

    """   
    @brief  获取参考坐标系下力/扭矩数据
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data=[fx,fy,fz,tx,ty,tz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetForceTorqueRCS(self):
        _error = self.robot.FT_GetForceTorqueRCS(0)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  获取力传感器原始力/扭矩数据
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data=[fx,fy,fz,tx,ty,tz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetForceTorqueOrigin(self):
        _error = self.robot.FT_GetForceTorqueOrigin(0)
        error = _error[0]
        return error
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  碰撞守护
    @param  [in] 必选参数 flag：0-关闭碰撞守护，1-开启碰撞守护；
    @param  [in] 必选参数 sensor_num：力传感器编号
    @param  [in] 必选参数 select：六个自由度是否检测碰撞 [fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    @param  [in] 必选参数 force_torque：碰撞检测力/力矩，单位 N 或 Nm
    @param  [in] 必选参数 max_threshold：最大阈值
    @param  [in] 必选参数 min_threshold：最小阈值
    力/力矩检测范围:(force_torque-min_threshold,force_torque+max_threshold)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_Guard(self, flag, sensor_num, select, force_torque, max_threshold, min_threshold):
        flag = int(flag)
        sensor_num = int(sensor_num)
        select = list(map(int, select))
        force_torque = list(map(float, force_torque))
        max_threshold = list(map(float, max_threshold))
        min_threshold = list(map(float, min_threshold))
        error = self.robot.FT_Guard(flag, sensor_num, select, force_torque, max_threshold, min_threshold)
        return error

    """   
    @brief  恒力控制
    @param  [in] 必选参数 flag：0-关闭碰撞守护，1-开启碰撞守护；
    @param  [in] 必选参数 sensor_num：力传感器编号
    @param  [in] 必选参数 select：[fx,fy,fz,mx,my,mz]六个自由度是否检测碰撞 ，0-不生效，1-生效
    @param  [in] 必选参数 force_torque：[fx,fy,fz,mx,my,mz]碰撞检测力/力矩，单位 N 或 Nm
    @param  [in] 必选参数 gain：[f_p,f_i,f_d,m_p,m_i,m_d], 力PID参数，力矩PID参数
    @param  [in] 必选参数 adj_sign：自适应启停状态，0-关闭，1-开启
    @param  [in] 必选参数 ILC_sign: ILC 控制启停状态，0-停止，1-训练，2-实操
    @param  [in] 必选参数 max_dis：最大调整距离，单位mm
    @param  [in] 必选参数 max_ang：最大调整角度，单位deg
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_Control(self, flag, sensor_num, select, force_torque, gain, adj_sign, ILC_sign, max_dis, max_ang):
        flag = int(flag)
        sensor_num = int(sensor_num)
        select = list(map(int, select))
        force_torque = list(map(float, force_torque))
        gain = list(map(float, gain))
        adj_sign = int(adj_sign)
        ILC_sign = int(ILC_sign)
        max_dis = float(max_dis)
        max_ang = float(max_ang)
        error = self.robot.FT_Control(flag, sensor_num, select, force_torque, gain, adj_sign, ILC_sign, max_dis,
                                      max_ang)
        return error

    """   
    @brief  螺旋线探索
    @param  [in] 必选参数 rcs 参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 ft：力或力矩阈值 (0~100)，单位 N 或 Nm
    @param  [in] 默认参数 dr：每圈半径进给量，单位 mm 默认0.7
    @param  [in] 默认参数 max_t_ms：最大探索时间，单位 ms 默认 60000
    @param  [in] 默认参数 max_vel：线速度最大值，单位 mm/s 默认 5
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SpiralSearch(self, rcs, ft, dr=0.7, max_t_ms=60000, max_vel=5):
        rcs = int(rcs)
        ft = float(ft)
        dr = float(dr)
        max_t_ms = float(max_t_ms)
        max_vel = float(max_vel)
        error = self.robot.FT_SpiralSearch(rcs, ft, dr, max_t_ms, max_vel)
        return error

    """   
    @brief  旋转插入
    @param  [in] 必选参数 rcs 参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 ft：力或力矩阈值 (0~100)，单位 N 或 Nm
    @param  [in] 必选参数 orn 力/扭矩方向，1-沿z轴方向，2-绕z轴方向
    @param  [in] 默认参数 angVelRot：旋转角速度，单位 °/s  默认 3
    @param  [in] 默认参数 angleMax：最大旋转角度，单位 ° 默认 5
    @param  [in] 默认参数 angAccmax：最大旋转加速度，单位 °/s^2，暂不使用 默认0
    @param  [in] 默认参数 rotorn：旋转方向，1-顺时针，2-逆时针 默认1
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_RotInsertion(self, rcs, ft, orn, angVelRot=3, angleMax=45, angAccmax=0, rotorn=1):
        rcs = int(rcs)
        ft = float(ft)
        orn = int(orn)
        angVelRot = float(angVelRot)
        angleMax = float(angleMax)
        angAccmax = float(angAccmax)
        rotorn = int(rotorn)
        error = self.robot.FT_RotInsertion(rcs, angVelRot, ft, angleMax, orn, angAccmax, rotorn)
        return error

    """   
    @brief  直线插入
    @param  [in] 必选参数 rcs 参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 ft：力或力矩阈值 (0~100)，单位 N 或 Nm
    @param  [in] 必选参数 disMax：最大插入距离，单位 mm
    @param  [in] 必选参数 linorn：插入方向:0-负方向，1-正方向
    @param  [in] 默认参数 lin_v：直线速度，单位 mm/s 默认1
    @param  [in] 默认参数 lin_a：直线加速度，单位 mm/s^2，暂不使用 默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_LinInsertion(self, rcs, ft, disMax, linorn, lin_v=1.0, lin_a=1.0):
        rcs = int(rcs)
        ft = float(ft)
        disMax = float(disMax)
        linorn = int(linorn)
        lin_v = float(lin_v)
        lin_a = float(lin_a)
        error = self.robot.FT_LinInsertion(rcs, ft, lin_v, lin_a, disMax, linorn)
        return error

    """   
    @brief  计算中间平面位置开始
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_CalCenterStart(self):
        error = self.robot.FT_CalCenterStart()
        return error

    """   
    @brief  计算中间平面位置结束
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）pos=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_CalCenterEnd(self):
        _error = self.robot.FT_CalCenterEnd()
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error

    """   
    @brief  表面定位
    @param  [in] 必选参数 rcs：参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 dir：移动方向，1-正方向，2-负方向
    @param  [in] 必选参数 axis：移动轴，1-x，2-y，3-z
    @param  [in] 必选参数 disMax：最大探索距离，单位 mm
    @param  [in] 必选参数 ft：动作终止力阈值，单位 N
    @param  [in] 默认参数 lin_v：探索直线速度，单位 mm/s 默认3
    @param  [in] 默认参数 lin_a：探索直线加速度，单位 mm/s^2 默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_FindSurface(self, rcs, dir, axis, disMax, ft, lin_v=3.0, lin_a=0.0):
        rcs = int(rcs)
        dir = int(dir)
        axis = int(axis)
        ft = float(ft)
        lin_v = float(lin_v)
        lin_a = float(lin_a)
        error = self.robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, disMax, ft)
        return error

    """   
    @brief  柔顺控制关闭
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_ComplianceStop(self):
        error = self.robot.FT_ComplianceStop()
        return error

    """   
    @brief  柔顺控制开启
    @param  [in] 必选参数 p: 位置调节系数或柔顺系数
    @param  [in] 必选参数 force：柔顺开启力阈值，单位 N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_ComplianceStart(self, p, force):
        p = float(p)
        force = float(force)
        error = self.robot.FT_ComplianceStart(p, force)
        return error

    """   
    @brief  负载辨识滤波初始化
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyDynFilterInit(self):
        error = self.robot.LoadIdentifyDynFilterInit()
        return error

    """   
    @brief  负载辨识变量初始化
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyDynVarInit(self):
        error = self.robot.LoadIdentifyDynVarInit()
        return error

    """   
    @brief  负载辨识主程序
    @param  [in] 必选参数 joint_torque 关节扭矩 j1-j6
    @param  [in] 必选参数 joint_pos 关节位置 j1-j6
    @param  [in] 必选参数 t 采样周期
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyMain(self, joint_torque, joint_pos, t):
        joint_torque = list(map(float, joint_torque))
        joint_pos = list(map(float, joint_pos))
        error = self.robot.LoadIdentifyMain(joint_torque, joint_pos, t)
        return error

    """   
    @brief  获取负载辨识结果
    @param  [in] 必选参数 gain  
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weight 负载重量
    @return 返回值（调用成功返回）cog 负载质心 [x,y,z]
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyGetResult(self, gain):
        gain = list(map(float, gain))
        _error = self.robot.LoadIdentifyGetResult(gain)
        error = _error[0]
        if error == 0:
            return error, _error[1], [_error[2], _error[3], _error[4]]
        else:
            return error

    """   
    ***************************************************************************传送带功能********************************************************************************************
    """

    """   
    @brief  传动带启动、停止
    @param  [in] 必选参数 status 状态，1-启动，0-停止 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorStartEnd(self, status):
        status = int(status)
        error = self.robot.ConveyorStartEnd(status)
        return error

    """   
    @brief  记录IO检测点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointIORecord(self):
        error = self.robot.ConveyorPointIORecord()
        return error

    """   
    @brief  记录A点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointARecord(self):
        error = self.robot.ConveyorPointARecord()
        return error

    """   
    @brief  记录参考点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorRefPointRecord(self):
        error = self.robot.ConveyorRefPointRecord()
        return error

    """   
    @brief  记录B点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointBRecord(self):
        error = self.robot.ConveyorPointBRecord()
        return error

    """   
    @brief  传送带工件IO检测
    @param  [in] 必选参数 max_t 最大检测时间，单位ms
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorIODetect(self, max_t):
        max_t = int(max_t)
        error = self.robot.ConveyorIODetect(max_t)
        return error

    """   
    @brief  获取物体当前位置
    @param  [in] 必选参数  mode 1-跟踪抓取 2-跟踪运动 3-TPD跟踪
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorGetTrackData(self, mode):
        mode = int(mode)
        error = self.robot.ConveyorGetTrackData(mode)
        return error

    """   
    @brief  传动带跟踪开始
    @param  [in] 必选参数  status 状态，1-启动，0-停止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackStart(self, status):
        status = int(status)
        error = self.robot.ConveyorTrackStart(status)
        return error

    """   
    @brief  传动带跟踪停止
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackEnd(self):
        error = self.robot.ConveyorTrackEnd()
        return error

    """   
    @brief  传动带参数配置
    @param  [in] 必选参数  param = [encChannel,resolution,lead,wpAxis,vision,speedRadio]  encChannel编码器通道 1-2,resolution 编码器分辨率 编码器旋转一圈脉冲个数,
    lead机械传动比 编码器旋转一圈传送带移动距离,wpAxis  工件坐标系编号 针对跟踪运动功能选择工件坐标系编号，跟踪抓取、TPD跟踪设为0,vision 是否配视觉  0 不配  1 配,
    speedRadio 速度比  针对传送带跟踪抓取速度范围为（1-100）  跟踪运动、TPD跟踪设置为1
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorSetParam(self, param):
        param = list(map(float, param))
        error = self.robot.ConveyorSetParam(param)
        return error

    """   
    @brief  传动带抓取点补偿
    @param  [in] 必选参数 cmp 补偿位置 [x,y,z]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorCatchPointComp(self, cmp):
        cmp = list(map(float, cmp))
        error = self.robot.ConveyorCatchPointComp(cmp)
        return error

    """   
    @brief  直线运动
    @param  [in] 必选参数  name cvrCatchPoint和cvrRaisePoint
    @param  [in] 必选参数 tool 工具号
    @param  [in] 必选参数 wobj 工件号
    @param  [in] 默认参数 vel 速度 默认20
    @param  [in] 默认参数 acc 加速度 默认100
    @param  [in] 默认参数 ovl 速度缩放因子 默认100
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackMoveL(self, name, tool, wobj, vel=20, acc=100, ovl=100, blendR=-1.0):
        name = str(name)
        tool = int(tool)
        wobj = int(wobj)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        error = self.robot.ConveyorTrackMoveL(name, tool, wobj, vel, acc, ovl, blendR, 0, 0)
        return error

    """   
    @brief  焊接开始 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 arcNum 焊机配置文件编号
    @param  [in] 必选参数 timeout 起弧超时时间
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ARCStart(self, ioType, arcNum, timeout):
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)
        error = self.robot.ARCStart(ioType, arcNum, timeout)
        return error

    """   
    @brief  焊接结束 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 arcNum 焊机配置文件编号
    @param  [in] 必选参数 timeout 熄弧超时时间
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ARCEnd(self, ioType, arcNum, timeout):
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)
        error = self.robot.ARCEnd(ioType, arcNum, timeout)
        return error

    """   
    @brief  设置焊接电流与输出模拟量对应关系 
    @param  [in] 必选参数 currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
    @param  [in] 必选参数 currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
    @param  [in] 必选参数 outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @param  [in] 必选参数 outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrentRelation(self, currentMin, currentMax, outputVoltageMin, outputVoltageMax):
        currentMin = float(currentMin)
        currentMax = float(currentMax)
        outputVoltageMin = float(outputVoltageMin)
        outputVoltageMax = float(outputVoltageMax)
        error = self.robot.WeldingSetCurrentRelation(currentMin, currentMax, outputVoltageMin, outputVoltageMax)
        return error

    """   
    @brief  设置焊接电压与输出模拟量对应关系 
    @param  [in] 必选参数 weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
    @param  [in] 必选参数 weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
    @param  [in] 必选参数 outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @param  [in] 必选参数 outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltageRelation(self, weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax):
        weldVoltageMin = float(weldVoltageMin)
        weldVoltageMax = float(weldVoltageMax)
        outputVoltageMin = float(outputVoltageMin)
        outputVoltageMax = float(outputVoltageMax)

        error = self.robot.WeldingSetVoltageRelation(weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax)
        return error

    """   
    @brief  获取焊接电流与输出模拟量对应关系 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
    @return 返回值（调用成功返回）currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
    @return 返回值（调用成功返回）outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @return 返回值（调用成功返回）outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetCurrentRelation(self):

        try:
            _error = self.robot.WeldingGetCurrentRelation()
            error = _error[0]
            if error == 0:
                return error, _error[1], _error[2], _error[3], _error[4]
            return _error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  获取焊接电压与输出模拟量对应关系 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
    @return 返回值（调用成功返回）weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
    @return 返回值（调用成功返回）outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @return 返回值（调用成功返回）outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetVoltageRelation(self):

        try:
            _error = self.robot.WeldingGetVoltageRelation()
            error = _error[0]
            if error == 0:
                return error, _error[1], _error[2], _error[3], _error[4]
            return _error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  设置焊接电流 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 float current 焊接电流值(A)
    @param  [in] 必选参数 int AOIndex 焊接电流控制箱模拟量输出端口(0-1)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrent(self, ioType, current, AOIndex):
        ioType = int(ioType)
        current = float(current)
        AOIndex = int(AOIndex)
        error = self.robot.WeldingSetCurrent(ioType, current, AOIndex)
        return error

    """   
    @brief  设置焊接电压 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 float voltage 焊接电压值(A)
    @param  [in] 必选参数 int AOIndex 焊接电压控制箱模拟量输出端口(0-1)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltage(self, ioType, voltage, AOIndex):
        ioType = int(ioType)
        voltage = float(voltage)
        AOIndex = int(AOIndex)
        error = self.robot.WeldingSetVoltage(ioType, voltage, AOIndex)
        return error

    """   
    @brief  设置摆动参数 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @param  [in] 必选参数 int weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
    @param  [in] 必选参数 float weaveFrequency 摆动频率(Hz)
    @param  [in] 必选参数 int weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
    @param  [in] 必选参数 float weaveRange 摆动幅度(mm)
    @param  [in] 必选参数 int weaveLeftStayTime 摆动左停留时间(ms)
    @param  [in] 必选参数 int weaveRightStayTime 摆动右停留时间(ms)
    @param  [in] 必选参数 int weaveCircleRadio 圆形摆动-回调比率(0-100%)
    @param  [in] 必选参数 int weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveSetPara(self, weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime,
                     weaveRightStayTime, weaveCircleRadio, weaveStationary):
        weaveNum = int(weaveNum)
        weaveType = int(weaveType)
        weaveFrequency = float(weaveFrequency)
        weaveIncStayTime = int(weaveIncStayTime)
        weaveRange = float(weaveRange)
        weaveLeftStayTime = int(weaveLeftStayTime)
        weaveRightStayTime = int(weaveRightStayTime)
        weaveCircleRadio = int(weaveCircleRadio)
        weaveStationary = int(weaveStationary)
        try:
            error = self.robot.WeaveSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                                            weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary)
            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  即时设置摆动参数 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @param  [in] 必选参数 int weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-
    @param  [in] 必选参数 float weaveFrequency 摆动频率(Hz)
    @param  [in] 必选参数 int weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
    @param  [in] 必选参数 float weaveRange 摆动幅度(mm)
    @param  [in] 必选参数 int weaveLeftStayTime 摆动左停留时间(ms)
    @param  [in] 必选参数 int weaveRightStayTime 摆动右停留时间(ms)
    @param  [in] 必选参数 int weaveCircleRadio 圆形摆动-回调比率(0-100%)
    @param  [in] 必选参数 int weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveOnlineSetPara(self, weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime,
                           weaveRightStayTime, weaveCircleRadio, weaveStationary):
        weaveNum = int(weaveNum)
        weaveType = int(weaveType)
        weaveFrequency = float(weaveFrequency)
        weaveIncStayTime = int(weaveIncStayTime)
        weaveRange = float(weaveRange)
        weaveLeftStayTime = int(weaveLeftStayTime)
        weaveRightStayTime = int(weaveRightStayTime)
        weaveCircleRadio = int(weaveCircleRadio)
        weaveStationary = int(weaveStationary)
        try:
            error = self.robot.WeaveOnlineSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                                                  weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio,
                                                  weaveStationary)
            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  摆动开始 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveStart(self, weaveNum):
        weaveNum = int(weaveNum)
        try:
            error = self.robot.WeaveStart(weaveNum)
            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  摆动结束 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveEnd(self, weaveNum):
        weaveNum = int(weaveNum)
        try:
            error = self.robot.WeaveEnd(weaveNum)
            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  正向送丝 
    @param  [in] 必选参数 int ioType io类型  0-控制器IO；1-扩展IO
    @param  [in] 必选参数 int wireFeed 送丝控制  0-停止送丝；1-送丝
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetForwardWireFeed(self, ioType, wireFeed):
        ioType = int(ioType)
        wireFeed = int(wireFeed)
        try:
            error = self.robot.SetForwardWireFeed(ioType, wireFeed)
            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  反向送丝 
    @param  [in] 必选参数 int ioType io类型  0-控制器IO；1-扩展IO
    @param  [in] 必选参数 int wireFeed 送丝控制  0-停止送丝；1-送丝
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetReverseWireFeed(self, ioType, wireFeed):
        ioType = int(ioType)
        wireFeed = int(wireFeed)
        try:
            error = self.robot.SetReverseWireFeed(ioType, wireFeed)
            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  送气
    @param  [in] 必选参数 int ioType io类型  0-控制器IO；1-扩展IO
    @param  [in] 必选参数 int airControl 送气控制  0-停止送气；1-送气
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAspirated(self, ioType, airControl):
        ioType = int(ioType)
        airControl = int(airControl)
        try:
            error = self.robot.SetAspirated(ioType, airControl)
            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  分段焊接启动
    @param  [in] 必选参数 startDesePos: 初始笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 endDesePos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 startJPos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 endJPos: 目标关节位置，单位 [°] 
    @param  [in] 必选参数 weldLength: 焊接长度，单位 [mm]
    @param  [in] 必选参数 noWeldLength: 非焊接长度，单位 [mm]    
    @param  [in] 必选参数 weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
    @param  [in] 必选参数 arcNum 焊机配置文件编号
    @param  [in] 必选参数 timeout 熄弧超时时间
    @param  [in] 必选参数 isWeave true-焊接 false-不焊接
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 search:[0]-不焊丝寻位，[1]-焊丝寻位
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SegmentWeldStart(self, startDesePos, endDesePos, startJPos, endJPos, weldLength, noWeldLength, weldIOType,
                         arcNum, weldTimeout, isWeave, weaveNum, tool, user,
                         vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0,
                         offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        startDesePos = list(map(float, startDesePos))
        endDesePos = list(map(float, endDesePos))
        startJPos = list(map(float, startJPos))
        endJPos = list(map(float, endJPos))

        weldLength = float(weldLength)
        noWeldLength = float(noWeldLength)
        weldIOType = int(weldIOType)
        arcNum = int(arcNum)
        weldTimeout = int(weldTimeout)
        isWeave = bool(isWeave)
        weaveNum = int(weaveNum)
        tool = int(tool)
        user = int(user)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        rtn = 0
        # 获取起点到终点之间的距离和各方向角度余弦值
        result = self.robot.GetSegWeldDisDir(startDesePos[0], startDesePos[1], startDesePos[2], endDesePos[0],
                                             endDesePos[1], endDesePos[2])
        if result[0] != 0:
            return int(result[0])

        distance = result[1]
        directionX = result[2]
        directionY = result[3]
        directionZ = result[4]
        endOffPos = list(offset_pos)

        rtn = self.robot.MoveJ(startJPos, startDesePos, tool, user, vel, acc, ovl, exaxis_pos, blendR, offset_flag,
                               offset_pos)
        if rtn != 0:
            return rtn

        weldNum = 0
        noWeldNum = 0
        i = 0
        while i < int(distance / (weldLength + noWeldLength)) * 2 + 2:
            if i % 2 == 0:
                weldNum += 1
                if weldNum * weldLength + noWeldNum * noWeldLength > distance:
                    endOffPos[0] = offset_pos[0] + distance * directionX;
                    endOffPos[1] = offset_pos[1] + distance * directionY;
                    endOffPos[2] = offset_pos[2] + distance * directionZ;
                    endOffPos[3] = offset_pos[4];
                    endOffPos[4] = offset_pos[4];
                    endOffPos[5] = offset_pos[5];

                    rtn = self.robot.ARCStart(weldIOType, arcNum, weldTimeout)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveStart(weaveNum)
                        if rtn != 0:
                            return rtn
                    rtn = self.robot.MoveL(startJPos, startDesePos, tool, user, vel, acc, ovl, blendR, exaxis_pos,
                                           search, 1, endOffPos)
                    if rtn != 0:
                        self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                        if isWeave:
                            rtn = self.robot.WeaveEnd(weaveNum)
                            if rtn != 0:
                                return rtn
                        return rtn
                    rtn = self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveEnd(weaveNum)
                        if rtn != 0:
                            return rtn
                    break
                else:
                    endOffPos[0] = offset_pos[0] + (weldNum * weldLength + noWeldNum * noWeldLength) * directionX
                    endOffPos[1] = offset_pos[1] + (weldNum * weldLength + noWeldNum * noWeldLength) * directionY
                    endOffPos[2] = offset_pos[2] + (weldNum * weldLength + noWeldNum * noWeldLength) * directionZ
                    endOffPos[3] = offset_pos[3]
                    endOffPos[4] = offset_pos[4]
                    endOffPos[5] = offset_pos[5]
                    rtn = self.robot.ARCStart(weldIOType, arcNum, weldTimeout)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveStart(weaveNum)
                        if rtn != 0:
                            return rtn
                    rtn = self.robot.MoveL(startJPos, startDesePos, tool, user, vel, acc, ovl, blendR, exaxis_pos,
                                           search, 1, endOffPos)
                    if rtn != 0:
                        self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                        if isWeave:
                            rtn = self.robot.WeaveEnd(weaveNum)
                            if rtn != 0:
                                return rtn
                        return rtn
                    rtn = self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveEnd(weaveNum)
                        if rtn != 0:
                            return rtn
            else:
                noWeldNum += 1
                if weldNum * weldLength + noWeldNum * noWeldLength > distance:
                    endOffPos[0] = offset_pos[0] + distance * directionX
                    endOffPos[1] = offset_pos[1] + distance * directionY
                    endOffPos[2] = offset_pos[2] + distance * directionZ
                    endOffPos[3] = offset_pos[3]
                    endOffPos[4] = offset_pos[4]
                    endOffPos[5] = offset_pos[5]
                    rtn = self.robot.MoveL(startJPos, startDesePos, tool, user, vel, acc, ovl, blendR, exaxis_pos,
                                           search, 1, endOffPos)
                    if rtn != 0:
                        return rtn
                    break
                else:
                    endOffPos[0] = offset_pos[0] + (weldNum * weldLength + noWeldNum * noWeldLength) * directionX
                    endOffPos[1] = offset_pos[1] + (weldNum * weldLength + noWeldNum * noWeldLength) * directionY
                    endOffPos[2] = offset_pos[2] + (weldNum * weldLength + noWeldNum * noWeldLength) * directionZ
                    endOffPos[3] = offset_pos[3]
                    endOffPos[4] = offset_pos[4]
                    endOffPos[5] = offset_pos[5]
                    rtn = self.robot.MoveL(startJPos, startDesePos, tool, user, vel, acc, ovl, blendR, exaxis_pos,
                                           search, 1, endOffPos)
                    if rtn != 0:
                        return rtn
            i += 1
        return rtn

    """   
    @brief  分段焊接终止
    @param  [in] 必选参数 ioType
    @param  [in] 必选参数 arcNum
    @param  [in] 必选参数 timeout

    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SegmentWeldEnd(self, ioType, arcNum, timeout):
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)

        rtn = self.robot.SegmentWeldEnd(ioType, arcNum, timeout)
        return rtn

    """   
    @brief  初始化日志参数
    @param  [in]默认参数 output_model：输出模式，0-直接输出；1-缓冲输出；2-异步输出，默认1
    @param  [in]默认参数 file_path： 文件保存路径+名称，名称必须是xxx.log的形式，比如/home/fr/linux/fairino.log。
                    默认执行程序所在路径，默认名称fairino_ year+month+data.log(如:fairino_2024_03_13.log);
    @param  [in]默认参数 file_num：滚动存储的文件数量，1~20个，默认值为5。单个文件上限50M;
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoggerInit(self, output_model=1, file_path="", file_num=5):
        return self.setup_logging(output_model, file_path, file_num)

    """   
    @brief  设置日志过滤等级
    @param  [in] 默认参数 lvl: 过滤等级值，值越小输出日志越少, 1-error, 2-warnning, 3-inform, 4-debug,默认值是1.
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoggerLevel(self, lvl=1):
        log_level = self.set_log_level(lvl)
        return 0

    """   
    @brief  下载点位表数据库
    @param  [in] pointTableName 要下载的点位表名称    pointTable1.db
    @param  [in] saveFilePath 下载点位表的存储路径   C://test/
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointTableDownLoad(self, point_table_name, save_file_path):
        if not os.path.exists(save_file_path):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND

        rtn = self.robot.PointTableDownload(point_table_name)
        if rtn == -1:
            return RobotError.ERR_POINTTABLE_NOTFOUND
        elif rtn != 0:
            return rtn
        port = 20011
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)
        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        total_buffer = bytearray(1024 * 1024 * 50)  # 50Mb
        total_size = 0
        recv_md5 = ""
        recv_size = 0
        find_head_flag = False
        while True:
            buffer = client.recv(1024)
            length = len(buffer)
            if length < 1:
                return RobotError.ERR_OTHER
            total_buffer[total_size:total_size + len(buffer)] = buffer

            total_size += len(buffer)
            if not find_head_flag and total_size > 4 and total_buffer[:4].decode('utf-8') == "/f/b":
                find_head_flag = True
            # 找到文件头后，提取文件大小和MD5校验码。文件大小的信息位于总数据的第5到第12个字节，MD5校验码的信息位于总数据的第13到第44个字节。
            if find_head_flag and total_size > 12 + 32:
                recv_size = int(total_buffer[4:12].decode('utf-8'))
                recv_md5 = total_buffer[12:44].decode('utf-8')
            # 接收到整个文件跳出循环
            if find_head_flag and total_size == recv_size:
                break
        if total_size == 0:
            return RobotError.ERR_OTHER
        file_buffer = total_buffer[12 + 32:total_size - 4]

        with open(os.path.join(save_file_path, point_table_name), 'wb') as file_writer:
            file_writer.write(file_buffer[:total_size - 16 - 32])

        check_md5 = calculate_file_md5(save_file_path + point_table_name)
        if check_md5 == recv_md5:
            client.send("SUCCESS".encode('utf-8'))
            return 0
        else:
            client.send("FAIL".encode('utf-8'))
            os.remove(os.path.join(save_file_path, point_table_name))
            return RobotError.ERR_OTHER

    """   
    @brief  上传点位表数据库
    @param  [in] pointTableFilePath 上传点位表的全路径名   C://test/pointTable1.db
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointTableUpLoad(self, point_table_file_path):
        MAX_UPLOAD_FILE_SIZE = 2 * 1024 * 1024;  # 最大上传文件为2Mb
        # 判断上传文件是否存在
        if not os.path.exists(point_table_file_path):
            return RobotError.ERR_UPLOAD_FILE_NOT_FOUND

        file_info = os.path.getsize(point_table_file_path)
        total_size = file_info + 16 + 32
        if total_size > MAX_UPLOAD_FILE_SIZE:
            print("Files larger than 2 MB are not supported!")
            return -1

        point_table_name = os.path.basename(point_table_file_path)

        rtn = self.robot.PointTableUpload(point_table_name)
        if rtn != 0:
            return rtn

        port = 20010

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)

        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER

        client.settimeout(2)

        # client.receive_timeout = 2000
        # client.send_timeout = 2000

        send_md5 = calculate_file_md5(point_table_file_path)

        head_data = f"/f/b{total_size:08d}{send_md5}"
        num = client.send(head_data.encode('utf-8'))
        if num < 1:
            return RobotError.ERR_OTHER

        with open(point_table_file_path, 'rb') as fs:
            file_bytes = fs.read()

        num = client.send(file_bytes)
        if num < 1:
            return RobotError.ERR_OTHER
        end_data = "/b/f"
        num = client.send(end_data.encode('utf-8'))
        if num < 1:
            return RobotError.ERR_OTHER

        result_buf = client.recv(1024)
        if result_buf[:7].decode('utf-8') == "SUCCESS":
            return RobotError.ERR_SUCCESS
        else:
            return RobotError.ERR_OTHER

    """   
    @brief  点位表切换
    @param  [in] PointTableSwitch 要切换的点位表名称   "pointTable1.db",当点位表为空，即""时，表示将lua程序更新为未应用点位表的初始程序
    @return 错误码 成功-0   失败-错误码 
    @return 错误errorStr
    """

    @log_call
    @xmlrpc_timeout
    def PointTableSwitch(self, point_table_name):
        rtn = self.robot.PointTableSwitch(point_table_name)  # 切换点位表
        if rtn != 0:
            if rtn == RobotError.ERR_POINTTABLE_NOTFOUND:
                error_str = "PointTable not Found!"
            else:
                error_str = "PointTable not Found!"
            return rtn, error_str

    """   
    @brief  点位表更新lua文件
    @param  [in] pointTableName 要切换的点位表名称   "pointTable1.db",当点位表为空，即""时，表示将lua程序更新为未应用点位表的初始程序
    @param  [in] luaFileName 要更新的lua文件名称   "testPointTable.lua"
    @return 错误码 成功-0   失败-错误码 
    @return 错误errorStr
    """

    @log_call
    @xmlrpc_timeout
    def PointTableUpdateLua(self, point_table_name, lua_file_name):
        try:

            rtn = self.robot.PointTableSwitch(point_table_name)  # 切换点位表
            if rtn != 0:
                if rtn == RobotError.ERR_POINTTABLE_NOTFOUND:
                    error_str = "PointTable not Found!"
                else:
                    error_str = "PointTable not Found!"
                return rtn, error_str

            time.sleep(0.3)  # 增加延时确保切换后后端确实收到切换后的点位表名称

            result = self.robot.PointTableUpdateLua(lua_file_name)
            error_str = result[1]
            if not error_str:
                error_str = "fail to update lua, please inspect pointtable"
            return result[0], error_str

        except Exception as e:
            return RobotError.ERR_RPC_ERROR, ""

    """   
    @brief  下载文件
    @param  [in] fileType 文件类型    0-lua文件
    @param  [in] fileName 文件名称    “test.lua”
    @param  [in] saveFilePath 保存文件路径    “C：//test/”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def __FileDownLoad(self, fileType, fileName, saveFilePath):
        if not os.path.exists(saveFilePath):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND
        rtn = self.robot.FileDownload(fileType, fileName)
        if rtn == -1:
            return RobotError.ERR_POINTTABLE_NOTFOUND
        elif rtn != 0:
            return rtn

        port = 20011
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)

        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        total_buffer = bytearray(1024 * 1024 * 50)  # 50Mb
        total_size = 0
        recv_md5 = ""
        recv_size = 0
        find_head_flag = False
        while True:
            buffer = client.recv(1024)
            length = len(buffer)
            if length < 1:
                return RobotError.ERR_OTHER
            total_buffer[total_size:total_size + len(buffer)] = buffer

            total_size += len(buffer)
            if not find_head_flag and total_size > 4 and total_buffer[:4].decode('utf-8') == "/f/b":
                find_head_flag = True
            # 找到文件头后，提取文件大小和MD5校验码。文件大小的信息位于总数据的第5到第12个字节，MD5校验码的信息位于总数据的第13到第44个字节。
            if find_head_flag and total_size > 12 + 32:
                recv_size = int(total_buffer[4:12].decode('utf-8'))
                recv_md5 = total_buffer[12:44].decode('utf-8')
            # 接收到整个文件跳出循环
            if find_head_flag and total_size == recv_size:
                break
        if total_size == 0:
            return RobotError.ERR_OTHER
        file_buffer = total_buffer[12 + 32:total_size - 4]

        with open(os.path.join(saveFilePath, fileName), 'wb') as file_writer:
            file_writer.write(file_buffer[:total_size - 16 - 32])

        check_md5 = calculate_file_md5(saveFilePath + fileName)
        if check_md5 == recv_md5:
            client.send("SUCCESS".encode('utf-8'))
            return 0
        else:
            client.send("FAIL".encode('utf-8'))
            os.remove(os.path.join(saveFilePath, fileName))
            return RobotError.ERR_OTHER

    """   
    @brief  上传文件
    @param  [in] fileType 文件类型    0-lua文件
    @param  [in] filePath上传文件的全路径名    C://test/test.lua     
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def __FileUpLoad(self, fileType, filePath):

        if not os.path.exists(filePath):
            return RobotError.ERR_POINTTABLE_NOTFOUND

        MAX_UPLOAD_FILE_SIZE = 500 * 1024 * 1024;  # 最大上传文件为500Mb
        file_info = os.path.getsize(filePath)
        total_size = file_info + 46 + 4
        if total_size > MAX_UPLOAD_FILE_SIZE:
            print("Files larger than 500 MB are not supported!")
            return -1
        file_name = os.path.basename(filePath)
        rtn = self.robot.FileUpload(fileType, file_name)
        if rtn != 0:
            return rtn

        port = 20010

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(20)

        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        client.settimeout(20)

        send_md5 = calculate_file_md5(filePath)
        head_data = f"/f/b{total_size:10d}{send_md5}"
        num = client.send(head_data.encode('utf-8'))

        if num < 1:
            return RobotError.ERR_OTHER

        with open(filePath, "rb") as f:
            while True:
                data = f.read(2 * 1024 * 1024)
                if not data:  # 如果读取到文件末尾
                    end_data = "/b/f"
                    num = client.send(end_data.encode('utf-8'))  # 发送文件传输完成的标志
                    if num < 1:
                        return RobotError.ERR_OTHER
                    break  # 跳出循环
                num = client.send(data)  # 将读取的数据通过socket连接发送给客户端
                if num < 1:
                    return RobotError.ERR_OTHER
        time.sleep(0.5)
        result_buf = client.recv(1024)
        if result_buf[:7].decode('utf-8') == "SUCCESS":
            return RobotError.ERR_SUCCESS
        else:
            return RobotError.ERR_OTHER

    """   
    @brief  删除文件
    @param  [in] fileType 文件类型    0-lua文件
    @param  [in] fileName 文件名称    “test.lua”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def __FileDelete(self, fileType, fileName):
        rtn = self.robot.FileDelete(fileType, fileName)
        return rtn

    """   
    @brief  下载Lua文件
    @param  [in] fileName 要下载的lua文件名“test.lua”
    @param  [in] savePath 保存文件本地路径“D://Down/”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LuaDownLoad(self, fileName, savePath):
        error = self.__FileDownLoad(0, fileName, savePath)
        return error

    """   
    @brief  上传Lua文件
    @param  [in] filePath上传文件的全路径名   C://test/test.lua  
    @return 错误码 成功-0  失败-错误码
    """

    def LuaUpload(self, filePath):
        error = self.__FileUpLoad(0, filePath)
        if error == 0:
            file_name = os.path.basename(filePath)
            _error = self.robot.LuaUpLoadUpdate(file_name)
            tmp_error = _error[0]
            if tmp_error == 0:
                return tmp_error
            else:
                return tmp_error, _error[1]
        else:
            return error

    """   
    @brief  删除Lua文件
    @param  [in] fileName 要删除的lua文件名“test.lua”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LuaDelete(self, fileName):
        error = self.__FileDelete(0, fileName)
        return error

    """   
    @brief  获取当前所有lua文件名称
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） luaNames lua文件名列表
    """

    @log_call
    @xmlrpc_timeout
    def GetLuaList(self):
        _error = self.robot.GetLuaList()
        size = len(_error)
        error = _error[0]
        if _error[0] == 0:
            lua_num = _error[1]
            lua_name = _error[2].split(';')
            return error, lua_num, lua_name
        else:
            return error

    """   
    @brief  设置485扩展轴参数
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 int servoCompany 伺服驱动器厂商，1-戴纳泰克
    @param  [in] 必选参数 int servoModel 伺服驱动器型号，1-FD100-750C
    @param  [in] 必选参数 int servoSoftVersion 伺服驱动器软件版本，1-V1.0
    @param  [in] 必选参数 int servoResolution 编码器分辨率
    @param  [in] 必选参数 float axisMechTransRatio 机械传动比  
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetParam(self, servoId, servoCompany, servoModel, servoSoftVersion, servoResolution,
                         axisMechTransRatio):
        servoId = int(servoId)
        servoCompany = int(servoCompany)
        servoModel = int(servoModel)
        servoSoftVersion = int(servoSoftVersion)
        servoResolution = int(servoResolution)
        axisMechTransRatio = float(axisMechTransRatio)
        error = self.robot.AuxServoSetParam(servoId, servoCompany, servoModel, servoSoftVersion, servoResolution,
                                            axisMechTransRatio)
        return error

    """   
    @brief  获取485扩展轴配置参数
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） int servoCompany 伺服驱动器厂商，1-戴纳泰克
    @return 返回值（调用成功返回） servoModel 伺服驱动器型号，1-FD100-750C 
    @return 返回值（调用成功返回） servoSoftVersion 伺服驱动器软件版本，1-V1.0
    @return 返回值（调用成功返回） int servoResolution 编码器分辨率
    @return 返回值（调用成功返回） float axisMechTransRatio 机械传动比
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetParam(self, servoId):
        servoId = int(servoId)
        _error = self.robot.AuxServoGetParam(servoId)
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5]
        else:
            return error

    """   
    @brief  设置485扩展轴使能/去使能
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 int status 使能状态，0-去使能， 1-使能
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoEnable(self, servoId, status):
        servoId = int(servoId)
        status = int(status)
        error = self.robot.AuxServoEnable(servoId, status)
        return error

    """   
    @brief  设置485扩展轴控制模式
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 mode 控制模式，0-位置模式，1-速度模式
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetControlMode(self, servoId, mode):
        servoId = int(servoId)
        mode = int(mode)
        error = self.robot.AuxServoSetControlMode(servoId, mode)
        return error

    """   
    @brief  设置485扩展轴目标位置(位置模式)
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 float pos 目标位置，mm或°
    @param  [in] 必选参数 float speed 目标速度，mm/s或°/s
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetPos(self, servoId, pos, speed):
        servoId = int(servoId)
        pos = float(pos)
        speed = float(speed)
        error = self.robot.AuxServoSetTargetPos(servoId, pos, speed)
        return error

    """   
    @brief  设置485扩展轴目标速度(速度模式)
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 float speed 目标速度，mm/s或°/s
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetSpeed(self, servoId, speed):
        servoId = int(servoId)
        speed = float(speed)
        error = self.robot.AuxServoSetTargetSpeed(servoId, speed)
        return error

    """   
    @brief  设置485扩展轴目标转矩(力矩模式)
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 float torque 目标力矩，Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetTorque(self, servoId, torque):
        servoId = int(servoId)
        torque = float(torque)
        error = self.robot.AuxServoSetTargetTorque(servoId, torque)
        return error

    """   
    @brief  设置485扩展轴回零
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 int mode 回零模式，1-当前位置回零；2-负限位回零；3-正限位回零
    @param  [in] 必选参数 float searchVel 回零速度，mm/s或°/s
    @param  [in] 必选参数 float latchVel 箍位速度，mm/s或°/s
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoHoming(self, servoId, mode, searchVel, latchVel):
        servoId = int(servoId)
        mode = int(mode)
        searchVel = float(searchVel)
        latchVel = float(latchVel)
        error = self.robot.AuxServoHoming(servoId, mode, searchVel, latchVel)
        return error

    """   
    @brief  清除485扩展轴错误信息
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoClearError(self, servoId):
        servoId = int(servoId)
        error = self.robot.AuxServoClearError(servoId)
        return error

    """   
    @brief  获取485扩展轴伺服状态
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） servoErrCode 伺服驱动器故障码
    @return 返回值（调用成功返回） servoState 伺服驱动器状态 bit0:0-未使能；1-使能;  bit1:0-未运动；1-正在运动;  bit4 0-未定位完成；1-定位完成；  bit5：0-未回零；1-回零完成
    @return 返回值（调用成功返回） servoPos 伺服当前位置 mm或°
    @return 返回值（调用成功返回） servoSpeed 伺服当前速度 mm/s或°/s
    @return 返回值（调用成功返回） servoTorque 伺服当前转矩Nm
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetStatus(self, servoId):
        servoId = int(servoId)
        _error = self.robot.AuxServoGetStatus(servoId)
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5]
        else:
            return error

    """   
    @brief  设置状态反馈中485扩展轴数据轴号
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServosetStatusID(self, servoId):
        servoId = int(servoId)
        error = self.robot.AuxServosetStatusID(servoId)
        return error

    """   
    @brief  设置机器人外设协议
    @param  [in] 必选参数 int protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExDevProtocol(self, protocol):
        protocol = int(protocol)
        error = self.robot.SetExDevProtocol(protocol)
        return error

    """   
    @brief  获取机器人外设协议
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） int protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster

    """

    @log_call
    @xmlrpc_timeout
    def GetExDevProtocol(self):
        _error = self.robot.GetExDevProtocol()
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error