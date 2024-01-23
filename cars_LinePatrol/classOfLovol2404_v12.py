import cantools
import os
import time
from datetime import datetime
import json
import numpy as np
import threading
import can
import cv2
import multiprocessing
import math
from pyzbar import pyzbar
from subprocess import PIPE, Popen
import subprocess
import classOfIMU_CGI610

class CANMsgTrans(object):
    db = 0

    def __init__(self, dbc_name):
        self.dbc_name = dbc_name
        dbc_path = os.path.join(os.getcwd(), dbc_name)
        self.db = cantools.db.load_file(self.dbc_name)  # db = cantools.database.load_file(dbcPath)原来的database已经改为db了
        # print(self.db)

    def can_msg_produce(self, msg_name, msg_list):
        msg = self.db.get_message_by_name(msg_name)
        # 消息发送初始化
        msg_data = {}
        j = 0
        for i in msg.signal_tree:
            msg_data[i] = msg_list[j]
            j = j + 1
        print(msg_data)
        data = msg.encode(msg_data)
        # message = can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False)
        msg_frame_id = msg.frame_id

        message = can.Message(arbitration_id=msg_frame_id, data=data, is_extended_id=True)
        return message


"""
__NCU1_msg_list对应 ：          
       0 AutomaticDriveEnable  : 0-自动驾驶使能关闭 1-开启 2-使能信号错误 
       1 FrontSteeringAngleCmd : 前轮转动角度

__NCU2_msg_list对应 ：       
       0 EHS4_OperatingMode，液压阀4   1-空档  4-浮动  16-正向动作  64-反向动作
       1 EHS3_OperatingMode，液压阀3
       2 EHS2_OperatingMode，液压阀2
       3 EHS1_OperatingMode，液压阀1

__NCU3_msg_list对应：
       0 Vehicle_ReControlEnable : 整机使能开关 0-使能关闭  1-开启  2-信号错误
       1 PTO_ReControlEnable : PTO使能开关 0-使能关闭  1-开启  2-信号错误
       2 EHS4_ReControlEnable : 阀4使能开关 0-使能关闭  1-开启  2-信号错误
       3 EHS3_ReControlEnable : 阀3使能开关 0-使能关闭  1-开启  2-信号错误
       4 EHS2_ReControlEnable : 阀2使能开关 0-使能关闭  1-开启  2-信号错误
       5 EHS1_ReControlEnable : 阀1使能开关 0-使能关闭  1-开启  2-信号错误
       6 RearPTO_Switchs : 后PTO开关 0-PTO断开  1-PTO结合  2-指令错误
       7 ReDiffLockCmd : 远程差速锁指令  0-差速锁断开    1-差速锁结合  2-指令错误
       8 Re4WD_Cmd : 0-四驱断开  1-四驱结合  2-指令错误
       9 ReDriveCommand : 方向档位控制指令  0-空档   1-后退档  18-前进档
       10 ReRatedvehicleSpeed : 目标车速  0-40km/h

__NCU4_msg_list对应：
        EHC_NO_1  ：   这个三个字段不知道干啥用，设置为 0xfa = 250
        EHC_NO_2  :    250
        EHC_NO_3  :    250
        RockerSwtich :  提升器控制 0-停止  1-上升（运输） 2-下降（控制） 3-快速下降  E-Fault
        Remote_Control_Enable :   外部使能开关  0-手动控制  1-导航控制  2-不使用  3-报错

"""


class Tractor(object):
    __NCU1_msg_list = [0, 0]  # 自动驾驶不使能 ，前轮转角0
    __NCU2_msg_list = [1, 1, 1, 1]  # 四路液压阀全设置为空档
    __NCU3_msg_list = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # debug 20210930 已修改完成
    __NCUEHC_msg_list = [250, 250, 250, 0, 2]
    # __can_msg_trans = CANMsgTrans("/home/wen/PycharmProjects/dongFengProj_1.0/component_0/dongFeng2204_2.dbc")
    """lyh"""
    __can_msg_trans = CANMsgTrans("2404_modify_2EHC.dbc")

    def __init__(self, flag):

        self.can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=250000)
        if flag == "send":  # 当前是发送消息的功能
            self.task_NCU_1 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_1", self.__NCU1_msg_list), period=0.1, duration=None,
                store_task=True)  # 一个task值对应一个can.Message.arbitration_id
            # task.modify_data(msg)

            self.task_NCU_2 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_2", self.__NCU2_msg_list), period=0.1,
                duration=None, store_task=True)
            # print('debug navi2')
            self.task_NCU_3 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_3", self.__NCU3_msg_list), period=0.1,
                duration=None, store_task=True)
            self.task_NCU_EHC = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_EHC", self.__NCUEHC_msg_list), period=0.1,
                duration=None, store_task=True)

        else:  # 当前是接受消息的功能
            pass

    def send_msg(self, msg_name, cmd_list):
        # 把can_msg的生成放进来
        can_msg = self.__can_msg_trans.can_msg_produce(msg_name, cmd_list)
        if msg_name == "NCU_1":
            self.task_NCU_1.modify_data(can_msg)
        if msg_name == "NCU_2":
            self.task_NCU_2.modify_data(can_msg)
        if msg_name == "NCU_3":
            self.task_NCU_3.modify_data(can_msg)
        if msg_name == "NCU_EHC":
            self.task_NCU_EHC.modify_data(can_msg)

    def receMsg(self):
        # 回调函数
        # recv() 返回的就是can.Message类，和上面的can.Message方法的参数一致
        get_data = self.can_bus.recv()  # recv()函数只管接收信息，会把CAN总线中所有的信息读取进来

        while 1:
            if get_data.arbitration_id == 0x18FFC203:  # 在这里进行信息的分拣
                # print("接收到了车辆状态信息")
                dipan_fankui = self.__can_msg_trans.db.decode_message(get_data.arbitration_id, get_data.data)
                print("dipan_fankui:   ", dipan_fankui)
                return [0x18FFC203, dipan_fankui]

            elif get_data.arbitration_id == 0x18FFC403:
                PTO_zhuangtai_fankui = self.__can_msg_trans.db.decode_message(get_data.arbitration_id, get_data.data)
                print("PTO_zhuangtai_fankui:   ", PTO_zhuangtai_fankui)
                return [0x18FFC403, PTO_zhuangtai_fankui]

            elif get_data.arbitration_id == 0x18FFDAF0:
                Front_SteerAngle_fankui = self.__can_msg_trans.db.decode_message(get_data.arbitration_id, get_data.data)
                print("Front_SteerAngle_fankui:   ", Front_SteerAngle_fankui)
                return [0x18FFDAF0, Front_SteerAngle_fankui]

            else:
                get_data = self.can_bus.recv()


class VCUCmd(object):
    """
        本类进行拖拉机整车（横纵向运动、提升器、PTO、液压输出）控制指令的封装，按特定功能来设计函数。
    """
    """lyh 列表中对应数值的顺序和Excel表顺序一致，如__NAVI_cmd_list[]与表0x18FF911C行的车速对应"""
    # 类成员变量

    __NCU1_cmd_list = [0, 0]  # 自动驾驶不使能    前轮转角为0
    __NCU2_cmd_list = [1, 1, 1, 1]  # 四路液压阀全设置为空档
    __NCU3_cmd_list = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 整机使能   PTO不使能   四路液压阀不使能   PTO断开   差速锁断开   四驱断开   空档  车速为0
    __NCUEHC_cmd_list = [250, 250, 250, 0, 2]  # 提升器停止  不使用

    def __init__(self, tractor):
        """
            进行变量的初始化
        """
        self.tractor = tractor
        self.send_init_msg()

    def send_init_msg(self):  # 解锁前轮转角控制，使能整车控制和导航控制
        self.tractor.send_msg("NCU_1", [0, 0])
        self.tractor.send_msg("NCU_3", [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.5)
        self.tractor.send_msg("NCU_1", [1, 0])

    def send_motion_ctrl_msg(self, AutomaticDriveEnable, Vehicle_ReControlEnable, ReDriveCommand, FrontSteeringAngleCmd,
                             ReRatedvehicleSpeed, Brk_En):
        """
            发送车辆横纵向控制指令。后期把参数换成英文。
        :param AutomaticDriveEnable:    1：自动驾驶模式、0：自动驾驶使能关闭
        param Vehicle_ReControlEnable    0:控制整机使能关闭 1：打开
        :param ReDriveCommand:字符串类型，分为“前进”、“后退”、“空挡”
        :param FrontSteeringAngleCmd:  前轮转角
        :param ReRatedvehicleSpeed:  速度
        :param Brk_En:  1:刹车  0，不刹车
        :return:
        """
        # 首先检查驾驶模式
        if AutomaticDriveEnable == 0:  # 手动模式
            self.__NCU1_cmd_list[0] = 0
            self.tractor.send_msg("NCU_1", self.__NCU1_cmd_list)
            # return "手动模式" # debug20211001注释掉本句，想在手动驾驶的时候进行方向盘的控制

        # TODO:检查下有车速情况下发空挡是什么反应
        # 刹车的优先级最高放在这里，刹车后就返回(刹车转换成速度为0、不管转角)
        if Brk_En == 1:
            self.__NCU3_cmd_list[-1] = 0  # 修改车速(后期看是否要挂空挡)
            # 发送指令直接返回
            self.tractor.send_msg("NCU_3", self.__NCU3_cmd_list)
            return "程序刹车制动"

        # 修改变量序列
        self.__NCU3_cmd_list[-1] = ReRatedvehicleSpeed  # 修改车速

        if ReDriveCommand == "前进":  # 测试下是否高效
            self.__NCU3_cmd_list[-2] = 18  # 修改档位(前进18、空挡0、后退1)
        elif ReDriveCommand == "后退":
            self.__NCU3_cmd_list[-2] = 1  # 修改档位(前进18、空挡0、后退1)
        else:
            # “空挡”
            self.__NCU3_cmd_list[-2] = 0  # 修改档位(前进18、空挡0、后退1)

        # self.__NAVI_cmd_list[2]   # 修改提升器指令
        self.__NCU3_cmd_list[0] = Vehicle_ReControlEnable  # 整车控制使能
        self.__NCU1_cmd_list[0] = AutomaticDriveEnable  # 修改导航模式(0:关闭、1：开启)
        # self.__NAVI_cmd_list[4]   # 修改点火熄火信号
        # self.__NAVI_cmd_list[5]   # 修改PTO指令

        # 前轮转角(后期读取前轮实际转角进行误差消除)
        self.__NCU1_cmd_list[1] = FrontSteeringAngleCmd  # 前轮目标转角

        # 发送修改的变量
        print('self.__NCU1_cmd_list', self.__NCU1_cmd_list)
        print('self.__NCU3_cmd_list', self.__NCU3_cmd_list)

        self.tractor.send_msg("NCU_1", self.__NCU1_cmd_list)
        self.tractor.send_msg("NCU_3", self.__NCU3_cmd_list)
        return "程序控制指令发送成功"

    def send_hoist_unlock_msg(self):  # 解锁提升器控制
        time.sleep(1)
        self.tractor.send_msg("NCU_EHC", [250, 250, 250, 0, 1])
        time.sleep(1)
        self.tractor.send_msg("NCU_EHC", [250, 250, 250, 1, 1])
        time.sleep(1)
        self.tractor.send_msg("NCU_EHC", [250, 250, 250, 0, 1])
        time.sleep(1)

    def send_hoist_msg(self, RockerSwtich, Remote_Control_Enable):

        self.__NCUEHC_cmd_list[4] = 1  # 0手动控制  1导航控制  2不使用

        if RockerSwtich == "up":  # 运输
            self.__NCUEHC_cmd_list[3] = 1

        elif RockerSwtich == "down":  # 控制
            self.__NCUEHC_cmd_list[3] = 2

        elif RockerSwtich == "quick_down":  # 快速下降
            self.__NCUEHC_cmd_list[3] = 3
        else:
            self.__NCUEHC_cmd_list[3] = 0  # 停止
        print('self.__NCUEHC_cmd_list', self.__NCUEHC_cmd_list)
        self.tractor.send_msg("NCU_EHC", self.__NCUEHC_cmd_list)

    def send_pto_msg(self, pto_en, pto_connected):
        """控制PTO"""
        self.__NCU3_cmd_list[1] = pto_en  # 1使能   0关闭
        self.__NCU3_cmd_list[6] = pto_connected  # 0断开  1结合 2错误
        self.tractor.send_msg("NCU_3", self.__NCU3_cmd_list)

    """
    EHS_name 可选EHS1-4对应液压阀1-4
    operate_command   1-空档 ；4-浮动 ； 16-正向 ； 64-反向
    """

    def send_EHS_msg(self, EHS_name, operate_command):

        if EHS_name == "EHS1":
            self.__NCU3_cmd_list[5] = 1
            self.__NCU2_cmd_list[3] = operate_command

        elif EHS_name == "EHS2":
            self.__NCU3_cmd_list[4] = 1
            self.__NCU2_cmd_list[2] = operate_command


        elif EHS_name == "EHS3":
            self.__NCU3_cmd_list[3] = 1
            self.__NCU2_cmd_list[1] = operate_command


        elif EHS_name == "EHS4":
            self.__NCU3_cmd_list[2] = 1
            self.__NCU2_cmd_list[0] = operate_command

        else:
            print("输入错误")
        print('self.__NCU3_cmd_list', self.__NCU3_cmd_list)
        print('self.__NCU2_cmd_list', self.__NCU2_cmd_list)

        self.tractor.send_msg("NCU_3", self.__NCU3_cmd_list)
        self.tractor.send_msg("NCU_2", self.__NCU2_cmd_list)




''' 新添加方法：找到前摄像头和后摄像头的索引号 '''
def find_cam(cam):
    cmd = ["/usr/bin/v4l2-ctl", "--list-devices"]
    out, err = Popen(cmd, stdout=PIPE, stderr=PIPE).communicate()

    out = out.strip()
    result = out.split(b"\n\t")

    print(result)

    for i in result:
        i = i.decode("utf-8","\n\t")
        if cam == i:
            return i[-1]

    return None

def log_message(message,file_name):
    """
    将消息记录到指定文件的函数。
    :param message: 要记录的消息。
    :param file_name: 日志将被写入的文件名。
    """
    with open(file_name, "a") as file:
        file.write(message + "\n")

def seconds_get():
    """
     获取当前时间的格式化
    :return: 返回格式化时间
    """
    time_get = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    seconds =  int(time_get[11:13]) * 3600 + int(time_get[14:16]) * 60 + int(time_get[17:19])
    return seconds
def get_camera_ids():
    """
    获取电脑中所有可用的摄像头索引
    :return: 返回列表，存放的是可用摄像头的索引
    """
    camera_ids = []
    max_camera_id= 5
    for camera_id in range(max_camera_id):
        try:
            cap = cv2.VideoCapture(camera_id)
            if cap.isOpened():
                camera_ids.append(camera_id)
            cap.release()
        except Exception:
            print(str(camera_id) + "can't open")

    return camera_ids

def find_inter_cam(command,i):
    """
    找到电脑内置摄像头对应的索引
    :param command: 执行列出摄像头详细指令
    :param i: 这条指令对应的摄像头索引
    :return: 内置摄像头索引
    """
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    result = str(result)
    # print(result[1311:1315])
    if (result[1311:1315] == "30/1"):
        # print("现在这个摄像头是电脑内置摄像头")
        return i

    return None

def getCameraIndex():
    """
    找到所有摄像头对应的索引值
    :return:
    """
    camera_ids = get_camera_ids()
    command_pre = "v4l2-ctl -d /dev/video"
    command_later = " --all"

    allCameraIndex = {'computer_interal_camera' : 0,
                      'font_camera' : 2,
                      'later_camera' : 4}

    # 找内置摄像头
    for i in camera_ids:
        command = command_pre + str(i) + command_later
        print("command = " + command)
        if find_inter_cam(command, i) is not None:
            allCameraIndex['computer_interal_camera'] = i

    # 找前摄像头
    for i in camera_ids:
        if i is not allCameraIndex['computer_interal_camera']:
            cap = cv2.VideoCapture(i)
            start_time = seconds_get()
            while True:
                # 捕获帧-by-帧
                ret, frame = cap.read()
                # ret1, frame1 = cap1.read()
                # 如果正确读取帧，ret为True
                if not ret:
                    print("无法读取摄像头画面")
                    break

                # 纠正畸变
                dst = frame
                # 把图像做一个灰度化处理
                gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像
                # 扫描二维码
                text = pyzbar.decode(gray)

                if text != []:
                    allCameraIndex["font_camera"] = i
                    # print("前面的摄像头为 ： " + str(Font_camera))
                    break

                # 如果找不到带二维码的图像
                end_time = seconds_get()
                if end_time - start_time > 1:
                    break

    for i in camera_ids:
        if (i is not allCameraIndex["computer_interal_camera"] and
                i is not allCameraIndex["font_camera"]):
            print("后摄像头 ： " + str(i))
            allCameraIndex["later_camera"] = i
            break

    print("All")
    print(allCameraIndex)
    return allCameraIndex

def producer_Out(fontIndex):

    # 摄像头部分的代码
    # 前摄像头代码
    cap = cv2.VideoCapture(fontIndex)
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开前摄像头")
        exit()
    cap.set(3, 1280)  # 设置分辨率
    cap.set(4, 768)
    # 后摄像头代码


    tractor = Tractor("send")
    vcu_cmd = VCUCmd(tractor)

    direct = '空挡'

    pre_angle = 0
    isFirst = 1
    isError = 0
    isError_pre = 0

    detectLog_folder = './detectLog/'
    if not os.path.exists(detectLog_folder):
        os.makedirs(detectLog_folder)

    log_path = detectLog_folder + str(datetime.now())
    log_path = log_path + ".txt"

    log_angle_path = log_path + "angle.txt"
    log_originAngle_path = log_path + "originangle.txt"

    while True:

        ret, frame = cap.read()

        '''以下是检测有没有二维码的代码'''

        # 纠正畸变
        dst = frame

        # 把图像做一个灰度化处理
        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像

        # 扫描二维码
        text = pyzbar.decode(gray)


        print("112231")
        print(text)

        # 这个地方加一个
        if text == []:
            # print("当前屏幕里面没有二维码")
            print("112232")
            if not ret :
                break
            print(direct)
            if direct == "前进":
                frame = cv2.flip(frame, 1, frame)  # 镜像图像


            '''1月5号：添加的新的将视野集中的模块'''
            # image_x = int(frame.shape[1] / 2)
            # image_y = int(frame.shape[0] / 2)
            # frame[0:image_y - 50, 0:image_x * 2] = [0, 0, 0]
            # frame[image_y + 50:image_y * 2, 0:image_x * 2] = [0, 0, 0]

            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # 转换为HSV
            # lower_white = np.array([80, 120, 30])
            # upper_white = np.array([100, 255, 255])
            # mask = cv2.inRange(hsv, lower_white, upper_white)  # 提取特殊颜色
            lower_white = np.array([80, 80, 30])
            upper_white = np.array([120, 255, 255])

            '''以下是计算角度的方法'''
            image_y, image_x = int(frame.shape[0] / 2), int(frame.shape[1] / 2)
            central_point = (image_x, image_y)  # 画面中心点

            try:
                mask = cv2.inRange(hsv, lower_white, upper_white)  # 提取特殊颜色
                contours, _ = cv2.findContours(cv2.cvtColor(cv2.bitwise_and(frame, frame, mask=mask),
                                                        cv2.COLOR_RGB2GRAY),
                                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 寻找物体边界


                # 这个地方加一个预警制度，防止突然挂掉
                cmax = max(contours, key=cv2.contourArea)

                M = cv2.moments(cmax)  # 得到最大矩阵

                x, y = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])  # 中心点的X


                cv2.drawContours(frame, cmax, -1, (0, 0, 255), -1)
                cv2.line(frame, (image_x, frame.shape[0]), (image_x, image_y), (255, 0, 255), 1, 4)
                cv2.line(frame, (image_x, frame.shape[0]), (x, y), (255, 255, 0), 1, 4)
                cv2.circle(frame, (x, y), 1, (0, 0, 255), 10)
                cv2.circle(frame, central_point, 1, (0, 255, 0), 10)

                line = pow(pow(x - image_x, 2) + pow(abs(frame.shape[0] - y), 2), 0.5)
                sin = (x - image_x) / line  # 算出夹角sin值
                angle = math.degrees(math.asin(sin))  # 算出角度
                log_message(str(datetime.now()) + " originAngle = " + str(angle), log_originAngle_path)
                isError = 0

            except Exception:
                # time.sleep(0.1)
                # 1. 设置另外的曝光
                # 2. 设置另外的色域
                # 3. 如果距离上一次的偏转角偏的太大，或者要打的角度太离谱，那么就需要把当前的角度设置成上一次的角度加5°或者减5°
                print("进入到识别不到的状态")
                isError = isError + 1
                log_message(str(datetime.now()) + " 进入到识别不到的状态",log_path)
                if pre_angle < 0:
                    angle = pre_angle + 1.5
                else:
                    angle = pre_angle - 1.5

            if angle > 35:
                angle = 35
            elif angle < -35:
                angle = -35
            '''这个地方新增角度判断，避免方向盘猛打的情况'''
            if isError == 0 and isError_pre > 1:
                pre_angle = angle
            else:
                if isFirst:
                    pre_angle = angle
                    isFirst = 0
                else:
                    if abs(pre_angle - angle) > 60 and not isError:
                        log_message(str(datetime.now()) + " 出现 : " + "较大的转向", log_path)
                        if pre_angle < 0:
                            angle = pre_angle + 3
                        else:
                            angle = pre_angle - 3
                    elif abs(pre_angle - angle) < 60 and not isError:
                        log_message(str(datetime.now()) + " 显示 : " + "正常的情况", log_path)
                    pre_angle = angle

            isError_pre = isError

            log_message(str(datetime.now()) + " 角度为 : " + str(angle) , log_angle_path)

            if direct == "前进" and math.fabs(angle) > 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "前进", angle * -0.82, 0.7, 0)
                print("传入的前进角度为:" + str(angle * -0.45))
            elif direct == "后退" and math.fabs(angle) > 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", angle * 0.6, 0.7, 0)
                print("传入的后退角度为:"+str(angle * 0.45))
            elif direct == "后退" and math.fabs(angle) <= 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", 0, 1, 0)
            elif direct == "前进" and math.fabs(angle) <= 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "前进", 0, 1, 0)
            else:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "空挡", 0, 1, 0)

            cv2.putText(frame, str(angle), (image_x, frame.shape[0]), cv2.FONT_HERSHEY_COMPLEX, 1.5, (0, 0, 255), 3)
            cv2.imshow('12', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif text != []:
            print("112233")
            for texts in text:
                textdate = texts.data.decode('utf-8')
                # print("test0")
                if textdate == 'stop' and direct == '空挡':
                    # print("test1")
                    direct = "前进"
                    vcu_cmd.send_motion_ctrl_msg(1, 1, "前进", 0, 0.7, 0)
                elif textdate == 'start' and direct == '前进':
                    # print("test2")
                    # direct = "空挡"
                    start_second = seconds_get()
                    Imu = classOfIMU_CGI610.Imu()
                    while 1:
                        end_second = seconds_get()

                        GPSWeek, GPSTime, x, y, Latitude, Longitude, headingAngle, V, Altitude, Warming, Pitch, Roll, gyro_x, gyro_y, gyro_z, \
                            acc_x, acc_y, acc_z, Ve, Vn, Vu, NSV1, NSV2, Status, Age = Imu.stateOfCar()  # 执行一次采样一次。这里用imu.stateOfCar实现
                        if headingAngle > 272.26 and end_second - start_second > 10:
                            cap.release()
                            cv2.destroyAllWindows()
                            vcu_cmd.send_motion_ctrl_msg(1, 1, "空挡", 0, 0, 0)
                            time.sleep(1)
                            return
                        vcu_cmd.send_motion_ctrl_msg(1, 1, "前进", 21, 0.6, 0)
                # elif textdate == 'start' and direct == '后退':
                #
                #     print(("空挡", 0))
                #     # 添加一个任务定时器，比如执行2秒钟
                #
                #     start_sencond = seconds_get()
                #
                #     while 1:
                #
                #         end_second = seconds_get()
                #
                #         if end_second - start_sencond > 35:
                #             cap.release()
                #             cv2.destroyAllWindows()
                #             return
                #         vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", 40, 1, 0)
                #
                # elif textdate == 'stop' and direct == '前进':
                #     return

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def producer_In(fontIndex, backIndex):

    """
    新增进机库方法，先巡线，识别到二维码停车
    :param fontIndex: 前摄像头编号
    :return:
    """
    # 摄像头部分的代码
    # 前摄像头代码
    cap = cv2.VideoCapture(fontIndex)
    cap1 = cv2.VideoCapture(backIndex)
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开前摄像头")
        exit()
    cap.set(3, 1280)  # 设置分辨率
    cap.set(4, 768)

    if not cap1.isOpened():
        print("无法打开后摄像头")
        exit()
    cap1.set(3, 1280)  # 设置分辨率
    cap1.set(4, 768)
    # TODO: 这个地方做了一个摄像头图像的垂直翻转

    NCU1list = [1, 30]
    NCU2list = [1, 16, 1, 1]
    NCU3list = [1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0]
    NCU4list = [0, 0]

    tractor = Tractor("send")
    vcu_cmd = VCUCmd(tractor)

    direct = '后退'

    pre_angle = 0
    isFirst = 1
    isError = 0
    isError_pre = 0

    detectLog_folder = './detectLog/'
    if not os.path.exists(detectLog_folder):
        os.makedirs(detectLog_folder)

    log_path = detectLog_folder + str(datetime.now())
    log_path = log_path + ".txt"

    log_angle_path = log_path + "angle.txt"
    log_originAngle_path = log_path + "originangle.txt"

    while True:

        ret, frame = cap.read()
        # 后摄像头做了一个水平加垂直的翻转功能
        ret1, frame1 = cap1.read()
        frame1 = cv2.flip(frame1,-1)

        '''以下是检测有没有二维码的代码'''

        # 纠正畸变
        dst = frame

        # 把图像做一个灰度化处理
        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像

        # 扫描二维码
        text = pyzbar.decode(gray)


        print("112231")
        print(text)

        # 这个地方加一个
        if text == []:
            # print("当前屏幕里面没有二维码")
            print("112232")
            if not ret:
                break
            print(direct)
            frame = cv2.flip(frame1, 1, frame1)  # 镜像图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # 转换为HSV
            lower_white = np.array([80, 100, 70])
            upper_white = np.array([120, 255, 255])

            '''以下是计算角度的方法'''
            image_y, image_x = int(frame.shape[0] / 2), int(frame.shape[1] / 2)
            central_point = (image_x, image_y)  # 画面中心点

            try:
                mask = cv2.inRange(hsv, lower_white, upper_white)  # 提取特殊颜色
                contours, _ = cv2.findContours(cv2.cvtColor(cv2.bitwise_and(frame, frame, mask=mask),
                                                        cv2.COLOR_RGB2GRAY),
                                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 寻找物体边界


                # 这个地方加一个预警制度，防止突然挂掉
                cmax = max(contours, key=cv2.contourArea)

                M = cv2.moments(cmax)  # 得到最大矩阵

                x, y = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])  # 中心点的X


                cv2.drawContours(frame, cmax, -1, (0, 0, 255), -1)
                cv2.line(frame, (image_x, frame.shape[0]), (image_x, image_y), (255, 0, 255), 1, 4)
                cv2.line(frame, (image_x, frame.shape[0]), (x, y), (255, 255, 0), 1, 4)
                cv2.circle(frame, (x, y), 1, (0, 0, 255), 10)
                cv2.circle(frame, central_point, 1, (0, 255, 0), 10)

                line = pow(pow(x - image_x, 2) + pow(abs(frame.shape[0] - y), 2), 0.5)
                sin = (x - image_x) / line  # 算出夹角sin值
                angle = math.degrees(math.asin(sin))  # 算出角度
                log_message(str(datetime.now()) + " originAngle = " + str(angle), log_originAngle_path)
                isError = 0

            except Exception:
                # time.sleep(0.1)
                # 1. 设置另外的曝光
                # 2. 设置另外的色域
                # 3. 如果距离上一次的偏转角偏的太大，或者要打的角度太离谱，那么就需要把当前的角度设置成上一次的角度加5°或者减5°
                print("进入到识别不到的状态")
                isError = isError + 1
                log_message(str(datetime.now()) + " 进入到识别不到的状态",log_path)
                if pre_angle < 0:
                    angle = pre_angle + 1.5
                else:
                    angle = pre_angle - 1.5

            if angle > 35:
                angle = 35
            elif angle < -35:
                angle = -35
            '''这个地方新增角度判断，避免方向盘猛打的情况'''
            if isError == 0 and isError_pre > 1:
                pre_angle = angle
            else:
                if isFirst:
                    pre_angle = angle
                    isFirst = 0
                else:
                    if abs(pre_angle - angle) > 60 and not isError:
                        log_message(str(datetime.now()) + " 出现 : " + "较大的转向", log_path)
                        if pre_angle < 0:
                            angle = pre_angle + 3
                        else:
                            angle = pre_angle - 3
                    elif abs(pre_angle - angle) < 60 and not isError:
                        log_message(str(datetime.now()) + " 显示 : " + "正常的情况", log_path)
                    pre_angle = angle

            isError_pre = isError

            log_message(str(datetime.now()) + " 角度为 : " + str(angle) , log_angle_path)

            if direct == "后退" and math.fabs(angle) > 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", angle * 0.82, 0.7, 0)
                print("传入的前进角度为:" + str(angle * 0.82))
            elif direct == "后退" and math.fabs(angle) <= 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", 0, 1, 0)
            else:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "空挡", 0, 1, 0)

            cv2.putText(frame, str(angle), (image_x, frame.shape[0]), cv2.FONT_HERSHEY_COMPLEX, 1.5, (0, 0, 255), 3)
            cv2.imshow('12', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif text != []:
            print("112233")
            for texts in text:
                textdate = texts.data.decode('utf-8')
                # if textdate == 'start' and direct == '空挡':
                #     vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", 0, 0.5, 0)
                #     direct = '后退'
                # TODO: 添加关闭摄像头部分代码
                if textdate == 'stop' and direct == '后退':
                    direct = '空挡'
                    vcu_cmd.send_motion_ctrl_msg(1, 1, "空挡", 0, 0, 0)
                    return


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def producer_In1(fontIndex, backIndex):

    """
    测试进机库需要打多少秒数，来完成进机库操作
    """


    tractor = Tractor("send")
    vcu_cmd = VCUCmd(tractor)

    # direct = '前进'



    # vcu_cmd.send_motion_ctrl_msg(1, 1, "前进", -35, 0.7, 0)

    time_start = seconds_get()
    while 1:
        end_start = seconds_get()
        if end_start - time_start > 81:
            vcu_cmd.send_motion_ctrl_msg(1, 1, "空挡", 0, 0, 0)
            return
        vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", 21.7, 0.7, 0)

# def testLine(backIndex):
#
#     cap = cv2.VideoCapture(backIndex)
#     if not cap.isOpened():
#         print("无法打开后摄像头")
#         exit()
#     cap.set(3, 1280)  # 设置分辨率
#     cap.set(4, 768)
#
#     while True:
        



'''
    这一版代码就是加上了try-except，还有定时器
'''

if __name__ == "__main__":


    # Carmea_Index = getCameraIndex()
    fontIndex = 4
    backIndex = 2
    # 执行出机库操作
    # producer_In1(fontIndex,backIndex)
    # producer_Out(fontIndex)
    # 执行进机库操作
    producer_In(fontIndex, backIndex)

   