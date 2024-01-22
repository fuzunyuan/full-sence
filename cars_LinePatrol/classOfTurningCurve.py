"""
这个类是利用巡线转弯的功能，从而实现移库动作
具体流程为：
1. 识别地面二维码开始巡线曲线
2. 识别二维码停止——此时保证前后摄像头都是能够检测到线
3. 此时开始倒车巡线——巡直线
4. 识别到停车二维码——完成移库动作
"""
import cv2
import os
from datetime import datetime
import numpy as np
import math
from pyzbar import pyzbar

def log_message(message,file_name):
    """
    将消息记录到指定文件的函数。
    :param message: 要记录的消息。
    :param file_name: 日志将被写入的文件名。
    """
    with open(file_name, "a") as file:
        file.write(message + "\n")

def takeTurnAction1(vcu_cmd,fontIndex):
    """
    移库动作一：巡线巡转弯曲线
    :param fontIndex: 前摄像头编号
    :return:
    """
    # 摄像头部分的代码
    # 前摄像头代码
    cap = cv2.VideoCapture(fontIndex)
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开前摄像头")
        exit()
    cap.set(3, 1280)  # 设置分辨率
    cap.set(4, 768)

    direct = '前进'

    pre_angle = 0
    isFirst = 1
    isError = 0
    isError_pre = 0

    detectLog_folder = './detectLogTruningCurve/'
    if not os.path.exists(detectLog_folder):
        os.makedirs(detectLog_folder)

    log_path = detectLog_folder + str(datetime.now())
    log_path = log_path + ".txt"

    log_angle_path = log_path + "angle1.txt"
    log_originAngle_path = log_path + "originangle1.txt"

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
            if not ret:
                break
            print(direct)
            frame = cv2.flip(frame, 1, frame)  # 镜像图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # 转换为HSV
            # TODO:这个地方的颜色需要加一个确定的颜色阈值，目前我们要巡线的颜色是黄色的巡线
            #       这个地方的阈值可以改成红色或者蓝色的阈值
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
                # TODO: 根据二维码来进行停止，这个二维码变成stay
                if textdate == 'stay' and direct == '前进':
                    vcu_cmd.send_motion_ctrl_msg(1, 1, "空挡", 0, 0, 0)
                    cap.release()
                    cv2.destroyAllWindows()
                    return

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def takeTurnAction2(vcu_cmd, fontIndex, backIndex):
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
    cap1 = cv2.VideoCapture(backIndex)
    if not cap1.isOpened():
        print("无法打开后摄像头")
        exit()
    cap1.set(3, 1280)  # 设置分辨率
    cap1.set(4, 768)


    direct = '空挡'

    pre_angle = 0
    isFirst = 1
    isError = 0
    isError_pre = 0

    detectLog_folder = './detectLogTruningCurve/'
    if not os.path.exists(detectLog_folder):
        os.makedirs(detectLog_folder)

    log_path = detectLog_folder + str(datetime.now())
    log_path = log_path + ".txt"

    log_angle_path = log_path + "angle2.txt"
    log_originAngle_path = log_path + "originangle2.txt"

    while True:

        ret, frame = cap.read()
        ret1, frame1 = cap1.read()
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
            if not ret and not ret1:
                break
            print(direct)
            if direct == "前进":
                frame = cv2.flip(frame, 1, frame)  # 镜像图像
            elif direct == "后退":
                frame = cv2.flip(frame1, 1, frame1)  # 镜像图像



            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # 转换为HSV
            # TODO: 这个地方还是要更新颜色的阈值,蓝色或者红色的阈值
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
                log_message(str(datetime.now()) + " 进入到识别不到的状态", log_path)
                if pre_angle < 0:
                    angle = pre_angle + 1.5
                else:
                    angle = pre_angle - 1.5

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

            log_message(str(datetime.now()) + " 角度为 : " + str(angle), log_angle_path)

            if direct == "前进" and math.fabs(angle) > 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "前进", angle * -0.82, 0.7, 0)
                print("传入的前进角度为:" + str(angle * -0.45))
            elif direct == "后退" and math.fabs(angle) > 3:
                vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", angle * 0.6, 0.7, 0)
                print("传入的后退角度为:" + str(angle * 0.45))
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

                if textdate == 'stay' and direct == '空挡':
                    direct = "后退"
                    vcu_cmd.send_motion_ctrl_msg(1, 1, "后退", 0, 0.7, 0)
                elif textdate == 'stop' and direct == '后退':
                    vcu_cmd.send_motion_ctrl_msg(1, 1, "空挡", 0, 0.7, 0)
                    return


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
