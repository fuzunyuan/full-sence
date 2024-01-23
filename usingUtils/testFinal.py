import cv2
import os
import numpy as np
import math
from datetime import datetime

'''function : 用来获取每一帧的转角'''
def log_message(message, file_name="log.txt"):
    """
    将消息记录到指定文件的函数。
    :param message: 要记录的消息。
    :param file_name: 日志将被写入的文件名。
    """
    with open(file_name, "a") as file:
        file.write(message + "\n")



def produce():
    log_path = "./detectLogs/" + str(datetime.now())
    log_path = log_path + ".txt"

    log_angle_path = log_path + "angle.txt"

    log_origin_path = log_path + "origin.txt"

    # 图片文件夹路径
    image_folder_path = 'resultBase/cropped_images'
    print(os.listdir(image_folder_path))
    h = os.listdir(image_folder_path)
    h.sort(key=lambda x: int(x.split('.')[0]))
    print(h)
    pre_x, pre_y = 0, 0
    pre_angle = 0
    isFirst = 1
    # 这个变量的作用是为了保证当我们从识别不到的情况转换到识别到的情况的时候，能够紧急多打方向回来
    isError = 0
    isError_pre = 0

    # 遍历文件夹中的所有图片
    for filename in h:
        if filename.endswith(('.png', '.jpg', '.jpeg')):  # 检查文件格式
            img_path = os.path.join(image_folder_path, filename)
            # frame = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)  # 以灰度模式读取
            frame = cv2.imread(img_path)


            if frame is not None:

                # print(f"图片: {filename}, 平均亮度: {mean}, 对比度(标准差): {std_dev}")
                # image_x = int(frame.shape[1] / 2)
                # image_y = int(frame.shape[0] / 2)
                # frame[0:image_y - 50, 0:image_x * 2] = [0, 0, 0]
                # frame[image_y + 50:image_y * 2, 0:image_x * 2] = [0, 0, 0]

                hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # 转换为HSV
                lower_white = np.array([80, 120, 30])
                upper_white = np.array([100, 255, 255])

                image_y, image_x = int(frame.shape[0] / 2), int(frame.shape[1] / 2)
                central_point = (image_x, image_y)  # 画面中心点

                try:
                    mask = cv2.inRange(hsv, lower_white, upper_white)  # 提取白色

                    contours, _ = cv2.findContours(
                        cv2.cvtColor(cv2.bitwise_and(frame, frame, mask=mask), cv2.COLOR_RGB2GRAY),
                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 寻找物体边界
                    # print(f"{filename} + tag1")
                    # TODO:这个地方加一个try-except，防止突然挂掉
                    cmax = max(contours, key=cv2.contourArea)
                    # print(f"{filename} + tag2")


                    # print("cmax = " + str(cmax))

                    M = cv2.moments(cmax)  # 矩阵无法使用
                    # print(f"{filename} + tag3")

                    if all(value == 0 for value in M.values()):
                        print("bad1")

                    x, y = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])  # 中心点的X

                    pre_x, pre_y = x, y
                    # print(f"{filename} + + x = " + str(x) + " ,y = " + str(y) + "pre_x, " + str(pre_x) + " pre_y ," + str(pre_y))
                    # log_message(f"{filename} , x = " + str(x) + " y = " + str(y),log_path)

                    cv2.drawContours(frame, cmax, -1, (0, 0, 255), -1)

                    cv2.line(frame, (image_x, frame.shape[0]), (image_x, image_y), (255, 0, 255), 1, 4)
                    cv2.line(frame, (image_x, frame.shape[0]), (x, y), (255, 255, 0), 1, 4)
                    cv2.circle(frame, (x, y), 1, (0, 0, 255), 10)
                    cv2.circle(frame, central_point, 1, (0, 255, 0), 10)

                    line = pow(pow(x - image_x, 2) + pow(abs(frame.shape[0] - y), 2), 0.5)
                    sin = (x - image_x) / line  # 算出夹角sin值
                    angle = math.degrees(math.asin(sin))  # 算出角度
                    log_message(f"{filename} originAngle = " + " : " + str(angle), log_origin_path)
                    isError = 0

                except Exception:
                    log_message(f"{filename} 进入到 : " + "识别不到",log_path)
                    isError = isError + 1

                    if pre_angle < 0:
                        angle = pre_angle + 1.5
                    else:
                        angle = pre_angle - 1.5

                # 这个表示，上一帧是Error,这一帧是正确的
                if isError == 0 and isError_pre > 1:
                    pre_angle = angle
                else:
                    if isFirst:
                        pre_angle = angle
                        isFirst = 0
                    else:
                        if abs(pre_angle - angle) > 60 and not isError:
                            log_message(f"{filename}出现 : "+"较大的转向" , log_path)
                            if pre_angle < 0:
                                angle = pre_angle + 3
                            else:
                                angle = pre_angle - 3
                        elif abs(pre_angle - angle) < 60 and not isError:
                            log_message(f"{filename}显示 : "+ "正常的情况", log_path)
                        pre_angle = angle

                isError_pre = isError

                log_message(f"{filename} 前轮转角为 ： " + str(angle), log_angle_path)
                cv2.putText(frame, str(angle), (image_x, frame.shape[0]),
                            cv2.FONT_HERSHEY_COMPLEX, 1.5, (0, 0, 255), 3)
                newfileName = "detectPic6/" + filename
                cv2.imwrite(newfileName, frame)
            else:
                print(f"无法加载图片: {filename} ", log_path)

if __name__ == "__main__":
    print("123")
    produce()