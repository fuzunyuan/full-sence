import cv2
from pyzbar import  pyzbar
from datetime import datetime
import subprocess

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


if __name__=="__main__":
    print("打开摄像头")
    allIndex = getCameraIndex()
    # 打开默认摄像头（通常是电脑内置摄像头）
    cap = cv2.VideoCapture(allIndex["later_camera"])  # 后面
    cap1 = cv2.VideoCapture(allIndex["font_camera"])  # 前面

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()

    while True:
        # 捕获帧-by-帧
        ret, frame = cap.read()
        ret1, frame1 = cap1.read()
        # 如果正确读取帧，ret为True
        if not ret and not ret1:
            print("无法读取摄像头画面")
            break

        # 显示结果帧
        cv2.imshow('Camera', frame)
        cv2.imshow('Creame1', frame1)
        # 按 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 完成所有操作后，释放摄像头
    cap.release()
    cv2.destroyAllWindows()