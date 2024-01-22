"""
    本类实现CGI610组合导航实时信息的读取,后期可抽象为组合导航类
"""
import serial
import time
import ll2xy
import os, sys, stat
import math


# portx = "/dev/ttyUSB0"
# # 新增权限赋予
# try_num = 0
# while not os.path.exists("/dev/ttyUSB0"):
#     # 检查串口线是否连接
#     print("串口未连接，重启连接中，尝试次数", try_num)
#     try_num = try_num + 1
#     time.sleep(1)
#
# # 赋予权限的方式更换成shell脚本
# # os.chmod("/dev/ttyUSB0", stat.S_IRWXU)
# print(os.system("./sudo_chmod_777")) # 返回值为0才往下走
#
# #bps = 115200  # 10hz时，5hz时对应波特率
# bps = 460800  # 100hz时对应波特率，这样应该就不会有数据延迟的问题
# timex = 5
# ser = serial.Serial(portx, bps, timeout=timex)
#
# # 解决：读取错误应该是要抛去第一次读取的，跟读取的快慢没关系
# # time.sleep(1)
# ser.readline()
# ser.readline()

# 测试,用阻塞的方式更好。
# ser.in_waiting

class Imu():
    def __init__(self):
        """初始化端口，等待调用读取即可"""
        portx = "/dev/ttyUSB0"
        # bps = 115200  # 10hz时，5hz时对应波特率
        bps = 460800  # 100hz时对应波特率，这样应该就不会有数据延迟的问题
        timex = 5
        self.ser = serial.Serial(portx, bps, timeout=timex)

        # 解决：读取错误应该是要抛去第一次读取的，跟读取的快慢没关系
        self.ser.readline()
        self.ser.readline()

    def stateOfCar(self):
        imuInfo = self.ser.readline().decode("gbk")

        # 调用一次，清空一次缓冲区,解决错误读取的问题
        self.ser.reset_input_buffer()
        self.ser.readline()  # 扔掉一条

        print('imuInfo=', imuInfo)
        # 对格式字符进行分割得到
        str = imuInfo.split(',')
        # print(str)
        # 'PC_time_stamp', 'utm_x', 'utm_y', 'cmd_steering', 'cmd_v', 'path_target_point_index',
        #                               'GPSWeek', 'GPSTime', 'Heading', 'Pitch', 'Roll', 'gyro_x', 'gyro_y', 'gyro_z',
        #                               'acc_x', 'acc_y', 'acc_z', 'Latitude', 'Longitude', 'Altitude', 'Ve', 'Vn', 'Vu', 'V',
        #                               'NSV1', 'NSV2', 'Status', 'Age', 'Warming'
        # 以下为GPCHC所有的字段
        GPSWeek = int(str[1])
        GPSTime = float(str[2])
        Heading = float(str[3])
        Pitch = float(str[4])
        Roll = float(str[5])
        gyro_x = float(str[6])
        gyro_y = float(str[7])
        gyro_z = float(str[8])
        acc_x = float(str[9])
        acc_y = float(str[10])
        acc_z = float(str[11])
        Latitude = float(str[12])
        Longitude = float(str[13])
        Altitude = float(str[14])
        Ve = float(str[15])
        Vn = float(str[16])
        Vu = float(str[17])
        V = float(str[18])
        NSV1 = int(str[19])
        NSV2 = int(str[20])
        Status = int(str[21])
        Age = int(str[22])
        Warming = str[23]
        Warming = Warming.split("*")
        Warming = float(Warming[0])

        # imu_vehicle_speed = float(str[18])

        # headingAngle = Heading
        # 经纬度转xy
        utm_x, utm_y = ll2xy.ll2xy(lat=Latitude, lon=Longitude)

        # v = imu_vehicle_speed  # 单位 m/s

        # 20211021增加矫正
        def geometric2utm(geo_point: tuple, datum_point: tuple, beta_ridian: float):
            """
                几何图形转utm坐标点
            :param geo_point:   根据想要的几何形状在直角坐标系中确定的点，通常在原点附近，根据原点和偏移来表示几何中的点
            :param datum_point: 变换基准点，指的是AB线中的B点，为UTM坐标系中的点
            :param beta_ridian: AB向量相对于正北方向的角度，0~360°
            :return:
            """
            x_skim = geo_point[0]
            y_skim = geo_point[1]
            utm_p_x = x_skim * math.cos(-beta_ridian) - y_skim * math.sin(-beta_ridian) + datum_point[0]
            utm_p_y = y_skim * math.cos(-beta_ridian) + x_skim * math.sin(-beta_ridian) + datum_point[1]
            utm_point = (utm_p_x, utm_p_y)
            return utm_point

        # B = (utm_x, utm_y)
        # B_ori = (0 + 0.78, 0)  # 把B点移到原点     不太明白为啥要弄个0.78，以车头为方向左侧是定位，定位蘑菇头距离拖拉机中轴线为0.78，这是将坐标点变换到车辆中轴线
        # B_planning = geometric2utm(geo_point=B_ori, datum_point=B, beta_ridian=math.radians(Heading))
        # utm_x = B_planning[0]
        # utm_y = B_planning[1]

        # gps_time,gps_status,gps_way=1,latitude,longitude,altitude,speed,satellite_num,warning
        # inte_navi_status = int(str[21])  # 车辆正常行驶需为42
        # imu_altitude = float(str[14])
        # imu_satellite_num = float(str[19])
        # imu_warning = str[23]    # 这里处理下
        # imu_warning = imu_warning.split("*")
        # imu_warning = float(imu_warning[0])

        # 考虑返回列表
        # return GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, inte_navi_status, imu_altitude, imu_satellite_num, imu_warning
        # print(str)
        # 'PC_time_stamp', 'cmd_steering', 'cmd_v', 'path_target_point_index', 'utm_x', 'utm_y',
        #                               'GPSWeek', 'GPSTime', 'Heading', 'Pitch', 'Roll', 'gyro_x', 'gyro_y', 'gyro_z',
        #                               'acc_x', 'acc_y', 'acc_z', 'Latitude', 'Longitude', 'Altitude', 'Ve', 'Vn', 'Vu', 'V',
        #                               'NSV1', 'NSV2', 'Status', 'Age', 'Warming'

        # return [utm_x, utm_y, GPSWeek, GPSTime, Heading, Pitch, Roll, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, Latitude, Longitude,
        #         Altitude, Ve, Vn, Vu, V, NSV1, NSV2, Status, Age, Warming]

        return [GPSWeek, GPSTime, utm_x, utm_y, Latitude, Longitude, Heading, V, Altitude,Warming, Pitch, Roll, gyro_x, gyro_y, gyro_z,
                acc_x, acc_y, acc_z,
                Ve, Vn, Vu, NSV1, NSV2, Status, Age]


# if __name__ == "__main__":
#     # 进行人工驾驶时开启
#     imu = Imu()
#     with open("记录.csv"+str(time.time()), "w") as file:
#         while True:
#             t1 = time.time()
#             # imuInfo = ser.readline().decode("gbk")
#             # print(imuInfo)
#             # print(imu.stateOfCar()[8])  # 测试下正常情况=


if __name__ == "__main__":
    while True:
        imu = Imu()
        print(imu.stateOfCar()[2:6])
