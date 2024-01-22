"""
    本类实现inte_navi信息的读取
"""
import serial
import time
import ll2xy

portx = "/dev/ttyUSB0"
#bps = 115200  # 10hz时，5hz时对应波特率
bps = 460800  # 100hz时对应波特率，这样应该就不会有数据延迟的问题
timex = 5
ser = serial.Serial(portx, bps, timeout=timex)

# 解决：读取错误应该是要抛去第一次读取的，跟读取的快慢没关系
# time.sleep(1)
ser.readline()
ser.readline()

# 测试,用阻塞的方式更好。
#ser.in_waiting

class Imu():
    def __init__(self):
        '''初始化端口，等待调用读取即可'''

        pass

    def stateOfCar(self):

        imuInfo = ser.readline().decode("gbk")

        # 调用一次，清空一次缓冲区
        ser.reset_input_buffer()
        ser.readline()  # 扔掉一条


        print('imuInfo=', imuInfo)
        # 对格式字符进行分割得到
        str = imuInfo.split(',')
        print(str)
        # 4为headingAngle，13为纬度，14为经度,19为车辆速度
        GPSWeek=float(str[1])
        GPSTime=float(str[2])
        imuAngle = float(str[3])
        imu_lat = float(str[12])
        imu_lon = float(str[13])
        imu_vehicle_speed = float(str[18])

        headingAngle = imuAngle
        # 经纬度转xy
        t_start = time.time()
        x, y = ll2xy.ll2xy(lat=imu_lat, lon=imu_lon)
        t_end = time.time()
        print(t_end-t_start)
        v = imu_vehicle_speed  # 单位 m/s

        # gps_time,gps_status,gps_way=1,latitude,longitude,altitude,speed,satellite_num,warning
        imu_gps_state = float(str[21])  # 车辆正常行驶需为42
        imu_altitude = float(str[14])
        imu_satellite_num = float(str[19])
        imu_warning = str[23]    # 这里处理下
        imu_warning = imu_warning.split("*")
        imu_warning = float(imu_warning[0])

        return GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, imu_gps_state, imu_altitude, imu_satellite_num, imu_warning


if __name__ == "__main__":
    imu = Imu()
    while True:
        t1 = time.time()
        imuInfo = ser.readline().decode("gbk")
        # print(imuInfo)
        print(imu.stateOfCar()[1])
        t2 = time.time()
        print("It tooks me "+str(t2-t1)+" s")
