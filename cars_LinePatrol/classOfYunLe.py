"""进行云乐小车的类型声明"""
# 小车要控制它，需要的信息如下：小车的属性和对它的操作，属性分别是五个，操作分别是发送和接收信息,发送和接收信息应该是接口
#   SCU_Drive_Mode_Req,SCU_ShiftLevel_Req,SCU_Steering_Wheel_Angle,SCU_Target_Speed,SCU_Brk_En五个参数在发送的时候需要

import cantools
import os
import time
import datetime
import json
import can

# 全局变量，dbc文件解析和can0通信初始化
dbcPath = os.path.join(os.getcwd(), "jiju.dbc")#os.getcwd()获取当前路径，join使用/将路径拼接在一起
db = cantools.db.load_file(dbcPath)

can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=250000)#接口和通道实例化can


class Car:
    # 实例化节点的时候默认开启
    def __init__(self, Car_Switch):
        """
        :param Car_Switch: Car_Switch=1 #默认为1，开启小车控制功能
        """
        self.Car_Switch = Car_Switch
        #sendMsg用的task
        # 2021/01/21 新增误差修正功能，小车开启对象并且转角不变化后，记录误差项并提供给后续命令的发送。这里默认小车前轮转角在上电后2秒内达到稳态



    def __msgInit(self,SCU_Drive_Mode_Req,SCU_ShiftLevel_Req,SCU_Steering_Wheel_Angle,SCU_Target_Speed,SCU_Brk_En):
        """
            初始化消息处理：处理dbc文件，这是通过处理dbc文件，最终返回要发送的消息
        :return:返回message，给send_period、modify作为其参数
        """
        scu_message = db.get_message_by_name('SCU')

        scuData = {}
        for i in scu_message.signal_tree:
            scuData[i] = 0
        scuData["SCU_Drive_Mode_Req"] = SCU_Drive_Mode_Req  # 模式标志位, 0:-不影响; 1:-自动驾驶模式请求（0x120）; 2:-驾驶员-PAD模式请求（0x100）;3：-驾驶员-方向盘模式请求（默认）
        scuData["SCU_ShiftLevel_Req"] = SCU_ShiftLevel_Req  ##挡位，D前：1; N空：2; R倒：3
        scuData["SCU_Steering_Wheel_Angle"] = SCU_Steering_Wheel_Angle  # 转向角度，±120.0（0.1）实际上发送的命令可以超过这个范围
        scuData["SCU_Target_Speed"] = SCU_Target_Speed  # 目标速度, 0.0-20.0km/h（0.1）
        scuData["SCU_Brk_En"] = SCU_Brk_En  # 是否刹车，否：0；是：1


        data = scu_message.encode(scuData)  #对消息进行编码，里面的参数是个字典
        # 得到发送的帧id
        # print(scu_message.frame_id)
        # scu_message.frame_id=0x51


        message = can.Message(arbitration_id=scu_message.frame_id, data=data, is_extended_id=False) #创建消息

        return message


    def __init_sendMsg(self,SCU_Drive_Mode_Req,SCU_ShiftLevel_Req,SCU_Steering_Wheel_Angle,SCU_Target_Speed,SCU_Brk_En):
        """
            (调用modify函数实现)小车会以50ms周期一直发送该指令，直到发送下一指令或函数Stop_Flag=0，周期设置参考自动驾驶说明第一页最后一句话
        :param SCU_Drive_Mode_Req:
        :param SCU_ShiftLevel_Req:
        :param SCU_Steering_Wheel_Angle:
        :param SCU_Target_Speed:
        :param SCU_Brk_En:
        :return:返回task，方便调用后续调用modify
        """
        if self.Car_Switch == 0:
            print('当前节点为信息读取节点')
        else:
            # 2021.4.5 速度安全限制
            if SCU_Target_Speed >=10.0:
                SCU_Target_Speed = 10.0
                print("warning：触发速度安全限制10km/h,from classOfYunLe.py")

            # 转换成CAN帧，发送
            message = self.__msgInit(SCU_Drive_Mode_Req, SCU_ShiftLevel_Req, SCU_Steering_Wheel_Angle, SCU_Target_Speed, SCU_Brk_En)

            print('发送的信息：', message)

            #send_periodic在特定的时间内每个特定时间发送消息，返回CyclicSendTaskABC类型的任务 period： 每条消息间隔时间（秒）duration：消息发送的总时长，如果不设置，会一直发送，store_task: 默认设置为true，会附加到bus实例中管理
            task = can_bus.send_periodic(message, period=0.05, duration=None, store_task=True)
            print('已开启sendperiod线程')
            # 假设task线程在本函数结束以后不停止
            return task

    def sendMsg(self, SCU_Drive_Mode_Req, SCU_ShiftLevel_Req, SCU_Steering_Wheel_Angle, SCU_Target_Speed, SCU_Brk_En):
        """
            发送车辆控制指令
        :param SCU_Drive_Mode_Req:
        :param SCU_ShiftLevel_Req:
        :param SCU_Steering_Wheel_Angle:
        :param SCU_Target_Speed:
        :param SCU_Brk_En:
        :return:
        """
        SCU_Steering_Wheel_Angle = SCU_Steering_Wheel_Angle - self.error
        message = self.__msgInit(SCU_Drive_Mode_Req, SCU_ShiftLevel_Req, SCU_Steering_Wheel_Angle,
                               SCU_Target_Speed, SCU_Brk_En)
        self.task.modify_data(message) # 修改data首字节,不改变周期发送，仅仅改变发送的数据
        pass

    def receMsg(self):
        # 回调函数
        #recv() 返回的就是can.Message类，和上面的can.Message方法的参数一致
        get_data = can_bus.recv()   # recv()函数只管接收信息，会把CAN总线中所有的信息读取进来


        #0x51为车辆状态参数介绍 0X51，参考自动驾驶使用说明第五页表格的表头
        #0x51对应dbc文件中BO_ 81 CCU_Status: 8 VCU的十进制81，与第五页表格内容对应
        """
        encode为编码
        >>> foo = db.get_did_by_name('Foo')
        >>> foo.encode({'Bar': 1, 'Fum': 5.0})
        b'\\x01\\x45\\x23\\x00\\x11'
        
        decode_message为解码
        >>>db.decode_message(158, b'\\x01\\x45\\x23\\x00\\x11')
        {'Bar': 1, 'Fum': 5.0}
        db.decode_message('Foo', b'\\x01\\x45\\x23\\x00\\x11')
        {'Bar': 1, 'Fum': 5.0}
        """
        # 设计成当读到0x51的时候返回，否则继续读
        while 1:
            if get_data.arbitration_id == 0x10FF01D2:  # 在这里进行信息的分拣
                #print("接收到了车辆状态信息")
                work_type = db.decode_message(get_data.arbitration_id, get_data.data)
                print(work_type)


            if get_data.arbitration_id == 0x10FF02D2:  # 在这里进行信息的分拣
                #print("接收到了车辆状态信息")
                taisheng_type = db.decode_message(get_data.arbitration_id, get_data.data)
                print(taisheng_type)

            if get_data.arbitration_id == 0x10FF03D2:  # 在这里进行信息的分拣
                #print("接收到了车辆状态信息")
                fault_type = db.decode_message(get_data.arbitration_id, get_data.data)
                print(fault_type)

            if get_data.arbitration_id == 0x10FF04D2:  # 在这里进行信息的分拣
                #print("接收到了车辆状态信息")
                jiju_staus_type = db.decode_message(get_data.arbitration_id, get_data.data)
                print(jiju_staus_type)

            get_data = can_bus.recv()


    @staticmethod
    def recv_msg_0x2AA():
        get_data = can_bus.recv()
        while 1:
            if get_data.arbitration_id == 0x2AA: #0X2AA对应dbc文件中BMU_System_Info十进制682
                BMU_System_Info = db.decode_message(get_data.arbitration_id, get_data.data)
                print(BMU_System_Info)
                #time.sleep(0.1)
                return BMU_System_Info
            else:
                get_data = can_bus.recv()


if __name__ == "__main__":
    car_rec = Car(0)

    time.sleep(3)
    while True:
        car_rec.receMsg()
        #time.sleep(0.2)
    #time.sleep(3)
