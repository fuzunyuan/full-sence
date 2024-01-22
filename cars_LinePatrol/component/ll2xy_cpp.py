# -*- coding: utf-8 -*-
import ctypes
import time
# 指定动态链接库
# lib = ctypes.cdll.LoadLibrary('/home/wen/PycharmProjects/dongFengProj_1.1/component_0/component/ll2xy_core.so')  # 这个库的地址是相对于调用模块的地址，后期写一个地址传递
lib = ctypes.cdll.LoadLibrary('/home/fuzunyuan/Downloads/lovol2404_last_1116/component/ll2xy_core.so')  # 这个库的地址是相对于调用模块的地址，后期写一个地址传递


#需要指定返回值的类型，默认是int
# lib.my_calculate.restype = ctypes.c_double
lib.my_trans.restype = ctypes.c_int
lib.x_is.restype = ctypes.c_double
lib.y_is.restype = ctypes.c_double


class Test(object):
    def __init__(self):
        # 动态链接对象
        #self.obj = lib.test_new()   # 这里调用了cpp的函数，返回了一个对象
        self.obj1 = lib.trans()

    def calculate(self, a, b):
        print(lib.my_trans(self.obj1, a, b))
        return lib.x_is(), lib.y_is()
        #res = lib.my_calculate(self.obj, a, b,c,d,e,f)
        #return res

#将python类型转换成c类型，支持int, float,string的变量和数组的转换
def convert_type(input):
    ctypes_map = {int:ctypes.c_int,
              float:ctypes.c_double,
              str:ctypes.c_char_p
              }
    input_type = type(input)
    if input_type is list:
        length = len(input)
        if length==0:
            print("convert type failed...input is "+input)  # 当列表为空时返回这个
            return None
        else:
            arr = (ctypes_map[type(input[0])] * length)()
            for i in range(length):
                arr[i] = bytes(input[i],encoding="utf-8") if (type(input[0]) is str) else input[i]
            return arr
    else:
        if input_type in ctypes_map:
            return ctypes_map[input_type](bytes(input,encoding="utf-8") if type(input) is str else input)
        else:
            print("convert type failed...input is "+input)
            return None


t = Test()


def transform_cpp(longitude, lattitude):
    x, y = t.calculate(convert_type(longitude), convert_type(lattitude))
    return x, y


if __name__ == '__main__':
    # 测试访问是否正常
    t = Test()
    # (40.005572, 116.349803), (40.005527, 116.349804), (40.005485, 116.349806)
    lon = 116.349803
    lat = 40.005572
    x,y = t.calculate(convert_type(lon), convert_type(lat))
    print("x,y=")
    print(x, y)

    lon = 116.349804
    lat = 40.005527
    x, y = t.calculate(convert_type(lon), convert_type(lat))
    print("x,y=")
    print(x, y)

    # 40.005335, 116.349835
    lon = 116.349806
    lat = 40.005485
    x, y = t.calculate(convert_type(lon), convert_type(lat))
    print("x,y=")
    print(x, y)
    # [(444503.6815634969, 4428578.091039354), (444503.73047871405, 4428573.095795492), (444503.86717714014, 4428568.432903707), (444503.91123426403, 4428562.771710446), (444504.3869197557, 4428557.7733532265), (444506.2209828042, 4428551.76610901), (444508.5736507279, 4428546.643062042), (444510.16704073234, 4428542.746526807), (444511.8555019847, 4428540.181268234), (444511.1524281005, 4428537.411459755), (444508.5901833002, 4428537.20815549), (444506.8187973662, 4428540.107012169), (444506.5049104183, 4428543.883216399), (444506.10243094625, 4428547.216077034), (445036.7208032364, 4428669.456629878)]

