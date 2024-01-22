"""
    pyproj算法复杂度太高，程序在这里时间开销太大
    tip：初始化的内容放在模块的前部，在模块一加载的时候就进行初始化
    更新：6.26 替换ll2xy函数，使用cpp混合编程
    更新：10.10更换pyproj包的使用方式，速度提升极大。202110101748更新完毕
"""
# from pyproj import Proj, transform
from pyproj import Transformer
import time
import component.ll2xy_cpp as ll2xy_cpp


# WGS84 = Proj(init='EPSG:4326')  # WGS84
# p = Proj(init="EPSG:32650")  # UTM 50N
transformer = Transformer.from_crs("epsg:32650", "epsg:4326")


def ll2xy(lat, lon):
    # lon = 116.35604 #经度东经，经度范围0-180
    # lat = 40.00643  #纬度北纬，纬度范围0-90
    # 对应xy为445036.7208032364 4428669.456629878
    # cpp test data:445036.720803445037 UTMN=4428669.456630445037

    #x, y = transform(WGS84, p, lon, lat)   # 平均调用一次50ms
    x, y = ll2xy_cpp.transform_cpp(lon, lat)  # 平均调用一次0.03ms
    return x, y


def xy2ll(x, y):
    # lon, lat = transform(p, WGS84, x, y)
    lat, lon = transformer.transform(x, y)
    return lon, lat


if __name__ == "__main__":
    # print(ll2xy(40.348790, 116.859382))     #  a
    # print(ll2xy(40.346937, 116.859309))     #  b


    pass
    # print(ll2xy(40.005614, 116.349793)) #(444502.86203611083, 4428582.758912923)
    # print(ll2xy(40.004659, 116.349871)) #(444508.74640294677, 4428476.713391734)
    # print("打印结束")
    # print(xy2ll(444508.691150676, 4428477.709124328))   # (116.34987026761654, 40.004667967137486)
    # print("二次打印结束")
    #
    # print(xy2ll(445036.7208032364, 4428669.456629878))
    print(xy2ll(570936.32, 4089403.10))
    print(xy2ll(570936.80, 4089403.35))

    #
    #
    # path_lat_lon = [(40.005572, 116.349803), (40.005527, 116.349804), (40.005485, 116.349806), (40.005434, 116.349807),
    #                 (40.005389, 116.349813), (40.005335, 116.349835), (40.005289, 116.349863), (40.005254, 116.349882),
    #                 (40.005231, 116.349902), (40.005206, 116.349894), (40.005204, 116.349864), (40.005230, 116.349843),
    #                 (40.005264, 116.349839), (40.005294, 116.349834), (40.00643, 116.35604)]
    # path_x_y = []
    # for i in path_lat_lon:
    #     x, y = ll2xy(i[0], i[1])
    #     path_x_y.append((x, y))
    # print(path_x_y)
