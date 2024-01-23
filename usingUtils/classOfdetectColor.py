import cv2
import numpy as np
import math


frame = cv2.imread("./color2.png")
hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # 转换为HSV
gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

average_brightness = cv2.mean(gray_image)[0]

print("average_brightness = " + str(average_brightness))
lower_white = np.array([80, 120, 30])
upper_white = np.array([100, 255, 255])

try:
    mask = cv2.inRange(hsv, lower_white, upper_white)  # 提取白色

    contours, _ = cv2.findContours(cv2.cvtColor(cv2.bitwise_and(frame, frame, mask=mask), cv2.COLOR_RGB2GRAY),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 寻找物体边界
    print("contors = " + str(contours))
    if not contours: # 找到不到这个颜色
        print("bad3")
        lower_white = np.array([60, 120, 30])
        upper_white = np.array([100, 255, 255])

    # TODO:这个地方加一个try-except，防止突然挂掉
    cmax = max(contours, key=cv2.contourArea)
    if not cmax:
        print("bad4")
    print("cmax = " + str(cmax))

    M = cv2.moments(cmax)  # 矩阵无法使用

    if all(value == 0 for value in M.values()):
        print("bad1")


    x, y = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])  # 中心点的X

except Exception:
    print("进入到识别不到的状态")


image_y, image_x = int(frame.shape[0] / 2), int(frame.shape[1] / 2)
central_point = (image_x, image_y)  # 画面中心点

cv2.drawContours(frame, cmax, -1, (0, 0, 255), -1)

cv2.line(frame, (image_x, frame.shape[0]), (image_x, image_y), (255, 0, 255), 1, 4)
cv2.line(frame, (image_x, frame.shape[0]), (x, y), (255, 255, 0), 1, 4)
cv2.circle(frame, (x, y), 1, (0, 0, 255), 10)
cv2.circle(frame, central_point, 1, (0, 255, 0), 10)

line = pow(pow(x - image_x, 2) + pow(abs(frame.shape[0] - y), 2), 0.5)
sin = (x - image_x) / line  # 算出夹角sin值
angle = math.degrees(math.asin(sin))  # 算出角度


cv2.putText(frame, str(angle), (image_x, frame.shape[0]), cv2.FONT_HERSHEY_COMPLEX, 1.5, (0, 0, 255), 3)
cv2.imwrite('12.png', frame)

if __name__ == "__main__":
    print("123")