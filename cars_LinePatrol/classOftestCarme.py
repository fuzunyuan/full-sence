import cv2

# 打开默认摄像头（通常是电脑内置摄像头）
cap = cv2.VideoCapture(2)  # 后面
cap1 = cv2.VideoCapture(4) # 前面


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
    
    # 图像做垂直翻转加水平翻转
    frame1 = cv2.flip(frame1,-1)
    
    # 显示结果帧
    cv2.imshow('Camera', frame)
    cv2.imshow('Creame1',frame1)
    # 按 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 完成所有操作后，释放摄像头
cap.release()
cv2.destroyAllWindows()

if __name__=="__main__":
    print("打开摄像头")