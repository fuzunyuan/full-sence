import cv2
import os

'''function ： 用来截取每一帧的图像'''
# 视频文件路径
video_path = 'videoBase/红色车道线.mp4'

# 创建保存图片的文件夹
save_folder = 'extractPic/temp1'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

# 读取视频
cap = cv2.VideoCapture(video_path)

# 检查是否成功打开视频文件
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# 设置提取帧的时间间隔（秒）
interval = 0.1

# 视频的帧率
fps = cap.get(cv2.CAP_PROP_FPS)

# 计算在每个间隔需要跳过的帧数
frame_skip = int(round(fps * interval))

# 用于命名图片的计数器
count = 1

while True:
    # 读取指定数量的帧
    for _ in range(frame_skip):
        ret, frame = cap.read()
        if not ret:
            break

    # 如果成功读取帧，则保存
    if ret:
        cv2.imwrite(os.path.join(save_folder, f"{count}.jpg"), frame)
        count += 1
    else:
        break

# 释放视频对象
cap.release()

print("帧提取完成")

if __name__ == "__main__":
    print("开始执行......")