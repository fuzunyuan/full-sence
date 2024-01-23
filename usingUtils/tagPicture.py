import cv2
import os

'''用来标记每一帧的图像的范围'''
# 图片文件夹路径
image_folder_path = './tempPicture'

# 要标记的区域（左上角x坐标，左上角y坐标，右下角x坐标，右下角y坐标）
mark_area = (430, 80, 1500, 870)

# 创建保存标记图片的文件夹
marked_folder = 'marked_images'
if not os.path.exists(marked_folder):
    os.makedirs(marked_folder)

# 遍历文件夹中的所有图片
for filename in os.listdir(image_folder_path):
    if filename.endswith(('.png', '.jpg', '.jpeg')): # 检查文件格式
        img_path = os.path.join(image_folder_path, filename)
        img = cv2.imread(img_path)

        # 在图片上画蓝色的框
        cv2.rectangle(img, (mark_area[0], mark_area[1]), (mark_area[2], mark_area[3]), (255, 0, 0), 2)

        # 保存标记过的图片
        cv2.imwrite(os.path.join(marked_folder, filename), img)

print("所有图片标记完成")

if __name__ == "__main__":
    print("执行完毕")