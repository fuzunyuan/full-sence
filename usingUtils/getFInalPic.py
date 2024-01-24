from PIL import Image
import os

'''将所用的图片图像范围截取出来'''
# 图片文件夹路径
image_folder_path = './extractPic/testFinal2'

# 创建保存裁剪图片的文件夹
cropped_folder = 'resultBase/cropped_images2'
if not os.path.exists(cropped_folder):
    os.makedirs(cropped_folder)

# 要裁剪的区域（左上角x坐标，左上角y坐标，右下角x坐标，右下角y坐标）
crop_area = ((100, 190, 1350, 870))

# 遍历文件夹中的所有图片
for filename in os.listdir(image_folder_path):
    if filename.endswith(('.png', '.jpg', '.jpeg')): # 检查文件格式
        img_path = os.path.join(image_folder_path, filename)
        with Image.open(img_path) as img:
            cropped_img = img.crop(crop_area)
            cropped_img.save(os.path.join(cropped_folder, filename))

print("所有图片裁剪完成")

if __name__ == "__main__":
    print("执行完毕")