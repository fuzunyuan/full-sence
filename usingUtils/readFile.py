'''function : 读取文件的每一行,并且显示出来'''
import matplotlib.pyplot as plt

result = []


with open('detectLogs/2024-01-05 11:01:52.041450.txtangle.txt', 'r') as file:
    n = 0
    for line in file:
        s = line.strip()  # 使用 strip() 去除行尾的换行符
        s = s.split(" ")
        result.append(float(s[-1]))

print(result)

# 解析数据
frames = list(range(1, len(result) + 1))

# 绘制图表
plt.figure(figsize=(15, 6))
plt.plot(frames, result, marker='o')
plt.title('angleChange')
plt.xlabel('EveryPicture')
plt.ylabel('angle')
plt.grid(True)
plt.show()

if __name__ == "__main__":
    print("读取每一行")