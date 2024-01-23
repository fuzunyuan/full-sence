from datetime import datetime
def log_message(message, file_name="log.txt"):
    """
    将消息记录到指定文件的函数。
    :param message: 要记录的消息。
    :param file_name: 日志将被写入的文件名。
    """
    with open(file_name, "a") as file:
        file.write(message + "\n")

if __name__ == "__main__":
# 示例用法
    log_path = "./detectLogs/"
    log_path = log_path + str(datetime.now()) + ".txt"
    print(log_path)
    log_message("这是一个测试日志消息。",log_path)
    log_message("另一个日志条目。", log_path)
