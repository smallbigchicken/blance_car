import os
import sys

# 打印当前 Python 解释器信息，便于调试与环境确认
print(f"当前 Python 解释器路径: {sys.executable}")
print(f"当前环境目录 (prefix): {sys.prefix}")

import time
import YOLOv10n_gestures
import camera
from collections import deque

# 命名管道路径（与 C++ 端约定）
pipe_path = "/tmp/my_pipe"

# 如果管道不存在则尝试创建（mkfifo），存在则忽略异常
if not os.path.exists(pipe_path):
    try:
        os.mkfifo(pipe_path)
    except FileExistsError:
        pass

print("Opening pipe (waiting for C++ reader)...")
try:
    # 以写模式打开命名管道：该调用会阻塞直到有读端打开（C++ 程序以 ifstream 打开）
    pipe_fd = os.open(pipe_path, os.O_WRONLY)
except OSError as e:
    # 无法打开管道时退出（例如权限或路径问题）
    print(f"Error opening pipe: {e}")
    sys.exit(1)

# 初始化摄像头与手势识别模型
cap1 = camera.VideoCamera()
gestures_model = YOLOv10n_gestures.YOLOv10ONNX()

# 使用长度为2的队列做简单去抖（连续两帧相同才视为有效）
queue = deque([-1, -1], maxlen=2)

try:
    while True:
        # 从摄像头读取原始帧（调用封装的接口）
        frame = cap1.get_raw_frame()
        if frame is None:
            # 若未获取到帧，稍作等待避免忙等
            time.sleep(0.1)
            continue

        # 对当前帧进行手势检测，得到业务编号（-1 表示无有效手势）
        det_result = gestures_model.detect(frame)
        queue.append(det_result)

        # 简单滤波：仅当队列内所有值相同才认为稳定
        queue_result = queue[0] if all(x == queue[0] for x in queue) else -1

        # 准备写入消息（换行分隔，便于 C++ 端按行读取）
        msg = str(queue_result) + "\n"

        # 将消息写入命名管道（阻塞或抛出 BrokenPipeError）
        os.write(pipe_fd, msg.encode())

        # 控制循环频率，避免占用过高 CPU
        time.sleep(0.05)

except BrokenPipeError:
    # 当 C++ 读端关闭管道时，写操作会触发此异常，应当优雅退出
    print("Reader closed pipe. Exiting...")
finally:
    # 关闭文件描述符并释放可能的资源
    os.close(pipe_fd)
    # cap1.release() # 若 VideoCamera 提供释放方法，请在此调用以释放摄像头资源
