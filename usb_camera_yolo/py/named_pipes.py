import os
import sys
print(f"当前 Python 解释器路径: {sys.executable}")
print(f"当前环境目录 (prefix): {sys.prefix}")
import time
import YOLOv10n_gestures
import camera
from collections import deque

pipe_path = "/tmp/my_pipe"

# 不需要 mkfifo 了，或者保留 try-except 以防万一
if not os.path.exists(pipe_path):
    try:
        os.mkfifo(pipe_path)
    except FileExistsError:
        pass

print("Opening pipe (waiting for C++ reader)...")
try:
    # 这一步会阻塞，直到 C++ 执行 ifstream pipe(pipe_path)
    pipe_fd = os.open(pipe_path, os.O_WRONLY)
except OSError as e:
    print(f"Error opening pipe: {e}")
    sys.exit(1)

cap1 = camera.VideoCamera()
gestures_model = YOLOv10n_gestures.YOLOv10ONNX()
queue = deque([-1, -1], maxlen=2)

try:
    while True:
        frame = cap1.get_raw_frame()
        if frame is None:
            time.sleep(0.1)
            continue

            # 检测手势
        det_result = gestures_model.detect(frame)
        queue.append(det_result)

        # 简单的滤波逻辑
        queue_result = queue[0] if all(x == queue[0] for x in queue) else -1

        msg = str(queue_result) + "\n"

        # 写入管道
        os.write(pipe_fd, msg.encode())

        time.sleep(0.05)

except BrokenPipeError:
    # 当 C++ 程序关闭管道（停止运行）时，Python 写入会触发此错误
    print("Reader closed pipe. Exiting...")
finally:
    os.close(pipe_fd)
    # cap1.release() # 如果有释放资源的函数记得调用