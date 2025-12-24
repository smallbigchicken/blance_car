# coding=utf-8
# camera.py
import cv2
import numpy as np


class VideoCamera:
    """
    视频摄像头类，用于捕获视频流并进行图像处理。
    """

    def __init__(self, brightness=60, contrast=30, exposure=60):
        self.cap = cv2.VideoCapture(0)  # 打开默认摄像头
        if not self.cap.isOpened():
            print("Open Camera Error!")
            return
        self.cap.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc("M", "J", "P", "G")
        )  # 设置视频编码格式为MJPG
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)  # 设置亮度
        self.cap.set(cv2.CAP_PROP_CONTRAST, contrast)  # 设置对比度
        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)  # 设置曝光
        self.cap.set(3, 640)  # 设置视频宽度
        self.cap.set(4, 480)  # 设置视频高度
        self.cap.set(5, 30)  # 设置视频帧率

    def __del__(self):
        self.cap.release()

    def get_raw_frame(self):
        ret, image = self.cap.read()
        if not ret:
            print("Capture Video Error!")
            return bytes({1})
        return image

    def get_optimized_frame(self):
        image = self.get_raw_frame()
        if isinstance(image, bytes) and image == bytes({1}):
            return image
        # 对图像进行优化处理
        image = self.white_balance_reference_area(image)
        return image

    @staticmethod
    def white_balance_reference_area(img, mask=None):
        img = img.astype(np.float32)

        if mask is not None:
            # 只使用 mask 中的区域（假设为纸）
            mean_vals = cv2.mean(img, mask=mask)[:3]
        else:
            # 整张图像做白平衡
            mean_vals = cv2.mean(img)[:3]

        scale = [255.0 / v if v > 0 else 1.0 for v in mean_vals]

        for c in range(3):
            img[:, :, c] *= scale[c]

        img = np.clip(img, 0, 255)
        return img.astype(np.uint8)


if __name__ == "__main__":
    camera = VideoCamera()
    import time

    # while True:
    #     frame = camera.get_jpeg_frame()
    #     if frame == bytes({1}):
    #         break
    #     cv2.imshow(
    #         "Camera", cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
    #     )
    #     if cv2.waitKey(1) & 0xFF == ord("q"):
    #         break
    # cv2.destroyAllWindows()
    # del camera
    # print("Camera closed.")

    # frame = camera.get_jpeg_frame()
    # # 保存
    # if frame != bytes({1}):
    #     with open("test.jpg", "wb") as f:
    #         f.write(frame)
    #     print("Frame saved as test.jpg")
    # else:
    #     print("Failed to capture frame.")

    fourcc = cv2.VideoWriter_fourcc(*"XVID")  # 常见编码还有 'MJPG', 'MP4V'
    out = cv2.VideoWriter("output.avi", fourcc, 30, (640, 480))
    start = time.time()
    while time.time() - start < 5:  # 录制5秒视频
        frame = camera.get_raw_frame()
        out.write(frame)  # 写入当前帧
        # cv2.imshow('frame', frame)  # 显示当前帧
        # if cv2.waitKey(1) & 0xFF == ord('q'):  # 按 'q' 退出
        #     break

    camera.__del__()
    out.release()
    # cv2.destroyAllWindows()
