import onnxruntime as ort
from collections import deque
import numpy as np
import cv2
import os
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, "YOLOv10n_gestures.onnx")
print(f"Loading model from: {model_path}")  # 调试用


class YOLOv10ONNX:
    """
    手搓的onnx模型推理类，使用YOLOv10n_gestures.onnx模型进行手势识别。
    """

    def __init__(self, min_conf: float = 0.8, onnx_path: str = model_path, imgsz: int = 480,
                 use_gpu: bool = True):
        self.imgsz = imgsz
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if use_gpu else ['CPUExecutionProvider']
        self.session = ort.InferenceSession(onnx_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]
        self.min_conf = min_conf
        self.label = {0: 'grabbing', 1: 'grip', 2: 'holy', 3: 'point', 4: 'call', 5: 'three3', 6: 'timeout', 7: 'xsign',
                      8: 'hand_heart', 9: 'hand_heart2', 10: 'little_finger', 11: 'middle_finger', 12: 'take_picture',
                      13: 'dislike',
                      14: 'fist', 15: 'four', 16: 'like', 17: 'mute', 18: 'ok', 19: 'one', 20: 'palm', 21: 'peace',
                      22: 'peace_inverted', 23: 'rock', 24: 'stop', 25: 'stop_inverted', 26: 'three', 27: 'three2',
                      28: 'two_up',
                      29: 'two_up_inverted', 30: 'three_gun', 31: 'thumb_index', 32: 'thumb_index2', 33: 'no_gesture'}
        self.label2num = {'grabbing': -1, 'grip': -1, 'holy': -1, 'point': -1, 'call': 6, 'three3': 3, 'timeout': -1,
                          'xsign': -1,
                          'hand_heart': -1, 'hand_heart2': -1, 'little_finger': 1, 'middle_finger': 1,
                          'take_picture': -1,
                          'dislike': -1,
                          'fist': -1, 'four': 4, 'like': -1, 'mute': 1, 'ok': 3, 'one': 1, 'palm': 5, 'peace': 2,
                          'peace_inverted': 2, 'rock': -1, 'stop': -1, 'stop_inverted': -1, 'three': 3, 'three2': 3,
                          'two_up': -1,
                          'two_up_inverted': -1, 'three_gun': -1, 'thumb_index': 8, 'thumb_index2': -1,
                          'no_gesture': -1}
        print("模型初始化完成，使用设备:", "GPU" if use_gpu else "CPU")

    def preprocess(self, img):
        """
        对输入图像进行预处理，包括调整大小、填充和归一化。
        """
        self.original_image = img.copy()
        h0, w0 = img.shape[:2]
        r = self.imgsz / max(h0, w0)
        new_unpad = (int(w0 * r), int(h0 * r))
        img_resized = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        dw, dh = self.imgsz - new_unpad[0], self.imgsz - new_unpad[1]
        dw /= 2
        dh /= 2
        img_padded = cv2.copyMakeBorder(img_resized, int(dh), int(dh), int(dw), int(dw),
                                        cv2.BORDER_CONSTANT, value=(114, 114, 114))
        img_rgb = cv2.cvtColor(img_padded, cv2.COLOR_BGR2RGB)
        img = img_rgb.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[None]  # shape (1,3,H,W)
        return img, r, dw, dh

    def postprocess(self, pred, r, dw, dh, conf_thres=0.25, iou_thres=0.45):
        """
        对模型预测结果进行后处理，包括坐标转换、置信度筛选和非极大值抑制。
        """
        pred = pred[0]
        boxes, scores, classes = pred[:, :4], pred[:, 4], pred[:, 5].astype(int)
        xyxy = boxes.copy()
        xyxy[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2 - dw) / r
        xyxy[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2 - dh) / r
        xyxy[:, 2] = (boxes[:, 0] + boxes[:, 2] / 2 - dw) / r
        xyxy[:, 3] = (boxes[:, 1] + boxes[:, 3] / 2 - dh) / r
        conf_mask = scores > conf_thres
        xyxy, scores, classes = xyxy[conf_mask], scores[conf_mask], classes[conf_mask]
        indices = cv2.dnn.NMSBoxes(xyxy.tolist(), scores.tolist(), conf_thres, iou_thres)
        if len(indices):
            indices = indices.flatten()
            return xyxy[indices], scores[indices], classes[indices]
        return np.array([]), np.array([]), np.array([])

    def infer(self, img, conf_thres=0.25, iou_thres=0.45, save_path=None):
        """
        对输入图像进行推理，返回检测到的手势框、置信度和类别。
        """
        img, r, dw, dh = self.preprocess(img)
        pred = self.session.run(self.output_names, {self.input_name: img})[0]
        boxes, scores, classes = self.postprocess(pred, r, dw, dh, conf_thres, iou_thres)
        # 按照置信度排序
        indices = np.argsort(scores)[::-1]
        boxes = boxes[indices]
        scores = scores[indices]
        classes = classes[indices]
        if len(boxes) == 0:
            return None, None, None, self.original_image
        if float(scores[0]) < self.min_conf:
            return None, None, None, self.original_image
        # 选取置信度最大的检测结果
        box = boxes[0]
        score = scores[0]
        classes = self.label[classes[0]]
        # 绘制检测结果
        img_drawn = self.visualize(box, score, classes)
        if save_path:
            cv2.imwrite(save_path, img_drawn)
        return box, score, classes, img_drawn

    def visualize(self, box, score, classes):
        """
        在图像上绘制检测结果，包括边界框和类别标签。
        """
        img = self.original_image.copy()
        cv2.putText(img, f"{classes} conf{score:.2f}", (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
        # cv2.imshow("YOLOv10 Gesture Detection", img)
        # cv2.waitKey(1)
        return img

    def detect(self, img, conf_thres=0.25, iou_thres=0.45, save_path=None):
        """
        对输入图像进行手势检测，返回检测到的手势类别编号。
        """
        box, score, classes, img_drawn = self.infer(img, conf_thres, iou_thres, save_path)
        if classes is None:
            return -1
        return self.label2num[classes]

    def stable_detect(self, cap):
        queue = deque([-1, -1, -1], maxlen=3)
        while True:
            frame = cap.get_raw_frame()
            if frame is None:
                continue
            queue.append(self.detect(frame))
            queue_result = queue[0] if all(x == queue[0] for x in queue) else -1
            if queue_result != -1:
                return queue_result


if __name__ == "__main__":
    from camera import VideoCamera

    # cap = cv2.VideoCapture('output.avi')
    cap1 = VideoCamera()
    gestures_model = YOLOv10ONNX()

    # frame_count=0
    # count_time=time.time()
    # while time.time()-count_time<=30:
    #     start = time.time()
    #     frame = cap1.get_raw_frame()
    #     if frame is None:
    #         break
    #     gestures_num = gestures_model.detect(frame)
    #     if frame_count % 10 == 0:
    #         cv2.imwrite(f'result_{frame_count}.jpg', frame)
    #     fps = 1.0 / (time.time() - start)
    #     frame_count+=1
    #     print(gestures_num,fps)

    while True:
        print(gestures_model.stable_detect(cap1))
