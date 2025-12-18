#include <opencv2/opencv.hpp>
#include <iostream>
#include "onnxruntime_cxx_api.h"

int main()
{
    // 打开默认 USB 摄像头（通常是 /dev/video0）
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera" << std::endl;
        return -1;
    }

    // 设置分辨率（可选）
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame;

    while (true) {
        cap >> frame; // 读取一帧
        if (frame.empty()) {
            std::cerr << "Error: Empty frame" << std::endl;
            break;
        }

        cv::imshow("USB Camera", frame);

        // 按 q 或 ESC 退出
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
