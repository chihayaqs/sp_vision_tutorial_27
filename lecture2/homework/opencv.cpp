#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

#include <exception>
#include <iostream>
#include <string>

int main()
{
    try {
        // 初始化相机和 AprilTag 识别器，读取配置中的 auto_charge 部分。
        Camera camera;
        auto_charge::AprilTagDetector detector("./configs/yolo.yaml");

        while (1) {
            // 调用相机读取图像
            cv::Mat img;
            if (!camera.read(img)) {
                if (cv::waitKey(1) == 'q') {
                    break;
                }
                continue;
            }

            // 识别作业图片中的 AprilTag 标记，画出轮廓并标注 ID。
            auto detections = detector.detect(img);
            for (const auto & detection : detections) {
                tools::draw_points(img, detection.corners, cv::Scalar(0, 255, 0));
                tools::draw_text(
                    img, "ID: " + std::to_string(detection.id), detection.center);
            }

            // 显示图像，按 q 退出。
            cv::resize(img, img, cv::Size(640, 480));
            cv::imshow("img", img);
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        cv::destroyAllWindows();
    } catch (const std::exception & e) {
        std::cerr << "运行失败: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}