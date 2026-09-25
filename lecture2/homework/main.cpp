#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

#include <exception>
#include <iostream>

int main()
{
    try {
        // 初始化相机、yolo类
        Camera camera;
        auto_aim::YOLO yolo("./configs/yolo.yaml", false);
        
        while (1) {
            // 调用相机读取图像
            cv::Mat img;
            if (!camera.read(img)) {
                if (cv::waitKey(1) == 'q') {
                    break;
                }
                continue;
            }

            // 调用yolo识别装甲板
            auto armors = yolo.detect(img);

            // 按关键点顺序画出绿色闭合轮廓。
            for (const auto & armor : armors) {
                tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0));
                // 在装甲板中心标注名称
                tools::draw_text(
                    img, auto_aim::ARMOR_NAMES.at(armor.name),
                    armor.center, cv::Scalar(0, 255, 0));
            }

            cv::resize(img, img, cv::Size(640, 480));
            cv::imshow("img", img);
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        cv::destroyAllWindows();
    } 
    catch (const std::exception & e) {
        // 统一打印初始化或运行异常，已构造的相机自动析构。
        std::cerr << "运行失败: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}