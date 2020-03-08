#include <iostream>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.h"

int main() {
    using namespace std;
    using namespace cv;
    Mat img = imread("D:\\GeePlayerDownload\\picture1.png");
    // imshow("Armor", mat);

    DetectorParam param;
    ArmorDetector detector(param, ArmorDetector::BLUE);

    VideoCapture cap;
    cap.open("D:\\GeePlayerDownload\\test1.avi");

    clock_t start, end;
    while (true) {
        start = clock();
        cap >> img;
        detector.loadImg(img);
        detector.detect();

        for (const auto &armor : detector.getArmorInfo()) {
            for (int i = 0; i < 4; i++)
                line(img, armor.vertices[i], armor.vertices[(i + 1) % 4], Scalar(0, 0, 255), 5);
        }
        end = clock();

        imshow("rectangles", img);
        if (waitKey(20) > 0) break;

//        int key = waitKey(0);
//        if (key == 115) {
//            string text = "D:\\IMG\\" + to_string(count) + ".png";
//            cout << text << endl;
//            // 保存当前图片
//            imwrite(text, img);
//            count++;
//        } else if (key == 97) {
//            continue;
//        } else if (key > 0) {
//            break;
//        }
    }


    return 0;
}
