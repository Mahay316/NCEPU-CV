#include <iostream>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.h"

int main() {
    using namespace std;
    using namespace cv;

    Mat img = imread("../armor3.png");

    DetectorParam param;
    ArmorDetector detector(param, ArmorDetector::RED);
    detector.loadImg(img);
    detector.detect();

    for (const auto &armor : detector.getArmorInfo()) {
        for (int i = 0; i < 4; i++)
            line(img, armor.vertices[i], armor.vertices[(i + 1) % 4], Scalar(0, 0, 255), 5);
    }
    imshow("rectangles", img);
    waitKey(0);

    return 0;
}
