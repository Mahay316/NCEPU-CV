#include <opencv2/opencv.hpp>
#include "armor/ArmorDetector.h"
#include "pose/PoseSolver.h"

int main() {
    using namespace std;
    using namespace cv;

    Mat img = imread("../armor/armor4.png");

    DetectorParam param;
    ArmorDetector detector(param, ArmorDetector::RED);
    detector.loadImg(img);
    detector.detect();

    SolverParam s_param;
    PoseSolver solver(s_param, "../calib/calib.xml");

    for (const auto &armor : detector.getArmorInfo()) {
        for (int i = 0; i < 4; i++)
            line(img, armor.vertices[i], armor.vertices[(i + 1) % 4], Scalar(0, 0, 255), 5);
        solver.setBulletSpeed(10000);
        solver.solve(armor.vertices);
        Point2f pnt((armor.vertices[2].x + armor.vertices[1].x) / 2,
                (armor.vertices[0].y + armor.vertices[1].y) / 2);
        solver.solve(pnt);
    }
    imshow("rectangles", img);
    waitKey(0);

    return 0;
}
