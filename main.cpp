#include <opencv2/opencv.hpp>
#include<opencv2/video/tracking.hpp>
#include <stdio.h>
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
//
//    VideoCapture capture("C:\\Users\\Administrator\\Desktop\\RM\\视频\\02.mp4");
//    Mat frame;
//
//    //卡尔曼滤波，初始化
//    const int stateNum = 4;//state(x,y,detaX,detaY)
//    const int measureNum = 2;//measurement(x,y)
//    KalmanFilter KF(stateNum, measureNum, 0, CV_32F);
////    Mat state (stateNum, 1, CV_32FC1);
////    Mat processNoise(stateNum, 1, CV_32F);
//    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
//    //随机生成一个矩阵，期望是0，标准差为0.1;
//    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
//    //F 状态转移矩阵
//    KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,
//            0, 1, 0, 1,
//            0, 0, 1, 0,
//            0, 0, 0, 1);
//    //setIdentity: 单位对角矩阵;
//    setIdentity(KF.errorCovPost, Scalar::all(1));//P(k|k-1) 预测估计协方差矩阵矩阵
//    setIdentity(KF.measurementMatrix);//H 观测矩阵,把真实状态空间X映射成观测空间
//    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Qk(Q) 估计噪声协方差矩阵
//    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//Rk(R) 观测噪声协方差矩阵
//
//    for (;;) {
//        capture >> frame;
//        if (frame.empty()) {
//            printf("end capture,break\n");
//            break;
//        }
//
//        DetectorParam param;
//        ArmorDetector detector(param, ArmorDetector::RED);
////        detector.loadImg(img);
//        detector.loadImg(frame);
//        detector.detect();
//
//
//        for (const auto &armor : detector.getArmorInfo()) {
//            for (int i = 0; i < 4; i++)
////                line(img, armor.vertices[i], armor.vertices[(i + 1) % 4], Scalar(0, 0, 255), 5);
//                line(frame, armor.vertices[i], armor.vertices[(i + 1) % 4], Scalar(0, 0, 255), 5);
//
//            cv::circle(frame, Point((armor.m_leftLight.center.x + armor.m_rightLight.center.x) / 2,
//                                    (armor.m_leftLight.center.y + armor.m_rightLight.center.y) / 2),
//                       10, cv::Scalar(0, 255, 0), 3);//画个圈
//
//            //预测模型
//            Mat prediction = KF.predict();
//            Point predict_pt = Point((int) prediction.at<float>(0), (int) prediction.at<float>(1));
//            //观测
//            double center_x = (armor.m_leftLight.center.x + armor.m_rightLight.center.x) / 2;
//            double center_y = (armor.m_leftLight.center.y + armor.m_rightLight.center.y) / 2;//装甲板中心坐标
//            measurement.at<float>(0) = (float) center_x;
//            measurement.at<float>(1) = (float) center_y;
//            KF.correct(measurement);
//            //绘制最优预测点
//            circle(frame, predict_pt, 6, Scalar(255, 0, 0), -1);
//
//            center_x = (int) prediction.at<float>(0);
//            center_y = (int) prediction.at<float>(1);
//        }
//        imshow(" ", frame);
//        if ((cv::waitKey(35) & 0xff) == 27) {
//            printf("Exit this program.\n");
//            break;
//        }
//    }
//    capture.release();
//    cv::destroyAllWindows();

    return 0;
}
