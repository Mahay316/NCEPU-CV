// Created by Mahay on 2020/3/25.
#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include "PoseSolver.h"

using namespace std;
using namespace cv;


PoseSolver::PoseSolver(const SolverParam &param, const string &filename) {
    m_param = param;

    // 读取相机的内参矩阵和畸变参数
    FileStorage file(filename, FileStorage::READ);

    if (!file.isOpened()) {
        cout << "打开" << filename << "失败！" << endl;
        return;
    }
    file["CameraMatrix"] >> m_cameraMatrix;
    file["DistCoeffs"] >> m_distCoeffs;
}

// 使用PnP解算，至少传入4个点
PoseSolver::SolverFlag PoseSolver::solve(const vector<Point2f> &target) {
    // 标志点数量不足，无法进行解算
    if (target.size() != 4)
        return FLAG_ERROR;

    mode = PNP;
    // 旋转向量
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1);
    // 平移向量
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1);
    //进行位置解算
    solvePnP(m_param.m_coordinateOfArmor, target, m_cameraMatrix, m_distCoeffs, rVec, tVec);

    // cout << "tvec: " << tVec << endl;
    // 目标在相机坐标系的坐标，单位mm
    double x = tVec.at<double>(0, 0);
    double y = tVec.at<double>(1, 0) - m_param.m_yOffset;
    double z = tVec.at<double>(2, 0) - m_param.m_zOffset;

    // 俯仰角，向上为负
    m_pitch = atan(y / z);
    // 偏航角，向左为负
    m_yaw = atan(x / z);
    m_distance = sqrt(x * x + y * y + z * z);

    if (m_distance > m_param.m_maxDistance) {
        // 距离太远，PnP解算误差过大，不可用
        return FLAG_TOO_FAR;
    }

    // 根据距离补偿重力
    // theta为需要补偿的角度
    // alpha为枪管的俯角
    // 公式为 theta = (asin((g*d*cos(alpha)/v^2 + tan(alpha)) / sqrt(1+tan(alpha)*tan(alpha))) - alpha)/2;
    // 由于alpha角无法得知，所以默认为0，则式子化简为 theta = asin(g*d/v^2) / 2
    // 由于未考虑到重力加速度的分量，射击较高或较低处的目标都会产生较大误差
    m_pitch -= asin(9800 * m_distance / (m_bulletSpeed * m_bulletSpeed)) / 2;

    cout << "Pitch: " << m_pitch << " Yaw: " << m_yaw << endl;
    cout << "Distance: " << m_distance << endl;

    return FLAG_ANGLE_AND_DIS;
}

// 使用单点解算
PoseSolver::SolverFlag PoseSolver::solve(const Point2f &target) {
    mode = SINGLE;

    vector<Point2f> in;
    vector<Point2f> out;
    in.push_back(target);
    undistortPoints(in, out, m_cameraMatrix, m_distCoeffs, noArray(), m_cameraMatrix);

    Point2f pnt = out.front();
    // cout << "Original: (" << target.x << ", " << target.y << ")\n";
    // cout << "Undistorted: (" << pnt.x << ", " << pnt.y << ")\n";

    double fx = m_cameraMatrix.at<double>(0, 0);
    double fy = m_cameraMatrix.at<double>(1, 1);
    double cx = m_cameraMatrix.at<double>(0, 2);
    double cy = m_cameraMatrix.at<double>(1, 2);

    // 俯仰角，向上为负
    m_pitch = atan((pnt.y - cy - m_param.m_yOffset) / fy) + m_param.m_gravityCompensation;
    // 偏航角，向左为负
    m_yaw = atan((pnt.x - cx) / fx);

    cout << "Pitch: " << m_pitch << " Yaw: " << m_yaw << endl;

    return FLAG_ANGLE;
}

// 设置弹丸速度，以进行重力补偿
void PoseSolver::setBulletSpeed(double speed) {
    m_bulletSpeed = speed;
}

// 得到将目标转到图像中心需要转动的俯仰角和偏航角
Vec2f PoseSolver::getAngle() const {
    return Vec2f(m_pitch, m_yaw);
}

// 得到相机坐标系原点到目标的距离
double PoseSolver::getDistance() const {
    return m_distance;
}

// 获取解算模式
PoseSolver::Mode PoseSolver::getMode() const {
    return mode;
}
