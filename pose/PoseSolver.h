// Created by Mahay on 2020/3/25.

#ifndef ROBOMASTER_POSESOLVER_H
#define ROBOMASTER_POSESOLVER_H

#include <vector>
#include <string>
#include <opencv2/core.hpp>

class SolverParam {
public:
    // 装甲板的世界坐标
    std::vector<cv::Point3f> m_coordinateOfArmor;
    // 摄像头与枪管的y坐标偏移量
    int m_pxPerMm;  // 每毫米对应的像素值
    double m_yOffset;  // 单位：mm
    double m_zOffset;  // 单位：mm
    int m_maxDistance;
    double m_gravityCompensation;

    SolverParam() {
        // 单位：mm
        m_coordinateOfArmor = {
                cv::Point3f(-65, 35, 0),  // 左下角
                cv::Point3f(-65, -35, 0),  // 左上角
                cv::Point3f(65, -35, 0),  // 右上角
                cv::Point3f(65, 35, 0),  // 右下角
        };
        m_pxPerMm = 1;
        m_yOffset = 0;
        m_zOffset = 0;
        m_maxDistance = 8500;
        m_gravityCompensation = 0;
    }
};

class PoseSolver {
public:
    enum Mode {
        SINGLE, PNP
    };
    // solve()成员函数的状态码
    enum SolverFlag {
        FLAG_ERROR,          // 解算错误
        FLAG_ANGLE,          // 单点解算成功，只有角度信息
        FLAG_ANGLE_AND_DIS,  // PnP解算成功，包含角度和距离信息
        FLAG_TOO_FAR         // PnP解算失败，目标太远，将导致误差过大
    };
    PoseSolver(const SolverParam &param, const std::string &filename);
    SolverFlag solve(const std::vector<cv::Point2f> &target);
    SolverFlag solve(const cv::Point2f &target);
    void setBulletSpeed(double speed);
    cv::Vec2f getAngle() const;
    double getDistance() const;
    Mode getMode() const;
private:
    Mode mode;  // 解算模式
    double m_pitch;  // 俯仰角
    double m_yaw;  // 偏航角
    double m_distance;  // 欧氏距离
    double m_bulletSpeed;  // 弹丸射速
    SolverParam m_param;  // 解算相关的参数
    cv::Mat m_cameraMatrix = cv::Mat(3, 3, CV_32FC1);  // 相机的内参矩阵
    cv::Mat m_distCoeffs = cv::Mat(1, 5, CV_32FC1);  // 相机的畸变参数
};


#endif //ROBOMASTER_POSESOLVER_H
