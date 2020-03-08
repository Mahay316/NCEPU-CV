// Created by Mahay on 2020/3/3.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.h"

using namespace std;
using namespace cv;

/**
 * 矫正RotatedRect，使其角度处于(-45, 45]度之间
 * @param rect 待矫正的RotatedRect
 */
void regulateRect(RotatedRect &rect) {
    // 将角度矫正到第一二象限
    while (rect.angle > 90) rect.angle -= 180;
    while (rect.angle <= -90) rect.angle += 180;
    // 将角度矫正到(-45, 45]度之间
    if (rect.angle > 45) {
        swap(rect.size.width, rect.size.height);
        rect.angle -= 90;
    } else if (rect.angle <= -45) {
        swap(rect.size.width, rect.size.height);
        rect.angle += 90;
    }
}

/**
 * 计算两点的欧式距离
 */
float distance(const Point2f &p1, const Point2f &p2) {
    float horDis = abs(p1.x - p2.x);
    float verDis = abs(p1.y - p2.y);

    return sqrt(horDis * horDis + verDis * verDis);
}

ArmorDescriptor::ArmorDescriptor(const LightDescriptor &light1, const LightDescriptor &light2, float rotScore,
                                 const DetectorParam &param) {
    m_leftLight = (light1.center.x < light2.center.x) ? light1.rect : light2.rect;
    m_rightLight = (light1.center.x > light2.center.x) ? light1.rect : light2.rect;

    m_rotScore = rotScore;
    m_param = param;

    cv::Point2f left[4];
    m_leftLight.points(left);
    cv::Point2f right[4];
    m_rightLight.points(right);


    vertices[0] = left[3];  // 装甲左下角
    vertices[1] = left[2];  // 装甲左上角
    vertices[2] = right[1];  // 装甲右上角
    vertices[3] = right[0];  // 装甲右下角
}

ArmorDetector::ArmorDetector(const DetectorParam &param, side enemy) {
    m_param = param;
    m_enemyColor = enemy;
};

/**
 * 加载待处理图片
 * @param img 待处理图片
 */
void ArmorDetector::loadImg(const Mat &img) {
    m_img = img;
    // 清空上一次的计算结果
    m_armorInfos.clear();
}

int ArmorDetector::detect() {
    vector<Mat> channels(3);
    // 分离颜色通道，顺序为BGR
    split(m_img, channels);

    // 分离出灯条
    Mat grayImg;
    if (m_enemyColor == RED) {
        grayImg = channels.at(2) - channels.at(0);
    } else {
        grayImg = channels.at(0) - channels.at(2);
    }

    Mat binaryImg;
    threshold(grayImg, binaryImg, m_param.m_threshold, 255, THRESH_BINARY);
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    // 膨胀，增大识别率
    dilate(binaryImg, binaryImg, kernel);

    // 识别轮廓
    vector<vector<Point>> lightContours;
    findContours(binaryImg, lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 筛选轮廓
    vector<LightDescriptor> lightInfos;
    for (const auto &con : lightContours) {
        double area = contourArea(con);
        // 拟合至少需要5个点，轮廓面积太小则抛弃
        if (con.size() < 5 || area < m_param.m_lightMinArea) continue;

        // 试用椭圆拟合轮廓，得到外接矩形
        RotatedRect lightRect = fitEllipse(con);
        // 矫正旋转矩形，以保证下面的判断正确
        regulateRect(lightRect);
        // 长宽比、凸度太小则抛弃
        if (lightRect.size.height / lightRect.size.width < m_param.m_lightMinHWRatio ||
            area / lightRect.size.area() < m_param.m_lightMinSolidity)
            continue;

        // 符合灯条的约束，保存灯条信息
        lightInfos.emplace_back(lightRect);
    }
    if (lightInfos.empty()) return ARMOR_NO;

    // 遍历灯条，匹配装甲板
    for (int i = 0; i < lightInfos.size(); i++) {
        for (int j = i + 1; j < lightInfos.size(); j++) {
            const LightDescriptor &light1 = lightInfos[i];
            const LightDescriptor &light2 = lightInfos[j];

            // 角度差，同一装甲板的两个灯条近似平行
            float angleDiff = abs(light1.angle - light2.angle);
            float meanHeight = (light1.height + light2.height) / 2;
            // 长度差比率，使用比率可以是筛选更具普适性
            float heightDiffRatio = abs(light1.height - light2.height) / meanHeight;
            if (angleDiff > m_param.m_lightMaxAngleDiff ||
                heightDiffRatio > m_param.m_lightMaxHeightDiffRatio)
                continue;

            // 灯条距离与灯条平均长度之比
            // 过大表示两个灯条并不属于同一装甲板
            // 过小则是拍摄视角太偏，不适合打击
            float disRatio = distance(light1.center, light2.center) / meanHeight;
            float yDiffRatio = abs(light1.center.y - light2.center.y) / meanHeight;
            if (yDiffRatio > m_param.m_lightMaxYDiffRatio ||
                disRatio < m_param.m_armorMinDisRatio ||
                disRatio > m_param.m_armorMaxDisRatio)
                continue;

            // 视角偏的程度
            float disRatioOffset = max(m_param.m_smallArmorRatio - disRatio, 0.0F);
            // 计算装甲板评分，以便后续选择最佳攻击目标
            float rotScore = -(disRatioOffset * disRatioOffset + yDiffRatio * yDiffRatio);


            // 符合装甲的约束，保存装甲信息
            m_armorInfos.emplace_back(light1, light2, rotScore, m_param);
        }
    }
    if (m_armorInfos.empty()) return ARMOR_NO;

    return ARMOR_FOUND;
}

vector<ArmorDescriptor> &ArmorDetector::getArmorInfo() {
    return m_armorInfos;
}
