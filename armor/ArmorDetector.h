// Created by Mahay on 2020/3/3.

#ifndef ROBOMASTER_ARMORDETECTOR_H
#define ROBOMASTER_ARMORDETECTOR_H

#include <opencv2/core.hpp>
#include <utility>
#include <vector>

struct LightDescriptor {
    float width;
    float height;
    float angle;
    cv::Point2f center;
    float area;
    cv::RotatedRect rect;

    LightDescriptor() {}

    explicit LightDescriptor(cv::RotatedRect &rr) {
        width = rr.size.width;
        height = rr.size.height;
        angle = rr.angle;
        center = rr.center;
        area = rr.size.area();
        rect = rr;
    }
};

struct DetectorParam {
    /* 用于识别灯条 */
    double m_threshold = 140.0;
    double m_lightMinArea = 15.0;
    float m_lightMinHWRatio = 0.7;  // 最小长宽比
    double m_lightMinSolidity = 0.5;  // 最小凸度，外界矩形面积 / 轮廓面积

    /* 用于匹配装甲板 */
    float m_lightMaxAngleDiff = 13.0;  // 两灯条最大角度差
    float m_lightMaxHeightDiffRatio = 0.3;  // 两灯条最大长度差比率
    float m_armorMinDisRatio = 1.0;
    float m_armorMaxDisRatio = 3;
    float m_lightMaxYDiffRatio = 0.6;

    // 标准小装甲模块两灯条距离与灯条长度比
    // 用于识别装甲类型（单项赛不需要）和计算装甲位置得分
    float m_smallArmorRatio = 2;

    // 识别装甲板上的数字分数最小值
    // 小于这个值则认为装甲板上没有数字
    float m_numberDetectMinScore = 0.6;
};

class ArmorDescriptor {
private:
    DetectorParam m_param;
    float m_rotScore;
    cv::Mat m_frontImg;
    cv::Mat m_grayImg;
public:
    cv::RotatedRect m_leftLight;
    cv::RotatedRect m_rightLight;
    std::vector<cv::Point2f> vertices;   // 装甲的四个顶点，不包括灯条
    ArmorDescriptor(const LightDescriptor &light1, const LightDescriptor &light2, float rotScore,
                    const cv::Mat &grayImg, const DetectorParam &param);
    int isRealArmor(std::vector<cv::Mat> &tmpl);
    void getFrontImg();
};

class ArmorDetector {
public:
    // 阵营颜色
    enum side {
        RED, BLUE
    };
    // 识别状态结果
    enum result {
        ARMOR_NO,
        ARMOR_FOUND
    };

    // tmplPath为装甲板数字匹配模板路径
    ArmorDetector(const DetectorParam &param, side enemy, const std::string &tmplPath = "../template/");
    void loadImg(const cv::Mat &mat);
    int detect();
    std::vector<ArmorDescriptor> &getArmorInfo();

private:
    cv::Mat m_img;
    side m_enemyColor;
    DetectorParam m_param;
    std::vector<ArmorDescriptor> m_armorInfos;
    std::vector<cv::Mat> m_tmpl;
};

#endif //ROBOMASTER_ARMORDETECTOR_H
