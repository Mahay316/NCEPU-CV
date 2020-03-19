// Created by Mahay on 2020/3/19.
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <iostream>

#define DRAW_CHESSBOARD_CORNER
#define UNDISTORT

using namespace std;
using namespace cv;

void runCalibration(const vector<Mat> &imgList, const Size &size);
void genObjPoints(const Size &boardSize, float squareSize, vector<Point3f> &dst);
bool saveCamParam(const Mat &cameraMatrix, const Mat &distCoeffs, const string &filename);

const float Square = 3;

int main() {
    vector<Mat> imgs;
    imgs.push_back(imread("../board.png"));
//    imgs.push_back(imread("../a1.jpg"));
//    imgs.push_back(imread("../a2.jpg"));
//    imgs.push_back(imread("../a3.jpg"));
    runCalibration(imgs, Size(9, 6));

    return 0;
}

void runCalibration(const vector<Mat> &imgList, const Size &size) {
    vector<vector<Point3f>> objPoints(1);  // 角点世界坐标
    vector<vector<Point2f>> imgPoints;  // 角点图片坐标
    // 每张图片上的角点世界坐标都相同
    // 重复使用一组世界坐标
    genObjPoints(size, Square, objPoints[0]);

    for (const Mat &img : imgList) {
        vector<Point2f> corners;
        // 找到棋盘上所有角点
        bool found = findChessboardCorners(img, size, corners);

#ifdef DRAW_CHESSBOARD_CORNER
        Mat tmp;
        img.copyTo(tmp);
        drawChessboardCorners(tmp, size, corners, found);
        imshow("Chessboard corners", tmp);
        waitKey(0);
#endif

        if (found) {
            Mat grayImg;
            cvtColor(img, grayImg, COLOR_BGR2GRAY);
            // 提高角点坐标精度
            cornerSubPix(grayImg, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.001));

            imgPoints.push_back(corners);
        }
    }

    // 相机内参矩阵
    Mat cameraMatrix(3, 3, CV_32FC1, cv::Scalar::all(0));
    // 相机的5个畸变系数：k1, k2, p1, p2, k3
    Mat distCoeffs(1, 5, CV_32FC1, cv::Scalar::all(0));
    // 世界坐标到相机坐标的旋转矩阵
    std::vector<cv::Mat> rvecsMat;
    // 世界坐标到相机坐标的平移矩阵
    std::vector<cv::Mat> tvecsMat;
    objPoints.resize(imgPoints.size(), objPoints[0]);

    calibrateCamera(objPoints, imgPoints, imgList[0].size(), cameraMatrix, distCoeffs, rvecsMat, tvecsMat);

    // 保存相机内参和畸变系数到文件
    saveCamParam(cameraMatrix, distCoeffs, "../calib.xml");

#ifdef UNDISTORT
    Mat final;
    undistort(imgList[0], final, cameraMatrix, distCoeffs);
    imshow("Final", final);
    waitKey(0);
#endif

}

bool saveCamParam(const Mat &cameraMatrix, const Mat &distCoeffs, const string &filename) {
    FileStorage f(filename, FileStorage::WRITE);
    if (!f.isOpened()) {
        cout << "打开文件失败: " << filename << endl;
        return false;
    }

    f << "CameraMatrix" << cameraMatrix;
    f << "distCoeffs" << distCoeffs;
    f.release();

    return true;
}

// 根据棋盘角点数量生成对应世界坐标
void genObjPoints(const Size &boardSize, float squareSize, vector<Point3f> &dst) {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            dst.push_back(Point3f(float(Square * i), float(Square * j), 0));
        }
    }
}
