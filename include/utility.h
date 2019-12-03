#ifndef UTILITY_H
#define UTILITY_H

#include <opencv2/opencv.hpp>

#define LOG(x) std::cout << x << std::endl
#define LOGV(x, y) std::cout << x << ": " << y << std::endl;

using namespace cv;
using namespace std;

/**
 * @brief 获得旋转degree（360）后的图片及旋转矩阵
 * 
 * @param src 
 * @param dst 
 * @param degree 
 * @return Mat 
 */
Mat warpRotation(Mat src, Mat &dst, double degree);

/**
 * @brief 点p通过矩阵R变换
 * 
 * @param p 
 * @param R 
 */
void change(Point2f& p, Mat R);

/**
 * @brief 获得矩阵R偏移后的矩阵
 * 
 * @param R 
 * @param dx 
 * @param dy 
 * @return Mat 
 */

Mat addDisplacement(Mat R, int dx, int dy);

/**
 * @brief 变为3*3方阵
 * 
 * @param M1 
 * @return Mat 
 */
Mat toSquare(Mat M1);

#endif