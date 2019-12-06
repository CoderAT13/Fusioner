#ifndef FUSIONER_H
#define FUSIONER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/aruco.hpp>
#include "utility.h"
 
#define SRC_WIDTH 1280
#define SRC_HEIGHT 960
#define S_WIDTH 320
#define S_HEIGHT 240
#define DST_WIDTH 800
#define DST_HEIGHT 600

using namespace std;
using namespace cv;
using namespace cv::detail;

struct ImageAruco
{
    Mat image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    Size size;
    Size first_size;
    Rect roi;
    Mat mask;
    Mat R;
    Mat T;
    Mat iT;
    double scale;
    int findIds(int id)
    {
        for (int i = 0; i < ids.size(); i++)
        {
            if (ids[i] == id)
                return i;
        }
        return -1;
    }
};

class Fusioner{
private:
    vector<ImageAruco> arucos;
    Mat resultImage;

    Mat dstR;
    Mat idstR;
    double dstScale, dstScaleQ;

    /**
     * @brief 寻找两张图片id匹配的aruco
     * 
     * @param src1 需要匹配的aruco
     * @param src2 需要匹配的aruco
     * @return vector<pair<int, int>> 匹配的aruco在ImageAruco.corners里的idx对
     */
    vector<pair<int, int>> match(ImageAruco src1, ImageAruco src2);

    /**
     * @brief 通过first的绝对roi及aruco相对位置，找到second的绝对roi
     * 
     * @param first 
     * @param second 
     * @return Rect Second的绝对roi
     */
    Rect findRoi(ImageAruco first, ImageAruco second);

    /**
     * @brief 计算src1相对于src0的旋转角度、以及resize src2（2019.12.2效果不好删除）
     * 
     * @param src0 
     * @param src1 
     * @return double src1的旋转角度（360）
     */
    double compute(ImageAruco& src0, ImageAruco& src1);

    /**
     * @brief 获取原图间坐标转换
     * 
     * @param x 输入横坐标 x
     * @param y 输入纵坐标 y
     * @param scale resize比例
     * @param isInverse 是否逆转换
     * @return Point2f 转换后的点
     */
    Point2f getOrigin(int x, int y,  bool isInverse = false);

    /**
     * @brief 处理结果图像到 800 * 600 留白边，计算 dstR 和 idstR
     * 
     * @param src 
     */
    void processDst(Mat& src);

    /**
     * @brief 根据现有的ImageAruco数据得到一张结果图，计算 T 和 iT
     * 会把arucos的image release
     */
    void getResult();
public:

    /**
     * @brief 初始化
     * 
     * @param srcs 输入带aruco的图片
     * 
     * 会把srcs清空释放内存
     */
    void init(vector<Mat>& srcs);

    /**
     * @brief 换一组图片更新（可不带Aruco）
     * 
     * @param srcs 
     */
    void changeImage(vector<Mat>& srcs);


    /**
     * @brief 展示结果图片
     * 
     */
    void showResult();

    /**
     * @brief 获取结果图片
     * 
     * @return Mat 
     */
    Mat getResultImage();

    /**
     * @brief 320 * 240 <==> 800 * 600的坐标转换
     * 
     * @param src 
     * @param idx 原图序号
     * @param isInverse false：320 * 240 <==> 800 * 600，true： 800 * 600 <==> 320 * 240
     * @return Point2f 转换后的点
     */
    Point2f getDstPos(Point2f src, int idx,  bool isInverse = false);

};

#endif