#include "utility.h"



Mat warpRotation(Mat src, Mat &dst, double degree){
    Point2f center;
	center.x = float(src.cols / 2.0 - 0.5);
	center.y = float(src.rows / 2.0 - 0.5);
 
	double angle = degree  * CV_PI / 180.; // 弧度    
	double a = sin(angle), b = cos(angle);
	int width = src.cols;
	int height = src.rows;
	int width_rotate = int(height * fabs(a) + width * fabs(b));
	int height_rotate = int(width * fabs(a) + height * fabs(b));
 
	Mat M1 = getRotationMatrix2D(center, degree, 1.0);

	Point2f srcPoints1[3];
	Point2f dstPoints1[3];
 
	srcPoints1[0] = Point2i(0, 0);
	srcPoints1[1] = Point2i(0, src.rows);
	srcPoints1[2] = Point2i(src.cols, 0);
 
	dstPoints1[0] = Point2i((width_rotate - width)/2 , (height_rotate - height)/2);
	dstPoints1[1] = Point2i((width_rotate - width)/2 , src.rows + (height_rotate - height)/2);
	dstPoints1[2] = Point2i(src.cols + (width_rotate - width)/2, (height_rotate - height)/2);
 
	Mat M2 = getAffineTransform(srcPoints1, dstPoints1);
	
	M1.at<double>(0, 2) = M1.at<double>(0, 2) + M2.at<double>(0, 2);
	M1.at<double>(1, 2) = M1.at<double>(1, 2) + M2.at<double>(1, 2);
 
	//Mat src2(Size(width_rotate, height_rotate), CV_8UC1, Scalar(0));
	Mat res5(width_rotate, height_rotate, CV_8UC1, Scalar(0));

	warpAffine(src, res5, M1, Size(width_rotate, height_rotate));

    dst = res5.clone();
	
	return M1;

}

void change(Point2f& p, Mat R){
	Mat tr;
	R.convertTo(tr,6);
	Mat p0 = Mat(3,1,6);
	p0.at<double>(0,0) = p.x;
	p0.at<double>(1,0) = p.y;
	p0.at<double>(2,0) = 1;
	Mat tmp = tr*p0;
	Point res = Point(tmp.at<double>(0,0),tmp.at<double>(1,0));
	p = res;
}



Mat addDisplacement(Mat R, int dx, int dy){
	Mat M1 = R.clone();
    Point2f srcPoints1[3];
	Point2f dstPoints1[3];
    srcPoints1[0] = Point2i(0, 0);
	srcPoints1[1] = Point2i(0, 100);
	srcPoints1[2] = Point2i(100, 0);
 
	dstPoints1[0] = Point2i(dx , dy);
	dstPoints1[1] = Point2i(dx , 100 + dy);
	dstPoints1[2] = Point2i(100 + dx, dy);
    Mat M2 = getAffineTransform(srcPoints1, dstPoints1);
    M1.at<double>(0, 2) = M1.at<double>(0, 2) + M2.at<double>(0, 2);
	M1.at<double>(1, 2) = M1.at<double>(1, 2) + M2.at<double>(1, 2);
    return M1;
}

Mat toSquare(Mat M1){
	Mat fm;
	M1.convertTo(fm, 6);
    Mat resM(3,3,6, Scalar(0));
	Mat t = resM(Rect(0,0,3,2));
	fm.copyTo(t);
	resM.at<double>(2,2) = 1;
    return resM;
}