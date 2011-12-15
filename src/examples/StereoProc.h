#ifndef STEREOPROC_H
#define STEREOPROC_H

#include <opencv2/calib3d/calib3d.hpp>

class StereoProc
{
public:
	bool init(const std::string& configFile);
	StereoProc(const std::string& configFile){
		init(configFile);
	}
	bool getImageInfo(cv::Mat& intrinsicMat) const;
	bool undistort(cv::Mat& img1,cv::Mat& img2,cv::Mat& imgUnd1,cv::Mat& imgUnd2) const;
	bool process(cv::Mat& img1, cv::Mat& img2,
				 cv::Mat& imgRectified, cv::Mat& imgDepth);

	cv::Mat P1;
	cv::Mat P2;
private:
	cv::StereoBM mStereoBM;

	cv::Mat mImgRectifiedLeft;
	cv::Mat mImgRectifiedRight;
	cv::Mat mImgDisparity;

	double mBaseline;
	double mFocalLength;

	cv::Mat mIntrinsicMat;

	cv::Mat mUndistortMapXLeft;
	cv::Mat mUndistortMapYLeft;
	cv::Mat mUndistortMapXRight;
	cv::Mat mUndistortMapYRight;

};

#endif
