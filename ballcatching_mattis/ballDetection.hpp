#ifndef __BALL_DETECTION_HPP
#define __BALL_DETECTION_HPP

#include "config.h"



class BallDetector{

  CvFGDetector* FGDetector;
  Balls balls;
  BallDetector();
  void addData(const Mat& image1,const Mat& P1,const Mat& image2,const Mat& P2,int time);
  Point3d predictImpact(double height);
  void render(Mat& image_left, Mat& image_right);

};
#endif
 
