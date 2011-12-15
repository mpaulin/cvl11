#ifndef __BALL_DETECTION_HPP
#define __BALL_DETECTION_HPP

#include "config.hpp"
#include "motionFiltering.hpp"

class BallDetector{
public:
  CvFGDetector* FGDetector_left;
  CvFGDetector* FGDetector_right;
  Balls balls;
  Parabola parabola;
  int frame;
  int beginFrame;
  BallDetector();
  void addData(const Mat& image1,const Mat& P1,const Mat& image2,const Mat& P2,long int time);
  Point3d predictImpact(double height);
  void render(Mat& image_left, Mat& image_right);

};
#endif
 
