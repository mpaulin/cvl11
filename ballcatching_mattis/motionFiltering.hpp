#ifndef __MOTION_FILTERING_HPP
#define __MOTION_FILTERING_HPP

#include "config.hpp"

class MotionFilteringParameters{
public:
  //Gravity
  double g;
  //Maximum mean error between points and estimated parabola.
  double maxMeanError;
  //Minimum blobs detected to start motion filtering
  int minToConsider;
  //Max vertical distance between left/right matching blobs
  int maxVertDistLR;
  //Multiplier for the plane projection error.
  double lambdaPlaneError;
  //Max distance between blobs to match
  double maxDist;


  MotionFilteringParameters(){
    g = -9.81/2;
    maxMeanError = 40;
    minToConsider = 3;
    maxVertDistLR = 2;
    lambdaPlaneError = 0.001;
    maxDist = 2000;
  }
};

//Class describing planes that contain the Z axis.
class Plane{
public:
  double a;
  double b;
  double c;
  double d;

  Plane();
  Plane(const vector<Point3d>& points);
  pair<Vec3d,Vec3d> getBase();
  Point3d getOrigin();
  Point2d project(const Point3d& p);
  Point3d retroProject(const Point2d& p);
  void render(Mat& image, const Mat& P);

  Point3d first;
  Point3d last;
};

class Parabola{
private:
  vector<Point3d> points;
  vector<double> times;
public:
  double a;
  double b;
  double c;
  Plane plane;
  double v0x;
  Parabola();
  Parabola(const vector<Point3d>& points,const vector<double>& times,double g);
  double eval(double x);
  void render(Mat& image, const Mat& P,const Scalar& col);
  double getError(const MotionFilteringParameters& params);
};

Parabola motionFilter(Balls& balls,MotionFilteringParameters p = MotionFilteringParameters());

#endif
