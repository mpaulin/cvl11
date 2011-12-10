#ifndef __MOTION_FILTERING_HPP
#define __MOTION_FILTERING_HPP

#include "config.hpp"

class MotionFilteringParameters{
public:
  //Gravity
  double g;
  //Maximum mean error between points and estimated parabola.
  double maxMeanError;


  MotionFilteringParameters(){
    g = 9.81;
    maxMeanError = 100;
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

};

class Parabola{
private:
  vector<Point3d> points; //For display
  
public:
  double a;
  double b;
  double c;
  Plane plane;

  Parabola();
  Parabola(const vector<Point3d>& points,const vector<double>& times,double g);
  double eval(double x);
  void render(Mat& image, const Mat& P,const Scalar& col);
};

Point3d triangulate(const Mat& P1, const Mat& P2, const Point3d& x1, const Point3d& x2);



#endif
