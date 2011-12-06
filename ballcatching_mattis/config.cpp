#include "config.hpp"


void renderPoint(Mat& image, const Mat& P,const Point3d& X, const Scalar& col){
  Vec4d Xh(X.x,X.y,X.z,1);
  Mat ph = P*Mat(Xh);
  Point2d pp(ph.at<double>(0,0)/ph.at<double>(2,0),ph.at<double>(1,0)/ph.at<double>(2,0));
  circle(image,pp,2,col);
}

//Same with a line
void renderLine(Mat& image, const Mat& P,const Point3d& X1,const Point3d& X2, const Scalar& col){
  Vec4d X1h(X1.x,X1.y,X1.z,1);
  Mat p1h = P*Mat(X1h);
  Point2d pp1(p1h.at<double>(0,0)/p1h.at<double>(2,0),p1h.at<double>(1,0)/p1h.at<double>(2,0)); 

  Vec4d X2h(X2.x,X2.y,X2.z,1);
  Mat p2h = P*Mat(X2h);
  Point2d pp2(p2h.at<double>(0,0)/p2h.at<double>(2,0),p2h.at<double>(1,0)/p2h.at<double>(2,0));

  line(image,pp1,pp2,col);
}
