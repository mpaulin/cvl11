#include "motionFiltering.hpp"


pair<double,double> linearRegression(const vector<Point2d>& points){
  Mat A(points.size(),2,CV_64F);
  Mat B(points.size(),1,CV_64F);
  for(unsigned int i = 0;i<points.size();i++){
    A.at<double>(i,0)= points[i].x;
    A.at<double>(i,1)= 1;
    B.at<double>(i,0)= points[i].y;
  }
  Mat X = A.inv(DECOMP_SVD)*B;
  double a = X.at<double>(0,0);
  double b = X.at<double>(1,0);

  return pair<double,double>(a,b);
}

Plane::Plane(){}

Plane::Plane(const vector<Point3d>& points){
  Mat A = Mat(points.size(),3,CV_64F);
  for(unsigned int i = 0;i<points.size();i++){
    A.at<double>(i,0) = points[i].x;
    A.at<double>(i,1) = points[i].y;
    A.at<double>(i,2) = 1;
  }
  SVD svd(A);
  Vec4d plane;
  a = svd.vt.at<double>(2,0);
  b = svd.vt.at<double>(2,1);
  c = 0;
  d = svd.vt.at<double>(2,2);
}


pair<Vec3d,Vec3d> Plane::getBase(){
  Vec3d b1(-b,a,0);
  b1 *= 1/norm(b1);
  Vec3d b2(0,0,1);
  return pair<Vec3d,Vec3d>(b1,b2);
}

Point3d Plane::getOrigin(){
  // TODO maybe replace by a point closer to the samples
  return Point3d(-d/a,0,0);
}


Point2d Plane::project(const Point3d& p){
  Point3d O = getOrigin(); 
  pair<Vec3d,Vec3d> base = getBase();
  return Point2d(
		 (Mat(p-O)).dot(base.first),
		 (Mat(p-O)).dot(base.second));
}


Point3d Plane::retroProject(const Point2d& p){
  Point3d O = getOrigin(); 
  pair<Vec3d,Vec3d> base = getBase();
  return O+p.x*Point3d(base.first)+p.y*Point3d(base.second);
}


void Plane::render(Mat& image, const Mat& P){
  pair<Vec3d,Vec3d> base = getBase();
  Point3d O = getOrigin();
  Point3d i = O - 10*Point3d(base.first);
  Point3d j = O + 10*Point3d(base.second);
  renderLine(image,P,O,i,Scalar(255,0,255));
  renderLine(image,P,O,j,Scalar(255,0,255));
  renderPoint(image,P,retroProject(Point2d(0,0)),Scalar(125,0,255));
}

Parabola::Parabola(){}

Parabola::Parabola(const vector<Point3d>& points,const vector<double>& times,double g){
  //method :  0 -> with positions only 1 -> with positions and speeds
  this->points = points;
  plane = Plane(points);

  //Evalute Vx0
  vector<Point2d> vx(points.size());
  for(unsigned int i = 0;i<vx.size();i++){
    vx[i].x = times[i];
    vx[i].y =  plane.project(points[i]).x;
  }
  double v0x = linearRegression(vx).first;
  cout << "V0x found : " << v0x << endl;
  vector<Point2d> pts(points.size());
  for(unsigned int i = 0;i<pts.size();i++){
    Point2d p = plane.project(points[i]);
    pts[i].x = p.x;
    pts[i].y = p.y - g*p.x*p.x/v0x;
  }
  pair<double,double> bc = linearRegression(pts);

  a = g/v0x;
  b = bc.first;
  c = bc.second;

}

double Parabola::eval(double x){
  return c+x*(b+x*a);
}

void Parabola::render(Mat& image, const Mat& P,const Scalar& col){
  plane.render(image,P);
  double steps = 100;
  double beg = plane.project(points[0]).x;
  double nb = 5;
  for(int i = 0;i<steps;i++){
    double x = beg-nb*i/steps;
    Point2d po(x,eval(x));
    Point3d X = plane.retroProject(po);
    
    renderPoint(image,P,X,col); 
  }
  
  for(vector<Point3d>::const_iterator it = points.begin();it!=points.end();it++){
    Point3d p(it->x,it->y,0);
    renderPoint(image,P,p,Scalar(255,255,255));
    renderPoint(image,P,*it,Scalar(125,125,125));
  }
}

