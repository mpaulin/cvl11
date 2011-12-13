#ifndef __CONFIG_HPP
#define __CONFIG_HPP

//A header file whith things that can be useful all the time

#include <opencv2/opencv.hpp>
#include <opencv2/legacy/blobtrack.hpp>
#include <string> 
#include <vector>
#include <time.h>
#include <algorithm>
#include <list>

using namespace cv;
using namespace std;

#define PI 3.14159265359

class Ellipse{
public:
  Point2f center;
  float a;
  float b;
  //Angle between Ox and the a axis
  float theta;

  Ellipse(){
    a = 0;
    b = 0;
    theta = 0;
  }

  Ellipse(const Point2f& cent,float aa,float bb,float t){
    center = cent;
    a = aa;
    b = bb;
    theta = t;
  }

  double distanceTo(const Ellipse& e) const{
    Point p = e.center;
    return sqrt((center.x-p.x)*(center.x-p.x)+(center.y-p.y)*(center.y-p.y));
  }

  void render(Mat& image){
    ellipse(image,center,Size(a,b),theta,0,360,Scalar(255,0,0));
  }

};
 
class Balls{
public:
  class Trajectory{
  public:
    int length;
    Trajectory(){
    	length = 0;
    }
    
    Ellipse getEllipse(int i) const{
      return ellipses[i];
    }

    int getFrame(int i) const{
      return frames[i];
    }

    vector<double> getHistogram(int i){
      return histograms[i];
    }

    void addPoint(const Ellipse& e, int frame, const vector<double>& hist){
      ellipses.push_back(e);
      frames.push_back(frame);
      histograms.push_back(hist);
      length++;
    }
  private:
    vector<Ellipse> ellipses;
    vector<int> frames;
    vector<vector<double> > histograms;
  };

  list<Trajectory> trajectories_left;
  list<Trajectory> trajectories_right;
  vector<Ellipse> currentBlobs_left;
  vector<Ellipse> currentBlobs_right;
  vector<double> times;
  vector<Mat> cameras_left;
  vector<Mat> cameras_right;
  list<int> matchings;
  list<vector<Point3d> > balls;

  void render(Mat&, Mat&);
};

void renderPoint(Mat& image, const Mat& P,const Point3d& X, const Scalar& col);

void renderLine(Mat& image, const Mat& P,const Point3d& X1,const Point3d& X2, const Scalar& col);

vector<double> getHistogram(const Ellipse&, const Mat&, int);

#endif
