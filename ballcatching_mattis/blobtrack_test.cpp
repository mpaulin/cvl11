#include <opencv2/opencv.hpp>
#include <opencv2/legacy/blobtrack.hpp>
#include <string>
#include<vector>

using namespace cv;
using namespace std;

#define PI 3.1415926535789

void renderPoint(Mat& image, const Mat& P,const Point3d& X, const Scalar& col){
  Vec4d Xh(X.x,X.y,X.z,1);
  Mat ph = P*Mat(Xh);
  Point2d pp(ph.at<double>(0,0)/ph.at<double>(2,0),ph.at<double>(1,0)/ph.at<double>(2,0));
  circle(image,pp,2,col);
}

void renderLine(Mat& image, const Mat& P,const Point3d& X1,const Point3d& X2, const Scalar& col){
  Vec4d X1h(X1.x,X1.y,X1.z,1);
  Mat p1h = P*Mat(X1h);
  Point2d pp1(p1h.at<double>(0,0)/p1h.at<double>(2,0),p1h.at<double>(1,0)/p1h.at<double>(2,0));

  Vec4d X2h(X2.x,X2.y,X2.z,1);
  Mat p2h = P*Mat(X2h);
  Point2d pp2(p2h.at<double>(0,0)/p2h.at<double>(2,0),p2h.at<double>(1,0)/p2h.at<double>(2,0));

  line(image,pp1,pp2,col);
}

//Class describing planes that contain the Z axis.
class Plane{


public:
  double a;
  double b;
  double c;
  double d;

  Plane(){};
  Plane(const vector<Point3d>& points){
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

  pair<Vec3d,Vec3d> getBase(){
    Vec3d b1(-b,a,0);
    b1 *= 1/norm(b1);
    Vec3d b2(0,0,1);
    return pair<Vec3d,Vec3d>(b1,b2);
  }

  Point3d getOrigin(){
    // TODO maybe replace by a point closer to the samples
    return Point3d(-d/a,0,0);
  }

  Point2d project(const Point3d& p){
    Point3d O = getOrigin(); 
    pair<Vec3d,Vec3d> base = getBase();
    return Point2d(
		   (Mat(p-O)).dot(base.first),
		   (Mat(p-O)).dot(base.second));
  }

  Point3d retroProject(const Point2d& p){
    Point3d O = getOrigin(); 
    pair<Vec3d,Vec3d> base = getBase();
    return O+p.x*Point3d(base.first)+p.y*Point3d(base.second);
  }

  void render(Mat& image, const Mat& P){
    pair<Vec3d,Vec3d> base = getBase();
    Point3d O = getOrigin();
    Point3d i = O - 10*Point3d(base.first);
    Point3d j = O + 10*Point3d(base.second);
    renderLine(image,P,O,i,Scalar(255,0,255));
    renderLine(image,P,O,j,Scalar(255,0,255));
    renderPoint(image,P,retroProject(Point2d(0,0)),Scalar(125,0,255));
  }

};

class Parabola{
private:
  vector<Point3d> points; //For display
  


public:
  double a;
  double b;
  double c;
  Plane p;

  Parabola(){}

  Parabola(const vector<Point3d>& points,int method ){
    //method :  0 -> with positions only 1 -> with positions and speeds
    this->points = points;
    p = Plane(points);

    if(method ==0){
      Mat A(points.size(),3,CV_64F);
      Mat B(points.size(),1,CV_64F);
      for(unsigned int i = 0;i<points.size();i++){
	Point2d x = p.project(points[i]);
	A.at<double>(i,0) = x.x*x.x;
	A.at<double>(i,1) = x.x;
	A.at<double>(i,2) = 1; 
	B.at<double>(i,0) = x.y;
      }
      
      Mat X = A.inv(DECOMP_SVD)*B;
      a = X.at<double>(0,0);
      b = X.at<double>(1,0);
      c = X.at<double>(2,0);
    }
    else if(method == 1){
      Mat A(2*points.size()-1,4,CV_64F);
      Mat B(2*points.size()-1,1,CV_64F);
      for(unsigned int i = 0;i<points.size();i++){
	Point2d x = p.project(points[i]);
	A.at<double>(i,0) = x.x*x.x;
	A.at<double>(i,1) = x.x;
	A.at<double>(i,2) = 1; 
	A.at<double>(i,3) = 0; 
	B.at<double>(i,0) = x.y;
	if(i>0){
	  A.at<double>(points.size()+i-1,0) = -2;
	  A.at<double>(points.size()+i-1,1) = 0;
	  A.at<double>(points.size()+i-1,2) = 0;
	  A.at<double>(points.size()+i-1,3) = 1;
	  B.at<double>(points.size()+i-1,0) = x.y-p.project(points[i-1]).y;
	}
      }
      Mat X = A.inv(DECOMP_SVD)*B;
      a = X.at<double>(0,0);
      b = X.at<double>(1,0);
      c = X.at<double>(2,0);
    }  
  }

  double eval(double x){
    return c+x*(b+x*a);
  }

  void render(Mat& image, const Mat& P,const Scalar& col){
    p.render(image,P);
    double steps = 100;
    double beg = p.project(points[0]).x;
    double nb = 5;
    for(int i = 0;i<steps;i++){
      double x = beg-nb*i/steps;
      Point2d po(x,eval(x));
      Point3d X = p.retroProject(po);
      
      renderPoint(image,P,X,Scalar(0,255,255)); 
    }

    for(vector<Point3d>::const_iterator it = points.begin();it!=points.end();it++){
      Point3d p(it->x,it->y,0);
      renderPoint(image,P,p,Scalar(255,255,255));
      renderPoint(image,P,*it,Scalar(125,125,125));
    }
  }
};




double p1[3][4] = {{0.9519,   0.0449,   0.003,   -6.4569},
		   {0.0962,   0.2388,   0.8671,  -4.2482},
		   {0.0004,   0.0009,   0.0000,  -0.0111}};

double p2[3][4] = {{-0.5845,    0.5768,   -0.0002,    2.3055},
		   {-0.2101,   -0.0791,   -0.7463,    3.9477},
		   {-0.0008,   -0.0003,   -0.0000,    0.0106}};

/**
   Creates a blob tracker with fixed parameters
 */
CvBlobTrackerAuto* createTracker(){
  CvGaussBGStatModelParams* FGparams = new CvGaussBGStatModelParams;		     
  FGparams->win_size=200;	
  FGparams->n_gauss=5;
  FGparams->bg_threshold=0.7;
  FGparams->std_threshold=2.5;
  FGparams->minArea=15.f;
  FGparams->weight_init=0.05;
  FGparams->variance_init=30; 
  CvFGDetector* FGDetector = cvCreateFGDetectorBase(CV_BG_MODEL_MOG,FGparams);
  CvBlobDetector* blobDetector = cvCreateBlobDetectorCC();
  CvBlobTracker* blobTracker = cvCreateBlobTrackerCC();
  CvBlobTrackPostProc* postProc = cvCreateModuleBlobTrackPostProcKalman();
  CvBlobTrackerAutoParam1* param = new CvBlobTrackerAutoParam1;
  CvBlobTrackerAuto* tracker;
  param->pBD = blobDetector;
  param->pFG = FGDetector;
  param->pBT = blobTracker;
  param->pBTPP = postProc;
  param->FGTrainFrames = 1;
  tracker = cvCreateBlobTrackerAuto1(param);
  return tracker;
}

Point3d triangulate(const Mat& P1, const Mat& P2, const Point3d& x1, const Point3d& x2){
  Mat A(6,6,CV_64F);
  for(int i = 0;i<3;i++){
    for(int j = 0;j<4;j++){
      A.at<double>(i,j) = P1.at<double>(i,j);
      A.at<double>(i+3,j) = P2.at<double>(i,j);
    }
  }
  A.at<double>(0,4)  = -x1.x;
  A.at<double>(1,4) =  -x1.y;
  A.at<double>(2,4) =  -x1.z;
  
  A.at<double>(3,5) = -x2.x;
  A.at<double>(4,5) = -x2.y;
  A.at<double>(5,5) = -x2.z;

  A.at<double>(3,4) = 0;
  A.at<double>(4,4) = 0;
  A.at<double>(5,4) = 0;
  A.at<double>(0,5) = 0;
  A.at<double>(1,5) = 0;
  A.at<double>(2,5) = 0;

  SVD svd(A);
  

  Point3d X;

  
  
  X.x = svd.vt.at<double>(5,0)/svd.vt.at<double>(5,3);
  X.y = svd.vt.at<double>(5,1)/svd.vt.at<double>(5,3);
  X.z = svd.vt.at<double>(5,2)/svd.vt.at<double>(5,3);

  //cout << "3D point found : " << X << endl;
  return X;
}

int main(int argc, char** argv)
{

  Mat P1(3,4,CV_64F,p1);
  Mat P2(3,4,CV_64F,p2);
 
  cout << P1 << endl;
  cout << P2 << endl;

  VideoCapture cap,cap2;
  cap.open(string(argv[1]));
  cap2.open(string(argv[2]));
  if(!cap.isOpened() || !cap2.isOpened()) return -1;
    
  namedWindow("Blob Tracking",1);
  namedWindow("Blob Tracking2",1);
  CvBlobTrackerAuto* tracker1 = createTracker(); 
  CvBlobTrackerAuto* tracker2 = createTracker();
  vector<Point3d> tracks;
  for(int i = 0;i<200;i++){
    Mat frame;
    Mat frame2;
    cap >> frame;
    cap2 >> frame2;

    cvtColor(frame,frame, CV_BGR2GRAY);
    cvtColor(frame2,frame2,CV_BGR2GRAY);
    
    IplImage* ipl = new IplImage(frame);
    IplImage* pMask = NULL;   
    tracker1->Process(ipl,pMask);

    IplImage* ipl2 = new IplImage(frame2);
    IplImage* pMask2 = NULL;
    tracker2->Process(ipl2,pMask2);

    cvtColor(frame,frame,CV_GRAY2BGR);
    cvtColor(frame2,frame2,CV_GRAY2BGR);

    if(tracker1->GetBlobNum()>0){
      for(int i = 0;i<tracker1->GetBlobNum();i++){
	CvBlob* blob = tracker1->GetBlob(i);
	ellipse(frame,Point2i(blob->x,blob->y),cv::Size(blob->w/2,blob->h/2),0,0,360,Scalar(255,255,0));
      }
    }    
    if(tracker2->GetBlobNum()>0){
      for(int i = 0;i<tracker2->GetBlobNum();i++){
	CvBlob* blob = tracker2->GetBlob(i);
	ellipse(frame2,Point2i(blob->x,blob->y),cv::Size(blob->w/2,blob->h/2),0,0,360,Scalar(255,255,0));
	ellipse(frame,Point2i(blob->x,blob->y),cv::Size(blob->w/2,blob->h/2),0,0,360,Scalar(0,0,100));
      }
    }

    renderLine(frame,P1,Point3d(0,0,0),Point3d(10,0,0),Scalar(125,255,125));
    renderLine(frame,P1,Point3d(0,0,0),Point3d(0,10,0),Scalar(125,255,125));
    renderLine(frame,P1,Point3d(0,0,0),Point3d(0,0,10),Scalar(125,255,125));

    if(tracker1->GetBlobNum()>0 && tracker2->GetBlobNum()>0){
      Point3d x1(tracker1->GetBlob(0)->x,tracker1->GetBlob(0)->y,1);
      Point3d x2(tracker2->GetBlob(0)->x,tracker2->GetBlob(0)->y,1);
      Point3d X = triangulate(P1,P2,x1,x2);

      tracks.push_back(X);
      if(i>1){
	//cout << "Speed " << X.z-tracks[tracks.size()-2].z << endl;
	cout << "Mean Momentum " << ((X.z - tracks[tracks.size()-2].z) - (tracks[1].z-tracks[0].z))/(tracks.size()-1) << endl;
      }

      Mat Xh(4,1,CV_64F);
      Xh.at<double>(0,0) = X.x;
      Xh.at<double>(1,0) = X.y;
      Xh.at<double>(2,0) = X.z;
      Xh.at<double>(3,0) = 1;
 
      Mat rep1 = P1*Xh;
      Mat rep2 = P2*Xh;

      circle(frame,Point2d(rep1.at<double>(0,0)/rep1.at<double>(2,0),rep1.at<double>(1,0)/rep1.at<double>(2,0)),2,Scalar(0,255,0),-1);
      circle(frame2,Point2d(rep2.at<double>(0,0)/rep2.at<double>(2,0),rep2.at<double>(1,0)/rep2.at<double>(2,0)),2,Scalar(0,255,0),-1);

      if(tracks.size()>10){
	Parabola p(tracks,0);
	p.render(frame,P1,Scalar(0,255,255));
	Parabola p2(tracks,1);
	p2.render(frame,P1,Scalar(0,0,255));
      }
    }
    //std::cout << "Found " << tracker1->GetBlobNum() << " and " << tracker2->GetBlobNum() << " Blobs in images." << std::endl;
   
    imshow("Blob Tracking", frame);
    imshow("Blob Tracking2",frame2);
    if(waitKey(20) >=0){
      break;
    }
  }
  while(waitKey(0)==-1){

  }
  return 0;
}
