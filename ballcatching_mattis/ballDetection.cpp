#include "blobDetection.hpp"
#include "blobTracking.hpp"
#include "trajectoryFiltering.hpp"
#include "blob3dMatching.hpp"

#include "ballDetection.hpp"

#include <sys/time.h>

BallDetector::BallDetector(){
  CvGaussBGStatModelParams* params = new CvGaussBGStatModelParams;		     
  params->win_size=200;	
  params->n_gauss=5;
  params->bg_threshold=0.7;
  params->std_threshold=2.5;
  params->minArea=15.f;
  params->weight_init=0.05;
  params->variance_init=30; 
  FGDetector_left = cvCreateFGDetectorBase(CV_BG_MODEL_MOG,params);
  FGDetector_right = cvCreateFGDetectorBase(CV_BG_MODEL_MOG,params);
  frame = 0;
  beginFrame = 2;
}


#ifdef MOTION_FIRST

void BallDetector::addData(const Mat& image_left,const Mat& P_left, const Mat& image_right, const Mat& P_right, long time){
	FGDetector_left->Process(new IplImage(image_left));
	FGDetector_right->Process(new IplImage(image_right));
	mask_left = cvarrToMat(FGDetector_left->GetMask());
	mask_right = cvarrToMat(FGDetector_right->GetMask());
	double time_s = double(time)/1000000;
	balls.times.push_back(time_s);
	balls.currentCamera_left = P_left;
	balls.currentCamera_right = P_right;
	balls.currentFrame = frame;
	cout << endl;
	cout << "Frame " << frame << endl;
	if(frame>=beginFrame){
		balls.currentBlobs_left = getBlobs(mask_left);
		balls.currentBlobs_right = getBlobs(mask_right);

		cout << "Blobs found : " << balls.currentBlobs_left.size() << " and " << balls.currentBlobs_right.size() << endl;

		balls.currentPoints = triangulate(balls.currentBlobs_left,balls.currentBlobs_right,P_left,P_right);

		cout << "Actually relevant points found: " << balls.currentPoints.size() << endl;

		parabola = motionFilter(balls);

		cout << "Trajectories: " << balls.trajectories.size() << endl;

		filterTrajectories(balls.trajectories,balls.times,balls.currentFrame);
	}
	frame++;
}

#else
void BallDetector::addData(const Mat& image_left,const Mat& P_left,const Mat& image_right,const Mat& P_right,long int time){
	struct timeval bchmrk;
	gettimeofday(&bchmrk,NULL);
	double t1 = double(bchmrk.tv_usec)/1000;
	FGDetector_left->Process(new IplImage(image_left));
	FGDetector_right->Process(new IplImage(image_right));
	mask_left = cvarrToMat(FGDetector_left->GetMask());
	mask_right = cvarrToMat(FGDetector_right->GetMask());
	gettimeofday(&bchmrk,NULL);
	double t2 = double(bchmrk.tv_usec)/1000;
	double time_s = double(time)/1000000;
	balls.times.push_back(time_s);
	balls.cameras_left.push_back(P_left);
	balls.cameras_right.push_back(P_right);

	if(frame>=beginFrame){
		balls.currentBlobs_left = getBlobs(mask_left);
		balls.currentBlobs_right = getBlobs(mask_right);
		gettimeofday(&bchmrk,NULL);
		double t3 = double(bchmrk.tv_usec)/1000;

		cout << "Blobs found : " << balls.currentBlobs_left.size() << " and " << balls.currentBlobs_right.size() << " ";
		cout << "Trajectories left :" << balls.trajectories_left.size() << " ";
		cout << "Trajectories right :" << balls.trajectories_right.size() << endl;

		updateTrajectories(balls.trajectories_left,balls.times,balls.currentBlobs_left,frame,image_left);
		updateTrajectories(balls.trajectories_right,balls.times,balls.currentBlobs_right,frame,image_right);
		gettimeofday(&bchmrk,NULL);
		double t4 = double(bchmrk.tv_usec)/1000;
		filterTrajectories(balls.trajectories_left,balls.times,frame);
		filterTrajectories(balls.trajectories_right,balls.times,frame);

		parabola = motionFilter(balls);
		gettimeofday(&bchmrk,NULL);
		double t5 = double(bchmrk.tv_usec)/1000;

		cout << "Benchmarking: All : " << t5-t1 << " | FG " << t2-t1 << " | Blobs " << t3-t2 << " | Tracking " << t4-t3 << " Parabola | " << t5-t4 << endl;

	}
	cout << endl;
	cout << "Frame " << frame << endl;
	frame++;
}
#endif

Point3d BallDetector::predictImpact(double height){
	const double a = parabola.a;
	const double b = parabola.b;
	const double c = parabola.c - height;
	double x1 = (-b-sqrt(b*b-4*a*c))/(2*a);
	double x2 = (-b+sqrt(b*b-4*a*c))/(2*a);
	if((x2-x1)*parabola.v0x>0) return parabola.plane.retroProject(Point2d(x2,height));
	else return parabola.plane.retroProject(Point2d(x1,height));
}

void BallDetector::render(Mat& image_left,Mat& image_right){
	if(frame<=beginFrame) return;
	for(vector<Ellipse>::const_iterator it = balls.currentBlobs_left.begin();it!=balls.currentBlobs_left.end();it++){
		ellipse(image_left,it->center,Size(it->a,it->b),it->theta,0,360,Scalar(255,255,0));
	}
	for(vector<Ellipse>::const_iterator it = balls.currentBlobs_right.begin();it!=balls.currentBlobs_right.end();it++){
			ellipse(image_right,it->center,Size(it->a,it->b),it->theta,0,360,Scalar(255,255,0));
	}
	balls.render(image_left,image_right);
	parabola.render(image_left,balls.currentCamera_left,Scalar(255,0,0));
	parabola.render(image_right,balls.currentCamera_right,Scalar(255,0,0));
}
