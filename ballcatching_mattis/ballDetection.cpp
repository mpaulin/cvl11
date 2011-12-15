#include "blobDetection.hpp"
#include "blobTracking.hpp"
#include "trajectoryFiltering.hpp"

#include "ballDetection.hpp"

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
  beginFrame = 100;
}

void BallDetector::addData(const Mat& image_left,const Mat& P_left,const Mat& image_right,const Mat& P_right,long int time){

  FGDetector_left->Process(new IplImage(image_left));
  FGDetector_right->Process(new IplImage(image_right));
  Mat mask_left = FGDetector_left->GetMask();
  Mat mask_right = FGDetector_right->GetMask();
  
  if(frame>=beginFrame){
    vector<Ellipse> blobs_left = getBlobs(mask_left);
    vector<Ellipse> blobs_right = getBlobs(mask_right);

    balls.currentBlobs_left = blobs_left;
    balls.currentBlobs_right = blobs_right;

    double time_s = double(time)/1000000;
    balls.times.push_back(time_s);

    updateTrajectories(balls.trajectories_left,balls.times,blobs_left,frame,image_left);
    updateTrajectories(balls.trajectories_right,balls.times,blobs_right,frame,image_right);
  
    filterTrajectories(balls.trajectories_left,balls.times,frame);
    filterTrajectories(balls.trajectories_right,balls.times,frame);

    parabola = motionFilter(balls);
  }
  cout << "Frame " << frame << endl;
  frame++;
}

Point3d predictImpact(){
  //TODO
  return Point3d(0,0,0);
}

void BallDetector::render(Mat& image_left,Mat& image_right){
	if(frame<=beginFrame) return;
	for(vector<Ellipse>::const_iterator it = balls.currentBlobs_left.begin();it!=balls.currentBlobs_left.end();it++){
		ellipse(image_left,it->center,Size(it->a,it->b),it->theta,0,360,Scalar(255,255,0));
	}
	balls.render(image_left,image_right);
	parabola.render(image_left,balls.cameras_left[balls.cameras_left.size()-1],Scalar(255,0,0));
}
