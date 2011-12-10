#include "ballDetection.hpp"

void BallDetector::BallDetector{
  CvGaussBGStatModelParams* params = new CvGaussBGStatModelParams;		     
  params->win_size=200;	
  params->n_gauss=5;
  params->bg_threshold=0.7;
  params->std_threshold=2.5;
  params->minArea=15.f;
  params->weight_init=0.05;
  params->variance_init=30; 
  FGDetector = cvCreateFGDetectorBase(CV_BG_MODEL_MOG,params);
}

Point3d BallDetector::addData(const Mat& image_left,const Mat& P_left,const Mat& image_right,const Mat& P_right,int time){

  FGDetector->Process(image_left);
  mask_left = FGDetector->GetMask();
  vector<Ellipse> blobs_left = getBlobs(mask_left);
  vector<Ellipse> blobs_right = getBlobs(mask_right);
  
  balls.times.push_back(time);

  updateTrajectories(balls.trajectories_left,balls.histograms_left,balls.times,blobs_left,time,image_left);
  updateTrajectories(balls.trajcetories_right,balls.histograms_right,balls.times,blobs_right,time,image_right);
  
  //filterTrajectories(trajectories_left,time);
  //filterTrajectories(trajectories_right,time);
}

Point3d predictImpact(){
  //TODO
  return Point3d(0,0,0);
}

void render(Mat& image_left,Mat& image_right){
  balls.render(image_left,image_right);
}
