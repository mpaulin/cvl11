#include "config.hpp"
#include "blobDetection.hpp"
#include "blobTracking.hpp"
#include "trajectoryFiltering.hpp"
#include <time.h>

double thresh = 150; 
void threshold(Mat& image){
  for(MatIterator_<double> it = image.begin<double>();it!=image.end<double>();++it){
    if((*it)<thresh) *it = 0;
  }
}

int main(int argc, char** argv){
  CvGaussBGStatModelParams* params = new CvGaussBGStatModelParams;		     
  params->win_size=200;	
  params->n_gauss=5;
  params->bg_threshold=0.7;
  params->std_threshold=2.5;
  params->minArea=15.f;
  params->weight_init=0.05;
  params->variance_init=30; 
  CvFGDetector* FGDetector = cvCreateFGDetectorBase(CV_BG_MODEL_MOG,params);
  
  namedWindow("Image",1);
  namedWindow("FG",1);
  //namedWindow("CC",1);
  VideoCapture cap;
  cap.open(string(argv[1]));
  if(!cap.isOpened()) return -1;
  //Trajectories trajectories;
  for(int i = 0;i<2000;i++){
    cout << "Read image " << i << endl;
    Mat frame;
    cap >> frame;
    //    threshold(frame);
   
    if(!frame.data) break;
    FGDetector->Process(new IplImage(frame));
    Mat mask = Mat(FGDetector->GetMask());
    imshow("FG",mask);
    
    //clock_t t = clock();
    vector<Ellipse> blobs = getBlobs(mask);
    cout << "Found " << blobs.size() << " blobs" << endl;
    //clock_t t2 = clock();
    //cout << " Elapsed time : "<< double(t2-t)/CLOCKS_PER_SEC << endl;
    //imshow("CC",clustersToImage(cluster(mask),frame.cols,frame.rows));
    


   
    /*
    int initFrames = 30;
    if(i<initFrames){
      trajectories = initTrajectories(blobs,frame);
    }
    else{
      vector<pair<Ellipse,int> > stampedBlobs;
      for(vector<Ellipse>::const_iterator it = blobs.begin();it!=blobs.end();it++){
	stampedBlobs.push_back(pair<Ellipse,int>(*it,i));
      }
      
      //updateTrajectories(trajectories,stampedBlobs,frame);
      //filterTrajectories(trajectories,i);
     
      cout <<"Trajectory size : " <<  trajectories.trajectories.size() << endl;
    } 
    */
  
    
    ///////////////////////////////////////////////
    //              Rendering part               
    ///////////////////////////////////////////////
    
    
    /*

    for(unsigned int j = 0;j<blobs.size();j++){
      ellipse(frame,blobs[j].center,Size(blobs[j].a,blobs[j].b),0,0,360,Scalar(255,0,0));
    } 
    //renderTrajectories(trajectories,frame);
    imshow("Image",frame);

    */
    if(waitKey(500) >= 0) break;
  }
  return 0;
}
