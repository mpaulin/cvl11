#include "config.hpp"
#include "blobDetection.hpp"


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
  VideoCapture cap;
  cap.open(string(argv[1]));
  if(!cap.isOpened()) return -1;
  for(int i = 0;i<2000;i++){
    cout << "Read image " << i << endl;
    Mat frame;
    cap >> frame;
    //    threshold(frame);
   
    if(!frame.data) break;
    FGDetector->Process(new IplImage(frame));
    Mat mask = Mat(FGDetector->GetMask());
    imshow("FG",mask);
    vector<Ellipse> blobs = getBlobs(mask);
    for(unsigned int i = 0;i<blobs.size();i++){
      ellipse(frame,blobs[i].center,Size(blobs[i].a,blobs[i].b),0,0,360,Scalar(255,0,0));
    }
    imshow("Image",frame);
    if(waitKey(30) >= 0) break;
  }
  return 0;
}
