#include <opencv2/opencv.hpp>
#include <opencv2/legacy/blobtrack.hpp>
#include <string>
#include <vector>
#include <time.h>

using namespace cv;
using namespace std;

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
  
  namedWindow("Thresh",1);
  namedWindow("FG",1);
  VideoCapture cap;
  cap.open(string(argv[1]));
  if(!cap.isOpened()) return -1;
  for(int i = 0;i<2000;i++){
    cout << "Read image " << i << endl;
    Mat frame;
    cap >> frame;
    //    threshold(frame);
    imshow("Thresh",frame);
    if(!frame.data) break;
    FGDetector->Process(new IplImage(frame));
    imshow("FG",Mat(FGDetector->GetMask()));
    if(waitKey(100) >= 0) break;
  }
  return 0;
}
