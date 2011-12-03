#include <opencv2/opencv.hpp>
#include <opencv2/legacy/blobtrack.hpp>
#include <string> 
#include <vector>
#include <time.h>
#include "pixelDisjointSet.hpp"

using namespace cv;
using namespace std;

double thresh = 150; 
void threshold(Mat& image){
  for(MatIterator_<double> it = image.begin<double>();it!=image.end<double>();++it){
    if((*it)<thresh) *it = 0;
  }
}

map<int,vector<Point2i> > cluster(const Mat& image){
  DisjointSet disjointSet(image.cols,image.rows);
  for(int y = 0; y < image.rows; y++){
    const unsigned char* prevRow = NULL;
    if(y>0) prevRow = image.ptr<unsigned char>(y-1);
    const unsigned char* My = image.ptr<unsigned char>(y);
    for(int x = 0; x < image.cols; x++){
      if(My[x]!=0){
	disjointSet.makeSet(x,y);
	if(prevRow!=NULL && prevRow[x]!=0){
	  disjointSet.unite(x,y,x,y-1);
	}
	if(x>0 && My[x-1]!=0){ 
	  disjointSet.unite(x,y,x-1,y);
	}
      }
    }
  }
  map<int,vector<Point2i> > clusters;
  for(int y = 0;y<image.rows;y++){
    for(int x = 0;x<image.cols;x++){
      Pixel p = disjointSet.find(x,y);
      int i = p.parent;
      if(!clusters.count(i)){
	clusters.insert(pair<int,vector<Point2i> >(i,vector<Point2i>(1,Point2i(x,y))));
      }
      else{
	clusters[i].push_back(Point2i(x,y));
      }
    }
  }
  return clusters;
}

Mat clustersToImage(const map<int,vector<Point2i> >& clusters, int width,int height){
  Mat clusteredImage(height,width,CV_8UC1);
  for(map<int,vector<Point2i> >::const_iterator it = clusters.begin();it!=clusters.end();it++){
    for(vector<Point2i>::const_iterator it2 = it->second.begin(); it2!=it->second.end();it2++){
      clusteredImage.at<unsigned char>(it2->y,it2->x) = (73153413*it->first)%(255);
    }
  }
  cout << "done" << endl;
  return clusteredImage;
}

vector<Mat> clustersAsMats(const map<int,vector<Point2i> >& clusters){
  vector<Mat> result;
   for(map<int,vector<Point2i> >::const_iterator it = clusters.begin();it!=clusters.end();it++){
     Mat m(2,it->second.size(),CV_64F);
     for(unsigned int i = 0;i<it->second.size();i++){
       m.at<double>(0,i) = it->second[i].x;
       m.at<double>(1,i) = it->second[i].y;
     }
     result.push_back(m);
   }
   return result;
}

vector<Point2i> getCenters(const vector<Mat>& clusters){
  vector<Point2i> centers;
  for(unsigned int i = 0;i<clusters.size();i++){
    int x = 0;
    int y = 0;
    Mat m = clusters[i];
    const double* rx = m.ptr<double>(0);
    const double* ry = m.ptr<double>(1);
    for(int j = 0;j<clusters[i].cols;j++){
      x+=rx[i];
      y+=ry[j];
    }
    x/=m.cols;
    y/=m.cols;
    centers.push_back(Point2i(x,y));
  }
  return centers;
}

class Ellipse{
public:
  Point2f center;
  float a;
  float b;
  float c;

  Ellipse(const Point2f& cent,float aa,float bb,float cc){
    center = cent;
    a = aa;
    b = bb;
    c = cc;
  }
};

vector<Ellipse> getBlobs(const map<int,vector<Point2i> >& clusters){
  vector<Ellipse> blobs;
  for(map<int,vector<Point2i> >::const_iterator it = clusters.begin();it != clusters.end();it++){
    float radius;
    Point2f center;
    minEnclosingCircle(it->second,center,radius);
    if(radius>1 && radius < 20){
      blobs.push_back(Ellipse(center,radius,radius,0));
    }
  }
  return blobs;
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
  namedWindow("Cluster",1);
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
    Mat mask = Mat(FGDetector->GetMask());
    imshow("FG",mask);
    map<int,vector<Point2i> > clusters = cluster(mask);
    cout << clusters.size() << endl;
    Mat clust = clustersToImage(clusters,mask.cols,mask.rows); 
    vector<Ellipse> blobs = getBlobs(clusters);
    for(unsigned int i = 0;i<blobs.size();i++){
      ellipse(clust,blobs[i].center,Size(blobs[i].a,blobs[i].b),0,0,360,255);
    }
    imshow("Cluster",clust);
   

    if(waitKey(30) >= 0) break;
  }
  return 0;
}
