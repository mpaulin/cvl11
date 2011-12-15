#include "config.hpp"


void renderPoint(Mat& image, const Mat& P,const Point3d& X, const Scalar& col){
  Vec4d Xh(X.x,X.y,X.z,1);
  Mat ph = P*Mat(Xh);
  Point2d pp(ph.at<double>(0,0)/ph.at<double>(2,0),ph.at<double>(1,0)/ph.at<double>(2,0));
  //cout << pp << endl;
  circle(image,pp,2,col);
}

//Same with a line
void renderLine(Mat& image, const Mat& P,const Point3d& X1,const Point3d& X2, const Scalar& col){
  Vec4d X1h(X1.x,X1.y,X1.z,1);
  Mat p1h = P*Mat(X1h);
  Point2d pp1(p1h.at<double>(0,0)/p1h.at<double>(2,0),p1h.at<double>(1,0)/p1h.at<double>(2,0)); 

  Vec4d X2h(X2.x,X2.y,X2.z,1);
  Mat p2h = P*Mat(X2h);
  Point2d pp2(p2h.at<double>(0,0)/p2h.at<double>(2,0),p2h.at<double>(1,0)/p2h.at<double>(2,0));

  line(image,pp1,pp2,col);
}


void Balls::render(Mat& image_left, Mat& image_right){
  for(list<Trajectory>::const_iterator it = trajectories_left.begin();it!=trajectories_left.end();it++){
    for(int j = 0;j<it->length-1;j++){
      Point2d p1 = it->getEllipse(j).center;
      Point2d p2 = it->getEllipse(j+1).center;
      line(image_left,p1,p2,Scalar(0,0,255));
    } 
    Ellipse blob = it->getEllipse(it->length-1);
    ellipse(image_left,blob.center,Size(blob.a,blob.b),blob.theta,0,360,Scalar(255,0,0));
  }

  for(list<Trajectory>::const_iterator it = trajectories_right.begin();it!=trajectories_right.end();it++){
    for(int j = 0;j<it->length-1;j++){
      Point2d p1 = it->getEllipse(j).center;
      Point2d p2 = it->getEllipse(j+1).center;
      line(image_right,p1,p2,Scalar(0,0,255));
    } 
    Ellipse blob = it->getEllipse(it->length-1);
    ellipse(image_right,blob.center,Size(blob.a,blob.b),blob.theta,0,360,Scalar(255,0,0));
  }
}

vector<double> getHistogram(const Ellipse & ellipse,const Mat& image,int nBins){
  
  int w = 2*max(ellipse.a,ellipse.b);
  int h = w;
  int cx = ellipse.center.x-w/2;
  int cy = ellipse.center.y-h/2;
  int nPix = 0;
  // Assign to histogram 
  vector<double> histo(nBins,0);
  for(int j = 0;j<h;j++){
    if(cy+j>=0 && cy+j<image.rows){
      const Vec3b* row = image.ptr<Vec3b>(cy+j);
      for(int i = 0;i<w;i++){
	if(cx+i>=0 && cx+i<image.cols){
	  double col = row[cx+i][0];
	  int clip = int((col*nBins)/255 + 0.5);
	  if(clip>=nBins){ 
	    clip = nBins-1;
	  }
	  histo[clip]+=1;
	  nPix ++;
	}
      }
    }
  }
  for(int k = 0;k<nBins;k++){
    histo[k]/=nPix;
  }
  /*
  cout << " Histogram with " << nBins << " bins" << endl;
  for(int i = 0;i<nBins;i++){
    cout << histos[i] << endl;
  }
  cout << " --------- " << endl;
  */
  return histo;
}
