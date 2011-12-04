#include "blobTracking.hpp"


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


double evalConfidence(const pair<Ellipse,int>& e1,
		      const pair<Ellipse,int>& e2,
		      pair<Ellipse,int>* le,
		      const vector<double>& hist1,
		      const vector<double>& hist2,
		      const Mat& image,
		      const BlobTrackingParameters& p){

  //TODO Compute speed and angle
    double v1r = 0;
    double v2r = 0;
    double v1t = 0;
    double v2t = 0;

    if(le != NULL){
      Point2d p1 = e1.first.center;
      Point2d p2 = e2.first.center;
      Point2d lp = le->first.center;
      v1r = (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
      v1r/= (e2.second-e1.second)*(e2.second-e1.second);
      
      v2r = (lp.x-p2.x)*(lp.x-p2.x)+(lp.y-p2.y)*(lp.y-p2.y);
      v2r/= (le->second-e2.second)*(le->second-e2.second);
      
      v1r = sqrt(v1r);
      v2r = sqrt(v2r);
      
      v1t = atan2(e2.first.center.y-e1.first.center.y,e2.first.center.x-e1.first.center.x);
      v2t = atan2(le->first.center.y-e2.first.center.y,le->first.center.x-e2.first.center.x);
  }
  
  //compute histogram distance
  double dHist = 0;
  for(int i = 0;i<p.nBins;i++){
    dHist += abs(hist1[i]-hist2[i]);
  }
  //cout << "Hist distance : " << dHist << endl;
    
  double dVR = abs(v1r-v2r);
  double dVT = min(min(abs(v1t-v2t),abs(v1t-v2t-2*PI)),abs(2*PI+v1t-v2t));
  double dRad = sqrt((e1.first.a-e2.first.a)*(e1.first.a-e2.first.a) 
		     +(e1.first.b-e2.first.b)*(e1.first.b-e2.first.b)) ;
  
  double dist = e1.first.distanceTo(e2.first);
  if(dist > p.maxDist || dist < p.minDist || dVT > p.maxRot) return 0;
  cout << dHist << " " << dRad << " " << dVR << " " << dVT << endl;
  double confidence = 1./(p.lambdaHist*dHist+p.lambdaRad*dRad+p.lambdaTSpeed*dVT); 
  //cout << v1r << endl;
  return confidence;
} 


void updateTrajectories(Trajectories& olds,
		       const vector<pair<Ellipse,int> >& blobs, 
		       const Mat& image,
		       const BlobTrackingParameters& p){
  vector<vector<double> > newHistos(blobs.size());
  for(unsigned int i = 0;i<blobs.size();i++){
    newHistos[i] = getHistogram(blobs[i].first,image,p.nBins);
  }

  for(unsigned int i = 0;i<olds.trajectories.size();i++){ 
    double maxConfidence = 0;
    int index = 0;
    for(unsigned int j = 0;j<blobs.size();j++){
      pair<Ellipse,int>* le = NULL;
      if(olds.trajectories[i].size()>0) le = &olds.trajectories[i][olds.trajectories[i].size()-2];
      double confidence = evalConfidence(blobs[j],
					 olds.trajectories[i][olds.trajectories[i].size()-1],
					 le,
					 newHistos[j],
					 olds.lastHistograms[i],
					 image,
					 p);
      cout << "confidence : " << confidence<< endl;
      if(confidence > maxConfidence){
	index = j;
	maxConfidence = confidence;
      }
    }
    if(maxConfidence>p.minConfidence){
      olds.trajectories[i].push_back(blobs[index]);
      for(int k = 0;k<p.nBins;k++){
	olds.lastHistograms[i][k] = newHistos[index][k];
      }
    }
  } 
}

Trajectories initTrajectories(const vector<Ellipse>& blobs,const Mat& image,const BlobTrackingParameters& p){
  Trajectories trajectories(blobs.size());
  for(unsigned int i = 0; i<blobs.size(); i++){
    trajectories.trajectories[i].push_back(pair<Ellipse,int>(blobs[i],0));
    trajectories.lastHistograms[i] = getHistogram(blobs[i],image,p.nBins);
  }
  return trajectories;
}

void renderTrajectories(const Trajectories& trajectories,Mat& image){
  for(unsigned int i = 0;i<trajectories.trajectories.size();i++){
    for(unsigned int j = 0;j<trajectories.trajectories[i].size()-1;j++){
      Point2d p1 = trajectories.trajectories[i][j].first.center;
      Point2d p2 = trajectories.trajectories[i][j+1].first.center;
      
      //cout << "Distance : " << sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)) << endl;
      line(image,p1,p2,Scalar(0,0,255));
    } 
  }
}
