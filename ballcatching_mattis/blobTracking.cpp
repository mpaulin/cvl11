#include "blobTracking.hpp"



double evalConfidence(const Ellipse& e1,
		      double t1,
		      const Ellipse& e2,
		      double t2,
		      const Ellipse& le,
		      double tl,
		      const vector<double>& hist1,
		      const vector<double>& hist2,
		      const Mat& image,
		      const BlobTrackingParameters& p){

  //TODO Compute speed and angle
    double v1r = 0;
    double v2r = 0;
    double v1t = 0;
    double v2t = 0;

    if(tl>=0){
      Point2d p1 = e1.center;
      Point2d p2 = e2.center;
      Point2d lp = le.center;
      v1r = (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
      v1r/= (t2-t1)*(t2-t1);
      
      v2r = (lp.x-p2.x)*(lp.x-p2.x)+(lp.y-p2.y)*(lp.y-p2.y);
      v2r/= (tl-t2)*(tl-t2);
      
      v1r = sqrt(v1r);
      v2r = sqrt(v2r);
      
      v1t = atan2(e2.center.y-e1.center.y,e2.center.x-e1.center.x);
      v2t = atan2(le.center.y-e2.center.y,le.center.x-e2.center.x);
  }
  
  //compute histogram distance
  double dHist = 0;
  for(int i = 0;i<p.nBins;i++){
    dHist += abs(hist1[i]-hist2[i]);
  }
  //cout << "Hist distance : " << dHist << endl;
    
  double dVR = abs(v1r-v2r);
  double dVT = min(min(abs(v1t-v2t),abs(v1t-v2t-2*PI)),abs(2*PI+v1t-v2t));
  double dRad = sqrt((e1.a-e2.a)*(e1.a-e2.a) 
		     +(e1.b-e2.b)*(e1.b-e2.b)) ;
  
  double dist = e1.distanceTo(e2);
  if(dist > p.maxDist || dist < p.minDist || dVT > p.maxRot) return 0;
  //cout << dHist << " " << dRad << " " << dVR << " " << dVT << endl;
  double confidence = 1./(p.lambdaHist*dHist+p.lambdaRad*dRad+p.lambdaTSpeed*dVT); 
  //cout << v1r << endl;
  return confidence;
} 

void updateTrajectories(list<Balls::Trajectory>& trajs,
			const vector<double>& times,
			const vector<Ellipse>& blobs, 
			int frame,
			const Mat& image,
			const BlobTrackingParameters& p){
  vector<vector<double> > newHistos(blobs.size());
  for(unsigned int i = 0;i<blobs.size();i++){
    newHistos[i] = getHistogram(blobs[i],image,p.nBins);
  }

  if(trajs.size()==0){//Initialization
    for(unsigned int i = 0;i<blobs.size();i++){
      Balls::Trajectory t;
      t.addPoint(blobs[i],frame,newHistos[i]);
      trajs.push_back(t);
    }
    return;
  }

  for(list<Balls::Trajectory>::iterator it = trajs.begin();
      it!=trajs.end();
      it++){
    double maxConfidence = 0;
    int index = 0;
    for(unsigned int j = 0;j<blobs.size();j++){
      Ellipse le;
      double tl = -1;
      if(it->length>0){
	le = it->getEllipse(it->length-2);
	tl = times[it->getFrame(it->length-2)];
      }
      double confidence = evalConfidence(blobs[j],
					 times[frame],
					 it->getEllipse(it->length-1),
					 times[it->getFrame(it->length-1)],
					 le,
					 tl,
					 newHistos[j],
					 it->getHistogram(it->length-1),
					 image,
					 p);
      //cout << "confidence : " << confidence<< endl;
      if(confidence > maxConfidence){
	index = j;
	maxConfidence = confidence;
      }
    }
    if(maxConfidence>p.minConfidence){
      it->addPoint(blobs[index],frame,newHistos[index]);
    }
  } 
}
