#ifndef __BLOB_TRACKING_HPP
#define __BLOB_TRACKING_HPP

#include "config.hpp"

class BlobTrackingParameters{
public:
  //Histogram size
  int nBins;
  //histogram confidence factor
  double lambdaHist;
  //Speed amplitude confidence factor
  double lambdaRSpeed;
  //Speed angle confidence factor
  double lambdaTSpeed;
  //Radius confidence factor
  double lambdaRad;
  //Minimal confidence needed to append a new blob to a trajectory
  double minConfidence;
  //Maximal distance for a blob between two frames
  double maxDist;
  //Minimal distance for a blob between two frames
  double minDist;
  //Maximal rotation
  double maxRot;

  BlobTrackingParameters(){
    nBins = 16;
    lambdaHist = 0.02;
    lambdaRSpeed = 1;
    lambdaTSpeed = 0.1;
    lambdaRad = 0.002;
    minConfidence = 0.1;
    maxDist = 150;
    minDist = 5;
    maxRot = 120*PI;
  }
}; 



void updateTrajectories(list<Balls::Trajectory>& trajs,
			const vector<double>& times,
			const vector<Ellipse> & blobs,
			double time,
			const Mat& image,
			const BlobTrackingParameters& p = 
	 		BlobTrackingParameters());


#endif
