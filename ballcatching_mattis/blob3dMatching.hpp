#ifndef BLOB_3D_MATCHING
#define BLOB_3D_MATCHING

#include "config.hpp"

class Blob3DMatchingParameters{
public:
	//Maximum y diff allowed
	int maxVertDiff;

	Blob3DMatchingParameters(){
		maxVertDiff = 5;
	}
};

vector<pair<int,int> > matchBlobs(const vector<Ellipse>& blobs_left,
								  const vector<Ellipse>& blobs_right,
								  const Mat& camera_left,
								  const Mat& camera_right,
								  const Blob3DMatchingParameters p = Blob3DMatchingParameters());












#endif
