#ifndef BLOB_3D_MATCHING
#define BLOB_3D_MATCHING

#include "config.hpp"

class Blob3DMatchingParameters{
public:
	//Maximum y diff allowed
	int maxVertDiff;
	//Radius thershold
	int maxRadiusDiff;
	//Bounding Box for blob detection
	double minBoundX;
	double maxBoundX;
	double minBoundY;
	double maxBoundY;
	double minBoundZ;
	double maxBoundZ ;

	Blob3DMatchingParameters(){
		maxVertDiff = 5;
		maxRadiusDiff = 5;
		minBoundX = -5000;
		maxBoundX = 5000;
		minBoundY = 0;
		maxBoundY = 5000;
		minBoundZ = -3000;
		maxBoundZ = 0;
	}
};

vector<pair<int,int> > matchBlobs(vector<Ellipse>& blobs_left,
								  vector<Ellipse>& blobs_right,
								  const Blob3DMatchingParameters& p = Blob3DMatchingParameters());



vector<Point3d> triangulate(vector<Ellipse>& blobs_left,
							vector<Ellipse>& blobs_right,
							const Mat& camera_left,
							const Mat& camera_right,
							const Blob3DMatchingParameters& params=Blob3DMatchingParameters());

#endif
