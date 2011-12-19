
#include "blob3dMatching.hpp"

bool comparison(const Ellipse& e1,const Ellipse& e2){
	return e1.center.y < e2.center.y;
}


vector<pair<int,int> > matchBlobs(vector<Ellipse>& blobs_left,
								  vector<Ellipse>& blobs_right,
								  const Mat& camera_left,
								  const Mat& camera_right,
								  const Blob3DMatchingParameters p){

	vector<pair<int,int> > matchings;

	//Sort the blobs according to y value.
	sort(blobs_left.begin(),blobs_left.end(),comparison);
	sort(blobs_right.begin(),blobs_right.end(),comparison);

	int i1 = 0;
	int i2 = 0;
	while(i1<blobs_left.size()){
		int oldI2 = i2;
		while(i2>=0 && i2<= blobs_right.size() && abs(blobs_left[i1].center.y-blobs_right[i2].center.y)<p.maxVertDiff){
			matchings.push_back(pair<int,int>(i1,i2));
			i2--;
		}
		i2 = oldI2;
		while(i2>=0 && i2<= blobs_right.size() && abs(blobs_left[i1].center.y-blobs_right[i2].center.y)<p.maxVertDiff){
					matchings.push_back(pair<int,int>(i1,i2));
					i2++;
		}
		i1++;
	}

	return matchings;
}
