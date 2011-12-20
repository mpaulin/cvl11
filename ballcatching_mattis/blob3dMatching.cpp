#include "blob3dMatching.hpp"

bool comparison(const Ellipse& e1,const Ellipse& e2){
	return e1.center.y < e2.center.y;
}


vector<pair<int,int> > matchBlobs(vector<Ellipse>& blobs_left,
								  vector<Ellipse>& blobs_right,
								  const Blob3DMatchingParameters& p){

	vector<pair<int,int> > matchings;

	//Sort the blobs according to y value.
	sort(blobs_left.begin(),blobs_left.end(),comparison);
	sort(blobs_right.begin(),blobs_right.end(),comparison);

	unsigned int i1 = 0;
	unsigned int i2 = 0;
	while(i1<blobs_left.size()){
		int oldI2 = i2;
		Ellipse b1 = blobs_left[i1];
		while(i2>=0 && i2< blobs_right.size() && abs(b1.center.y-blobs_right[i2].center.y)<p.maxVertDiff){
			Ellipse b2 = blobs_right[i2];
			if(0.5*sqrt((b1.a-b2.a)*(b1.a-b2.a)+(b1.b-b2.b)*(b1.b-b2.b)) < p.maxRadiusDiff){
				matchings.push_back(pair<int,int>(i1,i2));
			}
			i2--;
		}
		i2 = oldI2+1;
		while(i2>=0 && i2< blobs_right.size() && abs(b1.center.y-blobs_right[i2].center.y)<p.maxVertDiff){
			Ellipse b2 = blobs_right[i2];
			if(0.5*sqrt((b1.a-b2.a)*(b1.a-b2.a)+(b1.b-b2.b)*(b1.b-b2.b)) < p.maxRadiusDiff){
				matchings.push_back(pair<int,int>(i1,i2));
			}
			i2++;
		}
		i1++;
	}
	return matchings;
}

vector<Point3d> triangulate(vector<Ellipse>& blobs_left,
							vector<Ellipse>& blobs_right,
							const Mat& camera_left,
							const Mat& camera_right,
							const Blob3DMatchingParameters& params){
	vector<Point3d> points;
	vector<pair<int,int> > matchings = matchBlobs(blobs_left,blobs_right,params);
	for(unsigned int i = 0;i<matchings.size();i++){
		Ellipse e1 = blobs_left[matchings[i].first];
		Ellipse e2 = blobs_right[matchings[i].second];
		Point3d p = triangulate(camera_left,camera_right,e1.center,e2.center);
		if((p.x >= params.minBoundX) && (p.x < params.maxBoundX) && (p.y >= params.minBoundY) && (p.y < params.maxBoundY) && (p.z >= params.minBoundZ) && (p.z < params.maxBoundZ)){
			points.push_back(p);
		}
	}
	return points;
}
