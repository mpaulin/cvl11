#include "blobDetection.hpp"

#include <sys/time.h>

#if 1
unordered_map<int,vector<Point2i> > cluster(const Mat& image){
	struct timeval tv;
	gettimeofday(&tv,NULL);
	double t0 = double(tv.tv_usec)/1000;
	unordered_map<int,int> fg;
	int nFG = 0;
	for(int y = 0; y < image.rows; y++){
		const unsigned char* My = image.ptr<unsigned char>(y);
		for(int x = 0; x < image.cols; x++){
			if(My[x]!=0)
				fg.insert(pair<int,int>(y+image.rows*x,nFG++));
		}
	}

	gettimeofday(&tv,NULL);
	double t1 = double(tv.tv_usec)/1000;
	DisjointSet disjointSet(nFG);
	gettimeofday(&tv,NULL);
	double t2 = double(tv.tv_usec)/1000;
	for(unordered_map<int,int>::const_iterator it = fg.begin();it!=fg.end();++it){
		int y = it->first % image.rows;
		int x = it->first / image.rows;
		if(x>0){
			int left = y+image.rows*(x-1);
			if(fg.count(left)){
				disjointSet.unite(it->second,fg[left]);
			}
		}
		if(y>0){
			int up = y-1+image.rows*x;
			if(fg.count(up)){
				disjointSet.unite(it->second,fg[up]);
			}
		}
	}
	gettimeofday(&tv,NULL);
	double t3 = double(tv.tv_usec)/1000;
	unordered_map<int,vector<Point2i> > clusters;
	for(unordered_map<int,int>::const_iterator it = fg.begin();it!=fg.end();++it){
		int i = disjointSet.find(it->second);
		int y = it->first % image.rows;
		int x = it->first / image.rows;
		if(!clusters.count(i)){
			clusters.insert(pair<int,vector<Point2i> >(i,vector<Point2i>(1,Point2i(x,y))));
		}
		else{
			clusters[i].push_back(Point2i(x,y));
		}
	}
	gettimeofday(&tv,NULL);
	double t4 = double(tv.tv_usec)/1000;
	cout << "Benchmark blobs : All " << t4-t0 << " | Fg " << t1-t0 << " | Creation " << t2-t1 << " | CC : " << t3-t2 << " | Parents " << t4-t3 << endl;
	return clusters;
}
#else
unordered_map<int,vector<Point2i> > cluster(const Mat& image){
	struct timeval tv;
	gettimeofday(&tv,NULL);
	double t1 = double(tv.tv_usec)/1000;
	DisjointSet disjointSet(image.cols,image.rows);
	gettimeofday(&tv,NULL);
	double t2 = double(tv.tv_usec)/1000;
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
	gettimeofday(&tv,NULL);
	double t3 = double(tv.tv_usec)/1000;
	unordered_map<int,vector<Point2i> > clusters;
	for(int y = 0;y<image.rows;y++){
		const unsigned char* My = image.ptr<unsigned char>(y);
		for(int x = 0;x<image.cols;x++){
			if(My[x]!=0){
				Pixel p = disjointSet.find(x,y);
				int i = p.index;
				if(!clusters.count(i)){
					clusters.insert(pair<int,vector<Point2i> >(i,vector<Point2i>(1,Point2i(x,y))));
				}
				else{
					clusters[i].push_back(Point2i(x,y));
				}
			}
		}
	}
	gettimeofday(&tv,NULL);
	double t4 = double(tv.tv_usec)/1000;
	cout << "Benchmark blobs : All " << t4-t1 << " | Creation " << t2-t1 << " | CC : " << t3-t2 << " | parents " << t4-t3 << endl;
	return clusters;

}
#endif


Mat clustersToImage(const unordered_map<int,vector<Point2i> >& clusters, int width,int height){
	Mat clusteredImage(height,width,CV_8UC1);
	for(unordered_map<int,vector<Point2i> >::const_iterator it = clusters.begin();it!=clusters.end();it++){
		for(vector<Point2i>::const_iterator it2 = it->second.begin(); it2!=it->second.end();it2++){
			clusteredImage.at<unsigned char>(it2->y,it2->x) = (73153413*it->first)%(255);
		}
	}
	cout << "Found " << clusters.size() << " connected components." << endl;
	return clusteredImage;
}

vector<Mat> clustersAsMats(const unordered_map<int,vector<Point2i> >& clusters){
	vector<Mat> result;
	for(unordered_map<int,vector<Point2i> >::const_iterator it = clusters.begin();it!=clusters.end();it++){
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

vector<Ellipse> getBlobs(const unordered_map<int,vector<Point2i> >& clusters,BlobDetectionParameters p){
	vector<Ellipse> blobs;
	for(unordered_map<int,vector<Point2i> >::const_iterator it = clusters.begin();it != clusters.end();it++){
		float radius;
		Point2f center;
		minEnclosingCircle(it->second,center,radius);
		if(radius>p.minBlobSize && radius < p.maxBlobSize){
			blobs.push_back(Ellipse(center,radius,radius,0));
		}
	}
	return blobs;
}


vector<Ellipse> getBlobs(const Mat& image,BlobDetectionParameters p){
	unordered_map<int,vector<Point2i> > clusters = cluster(image);
	vector<Ellipse> blobs = getBlobs(clusters,p);
	return blobs;
}
