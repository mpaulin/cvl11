#include "blobDetection.hpp"

#include <sys/time.h>

int f = 0;

unordered_map<int,vector<Point2i> > cluster(const Mat& image){
	struct timeval tv;
	gettimeofday(&tv,NULL);
	double t0 = double(tv.tv_usec)/1000;
	vector<int> index;
	unordered_map<int,int> fg;
	int nFG = 0;
	for(int y = 0; y < image.rows; y++){
		const unsigned char* My = image.ptr<unsigned char>(y);
		for(int x = 0; x < image.cols; x++){
			if(My[x]!=0){
				fg.insert(pair<int,int>(y+image.rows*x,nFG++));
				index.push_back(y+image.rows*x);
			}
		}
	}

	gettimeofday(&tv,NULL);
	double t1 = double(tv.tv_usec)/1000;
	DisjointSet disjointSet(nFG);
	gettimeofday(&tv,NULL);
	double t2 = double(tv.tv_usec)/1000;
	for(unsigned int i = 0;i<index.size();i++){
		disjointSet.makeSet(fg[index[i]]);
		int y = index[i] % image.rows;
		int x = index[i] / image.rows;
		//cout << x << " " << y << endl;
		if(x>0){
			int left = y+image.rows*(x-1);
			if(fg.count(left)){
				disjointSet.unite(fg[index[i]],fg[left]);
				//cout << "Uniting " << fg[index[i]] << " (" << x << " " << y << ") with " << fg[left] << " (" << x-1 << " " << y << ")" << endl;
			}
		}
		if(y>0){
			int up = y-1+image.rows*x;
			if(fg.count(up)){
				//cout << "Uniting " << fg[index[i]] << " ("<< x << " " << y << ") with " << fg[up] << " (" << x << " " << y-1 << ")" << endl;
				disjointSet.unite(fg[index[i]],fg[up]);
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
	//cout << "Benchmark blobs : All " << t4-t0 << " | Fg " << t1-t0 << " | Creation " << t2-t1 << " | CC : " << t3-t2 << " | Parents " << t4-t3 << endl;

	//char name[1000];
	//sprintf(name,"/home/mattis/ETHZ/CVLabs/cvl11/tmp/cluters%04d.png",f++);
	//imwrite(string(name),clustersToImage(clusters,640,480));
	return clusters;
}


Mat clustersToImage(const unordered_map<int,vector<Point2i> >& clusters, int width,int height){
	Mat clusteredImage(height,width,CV_8UC1,Scalar(0,0,0));
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

vector<Point2f> getCenters(const vector<Mat>& clusters){
	vector<Point2f> centers;
	for(unsigned int i = 0;i<clusters.size();i++){
		int x = 0;
		int y = 0;
		Mat m = clusters[i];
		const double* rx = m.ptr<double>(0);
		const double* ry = m.ptr<double>(1);
		for(int j = 0;j<clusters[i].cols;j++){
			x+=rx[j];
			y+=ry[j];
		}
		x/=m.cols;
		y/=m.cols;
		centers.push_back(Point2f(x,y));
	}
	return centers;
}

vector<double> getRadii(const vector<Mat>& clusters,const vector<Point2f>& centers){
	vector<double> radii;
	for(unsigned int i = 0;i<clusters.size();i++){
		double r = 0;
		Mat m = clusters[i];
		Point2i center = centers[i];
		const double* px = m.ptr<double>(0);
		const double* py = m.ptr<double>(1);
		for(int j = 0;j<clusters[i].cols;j++){
			double rx = px[j]-center.x;
			double ry = py[j]-center.y;
			r+=sqrt(rx*rx+ry*ry);
		}
		r = 2*r/clusters[i].cols;
		radii.push_back(r);
	}
	return radii;
}

#if 1
vector<Ellipse> getBlobs(const unordered_map<int,vector<Point2i> >& clusters,const BlobDetectionParameters& p){
	vector<Ellipse> blobs;
	vector<Mat> cls = clustersAsMats(clusters);
	vector<Point2f> centers = getCenters(cls);
	vector<double> radii = getRadii(cls,centers);
	for(unsigned int i = 0;i<centers.size();i++){
		float radius = radii[i];
		Point2f center = centers[i];
		if(radius>p.minBlobSize && radius < p.maxBlobSize){
			blobs.push_back(Ellipse(center,radius,radius,0));
		}
	}
	return blobs;
}
#else
vector<Ellipse> getBlobs(const unordered_map<int,vector<Point2i> >& clusters,const BlobDetectionParameters& p){
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
#endif

vector<Ellipse> getBlobs(const Mat& image,const BlobDetectionParameters& p){
	unordered_map<int,vector<Point2i> > clusters = cluster(image);
	vector<Ellipse> blobs = getBlobs(clusters,p);
	return blobs;
}
