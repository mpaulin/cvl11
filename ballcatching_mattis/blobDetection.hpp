#ifndef __BLOB_DETECTION_HPP
#define __BLOB_DETECTION_HPP

#include "config.hpp"
#include "pixelDisjointSet.hpp"

class BlobDetectionParameters{
public:
  int minBlobSize;
  int maxBlobSize;

  BlobDetectionParameters(){
    minBlobSize = 1;
    maxBlobSize = 30;
  }
};

map<int,vector<Point2i> > cluster(const Mat& image);
Mat clustersToImage(const map<int,vector<Point2i> >& clusters, int width,int height);
vector<Mat> clustersAsMats(const map<int,vector<Point2i> >& clusters);
vector<Point2i> getCenters(const vector<Mat>& clusters);
vector<Ellipse> getBlobs(const map<int,vector<Point2i> >& clusters,BlobDetectionParameters p = BlobDetectionParameters());

/** Complete detection**/
vector<Ellipse> getBlobs(const Mat& image,BlobDetectionParameters p = BlobDetectionParameters());



#endif
