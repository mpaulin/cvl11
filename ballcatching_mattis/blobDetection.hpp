#ifndef __BLOB_DETECTION_HPP
#define __BLOB_DETECTION_HPP

#include <unordered_map>

#include "config.hpp"
#include "pixelDisjointSet.hpp"

class BlobDetectionParameters{
public:
  int minBlobSize;
  int maxBlobSize;

  BlobDetectionParameters(){
    minBlobSize = 2;
    maxBlobSize = 50;
  }
};

unordered_map<int,vector<Point2i> > cluster(const Mat& image);
Mat clustersToImage(const unordered_map<int,vector<Point2i> >& clusters, int width,int height);
vector<Mat> clustersAsMats(const unordered_map<int,vector<Point2i> >& clusters);
vector<Point2f> getCenters(const vector<Mat>& clusters);
vector<Ellipse> getBlobs(const map<int,vector<Point2i> >& clusters,const BlobDetectionParameters& p = BlobDetectionParameters());

/** Complete detection**/
vector<Ellipse> getBlobs(const Mat& image,const BlobDetectionParameters& p = BlobDetectionParameters());



#endif
