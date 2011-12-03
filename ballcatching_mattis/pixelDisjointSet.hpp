#ifndef _PIXEL_DISJOINT_SET_HPP
#define _PIXEL_DISJOINT_SET_HPP

#include <map>
#include <vector>

using namespace std;

class Pixel{
  public:
    
    int index;
    int parent;
    int rank;


    Pixel(){
      index = 0;
      parent = 0;
      rank = 0;
    }
    
    Pixel(int i,int p, int r){
      index = i;
      parent = p;
      rank = r;
    }

  };

class DisjointSet{
public:
  int width;
  int height;
  DisjointSet(int w,int h);
  void makeSet(int x, int y);
  int find(int);
  void unite(int p1, int p2);
  Pixel find(int x, int y);
  void unite(int,int,int,int);
private:
  vector<Pixel> pixels;
};













































#endif


