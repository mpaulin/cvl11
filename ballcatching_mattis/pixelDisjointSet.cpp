#include "pixelDisjointSet.hpp"
#include <iostream>

using namespace std;

DisjointSet::DisjointSet(int w, int h){
  width = w;
  height = h;
  pixels = vector<Pixel>(w*h);
}

void DisjointSet::makeSet(int x, int y){
  int index = y+height*x;
  pixels[index].index=index;
  pixels[index].parent =index;
  pixels[index].rank=0; 
}

int DisjointSet::find(int p){
  if(pixels[p].parent==p){
    return p;
  }
  else{
    int parent = find(pixels[p].parent);
    pixels[p].parent = parent; 
    return parent;
  }
}

void DisjointSet::unite(int p1, int p2){
  int root1 = find(p1);
  int root2 = find(p2);
  const Pixel pr1 = pixels[root1];
  const Pixel pr2 = pixels[root2];
  if(root1 == root2) return;

  if(pr1.rank < pr2.rank)
    pixels[root1].parent = root2;
  else if(pr1.rank > pr2.rank)
    pixels[root2].parent = root1;
  else{
    pixels[root2].parent = root1;
    pixels[root1].rank += 1;
  }  
}

Pixel DisjointSet::find(int x, int y){
  return pixels[find(y+height*x)];
}

void DisjointSet::unite(int x1, int y1, int x2, int y2){
  unite(y1+height*x1,y2+height*x2);
}
 
