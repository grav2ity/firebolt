#ifndef POLYGON_H
#define POLYGON_H

#include "mvector.h"

#include <vector>


class ConvexPolygon {

public:

  ConvexPolygon(std::vector<mVECTOR2> &vert);
  ConvexPolygon(unsigned int v, mVECTOR2 *d);
  ~ConvexPolygon();

  unsigned int getVertexCount() { return vertexCount; }
  mVECTOR2 &getVertex(int i) { return vertices[modularIndex(i)]; }
  mVECTOR2 getEdge(int i) { return ( vertices[modularIndex(i+1)] - vertices[modularIndex(i)] ); }
  mVECTOR2 getNormal(int);

  bool insidePolygon(mVECTOR2 v);

  std::vector<mVECTOR2> vertices2;

private:

  unsigned int modularIndex(int);

  int vertexCount;
  mVECTOR2 *vertices;

};


#endif
