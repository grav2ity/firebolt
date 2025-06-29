#define _USE_MATH_DEFINES
#include "polygon.h"

#include "direct3d.h"

#include <fstream>
#include <cmath>


ConvexPolygon::ConvexPolygon(std::vector<mVECTOR2> &vert) : vertexCount(vert.size()) {

  vertices = new mVECTOR2[vertexCount];
  for(unsigned int i =0; i<vert.size(); i++)
    memcpy(&vertices[i], &vert[i], sizeof(mVECTOR2));

  vertices2 = vert;

}

ConvexPolygon::ConvexPolygon(unsigned int v, mVECTOR2* d) : vertexCount(v) {

  vertices = new mVECTOR2[vertexCount];
  memcpy(vertices, d, sizeof(mVECTOR2) * vertexCount);

}

ConvexPolygon::~ConvexPolygon() {

  delete[] vertices;

}

bool ConvexPolygon::insidePolygon(mVECTOR2 v) {

  mVECTOR2 n1,n2,edge,edgeNormal;
  for(int i=0; i<vertexCount; i++) {

    edge = getEdge(i);
    edgeNormal = mVECTOR2(edge.y,-edge.x);
    n1 = getVertex(i);
    //float f = -mVec2Dot(&n1,&edgeNormal);
    //if ( (mVec2Dot(&edgeNormal,&v)+f) > -ER)
    //  return false;
    double f = mVec2Dot(&n1,&edgeNormal);
    if (mVec2Dot(&edgeNormal, &v) > (0.1 + f))
      return false;

  }

  return true;

}

unsigned int ConvexPolygon::modularIndex(int i) {

  if(i == 0)
    return 0;
  else if (i>0) {
    return i % vertexCount;
  }
  else {
    return vertexCount - ((-i) % vertexCount);
  }

}

mVECTOR2 ConvexPolygon::getNormal(int i) {

  mVECTOR2 e= getEdge(i);
  double temp = e.x;
  e.x = -e.y;
  e.y = temp;

  return e;

}
