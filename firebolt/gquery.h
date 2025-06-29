#ifndef GQUERY_H
#define GQUERY_H

#include <d3dx9.h>

#include "mvector.h"
#include "polygon.h"

#include <vector>

//basic geometry intersection queries

struct ray {

  ray(const D3DXVECTOR3 &s, const D3DXVECTOR3 &d) : start(s), direction(d) {}
  D3DXVECTOR3 start;
  D3DXVECTOR3 direction;
  D3DXVECTOR3 point(float t) { return start + t*direction; }

};

struct AABB {

  AABB() {}
  AABB(const D3DXVECTOR3 &a, const D3DXVECTOR3 &b) : max_(a), min_(b) {}
  D3DXVECTOR3 max_;
  D3DXVECTOR3 min_;

};

void sort(std::vector<mVECTOR2>&);


D3DXVECTOR3 edge_edge(D3DXVECTOR3, D3DXVECTOR3, D3DXVECTOR3, D3DXVECTOR3);

mVECTOR2 edge_edge(mVECTOR2, mVECTOR2, mVECTOR2, mVECTOR2);


void edge_polygon(std::vector<mVECTOR2> &polygon, mVECTOR2, mVECTOR2, std::vector<mVECTOR2> &result);

void polygon_polygon(ConvexPolygon&, ConvexPolygon&, std::vector<mVECTOR2> &result);


#endif // __GQUERY_
