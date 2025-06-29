#ifndef MESH_H
#define MESH_H

#include <d3dx9.h>
#include "gquery.h"


struct vertex {

  float v[3];
  float n[3];
  //float bitn[3];
  //float tn[3];
  //float uv[2];

};

struct face {

  unsigned short i[3];

};

struct Mesh {

  Mesh() : vertex_count(0), face_count(0), vertices(0), faces(0), aabb(0)/*, CPA(0)*/ {}
  ~Mesh();

  vertex &v(unsigned int i, unsigned int j) { return vertices[faces[i].i[j]]; }
  D3DXVECTOR3 face_normal(unsigned int i);
  void face_basis(unsigned int i, D3DXVECTOR3 &tangent, D3DXVECTOR3 &bitangent) ;

  void setup_per_face_normals();
  void setup_per_vertex_normals();
  void setup_per_vertex_angle_weighted_normals();
  void setup_tangent_basis();

  float face_angle(unsigned int i, unsigned int);

  void calcAABB();
  void calcBoundingSphereRadius();

  vertex *vertices;
  face *faces;

  unsigned int vertex_count;
  unsigned int face_count;

  int voffset;
  int ioffset;

  double boundingSphereRadius;

  AABB *aabb;
  //ConvexPolyhedronArchetype *CPA;

};

struct vertex_info {

  void operator=(vertex_info& r);
  ~vertex_info() { delete[] adjacent_faces; }
  unsigned int num_adjacent_faces;
  unsigned short *adjacent_faces;

};

#endif
