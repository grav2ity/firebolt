#include "mesh.h"

Mesh::~Mesh() {

  delete[] vertices;
  delete[] faces;

  delete aabb;
  /*delete CPA*/;
}

void Mesh::setup_per_vertex_angle_weighted_normals() {

  D3DXVECTOR3 *face_normals = new D3DXVECTOR3[face_count];
  for(unsigned int i=0; i<face_count; i++)
    face_normals[i] = face_normal(i);

  for(unsigned int i=0; i<vertex_count; i++)
    memset(vertices[i].n, 0, sizeof(float) *3);

  for(unsigned int i=0; i<face_count; i++) {
    reinterpret_cast<D3DXVECTOR3&>((v(i,0).n[0])) += face_normals[i]* face_angle(i,0);
    reinterpret_cast<D3DXVECTOR3&>((v(i,1).n[0])) += face_normals[i]* face_angle(i,1);
    reinterpret_cast<D3DXVECTOR3&>((v(i,2).n[0])) += face_normals[i]* face_angle(i,2);
  }

  for(unsigned int i=0; i<vertex_count; i++) {
    D3DXVec3Normalize(reinterpret_cast<D3DXVECTOR3*>(vertices[i].n), reinterpret_cast<D3DXVECTOR3*>(vertices[i].n));
  }

  delete[] face_normals;

}

float Mesh::face_angle(unsigned int i, unsigned int n) {

  int vert1 = n+1;
  int vert2 = n+2;
  if(vert1>=3) vert1-=3;
  if(vert2>=3) vert2-=3;

  D3DXVECTOR3 v1 = (reinterpret_cast<D3DXVECTOR3&> (v(i,vert1))) - (reinterpret_cast<D3DXVECTOR3&> (v(i,n)));
  D3DXVECTOR3 v2 = (reinterpret_cast<D3DXVECTOR3&> (v(i,vert2))) - (reinterpret_cast<D3DXVECTOR3&> (v(i,n)));

  D3DXVec3Normalize(&v1, &v1);
  D3DXVec3Normalize(&v2, &v2);
  return (float)acos(D3DXVec3Dot(&v1, &v2));

};

void Mesh::face_basis(unsigned int i, D3DXVECTOR3 &tangent, D3DXVECTOR3 &bitangent) {

  //D3DXVECTOR3 v1 = (reinterpret_cast<D3DXVECTOR3&> (v(i,1))) - (reinterpret_cast<D3DXVECTOR3&> (v(i,0)));
  //D3DXVECTOR3 v2 = (reinterpret_cast<D3DXVECTOR3&> (v(i,2))) - (reinterpret_cast<D3DXVECTOR3&> (v(i,0)));

  //D3DXVECTOR2 uv1 = (reinterpret_cast<D3DXVECTOR2&> (v(i,1).uv[0])) - (reinterpret_cast<D3DXVECTOR2&> (v(i,0).uv[0]));
  //D3DXVECTOR2 uv2 = (reinterpret_cast<D3DXVECTOR2&> (v(i,2).uv[0])) - (reinterpret_cast<D3DXVECTOR2&> (v(i,0).uv[0]));


  //float det = uv1.x * uv2.y - uv2.x * uv1.y;

  //if(det) {
  //
  //  tangent = (uv2.y * v1 - uv1.y * v2) / det;
  //  bitangent = ( -v1 * uv2.x + uv1.x *v2) /det;

  //}

}

void Mesh::setup_tangent_basis() {

  //D3DXVECTOR3 *face_tangents = new D3DXVECTOR3[face_count];
  //D3DXVECTOR3 *face_bitangents = new D3DXVECTOR3[face_count];

  //
  //for(int i=0; i<vertex_count; i++) {

  //  memset(vertices[i].tn, 0, sizeof(float) *3);
  //  memset(vertices[i].bitn, 0, sizeof(float) *3);

  //}

  //for(int i=0; i<face_count; i++) {
  //
  //  face_basis(i, face_tangents[i], face_bitangents[i]);

  //}

  //for(unsigned int i=0; i<face_count; i++) {

  //  reinterpret_cast<D3DXVECTOR3&>((v(i,0).tn[0])) += face_tangents[i];
  //  reinterpret_cast<D3DXVECTOR3&>((v(i,1).tn[0])) += face_tangents[i];
  //  reinterpret_cast<D3DXVECTOR3&>((v(i,2).tn[0])) += face_tangents[i];

  //  reinterpret_cast<D3DXVECTOR3&>((v(i,0).bitn[0])) += face_bitangents[i];
  //  reinterpret_cast<D3DXVECTOR3&>((v(i,1).bitn[0])) += face_bitangents[i];
  //  reinterpret_cast<D3DXVECTOR3&>((v(i,2).bitn[0])) += face_bitangents[i];

  //}

  //for(unsigned int i=0; i<vertex_count; i++) {

  //  D3DXVec3Normalize(reinterpret_cast<D3DXVECTOR3*>(vertices[i].tn), reinterpret_cast<D3DXVECTOR3*>(vertices[i].tn));
  //  D3DXVec3Normalize(reinterpret_cast<D3DXVECTOR3*>(vertices[i].bitn), reinterpret_cast<D3DXVECTOR3*>(vertices[i].bitn));
  //}
  //
  //delete[] face_tangents;
  //delete[] face_bitangents;

}

D3DXVECTOR3 Mesh::face_normal(unsigned int i) {

  D3DXVECTOR3 v1 = (reinterpret_cast<D3DXVECTOR3&> (v(i,1))) - (reinterpret_cast<D3DXVECTOR3&> (v(i,0)));
  D3DXVECTOR3 v2 = (reinterpret_cast<D3DXVECTOR3&> (v(i,2))) - (reinterpret_cast<D3DXVECTOR3&> (v(i,0)));
  D3DXVECTOR3 normal;
  D3DXVec3Cross(&normal, &v1, &v2);
  D3DXVec3Normalize(&normal, &normal);
  return normal;

}

void Mesh::setup_per_face_normals() {

  D3DXVECTOR3 normal;
  for(unsigned int i=0; i<face_count; i++) {
    normal = face_normal(i);
    reinterpret_cast<D3DXVECTOR3&>((v(i,0).n[0])) = normal;
    reinterpret_cast<D3DXVECTOR3&>((v(i,1).n[0])) = normal;
    reinterpret_cast<D3DXVECTOR3&>((v(i,2).n[0])) = normal;
  }

}

void Mesh::setup_per_vertex_normals() {

  D3DXVECTOR3 *face_normals = new D3DXVECTOR3[face_count];
  for(unsigned int i=0; i<face_count; i++)
    face_normals[i] = face_normal(i);

  for(unsigned int i=0; i<vertex_count; i++)
    memset(vertices[i].n, 0, sizeof(float) *3);

  for(unsigned int i=0; i<face_count; i++) {
    reinterpret_cast<D3DXVECTOR3&>((v(i,0).n[0])) += face_normals[i];
    reinterpret_cast<D3DXVECTOR3&>((v(i,1).n[0])) += face_normals[i];
    reinterpret_cast<D3DXVECTOR3&>((v(i,2).n[0])) += face_normals[i];
  }

  for(unsigned int i=0; i<vertex_count; i++) {
    D3DXVec3Normalize(reinterpret_cast<D3DXVECTOR3*>(vertices[i].n), reinterpret_cast<D3DXVECTOR3*>(vertices[i].n));
  }

  delete[] face_normals;

}

void Mesh::calcBoundingSphereRadius() {

  double maxDist = sqrt(vertices[0].v[0] * vertices[0].v[0] + vertices[0].v[1] * vertices[0].v[1] + vertices[0].v[2] * vertices[0].v[2]);
  double f;
  for(unsigned int i=1; i<vertex_count; i++) {
    f = sqrt(vertices[i].v[0] * vertices[i].v[0] + vertices[i].v[1] * vertices[i].v[1] + vertices[i].v[2] * vertices[i].v[2]);
    maxDist = (f>maxDist) ? f : maxDist;
  }

  boundingSphereRadius = maxDist;

}

void Mesh::calcAABB() {

  float max_x, max_y, max_z, min_x, min_y, min_z;

  max_x = min_x = vertices[0].v[0];
  max_y = min_y = vertices[0].v[1];
  max_z = min_z = vertices[0].v[2];

  for(unsigned int i=1; i<vertex_count; i++) {
    max_x = ( max_x > vertices[i].v[0] ) ? max_x : vertices[i].v[0];
    max_y = ( max_y > vertices[i].v[1] ) ? max_y : vertices[i].v[1];
    max_z = ( max_z > vertices[i].v[2] ) ? max_z : vertices[i].v[2];

    min_x = ( min_x < vertices[i].v[0] ) ? min_x : vertices[i].v[0];
    min_y = ( min_y < vertices[i].v[1] ) ? min_y : vertices[i].v[1];
    min_z = ( min_z < vertices[i].v[2] ) ? min_z : vertices[i].v[2];
  }

  aabb = new AABB(D3DXVECTOR3(max_x, max_y, max_z), D3DXVECTOR3(min_x, min_y, min_z));

}

void vertex_info::operator=(vertex_info &r) {

  if(this != &r) {

    num_adjacent_faces = r.num_adjacent_faces;
    adjacent_faces = new unsigned short[num_adjacent_faces];
    for(unsigned int i=0; i<num_adjacent_faces; i++)
      adjacent_faces[i]= r.adjacent_faces[i];

  }

}
