#ifndef INSECT_H
#define INSECT_H

#include "direct3d.h"

#include "mvector.h"
#include "body.h"
#include "gquery.h"
#include "oob.h"

#include <list>
#include <map>


class SceneReader;
class DMeshClass;
class DMeshManager;


//presently only a helper for file.cpp reading functioncs
struct Node {


  Node() : obb(0), parent(0), childrenCount(0) { }
  ~Node() { for (unsigned int i = 0; i<childrenCount; i++) delete children[i]; delete[] children; }

  unsigned int frozen;
  //
  D3DXMATRIX offsetMatrix;
  D3DXMATRIX staticMatrix;
  D3DXMATRIX worldMatrix;

  D3DXCOLOR color;

  Body *bodyRef;

  mMATRIX inertiaTensor;

  //geometry
  int meshIndex;
  OBB *obb;

  //hierarchy
  Node *parent;
  Node **children;
  unsigned int childrenCount;

};


class DMeshInstance {

  friend class DMeshClass;

public:

  DMeshInstance(Body*, D3DXMATRIX, D3DXCOLOR);

  void getAABB(AABB&);
  void getMatrix(D3DXMATRIX&, unsigned int);

  void UpdateState();



  D3DXVECTOR3 translation;
  D3DXMATRIX rotation;

  Body *bodyRef;
  D3DXMATRIX offsetMatrix;


  D3DXCOLOR color;

};

class DMeshClass {

friend class DMeshManager;

public:

  DMeshClass(int, unsigned int);
  ~DMeshClass();

  void newDMesh(Body*, D3DXMATRIX, D3DXCOLOR);
  void streamDMeshs();
  void updateDMeshs();


  unsigned int meshIndex;
  AABB *aabb;

  std::list<DMeshInstance> dMeshes;

  unsigned int maxDMeshCount;
  unsigned int curDMeshCount;

  IDirect3DVertexBuffer9 *matrixVertexBuffer;
};

class DMeshManager {

public:

  void newDMesh(unsigned int, Body*, D3DXMATRIX, D3DXCOLOR);
  void registerDMeshClass(unsigned int);
  void streamDMeshClasses();
  void updateDMeshClasses();

  void Reset();

  std::map<unsigned int, DMeshClass*> dmeshClasses;

};

extern DMeshManager dmeshManager;


#endif
