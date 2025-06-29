#ifndef POLYHEDRA_H
#define POLYHEDRA_H

#include <d3dx9.h>

#include "mesh.h"
#include "body.h"
#include "mvector.h"
#include "collision.h"

#include <vector>
#include <list>
#include <set>
#include <string>


ContactInfo GetContactInfo(OBB &a, OBB &c) ;

class OBB : public CollisionAvatar {

public:

  OBB(Body *b, double ee1, double ee2, double ee3) : CollisionAvatar(b), e1(ee1), e2(ee2), e3(ee3) { calcBoundingSphereRadius(); }

  ContactInfo GetContactInfo(CollisionAvatar *cB) { return cB->IntersectWithOBB(this); }
  ContactInfo IntersectWithOBB(OBB *cB) { return ::GetContactInfo(*this, *cB); }
  void BuildWorldMatrix();

  double e1, e2, e3;
  void TimeAlign(double);

  void calcBoundingSphereRadius() { boundingSphereRadius = sqrt(e1*e1 + e2*e2 + e3*e3); }


  unsigned int GetVertexCount()  { return 8; }
  unsigned int GetEdgeCount() { return 12; }
  unsigned int GetFaceCount() { return 6; }

  mVECTOR3 GetNormal(unsigned int i);
  mVECTOR3 GetVertex(unsigned int i);

  mVECTOR3 GetTransformedNormal(unsigned int i) { mVECTOR3 z; mVec3TransformNormal(&z, &GetNormal(i), &matrix); return z; }
  mVECTOR3 GetTransformedVertex(unsigned int i) {

    mVECTOR4 z;
    z = reinterpret_cast<mVECTOR4&>(GetVertex(i));
    z.w =1.0;
    mVec4Transform(&z, &z, &matrix);
    return mVECTOR3(z.x, z.y, z.z);

  }

  void TransformNormal(mVECTOR3&);
  void InverseTransformNormal(mVECTOR3&);
  void TransformVertex(mVECTOR3&);
  void InverseTransformVertex(mVECTOR3&);


  mMATRIX matrix, matrixInverse;
  mMATRIX offset;
  //mVECTOR3 *translation;
  mMATRIX*rotation;

};


void ComputeInterval(OBB &c, mVECTOR3 &d, ProjInfo &pInfo);
void FillExtremeVerticesSet(OBB &c, mVECTOR3 &d, ProjInfo &pInfo);
void GetContactInfo(double t1, CollisionAvatar *C0, CollisionAvatar *C1, ContactInfo &set, unsigned int maxIterations);


#endif
