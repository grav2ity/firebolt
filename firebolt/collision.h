#ifndef COLLSION_H
#define COLLSION_H

//#include "mesh.h"
#include "body.h"
#include "mvector.h"


#include <vector>
#include <list>
#include <set>
#include <string>
#include <algorithm>

///WORKFLOW
// ContactInfo -> ContactSet -> ContactCollection


class OBB;

struct ContactSet;
struct Mesh;
class CollisionAvatar;

struct ContactPointReference {

  ContactPointReference(ContactSet *c, unsigned int contactSetIndex, unsigned int matrixIndex, CollisionAvatar *cP, mVECTOR3 n);

  ContactSet *contactSetPointer;

  unsigned int matrixIndex; //index of the point in whole contact collection

  mVECTOR3 rel;
  mVECTOR3 normal;

};

struct ProjInfo {

  double min, max;
  int minIndex, maxIndex;
  int minCount, maxCount;
  std::vector<unsigned int> maxVertexIndices;
  std::vector<unsigned int> minVertexIndices;

};



struct ContactInfo {

  CollisionAvatar* cA;
  CollisionAvatar* cB;

  unsigned int aIndex, bIndex;
  unsigned int aCount, bCount;

  bool contact;
  double contactTime;

  bool volume; //interpenetration
  double overlapp; //amount of interpenetration

  bool rollback;

  mVECTOR3 contactNormal;

  ProjInfo projInfo0;
  ProjInfo projInfo1;

};

struct ContactSet {

  ContactSet() : cA(0), cB(0) {}

  std::vector<unsigned int> *aVertexIndices;
  std::vector<unsigned int> *bVertexIndices;

  CollisionAvatar* cA;
  CollisionAvatar* cB;
  std::vector<mVECTOR3> contactPoints;

  double overlapp;
  mVECTOR3 contactNormal;

};

struct ContactCollection {

  ContactCollection() : constraintCount(0), displacement(false), active(true) {}

  std::vector<ContactSet> contactSets;
  std::vector<ContactPointReference> contactPoints;

  double timeDelta;
  unsigned int constraintCount;
  bool displacement;
  bool active;

};

struct bilateralConstraint {

  bilateralConstraint(CollisionAvatar *a, CollisionAvatar *b) : A(a), B(b) {}
  CollisionAvatar *A, *B;

};

struct constraintGroup {

  std::vector<bilateralConstraint*> constraints;
  std::set<CollisionAvatar*> bodies;

};


class CollisionManager;

class CollisionAvatar {

  friend class CollisionManager;


public:

  CollisionAvatar(Body *b) : bodyRef(b), translation(&b->position), cGroup(0) {}

  virtual ~CollisionAvatar();

  virtual ContactInfo GetContactInfo(CollisionAvatar*) = 0;
  virtual ContactInfo IntersectWithOBB(OBB*) = 0;

  virtual mVECTOR3 GetTransformedVertex(unsigned int) = 0;
  virtual void BuildWorldMatrix() = 0;


  double BoundingSphereRadius() { return boundingSphereRadius; }
  Body *BodyRef() { return bodyRef; }
  mVECTOR3 &Translation() { return *translation; }


  Body *bodyRef;

  std::vector<ContactPointReference> contactPoints;

protected:

  double boundingSphereRadius;

  virtual void Zero();
  virtual void Rollback(double);
  virtual void TimeAlign(double);

  virtual void calcBoundingSphereRadius() { boundingSphereRadius = 0; }

private:

  double timeDelta;

  bool rollback;
  bool ignoreVolume;

  bool inTwo; //is in poolTwo
  bool moveOne; //is about to be moved back to poolOne

  ContactCollection *contactCollection;
  std::vector<ContactInfo*> contactInfos;

  constraintGroup *cGroup;

  mVECTOR3 *translation;

};


class CollisionManager {

public:

  void Frame(double);

  void AddCollisionAvatar(CollisionAvatar*);

  ContactCollection* NewContactCollection(ContactSet&);
  void AddToContactCollection(ContactCollection*, CollisionAvatar*);
  void AddToContactCollection(ContactCollection*, std::vector<ContactSet>::iterator, std::vector<ContactSet>::iterator);

  void ClearContactInfos(CollisionAvatar* cA);
  void Reset();
  void Clear();

  void TimeAlign(constraintGroup *cG);

  void ComputeContactSet(ContactInfo &s, CollisionAvatar& c0, CollisionAvatar& c1, ContactSet& cSet);
  void GetContactInfo(double t1, CollisionAvatar *C0, CollisionAvatar *C1, ContactInfo &I, unsigned int maxIterations);

  bool BoundingSphereIntersection(CollisionAvatar& s0, CollisionAvatar& s1);

  void BuildConstraintGroups();
  void ContactSetFromConstraint(CollisionAvatar*A, CollisionAvatar*B, ContactSet& cSet);


  std::vector<bilateralConstraint> constraints;

private:

  std::vector<CollisionAvatar*> collisionAvatars;
  std::vector<constraintGroup*> constraintGroups;


  std::list<ContactCollection*> contactCollections;
  std::list<ContactInfo*> contactInfos;

  std::list<CollisionAvatar*> poolOne, poolTwo;
  std::list<CollisionAvatar*>::iterator poolOneIt, poolTwoIt;

  std::vector<CollisionAvatar*> twoToOne;
  std::vector<CollisionAvatar*>::iterator twoToOneIt;

  ContactInfo * I;

};

extern CollisionManager *collisionManager;


#endif
