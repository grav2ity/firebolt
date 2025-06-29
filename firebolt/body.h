#ifndef BODY_H
#define BODY_H

#include "mvector.h"

#include <vector>
#include <set>


//based on 'An Introduction to Physically Based Modeling' by David Baraff


class CollisionAvatar;
struct constraintGroup;

struct Body {

  void SaveVectorState();
  void Rollback();

  void UpdateInertiaTensorInverse();
  bool immovable;


  //BODY CONSTATNS
  double mass;
  mMATRIX bodyInertiaTensor;
  mMATRIX bodyInertiaTensorInverse;

  //STATE VECTOR
  mVECTOR3 position;
  mMATRIX rotation;
  mVECTOR3 momentum;
  mVECTOR3 angularMomentum;

  //PER FRAME FORCE ACCUMULATOR
  mVECTOR3 force;
  mVECTOR3 torque;

  //OLD STATE VECTOR (for rollback)
  mVECTOR3 oldPosition;
  mMATRIX oldRotation;
  mVECTOR3 oldMomentum;
  mVECTOR3 oldAngularMomentum;
  mVECTOR3 oldForce;
  mVECTOR3 oldTorque;

  //HELPERS
  mMATRIX inertiaTensorInverse;
  mVECTOR3 velocity;
  mVECTOR3 angularVelocity;
};


void CrossProductMatrix(mMATRIX&, mVECTOR3&);

void BodyToArray(Body*,double*);
void ArrayToBody(Body*, double*);
void DerivativeToArray(Body*, double*);

//FUNCTIONS TO BE PASSED TO INTEGRATOR
void DerivativeAll(double*, double*, double);
void DerivativeOne(double*, double*, double);


class BodyManager {

public:

  BodyManager(unsigned int max = MAX_BODIES);
  ~BodyManager() { Reset(); }

  Body* NewBody(mMATRIX m, mMATRIX ineritaTensor);

  void Frame(double);
  void Reset();

  void IntegrateAll(double);
  void IntegrateOne(double, Body*);

  void DerivativeAll(double*, double*, double);
  void DerivativeOne(double*, double*, double);

  void RollbackAll();
  void RollbackOne(Body*);

  void ComputeForceAndTorque(Body*, double);

private:

  void SaveVectorStates();

  void BodiesToArray(double*, unsigned int, unsigned int);
  void ArrayToBodies(double*, unsigned int, unsigned int);

  unsigned int bodyCount;
  unsigned int maxBodies;

  Body* bodies;

  Body* currentBody;

  static const unsigned int BODY_STATE_SIZE = 18;
  static const unsigned int MAX_BODIES = 100;

};

extern BodyManager *bodyManager;


#endif
