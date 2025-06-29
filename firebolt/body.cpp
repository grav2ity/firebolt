#include "body.h"

#include "ode.h"

#include <fstream>


BodyManager *bodyManager;

//D3DXVECTOR3 gravity = D3DXVECTOR3(0.0f, -100.f, 0.0f);
mVECTOR3 gravity = mVECTOR3(0.0f, -100.f, 0.0f);
double inertiaTensorMultiplier = 400.0;

//Body///////////////////////////////////////////////////////


void Body::Rollback() {

  position = oldPosition;
  rotation = oldRotation;
  momentum = oldMomentum;
  angularMomentum = oldAngularMomentum;
  force = oldForce;
  torque = oldTorque;

}

void Body::SaveVectorState() {

  oldPosition = position;
  oldRotation = rotation;
  oldMomentum = momentum;
  oldAngularMomentum = angularMomentum;
  oldForce = force;
  oldTorque = torque;

}

void Body::UpdateInertiaTensorInverse() {

  mMATRIX rotationInverse;
  mMatrixTranspose(&rotationInverse, &rotation);
  mMatrixMultiply(&inertiaTensorInverse, &rotationInverse, &bodyInertiaTensorInverse);
  mMatrixMultiply(&inertiaTensorInverse, &inertiaTensorInverse, &rotation);

}


//BodyManager///////////////////////////////////////////////////////

BodyManager::BodyManager(unsigned int max) : bodyCount(0), maxBodies(max) {

  (char*&)bodies = (new char[maxBodies*sizeof(Body)]);
}



Body* BodyManager::NewBody(mMATRIX m, mMATRIX ineritaTensor) {

  Body* b = bodies + bodyCount;
  bodyCount++;


  b->immovable = false;
  b->mass = 4.0f ;
  b->bodyInertiaTensor = b->mass * ineritaTensor * inertiaTensorMultiplier; //not really true
  b->bodyInertiaTensor(3, 3) = 1.0f;

  mMatrixInverse(&b->bodyInertiaTensorInverse, 0, &b->bodyInertiaTensor);
  b->bodyInertiaTensor(3, 3) = 0.0f;
  b->bodyInertiaTensorInverse(3, 3) = 0.0f;
  mMatrixIdentity(&b->rotation);

  b->position = mVECTOR3(m(3, 0), m(3, 1), m(3, 2));
  m(3, 0) = m(3, 1) = m(3, 2) = 0;
  b->rotation = m;

  return bodies + bodyCount - 1;

}

void BodyManager::ArrayToBodies(double *a, unsigned int zeroIndex, unsigned int count) {

  for (unsigned int i = 0; i<count; i++)
    ArrayToBody(bodies + zeroIndex +i, (a + i * BODY_STATE_SIZE));

}

void BodyManager::BodiesToArray(double *a, unsigned int zeroIndex, unsigned int count) {

  for(unsigned int i=0; i<count; i++)
    BodyToArray(bodies + zeroIndex +i, (a + i * BODY_STATE_SIZE));

}



void BodyManager::ComputeForceAndTorque(Body*, double) {

  //presently response subsystem applies all additional forces

}

void BodyManager::DerivativeAll(double *a, double *b, double time) {

  ArrayToBodies(a, 0,bodyCount);

  for (unsigned int i = 0; i< bodyCount; i++) {

    ComputeForceAndTorque(&bodies[i], time);
    DerivativeToArray(&bodies[i], (b + i*BODY_STATE_SIZE));

  }

}

void BodyManager::DerivativeOne(double *a, double *b, double time) {

    ArrayToBody(currentBody,a);
    ComputeForceAndTorque(currentBody, time);
    DerivativeToArray(currentBody, b);

}

void BodyManager::Frame(double timeDelta) {

  IntegrateAll(timeDelta);

}

void BodyManager::IntegrateAll(double timeDelta) {

  SaveVectorStates();
  double *a = new double[bodyCount * BODY_STATE_SIZE];
  BodiesToArray(a, 0,bodyCount);
  MidPointIntegrate(a, bodyCount * BODY_STATE_SIZE, timeDelta, ::DerivativeAll);
  ArrayToBodies(a, 0,bodyCount);
  delete [] a;

}

void BodyManager::IntegrateOne(double timeDelta, Body* b) {

  b->SaveVectorState();
  currentBody = b;
  double *a = new double[BODY_STATE_SIZE];
  BodyToArray(b, a);
  MidPointIntegrate(a, BODY_STATE_SIZE, timeDelta, ::DerivativeOne);
  ArrayToBody(b, a);
  delete[] a;

}

void BodyManager::Reset() {

  memset(bodies, 0, sizeof(Body)*maxBodies);

  bodyCount = 0;
  currentBody = 0;

}

void BodyManager::RollbackAll() {

  for (unsigned int i = 0; i<bodyCount; i++)
    bodies[i].Rollback();

}

void BodyManager::RollbackOne(Body* b) {

  b->Rollback();

}

void BodyManager::SaveVectorStates() {

  for (unsigned int i = 0; i<bodyCount; i++)
    bodies[i].SaveVectorState();

}


void CrossProductMatrix(mMATRIX &m, mVECTOR3 &v) {

  m(0, 0) = 0.0f;
  m(1, 1) = 0.0f;
  m(2, 2) = 0.0f;

  m(0, 1) = +v[2];
  m(0, 2) = -v[1];

  m(1, 0) = -v[2];
  m(1, 2) = +v[0];

  m(2, 0) = +v[1];
  m(2, 1) = -v[0];

}

void BodyToArray(Body* b, double *a) {

  *a++ = b->position[0];
  *a++ = b->position[1];
  *a++ = b->position[2];

  for (int i = 0; i<3; i++)
    for (int j = 0; j<3; j++)
      *a++ = b->rotation(i, j);

  *a++ = b->momentum[0];
  *a++ = b->momentum[1];
  *a++ = b->momentum[2];

  *a++ = b->angularMomentum[0];
  *a++ = b->angularMomentum[1];
  *a++ = b->angularMomentum[2];

}

void ArrayToBody(Body* b, double *a) {

  b->position[0] = *a++;
  b->position[1] = *a++;
  b->position[2] = *a++;

  for (int i = 0; i<3; i++)
    for (int j = 0; j<3; j++)
      b->rotation(i, j) = *a++;

  b->momentum[0] = *a++;
  b->momentum[1] = *a++;
  b->momentum[2] = *a++;

  b->angularMomentum[0] = *a++;
  b->angularMomentum[1] = *a++;
  b->angularMomentum[2] = *a++;

  b->velocity = b->momentum / b->mass;

  mMATRIX rotationInverse;
  mMatrixTranspose(&rotationInverse, &b->rotation);
  mMatrixMultiply(&b->inertiaTensorInverse, &rotationInverse, &b->bodyInertiaTensorInverse);
  mMatrixMultiply(&b->inertiaTensorInverse, &b->inertiaTensorInverse, &b->rotation);
  mVec3TransformNormal(&b->angularVelocity, &b->angularMomentum, &b->inertiaTensorInverse);

}

void DerivativeToArray(Body* b, double *a) {

  if (fabs(b->velocity[0]) < 1e-2)
    b->velocity[0] = 0.0f;
  if (fabs(b->velocity[1]) < 1e-2)
    b->velocity[1] = 0.0f;
  if (fabs(b->velocity[2]) < 1e-2)
    b->velocity[2] = 0.0f;

  *a++ = b->velocity[0];
  *a++ = b->velocity[1];
  *a++ = b->velocity[2];

  if (fabs(b->angularVelocity.x) < 1e-2)
    b->angularVelocity.x = 0.0f;
  if (fabs(b->angularVelocity.y) < 1e-2)
    b->angularVelocity.y = 0.0f;
  if (fabs(b->angularVelocity.z) < 1e-2)
    b->angularVelocity.z = 0.0f;

  mMATRIX m;
  mMatrixIdentity(&m);
  CrossProductMatrix(m, b->angularVelocity);
  mMatrixMultiply(&m, &b->rotation, &m);

  for (int i = 0; i<3; i++)
    for (int j = 0; j<3; j++)
      *a++ = m(i, j);

  if (b->immovable == false)
    b->force += gravity*b->mass;

  *a++ = b->force[0];
  *a++ = b->force[1];
  *a++ = b->force[2];

  *a++ = b->torque[0];
  *a++ = b->torque[1];
  *a++ = b->torque[2];


  ///////////////////
  b->torque[0] = 0.0f;
  b->torque[1] = 0.0f;
  b->torque[2] = 0.0f;

  b->force[0] = 0.0f;
  b->force[1] = 0.0f;
  b->force[2] = 0.0f;
}

inline void DerivativeAll(double*a, double *b, double time) {
  bodyManager->DerivativeAll(a, b, time);
}

inline void DerivativeOne(double *a, double *b, double time) {
  bodyManager->DerivativeOne(a, b, time);
}

void BodyDerivative(Body* B, double *a, double *b, double time) {

  ArrayToBody(B, a);
  bodyManager->ComputeForceAndTorque(B, time);
  DerivativeToArray(B, b);

}
