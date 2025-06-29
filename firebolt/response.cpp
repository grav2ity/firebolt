#include "response.h"

#include "dantzig.h"

#include <set>

//Matrix *A;

//extern D3DXVECTOR3 gravity;
extern mVECTOR3 gravity;

//double E = 1.0f;


//some hand tuned magic variables changing of which may crash simulation
double IMPULSE_MULTIPLIER = 1.11;
double IMPULSE_THRESHOLD = 0;


//std::ofstream& operator<<(std::ofstream& o, mVECTOR3& v) {
//
//  o << v.x << ' ' << v.y << ' ' << v.z ;
//  return o;
//
//}
//std::stringstream& operator<<(std::stringstream& o, mVECTOR3& v) {
//
//  o << v.x << ' ' << v.y << ' ' << v.z ;
//  return o;
//
//}

void PointOnBodyVelocity(Body* b, mVECTOR3 r, mVECTOR3 &v) {

  mVec3Cross(&r, &b->angularVelocity, &r);
  v = b->velocity + r;

}

RResponse::RResponse(ContactCollection& cCollection) : cCollection(&cCollection), pointCount(0) {

  for (contactSetIt = cCollection.contactSets.begin(); contactSetIt != cCollection.contactSets.end(); contactSetIt++) {

    pointCount += (*contactSetIt).contactPoints.size();

    CCBodies.insert((*contactSetIt).cA);
    CCBodies.insert((*contactSetIt).cB);

  }

  velocityPass = false;
  impulse = false;

  relV = new double[pointCount];
  V = new double[pointCount];
  F = new double[pointCount];

  memset(V, 0, sizeof(double) * pointCount);
  memset(F, 0, sizeof(double) * pointCount);

  A = new Matrix(pointCount, pointCount);


  std::vector<mVECTOR3>::iterator it2;

  CollisionAvatar *A, *B;
  mVECTOR3 bArel;
  mVECTOR3 bBrel;
  mVECTOR3 vA, vB, vD;
  int l(0);
  for (contactSetIt = cCollection.contactSets.begin(); contactSetIt != cCollection.contactSets.end(); contactSetIt++) {

    A = (*contactSetIt).cA;
    B = (*contactSetIt).cB;

    for (it2 = (*contactSetIt).contactPoints.begin(); it2 != (*contactSetIt).contactPoints.end(); it2++, l++) {

      bArel = (*it2) - A->Translation();
      bBrel = (*it2) - B->Translation();

      PointOnBodyVelocity(A->bodyRef, bArel, vA);
      PointOnBodyVelocity(B->bodyRef, bBrel, vB);

      vD = vA - vB;

      V[l] = mVec3Dot(&vD, &(*contactSetIt).contactNormal);

      if (V[l] < 0) {

        impulse = true;
        velocityPass = true;

      }
    }
  }

  if (impulse) {

    for (unsigned int i = 0;  i< pointCount - cCollection.constraintCount; i++)
      V[i] *= IMPULSE_MULTIPLIER;

  }

  if (cCollection.constraintCount >0)
    velocityPass = true;

}

RResponse::~RResponse() {

  delete[] F;
  delete[] V;
  delete[] relV;
  delete A;

}

void RResponse::BuildMatrix(Matrix *MA) {

  std::set<CollisionAvatar*>::iterator CAit;
  std::vector<ContactPointReference>::iterator it1, it2;

  mVECTOR3 torque;
  mVECTOR3 ACC;
  mVECTOR3 A;
  mVECTOR3 r;

  double entry;
  unsigned int mCount(0);

  for (CAit = CCBodies.begin(); CAit != CCBodies.end(); CAit++) {

    if ((*CAit)->bodyRef->immovable == true)
      continue;

    for (it1 = (*CAit)->contactPoints.begin(); it1 != (*CAit)->contactPoints.end(); it1++) {

      mVec3Cross(&torque, &(*it1).rel, &(*it1).normal);
      A = (1.0 / (*CAit)->bodyRef->mass) * (*it1).normal;
      mVec3TransformNormal(&ACC, &torque, &(*CAit)->bodyRef->inertiaTensorInverse);

      for (it2 = (*CAit)->contactPoints.begin(); it2 != (*CAit)->contactPoints.end(); it2++) {

        entry = 0.0;
        mVec3Cross(&r, &ACC, &(*it2).rel);
        entry = mVec3Dot(&A, &(*it2).normal) + mVec3Dot(&(*it2).normal, &r);

        (*MA)((*it2).matrixIndex, (*it1).matrixIndex) += entry;

      }
    }
  }

}


void RResponse::BuildConstantVector(double *V) {

  std::vector<ContactSet>::iterator it;
  std::vector<mVECTOR3>::iterator it2;

  CollisionAvatar *A, *B;

  mVECTOR3 bArel;
  mVECTOR3 bBrel;

  mVECTOR3 vA, vB, vRel;
  mVECTOR3 forceA, forceB;
  mVECTOR3 axis, axisVelocity;

  unsigned int mCount(0);

  for (it = cCollection->contactSets.begin(); it != cCollection->contactSets.end(); it++) {

    A = (*it).cA;
    B = (*it).cB;

    axis = (*it).contactNormal;
    mVec3Cross(&axisVelocity, &B->bodyRef->angularVelocity, &axis);
    axisVelocity *= 2;

    for (it2 = (*it).contactPoints.begin(); it2 != (*it).contactPoints.end(); it2++, mCount++) {

      bArel = (*it2) - A->Translation();
      bBrel = (*it2) - B->Translation();

      PointOnBodyVelocity(A->bodyRef, bArel, vA);
      PointOnBodyVelocity(B->bodyRef, bBrel, vB);

      //vA = A->bodyRef->velocity;

      vRel = vA - vB;
      //vRel = A->bodyRef->velocity - bBrel;

      if (mCount < (pointCount - cCollection->constraintCount)) {

        mVec3Cross(&forceA, &A->bodyRef->angularVelocity, &bArel);
        mVec3Cross(&forceB, &B->bodyRef->angularVelocity, &bBrel);
        mVec3Cross(&forceA, &A->bodyRef->angularVelocity, &forceA);
        mVec3Cross(&forceB, &B->bodyRef->angularVelocity, &forceB);

      }
      else {

        forceA = mVECTOR3(0.0, 0.0, 0.0);
        forceB = mVECTOR3(0.0, 0.0, 0.0);

      }

      forceA += gravity;
      forceB += gravity;

      if (A->bodyRef->immovable == false) {

        V[mCount] += mVec3Dot(&(*it).contactNormal, &forceA);

      }

      if (B->bodyRef->immovable == false) {

        V[mCount] -= mVec3Dot(&(*it).contactNormal, &forceB);

      }

      if (mCount < (pointCount - cCollection->constraintCount))
        V[mCount] += mVec3Dot(&axisVelocity, &vRel);

    }
  }

}


void RResponse::ApplyForcesVector() {

  std::vector<ContactSet>::iterator it;
  std::vector<mVECTOR3>::iterator it2;

  CollisionAvatar *A, *B;
  mVECTOR3 bArel;
  mVECTOR3 bBrel;
  mVECTOR3 r, k;

  unsigned int mCount(0);

  for (it = cCollection->contactSets.begin(); it != cCollection->contactSets.end(); it++) {

    A = (*it).cA;
    B = (*it).cB;

    for (it2 = (*it).contactPoints.begin(); it2 != (*it).contactPoints.end(); it2++, mCount++) {

      if (A->bodyRef->immovable == false) {

        r = (*it2) - A->Translation();
        mVec3Cross(&k, &r, &(*it).contactNormal);
        A->bodyRef->torque += (F[mCount] * k);
        A->bodyRef->force += (*it).contactNormal * F[mCount];

      }

      if (B->bodyRef->immovable == false) {

        r = (*it2) - B->Translation();
        mVec3Cross(&k, &r, &(*it).contactNormal);
        B->bodyRef->torque -= (F[mCount] * k);
        B->bodyRef->force -= (*it).contactNormal * F[mCount];

      }
    }
  }

}

void RResponse::ApplyImpulsesVector() {

  std::vector<ContactSet>::iterator it;
  std::vector<mVECTOR3>::iterator it2;

  CollisionAvatar *A, *B;

  mVECTOR3 bArel;
  mVECTOR3 bBrel;

  unsigned int mCount(0);

  mVECTOR3 r, k;

  for (it = cCollection->contactSets.begin(); it != cCollection->contactSets.end(); it++) {

    A = (*it).cA;
    B = (*it).cB;

    for (it2 = (*it).contactPoints.begin(); it2 != (*it).contactPoints.end(); it2++, mCount++) {

      if (A->bodyRef->immovable == false) {

        r = (*it2) - A->Translation();
        mVec3Cross(&k, &r, &(*it).contactNormal);
        A->bodyRef->angularMomentum += (F[mCount] * k);
        A->bodyRef->momentum += (*it).contactNormal * F[mCount];

      }

      if (B->bodyRef->immovable == false) {

        r = (*it2) - B->Translation();
        mVec3Cross(&k, &r, &(*it).contactNormal);
        B->bodyRef->angularMomentum -= (F[mCount] * k);
        B->bodyRef->momentum -= (*it).contactNormal * F[mCount];

      }
    }
  }

  A->bodyRef->velocity = A->bodyRef->momentum / A->bodyRef->mass;
  B->bodyRef->velocity = B->bodyRef->momentum / B->bodyRef->mass;

  mMATRIX rotationInverse;
  mMatrixTranspose(&rotationInverse, &A->bodyRef->rotation);

  mMatrixMultiply(&A->bodyRef->inertiaTensorInverse, &rotationInverse, &A->bodyRef->bodyInertiaTensorInverse);
  mMatrixMultiply(&A->bodyRef->inertiaTensorInverse, &A->bodyRef->inertiaTensorInverse, &A->bodyRef->rotation);
  mVec3TransformNormal(&A->bodyRef->angularVelocity, &A->bodyRef->angularMomentum, &A->bodyRef->inertiaTensorInverse);

  mMatrixTranspose(&rotationInverse, &B->bodyRef->rotation);

  mMatrixMultiply(&B->bodyRef->inertiaTensorInverse, &rotationInverse, &B->bodyRef->bodyInertiaTensorInverse);
  mMatrixMultiply(&B->bodyRef->inertiaTensorInverse, &B->bodyRef->inertiaTensorInverse, &B->bodyRef->rotation);
  mVec3TransformNormal(&B->bodyRef->angularVelocity, &B->bodyRef->angularMomentum, &B->bodyRef->inertiaTensorInverse);

}

void RResponse::Response() {

  BuildMatrix(A);

  if (velocityPass) {

    dantzigSolver.Solve(pointCount, cCollection->constraintCount, A, V, F);
    ApplyImpulsesVector();

  }

  memset(V, 0, sizeof(double) * pointCount);
  memset(F, 0, sizeof(double) * pointCount);

  BuildConstantVector(V);

  dantzigSolver.Solve(pointCount, cCollection->constraintCount, A, V, F);

  ApplyForcesVector();

}


