#ifndef RESPONSE_H
#define RESPONSE_H

#include "collision.h"
#include "matrix.h"


class RResponse {

public:

  RResponse(ContactCollection& cCollection);
  ~RResponse();

  void Response();

private:

  void BuildMatrix(Matrix *MA);
  void BuildConstantVector(double *V);

  void ApplyForcesVector();
  void ApplyImpulsesVector();


  ContactCollection *cCollection;
  std::set<CollisionAvatar*> CCBodies;

  std::vector<ContactSet>::iterator contactSetIt;
  std::vector<mVECTOR3>::iterator contactPointIt;

  unsigned int pointCount;

  bool velocityPass;
  bool impulse;

  double *F;
  double *V;

  double *relV;

  Matrix *A;

};


#endif
