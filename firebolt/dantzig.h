#ifndef DANTZIG_H
#define DANTZIG_H

#include "matrix.h"

#include <set>

class DantzigSolver {

public:

  void Solve(unsigned int, unsigned int, Matrix*, double*, double*);

private:

  bool driveToZero(unsigned int);
  void fDirection(unsigned int);
  void maxStep(unsigned int);
  bool freezeDetect();

  int mCount;
  int size;

  Matrix *A;
  double *a; //constant vector
  double *f; //solution vector
  double *aDelta;
  double *fDelta;

  double ss;
  int jj;

  double *aBackUp;
  int stepsLog[4];


  std::set<int> C;
  std::set<int> NC;

};

extern DantzigSolver dantzigSolver;


#endif
