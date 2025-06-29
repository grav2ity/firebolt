#include "dantzig.h"

#include <fstream>
#include <math.h>


#ifdef _DEBUG
#define printD(f ,s) ( f << s)
std::ofstream fff6("response.txt");
#else
#define printD(f ,s)
#endif


DantzigSolver dantzigSolver;

double EE = 1e-13;

void DantzigSolver::Solve(unsigned int sizeh, unsigned int constraintCount, Matrix *Ah, double *ah, double *fh) {


  memset(stepsLog,-1,sizeof(int)*4);
  aBackUp = new double[sizeh];
  memcpy(aBackUp, ah, sizeof(double)*sizeh);

  size = sizeh;
  A = Ah;
  a = ah;
  f = fh;

  fDelta = new double[size];
  aDelta = new double[size];

  C.clear();
  NC.clear();

  A->r_count = A->c_count = size;

  //HANDLE CONSTRAINTS FIRST
  Matrix *ConstraintMatrix = new Matrix(constraintCount,constraintCount);
  Matrix *ConstraintInitialImpulseVector = new Matrix(constraintCount,1);
  mCount = constraintCount;
  for (unsigned int i = 0; i<constraintCount; i++) {
    (*ConstraintInitialImpulseVector)(i,0) = -a[size-constraintCount+i];
  }

  for (unsigned int i = 0; i<constraintCount; i++) {
    for(unsigned int j=0; j<constraintCount; j++) {
      (*ConstraintMatrix)(i,j) = (*A)(size-constraintCount+i, size-constraintCount+j);
    }
  }

  GaussJordanPartialPivoting((*ConstraintMatrix), (*ConstraintInitialImpulseVector));
  for (unsigned int i = 0; i<constraintCount; i++) {
    f[size-constraintCount+i] = (*ConstraintInitialImpulseVector)(i,0);
    C.insert(size-constraintCount+i);
  }

  A->MultiplyVector(f, aDelta);

  for (int i = 0; i<size; i++) {
    a[i] += aDelta[i];
  }

  delete ConstraintMatrix;
  delete ConstraintInitialImpulseVector;
  bool complete(0);
  bool restart(0);


  while(!complete) {

    complete =1;
    for (int i = 0; i< size; i++) {
      if(a[i] < -1e-4 ) {

        if(driveToZero(i)) {
          complete =true;
          restart = true;
          break;
        }
        complete=0;
      }
    }
  }

  if(restart) {
    delete[] aBackUp;
    delete[] aDelta;
    delete[] fDelta;

    Solve(size, mCount, A, a, f);
    return;
  }

  delete[] aBackUp;
  delete[] aDelta;
  delete[] fDelta;

}

void DantzigSolver::maxStep(unsigned int d) {

  ss = 1e18;
  jj = -100000000;
  double sp;

  if (aDelta[d] >0.0) {
    jj = d;
    ss = - a[d] / aDelta[d];
   }

  std::set<int>::iterator it, it2;
  for(it = C.begin(); it!= C.end(); it++) {

    if((*it) >= (size - mCount)) continue;
    if(aDelta[(*it)] < 0.0)
      aDelta[(*it)] = 0.0;
    if(fDelta[(*it)] < 0.0) {
      sp = (- f[(*it)]) / fDelta[(*it)];
      if( (sp) < ss) {
        ss = sp;
        jj = (*it);
      }
    }
  }

  for(it2 = NC.begin(); it2!= NC.end(); it2++) {

    if(aDelta[(*it2)] < 0.0) {
      sp =  (- a[(*it2)]  )/aDelta[(*it2)];
      if((sp) < ss) {
        ss = sp;
        jj = (*it2);

      }
    }
  }

}


void DantzigSolver::fDirection(unsigned int d) {

  memset(fDelta, 0, sizeof(double) * size);
  fDelta[d] = 1.0;

  Matrix *AA = new Matrix(C.size(), C.size());
  Matrix *V = new Matrix(C.size(), 1);

  std::set<int>::iterator it;
  std::set<int>::iterator it2;
  unsigned int i,j;

  i=0;
  for( it=C.begin(); it!=C.end(); it++,i++) {
    (*V)(i, 0) = - (*A)(*it, d);
  }

  i=0;
  for( it=C.begin(); it!=C.end(); it++, i++) {
    j=0;
    for(it2=C.begin(); it2!=C.end(); it2++,j++) {
      (*AA)(i,j) = (*A)(*it, *it2);
    }
  }


  //SOLVE MA*x = - Mv
  GaussJordanPartialPivoting((*AA), (*V));

  i=0;
  for(it=C.begin(); it!=C.end(); it++, i++) {
    fDelta[*it] = (*V)(i,0);
  }
  delete V;
  delete AA;

}


bool DantzigSolver::driveToZero(unsigned int d) {

  while(true) {

    memset(fDelta,0,sizeof(double)*size);
    memset(aDelta,0,sizeof(double)*size);

    fDirection(d);

    for (int i = 0; i<size; i++) {
      if(fabs(fDelta[i]) < EE)
        fDelta[i] = 0.0;
      }

    A->MultiplyVector(fDelta, aDelta);

    for (int i = 0; i<size; i++) {
      if(fabs(aDelta[i]) < EE)
        aDelta[i] = 0.0;
      }

    std::set<int>::const_iterator it;
    std::set<int>::const_iterator it2;

#ifdef _DEBUG
    for(int i=0; i<size; i++) {

      it = C.find(i);
      it2 = NC.find(i);

      printD(fff6,a[i] << " () "  << aDelta[i] << " () " << f[i] << " () " << fDelta[i] << " () ");

      if(it != C.end() )
        printD(fff6,"C");
      else if(it2 != NC.end() )
        printD(fff6,"NC");
      else
        printD(fff6,"(((");

      printD(fff6,std::endl);

    }
    printD(fff6,std::endl);
#endif //_DEBUG

    //MAXSTEP
    maxStep(d);
    if(freezeDetect())
      return true;

    printD(fff6,"SS" << ss << std::endl);
    printD(fff6,"JJ" << jj << std::endl);

    for(int i=0; i<size; i++) {

      f[i] += ss* fDelta[i];
      a[i] += ss * aDelta[i];

      if(fabs(a[i]) < EE)
        a[i] = 0.0;

      if(fabs(f[i]) < EE)
        f[i] = 0.0;

      printD(fff6, a[i] << " () "  << aDelta[i] << " () " << f[i] << " () " << fDelta[i] << " () ");
      printD(fff6,std::endl);

    }
    printD(fff6,std::endl);

    it = C.find(jj);
    it2 = NC.find(jj);

    if(it != C.end() ) {

      f[(*it)] = 0.0;
      C.erase(it);
      NC.insert(jj);
    }
    else if (it2 != NC.end()) {

      a[(*it2)] = 0.0;
      NC.erase(it2);
      C.insert(jj);
    }
    else {
      C.insert(jj);
      a[d] =0.0;
      return false;
    }

  }

}



bool DantzigSolver::freezeDetect() {

  stepsLog[3] = stepsLog[2];
  stepsLog[2] = stepsLog[1];
  stepsLog[1] = stepsLog[0];

  if(ss == 0.0) {

    stepsLog[0] = jj;
    if( (stepsLog[1] != -1) && (stepsLog[0] == stepsLog[1]) ) {

      for (int i=0; i<size; i++)
        //aBackUp[i] *=1.11;
        aBackUp[i] *=0.01;
      memcpy(a, aBackUp, sizeof(double)*size);
      memset(f, 0, sizeof(double)*size);
      return true;
    }
    else if( (stepsLog[1] != -1) && (stepsLog[2] != -1) &&(stepsLog[0] == stepsLog[2]) && (stepsLog[1] == stepsLog[3]) ) {

      for(int i=0; i<size; i++)
        //aBackUp[i] *=1.11;
        aBackUp[i] *= 0.01;
      memcpy(a, aBackUp, sizeof(double)*size);
      memset(f, 0, sizeof(double)*size);
      return true;
    }
    else
      return false;
  }

  else {
    stepsLog[0] = -1;
    return false;
  }


}
