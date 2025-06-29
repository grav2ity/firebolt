#include "ode.h"


void EulerIntegrate(double *vector, unsigned int count, double timeDelta, void(*derivativeFunction)(double*, double*, double)) {

  double *derivative1 = new double[count];

  derivativeFunction(vector, derivative1, timeDelta);

  for (unsigned int i = 0; i<count; i++)
    vector[i] += derivative1[i] * timeDelta;

  delete[] derivative1;

}

void MidPointIntegrate(double *vector, unsigned int count, double timeDelta, void(*derivativeFunction)(double*, double*, double)) {

  double *derivative1 = new double[count];
  double *V = new double[count];

  derivativeFunction(vector, derivative1, timeDelta);

  for (unsigned int i = 0; i<count; i++)
    V[i] = vector[i] + 0.5 * derivative1[i] * timeDelta;

  derivativeFunction(V, derivative1, timeDelta);

  for (unsigned int i = 0; i<count; i++)
    vector[i] += timeDelta *derivative1[i];

  delete[] derivative1;
  delete[] V;

}
