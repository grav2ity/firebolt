#ifndef ODE_H
#define ODE_H

void EulerIntegrate(double*, unsigned int, double, void(*)(double*, double*, double));
void MidPointIntegrate(double *vector, unsigned int count, double timeDelta, void(*)(double*, double*, double));

#endif
