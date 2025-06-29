#ifndef MATRIX_H
#define MATRIX_H

#include <memory.h>

#include <vector>


struct Matrix {


  double& operator()(long row, long col) ;
  Matrix(unsigned int rc, unsigned int cc);
  ~Matrix() { delete[] data;  }
  void MultiplyVector(double*, double*);
  unsigned int r_count;
  unsigned int c_count;

  double *data;


  int row_count() {  return r_count; }
  int col_count() {   return c_count; }
};

struct MatrixMask {

  double& operator()(int row, int col) { return m->operator()(rows[row], columns[col]); }

  Matrix *m;
  std::vector<int> columns;
  std::vector<int> rows;

};

void GaussJordanPartialPivoting(Matrix& M, Matrix& R);

#endif
