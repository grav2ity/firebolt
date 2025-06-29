#include "matrix.h"

#include <cmath>


double& Matrix::operator()(long row, long col) {

  return *(data + ( row*col_count() + col  ));

}

Matrix::Matrix(unsigned int rc, unsigned int cc) : r_count(rc), c_count(cc) {

  data = new double[row_count()*col_count()];
  memset(data,0, sizeof(double)*row_count()*col_count());

}

void Matrix::MultiplyVector(double *in, double *out) {

  double r =0.0;
  for(int i=0; i<row_count(); i++) {

    for(int j=0; j<col_count(); j++)
      r += operator()(i,j) * in[j];

    out[i] =r;
    r = 0.0;

  }

}

void GaussJordanPartialPivoting2(Matrix& M, Matrix& R) {

  //for row swap bookeeping
  int *rowMask = new int[M.row_count()];

  for(int i=0; i<M.row_count(); i++)
    rowMask[i] =i;


  double *perRowMaxValReciprocals = new double[M.row_count()];

  for(int i=0; i<M.row_count(); i++) {
    double maxRowVal = M(i,0);

    for(int j=1; j<M.col_count(); j++) {

      if(fabs(M(i,j)) > fabs(maxRowVal))
        maxRowVal = M(i,j);

    }

    perRowMaxValReciprocals[i] = 1.0f / maxRowVal;

  }


  //double maxRowVal;
  double maxRowColumnVal;
  int nextRow;

  for(int c=0; c<M.col_count(); c++) {

    //FIND NEXT ROW
    nextRow = c;
    maxRowColumnVal = M(rowMask[c],c) * perRowMaxValReciprocals[rowMask[c]];

    for(int r=c; r<M.row_count(); r++) {

      if(fabs(M(rowMask[r],c)  * perRowMaxValReciprocals[rowMask[r]]) > fabs(maxRowColumnVal) ) {

        maxRowColumnVal = M(rowMask[r],c) * perRowMaxValReciprocals[rowMask[r]];
        nextRow = r;

      }

    }

    if(nextRow != c) {

      int temp  = rowMask[c];
      rowMask[c] = nextRow;
      rowMask[nextRow] = temp;

    }

    double pivotReciprocal;
    double rat;

    pivotReciprocal = 1.0f / M(rowMask[c], c);

    for(int cc =c; cc < M.col_count(); cc++) {

      M(rowMask[c],cc) *= pivotReciprocal;

    }
    R(rowMask[c],0) *= pivotReciprocal;

    for(int r=0; r < M.row_count(); r++) {

      if(rowMask[r]!=rowMask[c]) {
        rat = M(rowMask[r],c); ///M(rowMask[c],c);
        M(rowMask[r],c) = 0.0f;
        for(int cc =c+1; cc < M.col_count(); cc++)
          M(rowMask[r],cc) = M(rowMask[r],cc) -   rat * M(rowMask[c],cc);

        R(rowMask[r],0) = R(rowMask[r],0) -   rat * R(rowMask[c],0);

      }

    }

  }

  double temp;
  for(int i=0; i< M.row_count(); i++) {

    if (rowMask[i] != i) {

      temp = R(rowMask[i], 0) ;
      R(rowMask[i], 0) = R(i, 0);
      R(i,0) = temp;
      rowMask[rowMask[i]] = rowMask[i];

    }

  }

  delete []rowMask;

}



void GaussJordanPartialPivoting0(Matrix& M, Matrix& R) {

  //for row swap bookeeping
  int *rowMask = new int[M.row_count()];

  for(int i=0; i<M.row_count(); i++)
    rowMask[i] =i;

  double maxRowVal;
  double maxRowColumnVal;
  int nextRow;

  for(int c=0; c<M.col_count(); c++) {

    //FIND NEXT ROW
    nextRow = c;
    maxRowColumnVal =0.0f;

    for(int r=c; r<M.row_count(); r++) {

      maxRowVal = 0.0f;
      for(int cc=c; cc<M.col_count(); cc++)
        if(fabs(M(rowMask[r],cc)) > maxRowVal)
          maxRowVal = fabs(M(rowMask[r] ,cc));

      if(fabs(M(rowMask[r],c)) / maxRowVal > maxRowColumnVal) {

        maxRowColumnVal = fabs(M(rowMask[r],c)) / maxRowVal;
        nextRow = r;

      }

    }

    if(nextRow != c) {

      int temp  = rowMask[c];
      rowMask[c] = nextRow;
      rowMask[nextRow] = temp;

    }

    double pivotReciprocal;
    double rat;


    pivotReciprocal = 1.0f / M(rowMask[c], c);

    for(int cc =c; cc < M.col_count(); cc++)
      M(rowMask[c],cc) *= pivotReciprocal;

    R(rowMask[c],0) *= pivotReciprocal;

    for(int r=0; r < M.row_count(); r++) {

      if(rowMask[r]!=rowMask[c])  {
         rat = M(rowMask[r],c); ///M(rowMask[c],c);
         M(rowMask[r],c) = 0.0f;
        for(int cc =c+1; cc < M.col_count(); cc++)
          M(rowMask[r],cc) = M(rowMask[r],cc) -   rat * M(rowMask[c],cc);

        R(rowMask[r],0) = R(rowMask[r],0) -   rat * R(rowMask[c],0);

      }

    }

  }

  double temp;
  for(int i=0; i< M.row_count(); i++) {

    if (rowMask[i] != i) {

      temp = R(rowMask[i], 0) ;
      R(rowMask[i], 0) = R(i, 0);
      R(i,0) = temp;

      rowMask[rowMask[i]] = rowMask[i];

    }

  }

  delete []rowMask;

}


void GaussJordanPartialPivoting(Matrix& M, Matrix& R) {

  int rowCount, colCount;
  rowCount = colCount = M.row_count();
  if (!rowCount)
    return;

  //for row swap bookeeping
  int *rowMask = new int[rowCount];


  for(int i=0; i<rowCount; i++) {
    rowMask[i] =i;

    //!!! test delete that doh
    // R(i,0) = i;
  }



  double maxRowVal;
  double maxRowColumnVal;
  int nextRow;

  for(int c=0; c<colCount; c++) {


    //FIND NEXT ROW
    nextRow = c;
    maxRowColumnVal =0.0;

    for(int r=c; r<rowCount; r++) {

      maxRowVal = 0.0;
      for(int cc=c; cc<colCount; cc++)
        if( fabs(M( rowMask[r],cc)) > maxRowVal)
          maxRowVal = fabs(M( rowMask[r] ,cc));

      if( fabs(M(rowMask[r],c)) / maxRowVal > maxRowColumnVal) {
        _ASSERT(maxRowVal != 0.0);
        maxRowColumnVal = fabs(M(rowMask[r],c)) / maxRowVal;
        nextRow = r;

      }

    }

    if(nextRow != c) {

      int temp  = rowMask[c];
      rowMask[c] = nextRow;
      rowMask[nextRow] = temp;

    }


    double rat;

    for(int r=c+1; r < rowCount; r++) {


      if(rowMask[r]!=rowMask[c])  {
      //  _ASSERT(M(rowMask[c], c) != 0.0);
        if(M(rowMask[c], c) == 0.0)
          M(rowMask[c], c) = 0.001;

         rat = M(rowMask[r],c) / M(rowMask[c], c); ///M(rowMask[c],c);
         M(rowMask[r],c) = 0.0f;

        for(int cc =c+1; cc < colCount; cc++)
          M(rowMask[r],cc) = M(rowMask[r],cc) -   rat * M(rowMask[c],cc);

        R(rowMask[r],0) = R(rowMask[r],0) -   rat * R(rowMask[c],0);

      }

    }


  }
    double t;
    if(M(rowMask[rowCount-1],rowCount-1) == 0.0)
      R(rowMask[rowCount-1],0) =0;
    else {
      _ASSERT(M(rowMask[rowCount-1],rowCount-1) != 0.0);
      R(rowMask[rowCount-1],0) = R(rowMask[rowCount-1],0) / M(rowMask[rowCount-1],rowCount-1);
    }
    for(int i=rowCount-2; i >=0; i--) {

      t= R(rowMask[i],0);
      for(int j=rowCount-1; j>i; j--) {

        t-= M(rowMask[i],j)*R(rowMask[j],0);

      }
      _ASSERT( M(rowMask[i],i) != 0.0);
      R(rowMask[i],0) = 1.0 / M(rowMask[i],i) * t;

  }

  double temp;
  for(int i=0; i< rowCount; i++) {

    if (rowMask[i] != i) {

      temp = R(rowMask[i], 0) ;
      R(rowMask[i], 0) = R(i, 0);
      R(i,0) = temp;

      rowMask[rowMask[i]] = rowMask[i];

    }

  }

  delete []rowMask;

}
