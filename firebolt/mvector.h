#ifndef MVECTOR_H
#define MVECTOR_H

#include <d3dx9.h>



class mVECTOR2 {

public:

  double x, y;

  mVECTOR2();
  mVECTOR2(const double*);
  mVECTOR2(double x, double y);

  operator double* ();
  operator const double* () const;

  mVECTOR2& operator += (const mVECTOR2&);
  mVECTOR2& operator -= (const mVECTOR2&);
  mVECTOR2& operator *= (double);
  mVECTOR2& operator /= (double);

  mVECTOR2 operator + () const;
  mVECTOR2 operator - () const;

  mVECTOR2 operator + (const mVECTOR2&) const;
  mVECTOR2 operator - (const mVECTOR2&) const;
  mVECTOR2 operator * (double) const;
  mVECTOR2 operator / (double) const;

  friend mVECTOR2 operator * (double, const mVECTOR2&);

  bool operator == (const mVECTOR2&) const;
  //bool operator != (const mVECTOR2&) const;

};

double mVec2Dot(const mVECTOR2*, const mVECTOR2*);
mVECTOR2* mVec2Normalize(mVECTOR2*, const mVECTOR2*);


class mVECTOR3 {

public:

  double x, y, z;

  mVECTOR3();
  mVECTOR3(D3DXVECTOR3);
  mVECTOR3(const double*);
  mVECTOR3(double x, double y, double z);

  operator double* ();
  operator const double* () const;

  //operator D3DXVECTOR3 ();
  D3DXVECTOR3 toDxVector();

  mVECTOR3& operator += (const mVECTOR3&);
  mVECTOR3& operator -= (const mVECTOR3&);
  mVECTOR3& operator *= (double);
  mVECTOR3& operator /= (double);

  mVECTOR3 operator + () const;
  mVECTOR3 operator - () const;

  mVECTOR3 operator + (const mVECTOR3&) const;
  mVECTOR3 operator - (const mVECTOR3&) const;
  mVECTOR3 operator * (double) const;
  mVECTOR3 operator / (double) const;

  friend mVECTOR3 operator * (double, const mVECTOR3&);

  bool operator == (const mVECTOR3&) const;
  //bool operator != (const mVECTOR3&) const;

};


double mVec3Dot(const mVECTOR3*, const mVECTOR3*);
mVECTOR3* mVec3Normalize(mVECTOR3*, const mVECTOR3*);
mVECTOR3* mVec3Cross(mVECTOR3*, const mVECTOR3*, const mVECTOR3*);


class mVECTOR4 {

public:

  double x, y, z, w;

  mVECTOR4();
  mVECTOR4(const double*);
  mVECTOR4(double x, double y, double z, double w);

  operator double* ();
  operator const double* () const;

  mVECTOR4& operator += (const mVECTOR4&);
  mVECTOR4& operator -= (const mVECTOR4&);
  mVECTOR4& operator *= (double);
  mVECTOR4& operator /= (double);

  mVECTOR4 operator + () const;
  mVECTOR4 operator - () const;

  mVECTOR4 operator + (const mVECTOR4&) const;
  mVECTOR4 operator - (const mVECTOR4&) const;
  mVECTOR4 operator * (double) const;
  mVECTOR4 operator / (double) const;

  friend mVECTOR4 operator * (double, const mVECTOR4&);

  bool operator == (const mVECTOR4&) const;
  //bool operator != (const mVECTOR4&) const;

};


class mMATRIX {

public:

  double m[4][4];

  mMATRIX();
  mMATRIX(const double*);
  mMATRIX(const mMATRIX&);

  explicit mMATRIX(const D3DXMATRIX&);

  mMATRIX( double _11, double _12, double _13, double _14,
           double _21, double _22, double _23, double _24,
           double _31, double _32, double _33, double _34,
           double _41, double _42, double _43, double _44 );

  double& operator () (unsigned int Row, unsigned int Col);
  double operator () (unsigned int Row, unsigned int Col) const;

  operator double* ();
  operator const double* () const;

  operator D3DXMATRIX ();

  mMATRIX& operator *= (const mMATRIX&);
  mMATRIX& operator += (const mMATRIX&);
  mMATRIX& operator -= (const mMATRIX&);
  mMATRIX& operator *= (const double);
  mMATRIX& operator /= (const double);

  mMATRIX operator + () const;
  mMATRIX operator - () const;


  mMATRIX operator * (const mMATRIX&) const;
  mMATRIX operator + (const mMATRIX&) const;
  mMATRIX operator - (const mMATRIX&) const;
  mMATRIX operator * (double) const;
  mMATRIX operator / (double) const;

  friend mMATRIX operator * (double, const mMATRIX&);

  //bool operator == (const mMATRIX&) const;
  //bool operator != (const mMATRIX&) const;

};

mMATRIX* mMatrixIdentity(mMATRIX*);
mMATRIX* mMatrixInverse(mMATRIX*, float*, const mMATRIX*);
mMATRIX* mMatrixSmallInverse(mMATRIX*, float*, const mMATRIX*);
mMATRIX* mMatrixMultiply(mMATRIX*, const mMATRIX*, const mMATRIX*);
mVECTOR3* mVec3TransformCoord(mVECTOR3*, const mVECTOR3*, const mMATRIX*);
mVECTOR3* mVec3TransformNormal(mVECTOR3*, const mVECTOR3*, const mMATRIX*);
mVECTOR4* mVec4Transform(mVECTOR4*, const mVECTOR4*, const mMATRIX*);

mMATRIX* mMatrixTranslation(mMATRIX*, double, double, double);
mMATRIX* mMatrixTranspose(mMATRIX*, const mMATRIX*);
mMATRIX* mMatrixSmallTranspose(mMATRIX*, const mMATRIX*);


#endif
