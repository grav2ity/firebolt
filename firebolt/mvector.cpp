#include "mvector.h"


mVECTOR2::mVECTOR2() {}
mVECTOR2::mVECTOR2( const double *d ) : x(d[0]), y(d[1]) {}
mVECTOR2::mVECTOR2( double xx, double yy ) : x(xx), y(yy) {}

mVECTOR2::operator double* () { return reinterpret_cast<double*>(this); }
mVECTOR2::operator const double* () const { return reinterpret_cast<const double*>(this); }


mVECTOR2&  mVECTOR2::operator += ( const mVECTOR2& r ) { x += r.x; y+= r.y; return *this; }
mVECTOR2& mVECTOR2::operator -= ( const mVECTOR2& r) { x -= r.x; y-= r.y; return *this; }
mVECTOR2& mVECTOR2::operator *= ( double r ) { x *= r; y*= r; return *this; }
mVECTOR2& mVECTOR2::operator /= ( double r ) { x /= r; y/= r; return *this; }


mVECTOR2 mVECTOR2::operator + () const { return *this; }
mVECTOR2 mVECTOR2::operator - () const { return mVECTOR2(-x,-y); }


mVECTOR2 mVECTOR2::operator + ( const mVECTOR2& r) const { return mVECTOR2(x+r.x, y+r.y); }
mVECTOR2 mVECTOR2::operator - ( const mVECTOR2& r) const { return mVECTOR2(x-r.x, y-r.y); }
mVECTOR2 mVECTOR2::operator * ( double r) const { return mVECTOR2(x*r, y*r); }
mVECTOR2 mVECTOR2::operator / ( double r) const { return mVECTOR2(x/r, y/r); }

mVECTOR2 operator * ( double l, const mVECTOR2& r) { return r*l; }

bool mVECTOR2::operator == ( const mVECTOR2& l) const {

  if( (x==l.x) && (y==l.y) )
    return true;
  else
    return false;

}

double mVec2Dot(const mVECTOR2* r, const mVECTOR2* l) {
  return r->x * l->x + r->y * l->y;
}

mVECTOR2* mVec2Normalize(mVECTOR2* d, const mVECTOR2* s) {

  double ln = sqrt(s->x*s->x + s->y*s->y);

  if(ln != 0.0)
    *d = (*s)/ln;
  else
    d->x = d->y = 0.0;

  return d;

}


mVECTOR3::mVECTOR3() {}
mVECTOR3::mVECTOR3(D3DXVECTOR3 d) :x(d.x), y(d.y), z(d.z) {}
mVECTOR3::mVECTOR3( const double *d ) : x(d[0]), y(d[1]), z(d[2]) {}
mVECTOR3::mVECTOR3( double xx, double yy, double zz ) : x(xx), y(yy), z(zz) {}

mVECTOR3::operator double* () { return reinterpret_cast<double*>(this); }
mVECTOR3::operator const double* () const { return reinterpret_cast<const double*>(this); }


D3DXVECTOR3 mVECTOR3::toDxVector() { return D3DXVECTOR3((float)x, (float)y, (float)z); }

mVECTOR3&  mVECTOR3::operator += ( const mVECTOR3& r ) { x += r.x; y+= r.y; z+= r.z; return *this; }
mVECTOR3& mVECTOR3::operator -= ( const mVECTOR3& r) { x -= r.x; y-= r.y; z-= r.z; return *this; }
mVECTOR3& mVECTOR3::operator *= ( double r ) { x *= r; y*= r; z*= r; return *this; }
mVECTOR3& mVECTOR3::operator /= ( double r ) { x /= r; y/= r; z/= r; return *this; }


mVECTOR3 mVECTOR3::operator + () const { return *this; }
mVECTOR3 mVECTOR3::operator - () const { return mVECTOR3(-x, -y, -z); }


mVECTOR3 mVECTOR3::operator + ( const mVECTOR3& r) const { return mVECTOR3(x+r.x, y+r.y, z+r.z); }
mVECTOR3 mVECTOR3::operator - ( const mVECTOR3& r) const { return mVECTOR3(x-r.x, y-r.y, z-r.z); }
mVECTOR3 mVECTOR3::operator * ( double r) const { return mVECTOR3(x*r, y*r, z*r); }
mVECTOR3 mVECTOR3::operator / ( double r) const { return mVECTOR3(x/r, y/r, z/r); }

mVECTOR3 operator * ( double l, const mVECTOR3& r) { return r*l; }

bool mVECTOR3::operator == ( const mVECTOR3& l) const {

  if( (x==l.x) && (y==l.y) && (z==l.z) )
    return true;
  else
    return false;


}

double mVec3Dot(const mVECTOR3* r, const mVECTOR3* l) {
  return r->x * l->x + r->y * l->y + r->z * l->z;
}

mVECTOR3* mVec3Normalize(mVECTOR3* d, const mVECTOR3* s) {

  double ln = sqrt(s->x*s->x + s->y*s->y + s->z*s->z);
  if(ln != 0.0)
    *d = (*s)/ln;
  else
    d->x = d->y = d->z = 0.0;

  return d;

}
mVECTOR3* mVec3Cross(mVECTOR3* o, const mVECTOR3* s1, const mVECTOR3* s2) {

  mVECTOR3 temp;
  temp.x = s1->y*s2->z - s1->z*s2->y;
  temp.y = s1->z*s2->x - s1->x*s2->z;
  temp.z = s1->x*s2->y - s1->y*s2->x;
  *o = temp;
  return o;

}


mVECTOR4::mVECTOR4() {}
mVECTOR4::mVECTOR4( const double *d ) : x(d[0]), y(d[1]), z(d[2]), w(d[3]) {}
mVECTOR4::mVECTOR4( double xx, double yy, double zz, double ww ) : x(xx), y(yy), z(zz), w(ww) {}

mVECTOR4::operator double* () { return reinterpret_cast<double*>(this); }
mVECTOR4::operator const double* () const { return reinterpret_cast<const double*>(this); }


mVECTOR4&  mVECTOR4::operator += ( const mVECTOR4& r ) { x += r.x; y+= r.y; z+= r.z; w+=r.w; return *this; }
mVECTOR4& mVECTOR4::operator -= ( const mVECTOR4& r) { x -= r.x; y-= r.y; z-= r.z; w-=r.w; return *this; }
mVECTOR4& mVECTOR4::operator *= ( double r ) { x *= r; y*= r; z*= r; w*= r; return *this; }
mVECTOR4& mVECTOR4::operator /= ( double r ) { x /= r; y/= r; z/= r; w/= r; return *this; }


mVECTOR4 mVECTOR4::operator + () const { return *this; }
mVECTOR4 mVECTOR4::operator - () const { return mVECTOR4(-x,-y,-z,-w); }


mVECTOR4 mVECTOR4::operator + ( const mVECTOR4& r) const { return mVECTOR4(x+r.x, y+r.y, z+r.z, w+r.w); }
mVECTOR4 mVECTOR4::operator - ( const mVECTOR4& r) const { return mVECTOR4(x-r.x, y-r.y, z-r.z, w-r.w); }
mVECTOR4 mVECTOR4::operator * ( double r) const { return mVECTOR4(x*r, y*r, z*r, w*r); }
mVECTOR4 mVECTOR4::operator / ( double r) const { return mVECTOR4(x/r, y/r, z/r, w/r); }

mVECTOR4 operator * ( double l, const mVECTOR4& r) { return r*l; }

bool mVECTOR4::operator == ( const mVECTOR4& l) const {

  if( (x==l.x) && (y==l.y) && (z==l.z) && (w==l.w) )
    return true;
  else
    return false;

}


mMATRIX::mMATRIX() {};
mMATRIX::mMATRIX( const double* d) { memcpy(this,d,sizeof(mMATRIX)); }
mMATRIX::mMATRIX( const mMATRIX& d) { memcpy(this,&d,sizeof(mMATRIX)); }


mMATRIX::mMATRIX( const D3DXMATRIX& d) {

  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      m[i][j] = d(i,j);

}

mMATRIX::mMATRIX( double _11, double _12, double _13, double _14,
            double _21, double _22, double _23, double _24,
            double _31, double _32, double _33, double _34,
            double _41, double _42, double _43, double _44 ) {

  m[0][0] = _11;
  m[0][1] = _12;
  m[0][2] = _13;
  m[0][3] = _14;

  m[1][0] = _21;
  m[1][1] = _22;
  m[1][2] = _23;
  m[1][3] = _24;

  m[2][0] = _31;
  m[2][1] = _32;
  m[2][2] = _33;
  m[2][3] = _34;

  m[2][0] = _41;
  m[2][1] = _42;
  m[2][2] = _43;
  m[2][3] = _44;

}


double& mMATRIX::operator () ( unsigned int Row, unsigned int Col ) { return m[Row][Col]; }
double mMATRIX::operator () ( unsigned int Row, unsigned int Col ) const { return m[Row][Col]; }


mMATRIX::operator double* () { return reinterpret_cast<double*>(this); }
mMATRIX::operator const double* () const { return reinterpret_cast<const double*>(this); }

mMATRIX::operator D3DXMATRIX () {

  D3DXMATRIX result;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = (float)m[i][j];
  return result;

}

mMATRIX& mMATRIX::operator *= ( const mMATRIX& r) {

  *this = (*this) * r;
  return *this;

}

mMATRIX& mMATRIX::operator += ( const mMATRIX& r) {

  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      m[i][j] += r(i,j);

  return *this;

}
mMATRIX& mMATRIX::operator -= ( const mMATRIX& r) {

  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      m[i][j] -= r(i,j);

  return *this;

}
mMATRIX& mMATRIX::operator *= ( const double r) {

  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      m[i][j] *= r;

  return *this;

}
mMATRIX& mMATRIX::operator /= ( const double r) {

  if(r!=0.0) {

  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      m[i][j] /= r;

  }
  return *this;

}

mMATRIX mMATRIX::operator + () const { return *this; }
mMATRIX mMATRIX::operator - () const {

  mMATRIX result;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = -m[i][j];
  return result;

}

mMATRIX mMATRIX::operator * ( const mMATRIX& r ) const {

  mMATRIX result;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = m[i][0]*r(0,j) + m[i][1]*r(1,j) + m[i][2]*r(2,j) + m[i][3]*r(3,j);
  return result;

}

mMATRIX mMATRIX::operator + ( const mMATRIX& r) const {

  mMATRIX result;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = m[i][j]+r(i,j);
  return result;

}

mMATRIX mMATRIX::operator - ( const mMATRIX& r) const {

  mMATRIX result;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = m[i][j]-r(i,j);
  return result;

}

mMATRIX mMATRIX::operator * ( double r) const {

  mMATRIX result;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = m[i][j] * r;
  return result;

}

mMATRIX mMATRIX::operator / ( double r) const {

  if(r!=0.0) {

  mMATRIX result;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = m[i][j] / r;

  return result;

  }
  else
    return *this;

}

mMATRIX operator * ( double l, const mMATRIX& r) {

  return r*l;

}


mMATRIX* mMatrixIdentity(mMATRIX* s) {

  memset(s,0,sizeof(mMATRIX));
  (*s)(0,0) = 1.0;
  (*s)(1,1) = 1.0;
  (*s)(2,2) = 1.0;
  (*s)(3,3) = 1.0;

  return s;

}

mMATRIX* mMatrixInverse(mMATRIX* o, float* det, const mMATRIX* s) {

  D3DXMATRIX temp;
  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      temp(i,j) = (float)((*s)(i,j));

  D3DXMatrixInverse(&temp,det,&temp);

  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      (*o)(i,j) = (temp)(i,j);

  return o;

}

mMATRIX* mMatrixSmallInverse(mMATRIX* o, float* det, const mMATRIX* s) {

  mMatrixSmallTranspose(o,s);
  return o;

}

mMATRIX* mMatrixMultiply(mMATRIX* o, const mMATRIX* s1, const mMATRIX* s2) {

  *o = (*s1) * (*s2);
  return o;
}

mVECTOR3* mVec3TransformCoord(mVECTOR3* o, const mVECTOR3* s1, const mMATRIX* s2) {

  mVECTOR4 temp(s1->x, s1->y, s1->z, 1.0);
  mVec4Transform(&temp, &temp, s2);
  temp /= temp.w;
  o->x = temp.x;
  o->y = temp.y;
  o->z = temp.z;
  return o;

}
mVECTOR3* mVec3TransformNormal(mVECTOR3* o, const mVECTOR3* s1, const mMATRIX* s2) {

  mVECTOR3 temp;
  temp.x = s1->x * (*s2)(0,0) + s1->y *  (*s2)(1,0) + s1->z * (*s2)(2,0);
  temp.y = s1->x * (*s2)(0,1) + s1->y *  (*s2)(1,1) + s1->z * (*s2)(2,1);
  temp.z = s1->x * (*s2)(0,2) + s1->y *  (*s2)(1,2) + s1->z * (*s2)(2,2);
  *o = temp;
  return o;

}

mVECTOR4* mVec4Transform(mVECTOR4* o, const mVECTOR4* s1, const mMATRIX* s2) {

  mVECTOR4 temp;
  temp.x = s1->x * (*s2)(0,0) + s1->y *  (*s2)(1,0) + s1->z * (*s2)(2,0) + s1->w * (*s2)(3,0);
  temp.y = s1->x * (*s2)(0,1) + s1->y *  (*s2)(1,1) + s1->z * (*s2)(2,1) + s1->w * (*s2)(3,1);
  temp.z = s1->x * (*s2)(0,2) + s1->y *  (*s2)(1,2) + s1->z * (*s2)(2,2) + s1->w * (*s2)(3,2);
  temp.w = s1->x * (*s2)(0,3) + s1->y *  (*s2)(1,3) + s1->z * (*s2)(2,3) + s1->w * (*s2)(3,3);
  *o = temp;
  return o;

}

mMATRIX* mMatrixTranslation(mMATRIX* s, double x, double y, double z) {

  memset(s,0,sizeof(mMATRIX));

  (*s)(0,0) = 1.0;
  (*s)(1,1) = 1.0;
  (*s)(2,2) = 1.0;

  (*s)(3,0) = x;
  (*s)(3,1) = y;
  (*s)(3,2) = z;
  (*s)(3,3) = 1.0;

  return s;

}

mMATRIX* mMatrixTranspose(mMATRIX* d, const mMATRIX* s) {

  mMATRIX result;

  for(unsigned int i=0; i<4; i++)
    for(unsigned int j=0; j<4; j++)
      result(i,j) = (*s)(j,i);

  *d = result;
  return d;

}

mMATRIX* mMatrixSmallTranspose(mMATRIX* d, const mMATRIX* s) {

  mMATRIX result;

  for(unsigned int i=0; i<3; i++)
    for(unsigned int j=0; j<3; j++)
      result(i,j) = (*s)(j,i);

  *d = result;
  return d;

}
