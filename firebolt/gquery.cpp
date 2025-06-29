#include "gquery.h"


void sort(std::vector<mVECTOR2> &v) {

  mVECTOR2 temp;
  int index;


  mVECTOR2 ve1 = v[0];
  mVECTOR2 ve2;
  mVECTOR2 ve3;
  mVECTOR2 n;
  mVECTOR2 np;

  double a;

  for (unsigned int i = 1; i<v.size(); i++) {

    ve2 = v[i];
    n = ve2 - ve1;
    np.x = -n.y;
    np.y = n.x;
    bool good = true;

    a = -(np.x * ve1.x + np.y * ve1.y);

    for (unsigned int j = 1; j<v.size(); j++) {

      if (i != j) {

        if ((np.x * v[j].x + np.y * v[j].y + a) < 0.0f)  {

          good = false;
          break;
        }
      }

    }
    if (good) {

      temp = v[1];
      v[1] = v[i];
      v[i] = temp;


      break;
    }

  }


  double maxdot;

  for (unsigned int i = 2; i< v.size(); i++) {

    ve1 = v[i - 2];
    ve2 = v[i - 1];
    n = ve2 - ve1;
    mVec2Normalize(&n, &n);
    index = i;
    ve3 = v[i];
    np = ve3 - ve2;
    mVec2Normalize(&np, &np);
    maxdot = (n.x * np.x + n.y *np.y);

    for (unsigned int j = i + 1; j<v.size(); j++) {

      ve3 = v[j];
      np = ve3 - ve2;
      mVec2Normalize(&np, &np);
      if ((n.x * np.x + n.y *np.y) > maxdot) {
        index = j;
        maxdot = (n.x * np.x + n.y *np.y);
      }

    }

    if (index != i) {

      temp = v[i];
      v[i] = v[index];
      v[index] = temp;

    }

  }

}


D3DXVECTOR3 edge_edge(D3DXVECTOR3 B0, D3DXVECTOR3 M0, D3DXVECTOR3 B1, D3DXVECTOR3 M1) {
//WARNING THIS MIGHT BE buggy/untested CASE 0 WORKS THOUGH


  D3DXVECTOR3 result;
  D3DXVECTOR3 temp;
  float a,b,c,d,e,f;

  a = D3DXVec3Dot(&M0, &M0);
  b = -D3DXVec3Dot(&M0, &M1);
  c = D3DXVec3Dot(&M1, &M1);
  temp = B0 - B1;
  d = D3DXVec3Dot(&M0, &temp);
  e = -D3DXVec3Dot(&M1, &temp);
  f = D3DXVec3Dot(&temp, &temp);


  float det,s,t;
  int region;
  float invDet;
  float tmp;

  det = a*c - b*b;
  s = b*e - c*d;
  t= b*d - a*e;


  if(s >= 0.0f) {

    if (s <= det) {

      if( t>= 0.0f) {

        if( t<= det) region =0;
        else region =3;

      }
      else region = 7;


    }
    else {

      if( t>= 0.0f) {

        if( t<= det) region =1;
        else region =2;

      }
      else region = 8;


    }

  }
  else {

      if( t>= 0.0f) {

        if( t<= det) region =5;
        else region =4;

      }
      else region = 6;


  }


  switch (region) {

    case 0:

      invDet = 1/det;
      s*= invDet;
      t*= invDet;

      break;

    case 1:

      s = 1;
      tmp = b+e;
      if ( tmp > 0 )
        t = 0;
      else if ( -tmp > c )
        t = 1;
      else
        t = -tmp/c;

      break;

    case 3:

      t = 1;
      tmp = b+d;
      if ( tmp > 0 )
        s = 0;
      else if ( -tmp > a )
        s = 1;
      else
        s = -tmp/a;

      break;


    case 5:

      s = 0;
      tmp = e;
      if ( tmp > 0 )
        t = 0;
      else if ( -tmp > c )
        t = 1;
      else
        t = -tmp/c;

      break;

    case 7:

      t = 1;
      tmp = d;
      if ( tmp > 0 )
        s = 0;
      else if ( -tmp > a )
        s = 1;
      else
        s = -tmp/a;

      break;


    case 2:

      tmp = b+d;
      if ( -tmp < a ) // Q_s(1,1) > 0
      {
        t = 1;
        if ( tmp > 0 ) // S < 0, so minimum at s = 0
          s = 0;
        else // 0 <= S < 1, so minimum at s = S
          s = -tmp/a;
      }
      else // Q_s(1,1) <= 0
      {
      s = 1;
      tmp = b+e;
        if ( -tmp < c ) // Q_t(1,1) > 0
        {
          if ( tmp > 0 ) // T < 0, so minimum at t = 0
            t = 0;
          else // 0 <= T < 1, so minimum at t = T
            t = -tmp/c;
        }
        else // Q_t(1,1) <= 0, gradient points to region 2, so minimum at t = 1
          t = 1;
      }

      break;


    case 4:

      tmp = b+d;
      if ( tmp < 0 )
      {
        t = 1;
        if ( -tmp > a )
          s = 1;
        else
          s = -tmp/a;
      }
      else
      {
        s = 0;
        tmp = e;
        if ( tmp > -c ) {

          if ( e > 0 )
            t = 0;
          else
            t = -tmp/c;
        }
        else
          t = 1;
      }

      break;



    case 6:

      tmp = d;
      if ( tmp < 0 )
      {
        t = 0;
         if ( -tmp > a )
          s = 1;
        else
          s = -tmp/a;

      }
      else
      {
        s = 0;
        tmp = e;
        if ( tmp < 0 ) {

          if ( -tmp > c )
            t = 1;
          else
            t = -tmp/c;
        }
        else
          t = 0;
      }


      break;


    case 8:

      tmp = a+d;
      if ( tmp > 0 )
      {
        t = 0;

        if ( d > 0 )
          s = 0;
        else
          s = -d/a;


      }
      else
      {
        s = 1;
        tmp = b+e;
        if ( tmp < 0 ) {

          if ( -tmp > c )
            t = 1;
          else
            t = -tmp/c;

        }
        else
          t = 0;
      }

      break;

  }

  return (B0 + s * M0 + B1 + t * M1 )/2;

}


mVECTOR2 edge_edge(mVECTOR2 s0, mVECTOR2 e0, mVECTOR2 s1, mVECTOR2 e1) {

  const double ER = 1e-1;

  mVECTOR2 R;

  double A0, B0, C0, A1, B1, C1;
  double det;

  A0 = -e0.y;
  B0 = e0.x;
  C0 = (A0*s0.x + B0*s0.y);

  A1 = -e1.y;
  B1 = e1.x;
  C1 = (A1*s1.x + B1*s1.y);


  det = A0*B1 - B0*A1;


  if( fabs(det) < ER) {

    R.x = -1;
    R.y = -1;
    return R;
  }


  double detX = C0*B1 - B0*C1;
  double detY = A0*C1 - C0*A1;

  double x = detX / det;
  double y = detY / det;

  mVECTOR2 v(x,y);
  mVECTOR2 v0, v1;

  v0 = v - s0;
  v1 = v - s1;

  R.x = (v0.x*e0.x + v0.y*e0.y) / (e0.x*e0.x + e0.y*e0.y);
  R.y = (v1.x*e1.x + v1.y*e1.y) / (e1.x*e1.x + e1.y*e1.y);

  //
  //if( (R.x >= -0.01) && (R.x <= 0.0f))
  //  R.x =0.0f;
  //if( (R.y >= -0.01) && (R.y <= 0.0f))
  //  R.y =0.0f;

  //if( (R.x >= 1.0) && (R.x <= 1.01f))
  //  R.x =1.0f;
  //if( (R.y >= 1.0) && (R.y <= 1.01f))
  //  R.y =1.0f;

  if (R.x >= 0.0f && R.x <= 1.0f && R.y >= 0.0f && R.y <= 1.0f )
    return R;
  else {

    R.x = -1;
    R.y = -1;

    return R;

  }

}


void edge_polygon(std::vector<mVECTOR2> &p, mVECTOR2 s, mVECTOR2 e, std::vector<mVECTOR2> &pR) {

  //std::vector<mVECTOR2> pR;

  mVECTOR2 res;

  for (unsigned int i = 0; i< p.size(); i++) {

    int ii = (i + 1) % p.size();
    res = edge_edge(p[i], p[ii] - p[i], s, e);
    if ((res.x != -1) && (res.y != -1)) {

      pR.push_back(p[i] + res.x * (p[ii] - p[i]));

    }

  }
  if (pR.size() == 1) {

    mVECTOR2 out, point;
    double y;

    for (unsigned int i = 0; i< p.size(); i++) {

      int ii = (i + 1) % p.size();

      out.x = p[ii].y - p[i].y;
      out.y = p[i].x - p[ii].x;
      point.x = p[i].x;
      point.y = p[i].y;
      y = mVec2Dot(&out, &point);


      if ((mVec2Dot(&out, &s) - y) >= 0.0f) {

        pR.push_back(s + e);
        return;

      }


    }
    pR.push_back(s);
    return;

  }

  if (pR.size() == 0) {

    pR.push_back(s);
    pR.push_back(s + e);
  }
  return;

}

void polygon_polygon(ConvexPolygon &P, ConvexPolygon &Q, std::vector<mVECTOR2> &R) {

  mVECTOR2 s, e;
  std::vector<mVECTOR2> points;

  bool whole(1);

  for (unsigned int i = 0; i < P.getVertexCount(); i++) {
    if (Q.insidePolygon(P.getVertex(i)))
      R.push_back(P.getVertex(i));
    else
      whole = 0;
  }

  if (whole)
    return;
  whole = 1;

  for (unsigned int i = 0; i < Q.getVertexCount(); i++) {
    if (P.insidePolygon(Q.getVertex(i)))
      R.push_back(Q.getVertex(i));
    else
      whole = 0;
  }

  if (whole)
    return;

  for (unsigned int i = 1; i < P.getVertexCount(); i++) {

    s = P.getVertex(i);
    e = P.getEdge(i);

    edge_polygon(Q.vertices2, s, e, points);

    R.insert(R.end(), points.begin(), points.end());
    points.clear();
  }

}
