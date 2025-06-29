#include "oob.h"

#include "gquery.h"

#include <set>


//assorted magic constants
const double prec = 0.05;
const double motest = 0.13;



//OBB////////////////////////////////////////////////////////


mVECTOR3 OBB::GetNormal(unsigned int i) { 
  switch (i) {

    case 0:
      return mVECTOR3(1.0,0.0,0.0);
    case 1:
      return mVECTOR3(0.0,1.0,0.0);
    case 2:
      return mVECTOR3(0.0,0.0,1.0);
    case 3:
      return mVECTOR3(-1.0,0.0,0.0);
    case 4:
      return mVECTOR3(0.0,-1.0,0.0);
    case 5:
      return mVECTOR3(0.0,0.0,-1.0);

    default:
      return mVECTOR3(0.0, 0.0, -1.0);
  }
}

mVECTOR3 OBB::GetVertex(unsigned int i) { 
    
  switch (i) {
    case 0:
      return mVECTOR3(e1,e2,e3);
    case 1:
      return mVECTOR3(-e1,e2,e3);
    case 2:
      return mVECTOR3(e1,e2,-e3);
    case 3:
      return mVECTOR3(-e1,e2,-e3);
    case 4:
      return mVECTOR3(e1,-e2,e3);
    case 5:
      return mVECTOR3(-e1,-e2,e3);
    case 6:
      return mVECTOR3(e1,-e2,-e3);
    case 7:
      return mVECTOR3(-e1,-e2,-e3);

    default:
      return mVECTOR3(-e1, -e2, -e3);
  }
}

void OBB::TransformNormal(mVECTOR3& v) {

  mVec3TransformNormal(&v, &v, &matrix);

}

void OBB::InverseTransformNormal(mVECTOR3& v) {

  mVec3TransformNormal(&v, &v, &matrixInverse);

}

void  OBB::TransformVertex(mVECTOR3& v) {

  mVec3TransformCoord(&v, &v, &matrix);

}

void OBB::InverseTransformVertex(mVECTOR3& v) {

  mVec3TransformCoord(&v, &v, &matrixInverse);

}

void OBB::BuildWorldMatrix() {

  mMATRIX trans, rot;

  mMatrixTranslation(&trans, Translation().x, Translation().y, Translation().z);
  //  D3DXMatrixRotationQuaternion(&rot, rotation);

  matrix = (*rotation) * trans;

  mMatrixSmallInverse(&matrixInverse, 0, &matrix);

}

void OBB::TimeAlign(double tDelta) {

  BuildWorldMatrix();
  CollisionAvatar::TimeAlign(tDelta);
}

void ComputeInterval(OBB& c, mVECTOR3& d, ProjInfo& pInfo) {

  double min, max, val;
  min = max = mVec3Dot(&d, &c.GetVertex(0));

  for (unsigned int i = 1; i < c.GetVertexCount(); i++) {

    val = mVec3Dot(&d, &c.GetVertex(i));
    if (val < min)
      min = val;
    else if (val > max)
      max = val;
  }

  pInfo.max = max;
  pInfo.min = min;

}

void FillExtremeVerticesSet(OBB& c, mVECTOR3& d, ProjInfo &pInfo) {

  double val, min, max;
  min = max = mVec3Dot(&d, &c.GetVertex(0));
  pInfo.minVertexIndices.clear();
  pInfo.maxVertexIndices.clear();
  pInfo.minVertexIndices.push_back(0);
  pInfo.maxVertexIndices.push_back(0);

  for (unsigned int i = 1; i< c.GetVertexCount(); i++) {

    val = mVec3Dot(&d, &c.GetVertex(i));
    if (fabs(val - min) < prec) {
      pInfo.minVertexIndices.push_back(i);
    }
    else if (fabs(val - min) > prec && (val < min)) {
      min = val;
      pInfo.minVertexIndices.clear();
      pInfo.minVertexIndices.push_back(i);
    }
    if (fabs(val - max) < prec) {
      pInfo.maxVertexIndices.push_back(i);
    }
    else if (fabs(val - max) > prec && (val > max)) {
      max = val;
      pInfo.maxVertexIndices.clear();
      pInfo.maxVertexIndices.push_back(i);
    }

  }


}


ContactInfo GetContactInfo(OBB& c0, OBB& c1) { 

  bool cross(false);


  mVECTOR3 d, d0, d1, v, v1;

  double minOverlapp(1000.0);
  double maxOverlapp(0.0);
  ContactInfo s;
  s.cA = &c0;
  s.cB = &c1;


  mVECTOR3 E1 = mVECTOR3(1.0,0.0,0.0);
  mVECTOR3 E2 = mVECTOR3(0.0,1.0,0.0);
  mVECTOR3 E3 = mVECTOR3(0.0,0.0,1.0);

  ProjInfo projInfo0, projInfo1, ble, ble2;
  ProjInfo projInfo0M, projInfo1M;

  //double max0,max1,min0,min1;

  mVECTOR3 c0translation(reinterpret_cast<mVECTOR3&>(c0.matrix.m[3][0])), c1translation(reinterpret_cast<mVECTOR3&>(c1.matrix.m[3][0]));
  double c0transDot, c1transDot;


  for(int i =0; i< 3; i++) {

    if(i==0) {
    d= d0= mVECTOR3(1.0,0.0,0.0);
        c0.TransformNormal(d0);
        c0transDot =  mVec3Dot(&d0, &c0translation);
        projInfo0.max = c0transDot + c0.e1;
        projInfo0.min = c0transDot - c0.e1;
    }
    if(i==1) {
    d= d0= mVECTOR3(0.0,1.0,0.0);
        c0.TransformNormal(d0);
        c0transDot =  mVec3Dot(&d0, &c0translation);
        projInfo0.max = c0transDot + c0.e2;
        projInfo0.min = c0transDot - c0.e2;

    }
    if(i==2) {
    d= d0= mVECTOR3(0.0,0.0,1.0);
        c0.TransformNormal(d0);
        c0transDot =  mVec3Dot(&d0, &c0translation);
        projInfo0.max = c0transDot + c0.e3;
        projInfo0.min = c0transDot - c0.e3;


    }



    d1= d0;
    c1transDot =  mVec3Dot(&d1, &c1translation);
    c1.InverseTransformNormal(d0);


    projInfo1.min = c1transDot - fabs(mVec3Dot(&d0,&(E1 *c1.e1))) - fabs(mVec3Dot(&d0,&(E2 *c1.e2))) - fabs(mVec3Dot(&d0,&(E3 *c1.e3)));
    projInfo1.max = c1transDot + fabs(mVec3Dot(&d0,&(E1 *c1.e1))) + fabs(mVec3Dot(&d0,&(E2 *c1.e2))) + fabs(mVec3Dot(&d0,&(E3 *c1.e3)));


    if(projInfo1.max < projInfo0.min || projInfo0.max <projInfo1.min ) {

      s.contact = false;
      s.volume = false;
      return s;
    }
    else if( projInfo1.max >= projInfo0.min && projInfo1.max <= projInfo0.max ) {
    
      if(minOverlapp > fabs(projInfo1.max - projInfo0.min) ) {
        minOverlapp = fabs(projInfo1.max - projInfo0.min);
        s.contactNormal = -d1;
        s.aIndex = projInfo0.minIndex;
        s.aCount = projInfo0.minCount;
        s.bIndex = projInfo1.maxIndex;
        s.bCount = projInfo1.maxCount;


        projInfo0M = projInfo0;
        projInfo1M = projInfo1;
        cross = false;
      }
    }
    else if( projInfo1.min <= projInfo0.max && projInfo1.min >= projInfo0.min) {
      if(minOverlapp > fabs(projInfo0.max - projInfo1.min) ) {
        minOverlapp = fabs(projInfo0.max - projInfo1.min);
        s.contactNormal = d1;
        s.aIndex = projInfo0.maxIndex;
        s.aCount = projInfo0.maxCount;
        s.bIndex = projInfo1.minIndex;
        s.bCount = projInfo1.minCount;


        projInfo0M = projInfo0;
        projInfo1M = projInfo1;
        cross = false;
      }
    }

  }

  for(int i =0; i< 3; i++) {

    if(i==0) {
    d= d0= mVECTOR3(1.0,0.0,0.0);
        c1.TransformNormal(d0);
        c1transDot =  mVec3Dot(&d0, &c1translation);
        projInfo1.max = c1transDot + c1.e1;
        projInfo1.min = c1transDot - c1.e1;
    }
    if(i==1) {
    d= d0= mVECTOR3(0.0,1.0,0.0);
        c1.TransformNormal(d0);
        c1transDot =  mVec3Dot(&d0, &c1translation);
        projInfo1.max = c1transDot + c1.e2;
        projInfo1.min = c1transDot - c1.e2;

    }
    if(i==2) {
    d= d0= mVECTOR3(0.0,0.0,1.0);
        c1.TransformNormal(d0);
        c1transDot =  mVec3Dot(&d0, &c1translation);
        projInfo1.max = c1transDot + c1.e3;
        projInfo1.min = c1transDot - c1.e3;


    }



    d1= d0;
    c0transDot =  mVec3Dot(&d1, &c0translation);
    c0.InverseTransformNormal(d0);


    projInfo0.min = c0transDot - fabs(mVec3Dot(&d0,&(E1 *c0.e1))) - fabs(mVec3Dot(&d0,&(E2 *c0.e2))) - fabs(mVec3Dot(&d0,&(E3 *c0.e3)));
    projInfo0.max = c0transDot + fabs(mVec3Dot(&d0,&(E1 *c0.e1))) + fabs(mVec3Dot(&d0,&(E2 *c0.e2))) + fabs(mVec3Dot(&d0,&(E3 *c0.e3)));



    if(projInfo1.max < projInfo0.min || projInfo0.max <projInfo1.min ) {
  
      s.contact = false;
      s.volume = false;
      return s;
    }
    else if( projInfo1.max >= projInfo0.min && projInfo1.max <= projInfo0.max ) {
    
      if(minOverlapp > fabs(projInfo1.max - projInfo0.min) ) {    
        minOverlapp = fabs(projInfo1.max - projInfo0.min);
        s.contactNormal = -d1;
        s.aIndex = projInfo1.minIndex;
        s.aCount = projInfo1.minCount;
        s.bIndex = projInfo0.maxIndex;
        s.bCount = projInfo0.maxCount;

        projInfo0M = projInfo0;
        projInfo1M = projInfo1;
        cross = false;
      }
    }
    else if( projInfo1.min <= projInfo0.max && projInfo1.min >= projInfo0.min) {
      if(minOverlapp > fabs(projInfo0.max - projInfo1.min) ) {  
        minOverlapp = fabs(projInfo0.max - projInfo1.min);
        s.contactNormal = d1;
        s.aIndex = projInfo1.maxIndex;
        s.aCount = projInfo1.maxCount;
        s.bIndex = projInfo0.minIndex;
        s.bCount = projInfo0.minCount;


        projInfo0M = projInfo0;
        projInfo1M = projInfo1;
        cross = false;
      }
    }

  }

  mVECTOR3 ddd;

  for(int i=0; i< 3; i++) {
  
    if(i==0)
    ddd= mVECTOR3(1.0,0.0,0.0);
    if(i==1)
    ddd = mVECTOR3(0.0,1.0,0.0);  
    if(i==2)
    ddd= mVECTOR3(0.0,0.0,1.0);
      c0.TransformNormal(ddd);

    for(int j=0; j< 3; j++) {
    
    if(j==0)
    d1=  mVECTOR3(1.0,0.0,0.0);
    if(j==1)
    d1 = mVECTOR3(0.0,1.0,0.0);  
    if(j==2)
    d1= mVECTOR3(0.0,0.0,1.0);

    
      c1.TransformNormal(d1);
  
      mVec3Cross(&d, &ddd, &d1);
      if((d.x * d.x + d.y * d.y + d.z * d.z) < 0.001f )
        continue;

      mVec3Normalize(&d, &d);
      d0 = d1 = d;

      c1transDot =  mVec3Dot(&d0, &c1translation);
      c0transDot =  mVec3Dot(&d0, &c0translation);

      c0.InverseTransformNormal(d);

    projInfo0.min = c0transDot - fabs(mVec3Dot(&d,&(E1 *c0.e1))) - fabs(mVec3Dot(&d,&(E2 *c0.e2))) - fabs(mVec3Dot(&d,&(E3 *c0.e3)));
    projInfo0.max = c0transDot + fabs(mVec3Dot(&d,&(E1 *c0.e1))) + fabs(mVec3Dot(&d,&(E2 *c0.e2))) + fabs(mVec3Dot(&d,&(E3 *c0.e3)));



      d = d0;
      c1.InverseTransformNormal(d);
    projInfo1.min = c1transDot - fabs(mVec3Dot(&d,&(E1 *c1.e1))) - fabs(mVec3Dot(&d,&(E2 *c1.e2))) - fabs(mVec3Dot(&d,&(E3 *c1.e3)));
    projInfo1.max = c1transDot + fabs(mVec3Dot(&d,&(E1 *c1.e1))) + fabs(mVec3Dot(&d,&(E2 *c1.e2))) + fabs(mVec3Dot(&d,&(E3 *c1.e3)));


      d1 = d;
  

      if(projInfo1.max < projInfo0.min || projInfo0.max<projInfo1.min ) {

        s.contact = false;
        s.volume = false;
        return s;
      }
      else if( projInfo1.max >= projInfo0.min && projInfo1.max <= projInfo0.max) {
    
        if(minOverlapp > fabs(projInfo1.max - projInfo0.min) ) {
          minOverlapp = fabs(projInfo1.max - projInfo0.min);
          s.contactNormal = -d0;
        s.aIndex = projInfo0.minIndex;
        s.aCount = projInfo0.minCount;
        s.bIndex = projInfo1.maxIndex;
        s.bCount = projInfo1.maxCount;

          projInfo0M = projInfo0;
          projInfo1M = projInfo1;
          cross = true;
        }
      }
      else if( projInfo1.min <= projInfo0.max&& projInfo1.min >= projInfo0.min) {

        if(minOverlapp > fabs(projInfo0.max- projInfo1.min) ) {
          minOverlapp = fabs(projInfo0.max- projInfo1.min);
          s.contactNormal = d0;
        s.aIndex = projInfo0.maxIndex;
        s.aCount = projInfo0.maxCount;
        s.bIndex = projInfo1.minIndex;
        s.bCount = projInfo1.minCount;

          projInfo0M = projInfo0;
          projInfo1M = projInfo1;
          cross = true;
        }
      }

    }

  }


    mVECTOR3 n = s.contactNormal;
    c0.InverseTransformNormal(n);
    FillExtremeVerticesSet(c0, n, projInfo0M);

    n = s.contactNormal;
    c1.InverseTransformNormal(n);
    FillExtremeVerticesSet(c1, n, projInfo1M);



  s.projInfo0 = projInfo0M;
  s.projInfo1 = projInfo1M;


  s.contact = true;
  s.overlapp = minOverlapp;

  if(minOverlapp <= motest) {

    s.volume =false;
  }
  else  {

    s.volume =true;
  }

  
  return s;

}






