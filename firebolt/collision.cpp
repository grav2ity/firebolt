#include "collision.h"

#include "body.h"
#include "dmesh.h"
#include "response.h"
#include "gquery.h"

#include <set>


unsigned int maxIterations = 50;


CollisionAvatar::~CollisionAvatar() {}

void CollisionAvatar::Zero() {

  contactCollection = 0;
  contactPoints.clear();
  contactInfos.clear();

  rollback = false;
  ignoreVolume = false;

  inTwo = false;
  moveOne = false;
}

void CollisionAvatar::Rollback(double tm) {

  if (timeDelta > tm) {
    rollback = true;
    bodyManager->RollbackOne(bodyRef);
    bodyManager->IntegrateOne(tm, bodyRef);
    TimeAlign(tm);
  }
}

void CollisionAvatar::TimeAlign(double tDelta) {

  timeDelta = tDelta;
}

void CollisionManager::ComputeContactSet(ContactInfo &s, CollisionAvatar& c0, CollisionAvatar& c1, ContactSet& cSet) {

  std::vector<mVECTOR3> c0Vertices;
  std::vector<mVECTOR3> c1Vertices;

  std::vector<unsigned int> *polyhedra0ContactInfo = &s.projInfo0.maxVertexIndices;
  std::vector<unsigned int> *polyhedra1ContactInfo = &s.projInfo1.minVertexIndices;

  cSet.overlapp = s.overlapp;
  mVECTOR3 d = s.contactNormal;

  //WARNING
  //apparently I haven't even implemented tests for all the '-vertex-' cases

  if (polyhedra0ContactInfo->size() == 1) { //vertex-

    if (polyhedra1ContactInfo->size() > 2) { //vertex-polygon

      cSet.cA = &c0;
      cSet.cB = &c1;
      cSet.contactPoints.push_back(cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[0]));
      cSet.contactNormal = -d;

    }
    else if ((polyhedra1ContactInfo->size() == 2) || (polyhedra1ContactInfo->size() == 1)) { //vertex-edge || vertex-vertex

      cSet.cA = &c0;
      cSet.cB = &c1;
      cSet.contactPoints.push_back(cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[0]));
      cSet.contactNormal = -d;


    }

  }
  else if (polyhedra0ContactInfo->size() == 2) { //edge-

    if (polyhedra1ContactInfo->size() > 2) { //edge-polygon

      cSet.cA = &c0;
      cSet.cB = &c1;
      cSet.contactNormal = -d;
      std::vector<mVECTOR2> pAvertices;
      std::vector<mVECTOR2> pBvertices;

      double y;

      std::vector<mVECTOR2> result;

      mVECTOR3 b0, b1;
      mVECTOR3 temp;

      mVECTOR3 planeOrigin = cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[0]);

      b0 = cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[1]) - planeOrigin;
      mVec3Cross(&b1, &b0, &cSet.contactNormal);
      mVec3Normalize(&b0, &b0);
      mVec3Normalize(&b1, &b1);

      y = mVec3Dot(&cSet.contactNormal, &planeOrigin);


      for (int i = polyhedra1ContactInfo->size() - 1; i >= 0; i--) {
        pBvertices.push_back(mVECTOR2(mVec3Dot(&cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[i]), &b0),
          mVec3Dot(&cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[i]), &b1)));

      }

      for (unsigned int i = 0; i<polyhedra0ContactInfo->size(); i++) {
        pAvertices.push_back(mVECTOR2(mVec3Dot(&cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[i]), &b0),
          mVec3Dot(&cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[i]), &b1)));

      }

      sort(pBvertices);
      edge_polygon(pBvertices, pAvertices[0], pAvertices[1] - pAvertices[0], result);


      for (unsigned int i = 0; i<result.size(); i++) {

        temp = b0 * result[i].x + b1 * result[i].y + y * cSet.contactNormal;
        cSet.contactPoints.push_back(temp);

      }

    }
    else if (polyhedra1ContactInfo->size() == 2) { //edge-edge

      cSet.cA = &c0;
      cSet.cB = &c1;

      mVECTOR3 s1, s2, n;
      mVECTOR3 edgeA, edgeB;

      s1 = cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[1]);
      n = cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[0]);
      edgeA = n - s1;

      s2 = cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[1]);
      n = cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[0]);
      edgeB = n - s2;


      mVec3Cross(&cSet.contactNormal, &edgeA, &edgeB);
      mVec3Normalize(&cSet.contactNormal, &cSet.contactNormal);

      mVECTOR3 r = edge_edge(s1.toDxVector(), edgeA.toDxVector(), s2.toDxVector(), edgeB.toDxVector());
      cSet.contactPoints.push_back(r);

      cSet.contactNormal = -d;

    }
    else if (polyhedra1ContactInfo->size() == 1) { //edge-vertex {

      cSet.cA = &c1;
      cSet.cB = &c0;

      cSet.contactPoints.push_back(cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[0]));
      cSet.contactNormal = d;

    }

  }
  else if (polyhedra0ContactInfo->size() >2) {//polygon-

    if (polyhedra1ContactInfo->size() > 2) { //polygon-polygon

      cSet.cA = &c0;
      cSet.cB = &c1;

      mVECTOR3 a, b, D;
      a = reinterpret_cast<mVECTOR3&>(cSet.cA->bodyRef->position);
      b = reinterpret_cast<mVECTOR3&>(cSet.cB->bodyRef->position);

      D = a - b;

      if (fabs(d.x) < 1e-4)
        d.x = 0.0f;
      if (fabs(d.y) < 1e-4)
        d.y = 0.0f;
      if (fabs(d.z) < 1e-4)
        d.z = 0.0f;


      if (mVec3Dot(&D, &d) >= 0.0f) {

        cSet.contactNormal = d;

      }
      else {
        cSet.contactNormal = -d;

      }

      // SOME polygon-polygon intersection routine
      std::vector<mVECTOR2> pAvertices;
      std::vector<mVECTOR2> pBvertices;

      double y;

      std::vector<mVECTOR2> result;

      mVECTOR3 b0, b1, b2, normal;
      mVECTOR3 temp;


      mVECTOR3 planeOrigin = cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[0]);
      b1 = cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[2]) - planeOrigin;
      b0 = cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[1]) - planeOrigin;
      //  mVec3Cross(&b1, &b0, &cSet.contactNormal);
      mVec3Normalize(&b0, &b0);
      mVec3Normalize(&b1, &b1);

      mVec3Cross(&normal, &b0, &b1);
      mVec3Cross(&b1, &b0, &normal);
      mVec3Normalize(&b0, &b0);
      mVec3Normalize(&b1, &b1);
      mVec3Normalize(&normal, &normal);

      y = mVec3Dot(&normal, &planeOrigin);


      for (int i = polyhedra0ContactInfo->size() - 1; i >= 0; i--) {
        pAvertices.push_back(mVECTOR2(mVec3Dot(&cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[i]), &b0),
          mVec3Dot(&cSet.cA->GetTransformedVertex((*polyhedra0ContactInfo)[i]), &b1)));
      }

      for (unsigned int i = 0; i<polyhedra1ContactInfo->size(); i++) {
        pBvertices.push_back(mVECTOR2(mVec3Dot(&cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[i]), &b0),
          mVec3Dot(&cSet.cB->GetTransformedVertex((*polyhedra1ContactInfo)[i]), &b1)));
      }

      sort(pAvertices);
      sort(pBvertices);


      ConvexPolygon CP0(pAvertices);

      ConvexPolygon CP1(pBvertices);


      polygon_polygon(CP0, CP1, result);

      if (result.size())
        sort(result);

      int l = result.size();

      for (unsigned int i = 0; i<result.size(); i++) {

        temp = b0 * result[i].x + b1 * result[i].y + y * normal;
        cSet.contactPoints.push_back(temp);

      }


    }
    else if (polyhedra1ContactInfo->size() == 2) { //polygon-edge

      cSet.cA = &c1;
      cSet.cB = &c0;

      cSet.contactNormal = d;

      std::vector<mVECTOR2> pAvertices;
      std::vector<mVECTOR2> pBvertices;

      double y;

      std::vector<mVECTOR2> result;

      mVECTOR3 b0, b1;
      mVECTOR3 temp;

      mVECTOR3 planeOrigin = cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[0]);

      b0 = cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[1]) - planeOrigin;
      mVec3Cross(&b1, &b0, &cSet.contactNormal);
      mVec3Normalize(&b0, &b0);
      mVec3Normalize(&b1, &b1);

      y = mVec3Dot(&cSet.contactNormal, &planeOrigin);
      mVECTOR3 pre1, pre2;
      pre1 = cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[0]);
      pre2 = cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[1]);


      for (int i = polyhedra0ContactInfo->size() - 1; i >= 0; i--) {
        pBvertices.push_back(mVECTOR2(mVec3Dot(&cSet.cB->GetTransformedVertex((*polyhedra0ContactInfo)[i]), &b0),
          mVec3Dot(&cSet.cB->GetTransformedVertex((*polyhedra0ContactInfo)[i]), &b1)));


      }

      for (unsigned int i = 0; i<polyhedra1ContactInfo->size(); i++) {
        pAvertices.push_back(mVECTOR2(mVec3Dot(&cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[i]), &b0),
          mVec3Dot(&cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[i]), &b1)));

      }

      // SOME edge polygon intersection routine

      sort(pBvertices);
      edge_polygon(pBvertices, pAvertices[0], pAvertices[1] - pAvertices[0], result);

      for (unsigned int i = 0; i<result.size(); i++) {

        temp = b0 * result[i].x + b1 * result[i].y + y * cSet.contactNormal;
        cSet.contactPoints.push_back(temp);

      }

    }
    else if (polyhedra1ContactInfo->size() == 1) { //polygon-vertex

      cSet.cA = &c1;
      cSet.cB = &c0;


      cSet.contactPoints.push_back(cSet.cA->GetTransformedVertex((*polyhedra1ContactInfo)[0]));

      cSet.contactNormal = d;
    }
  }

}

void CollisionManager::GetContactInfo(double t1, CollisionAvatar *C0, CollisionAvatar *C1, ContactInfo &I, unsigned int maxIterations) {

  double t0 = 0.0;
  I = C0->GetContactInfo(C1);
  I.rollback = false;


  if (!I.contact) {
    return;
  }
  if (!I.volume) {
    I.contactTime = t1;
    return;
  }

  double tm;
  for (unsigned int i = 1; i< maxIterations; i++) {

    tm = (t0 + t1) * 0.5;

    if (C0->timeDelta > tm) {
      I.rollback = true;
      C0->Rollback(tm);
    }
    if (C1->timeDelta > tm) {
      I.rollback = true;
      C1->Rollback(tm);
    }

    I = C0->GetContactInfo(C1);
    ///???
    I.rollback = true;

    if (!I.contact)
      t0 = tm;
    else if (I.volume)
      t1 = tm;
    else {
      I.contactTime = tm;
      return;
    }
  }

}



void CollisionManager::ContactSetFromConstraint(CollisionAvatar*A, CollisionAvatar*B, ContactSet& cSet) {

  cSet.cA = A;
  cSet.cB = B;

  cSet.contactNormal = A->Translation() - B->Translation();
  cSet.contactPoints.push_back((B->Translation() + cSet.contactNormal / 2));
  mVec3Normalize(&cSet.contactNormal, &cSet.contactNormal);

}



ContactPointReference::ContactPointReference(ContactSet *c, unsigned int contactSetIndex, unsigned int b, CollisionAvatar *cP, mVECTOR3 n)
  :contactSetPointer(c), matrixIndex(b)
{
  rel = c->contactPoints[contactSetIndex] - cP->Translation();

  normal = n;
}


//CollisionManager///////////////////////////////////////////////////////

void CollisionManager::AddCollisionAvatar(CollisionAvatar* c) {

  collisionAvatars.push_back(c);

}

void CollisionManager::BuildConstraintGroups() {

  std::vector<bilateralConstraint>::iterator it;
  for (it = constraints.begin(); it != constraints.end(); it++) {

    if ((*it).A->cGroup) {

      if ((*it).B->cGroup) { //both in a group

        if ((*it).A->cGroup == (*it).B->cGroup) { //both in the same group

          constraintGroup *CG = (*it).A->cGroup;
          CG->constraints.push_back(&(*it));

        }
        else { //merge groups

          constraintGroup *CG = new constraintGroup;
          constraintGroup *CG1 = (*it).A->cGroup;
          constraintGroup *CG2 = (*it).B->cGroup;
          std::vector<bilateralConstraint*>::iterator itBc;
          for (itBc = CG1->constraints.begin(); itBc != CG1->constraints.end(); itBc++)
            CG->constraints.push_back(*itBc);
          for (itBc = CG2->constraints.begin(); itBc != CG2->constraints.end(); itBc++)
            CG->constraints.push_back(*itBc);
          delete CG1;
          delete CG2;
          (*it).A->cGroup = CG;
          (*it).B->cGroup = CG;


        }


      }
      else { //only first in a group

        constraintGroup *CG = (*it).A->cGroup;
        CG->constraints.push_back(&(*it));
        (*it).B->cGroup = CG;

      }

    }
    else if ((*it).B->cGroup) { //only second in a group

      constraintGroup *CG = (*it).B->cGroup;
      CG->constraints.push_back(&(*it));
      (*it).A->cGroup = CG;

    }
    else { //none in a group

      constraintGroup *CG = new constraintGroup;
      CG->constraints.push_back(&(*it));
      (*it).B->cGroup = CG;
      (*it).A->cGroup = CG;
      constraintGroups.push_back(CG);

    }

  }

  std::vector<constraintGroup*>::iterator itCg;
  std::vector<bilateralConstraint*>::iterator itBc;
  for (itCg = constraintGroups.begin(); itCg != constraintGroups.end(); itCg++) {

    for (itBc = (*itCg)->constraints.begin(); itBc != (*itCg)->constraints.end(); itBc++) {

      (*itCg)->bodies.insert((*itBc)->A);
      (*itCg)->bodies.insert((*itBc)->B);

    }

  }


}

void CollisionManager::Clear() {

  std::list<ContactCollection*>::iterator it;
  for (it = contactCollections.begin(); it != contactCollections.end(); it++)
    delete (*it);
  contactCollections.clear();

  std::list<ContactInfo*>::iterator it2;
  for (it2 = contactInfos.begin(); it2 != contactInfos.end(); it2++)
    delete (*it2);
  contactInfos.clear();

  poolOne.clear();
  poolTwo.clear();

  twoToOne.clear();

  I = 0;

}

void CollisionManager::ClearContactInfos(CollisionAvatar* cA) {

  for (unsigned int i = 0; i<cA->contactInfos.size(); i++) {
    contactInfos.remove(cA->contactInfos[i]);
  }
  cA->contactInfos.clear();

}

void CollisionManager::Frame(double timeDelta) {

  I = 0;

  std::vector<CollisionAvatar*>::iterator it;
  std::vector<CollisionAvatar*>::iterator it2;


  for (it = collisionAvatars.begin(); it != collisionAvatars.end(); it++){
    (*it)->Zero();
    (*it)->TimeAlign(timeDelta);
  }

  poolOne.insert(poolOne.begin(), collisionAvatars.begin(), collisionAvatars.end());

  //MAIN PASS
  while ((poolOneIt = poolOne.begin()) != poolOne.end()) {

    bool moveOne(true);
    for (poolTwoIt = poolTwo.begin(); poolTwoIt != poolTwo.end(); poolTwoIt++) {

      if (!BoundingSphereIntersection(**poolOneIt, **poolTwoIt))
        continue;

      CollisionAvatar *one = *poolOneIt;
      CollisionAvatar *two = *poolTwoIt;

      I = new ContactInfo;

      GetContactInfo(timeDelta, one, two, *I, ::maxIterations);

      if (I->rollback) {

        if (I->volume && one->ignoreVolume && two->ignoreVolume) {
          contactInfos.push_back(I);
          one->contactInfos.push_back(I);
          two->contactInfos.push_back(I);
        }

        constraintGroup *cGOne = one->cGroup;
        constraintGroup *cGTwo = two->cGroup;

        if (!two->ignoreVolume) {

          if (I->volume)
            two->ignoreVolume = true;
          ClearContactInfos(two);

          twoToOne.push_back(two);
          two->moveOne = true;

          //////
          TimeAlign(cGTwo);
        }

        if (!one->ignoreVolume) {

          if (I->volume)
            one->ignoreVolume = true;
          ClearContactInfos(one);

          moveOne = false;

          TimeAlign(cGOne);
        }
        if (!two->ignoreVolume) {
          delete I;
          I = 0;
        }
        if (!one->ignoreVolume)
          break;
      }
      else if (I->contact) {
        contactInfos.push_back(I);
        one->contactInfos.push_back(I);
        two->contactInfos.push_back(I);
      }
      else {
        delete I;
      }

    }
    if (moveOne) {
      (*poolOneIt)->inTwo = true;

      poolTwo.push_back((*poolOneIt));
      poolOne.remove((*poolOneIt));

    }
    for (twoToOneIt = twoToOne.begin(); twoToOneIt != twoToOne.end(); twoToOneIt++) {
      (*twoToOneIt)->inTwo = false;
      (*poolOneIt)->moveOne = false;
      poolOne.push_back((*twoToOneIt));
      poolTwo.remove((*twoToOneIt));

    }
    twoToOne.clear();

  }

  std::list<ContactInfo*>::iterator cIt;
  //COMPUTE CONTACT SETS AND BUILD CONTACT GROUPS
  for (cIt = contactInfos.begin(); cIt != contactInfos.end(); cIt++) {

    ContactSet C;
    ComputeContactSet(**cIt, *((*cIt)->cA), *((*cIt)->cB), C);

    if ((*cIt)->cA->contactCollection) {

      if ((*cIt)->cB->contactCollection) { //both are in a contact group

        if ((*cIt)->cA->contactCollection == (*cIt)->cB->contactCollection) { // both are in the same group

          ContactCollection *CC = (*cIt)->cA->contactCollection;
          CC->contactSets.push_back(C);

        }
        else { //merge groups

          ContactCollection *C1 = (*cIt)->cA->contactCollection;
          ContactCollection *C2 = (*cIt)->cB->contactCollection;

          AddToContactCollection(C1, C2->contactSets.begin(), C2->contactSets.end());

          C1->contactSets.push_back(C);
          delete C2;
          contactCollections.remove(C2);
        }

      }
      else { // only first in the group

        ContactCollection *CC = (*cIt)->cA->contactCollection;
        CC->contactSets.push_back(C);
        AddToContactCollection(CC, (*cIt)->cB);
      }

    }
    else if ((*cIt)->cB->contactCollection) { //only second in the group

      ContactCollection *CC = (*cIt)->cB->contactCollection;
      CC->contactSets.push_back(C);
      AddToContactCollection(CC, (*cIt)->cA);
    }
    else { //none has a group

      ContactCollection *CC = NewContactCollection(C);
    }

  }


  //handle constraints
  std::vector<bilateralConstraint>::iterator IT;
  for (IT = constraints.begin(); IT != constraints.end(); IT++) {
    ContactSet C;// ((*IT).A->polyRef, (*IT).B->polyRef);
    ContactSetFromConstraint( (*IT).A, (*IT).B, C);
    if (((*IT).A->contactCollection) && ((*IT).A->contactCollection->active)){

      if (((*IT).B->contactCollection) && ((*IT).B->contactCollection->active)) { //both are in a contact group

        if ((*IT).A->contactCollection == (*IT).B->contactCollection) {

          ContactCollection *CC = (*IT).A->contactCollection;
          CC->contactSets.push_back(C);
          CC->constraintCount++;

        }
        else { //merge groups

          ContactCollection *C1 = (*IT).A->contactCollection;
          ContactCollection *C2 = (*IT).B->contactCollection;
          C1->constraintCount = C1->constraintCount + C2->constraintCount + 1;

          AddToContactCollection(C1, C2->contactSets.begin(), C2->contactSets.end());

          C1->contactSets.push_back(C);
          delete C2;
          contactCollections.remove(C2);

        }

      }
      else { // only first in the group

        ContactCollection *CC = (*IT).A->contactCollection;
        CC->contactSets.push_back(C);
        if ((*IT).B->bodyRef->immovable == false)
          (*IT).B->contactCollection = CC;

        CC->constraintCount++;

      }

    }
    else if (((*IT).B->contactCollection) && ((*IT).B->contactCollection->active)) { //only second in the group

      ContactCollection *CC = (*IT).B->contactCollection;
      CC->contactSets.push_back(C);
      if ((*IT).A->bodyRef->immovable == false)
        (*IT).A->contactCollection = CC;

      CC->constraintCount++;

    }
    else { //none has a group

      ContactCollection *CC = NewContactCollection(C);
      CC->constraintCount++;
    }

  }

  ////////
  int l(0), k(0);
  std::list<ContactCollection*>::iterator it3;
  std::vector<ContactSet>::iterator it4;
  std::vector<mVECTOR3>::iterator it5;

  //add contact point reference to collision avatars
  for (it3 = contactCollections.begin(); it3 != contactCollections.end(); it3++, l = 0)
    for (it4 = (*it3)->contactSets.begin(); it4 != (*it3)->contactSets.end(); it4++)
      for (k = 0, it5 = (*it4).contactPoints.begin(); it5 != (*it4).contactPoints.end(); it5++, k++) {
        (*it4).cA->contactPoints.push_back(ContactPointReference(&(*it4), k, l, (*it4).cA, (*it4).contactNormal));
        (*it4).cB->contactPoints.push_back(ContactPointReference(&(*it4), k, l, (*it4).cB, -(*it4).contactNormal));
        l++;
      }

  //response
  for (it3 = contactCollections.begin(); it3 != contactCollections.end(); it3++) {

    RResponse R(*(*it3));
    R.Response();
  }


  Clear();

}

ContactCollection* CollisionManager::NewContactCollection(ContactSet &cS) {

  ContactCollection *CC = new ContactCollection();
  CC->contactSets.push_back(cS);

  if ( cS.cA->ignoreVolume || cS.cB->ignoreVolume )
    CC->displacement = true;

  if (cS.cA->BodyRef()->immovable == false)
    cS.cA->contactCollection = CC;
  if (cS.cB->BodyRef()->immovable == false)
    cS.cB->contactCollection = CC;


  contactCollections.push_back(CC);

  return CC;

}

void CollisionManager::AddToContactCollection(ContactCollection *CC, CollisionAvatar *cA) {

  if (cA->ignoreVolume)
    CC->displacement = true;

  if (cA->bodyRef->immovable == false)
    cA->contactCollection = CC;
}

void CollisionManager::AddToContactCollection(ContactCollection *CC, std::vector<ContactSet>::iterator first, std::vector<ContactSet>::iterator last) {

  for (std::vector<ContactSet>::iterator it = first; it != last; it++) {

    AddToContactCollection(CC, (*it).cA);
    AddToContactCollection(CC, (*it).cB);
  }
  CC->contactSets.insert(CC->contactSets.end(), first, last);
  //std::for_each(first, last, AddToContactCollection(CC, )

}

bool CollisionManager::BoundingSphereIntersection(CollisionAvatar& s0, CollisionAvatar& s1) {

  mVECTOR3 v = *(s1.translation) - *(s0.translation);
  return ((v.x*v.x + v.y*v.y + v.z*v.z) <= (s1.BoundingSphereRadius() + s0.BoundingSphereRadius())*(s1.BoundingSphereRadius() + s0.BoundingSphereRadius() ));

}


void CollisionManager::Reset() {

  Clear();

  std::vector<constraintGroup*>::iterator itC;
  for (itC = constraintGroups.begin(); itC != constraintGroups.end(); itC++) {
    delete *itC;
  }
  constraints.clear();
  constraintGroups.clear();

  std::vector<CollisionAvatar*>::iterator it;
  for (it = collisionAvatars.begin(); it != collisionAvatars.end(); it++) {
    delete (*it);
  }
  collisionAvatars.clear();

}

void CollisionManager::TimeAlign(constraintGroup *cG) {

  if (cG) {
    std::set<CollisionAvatar*>::iterator bIt;

    for (bIt = cG->bodies.begin(); bIt != cG->bodies.end(); bIt++) {

      if ((*bIt)->timeDelta > (*poolOneIt)->timeDelta) {

        if (I->volume)
          (*bIt)->ignoreVolume = true;
        ClearContactInfos((*bIt));
        (*bIt)->Rollback((*poolOneIt)->timeDelta);

        if (((*bIt)->inTwo) && !((*bIt)->moveOne)) {
          twoToOne.push_back((*bIt));
        }
      }
    }
  }

}

CollisionManager *collisionManager;
