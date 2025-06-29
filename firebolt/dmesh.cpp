#include "dmesh.h"


DMeshManager dmeshManager;

DMeshInstance::DMeshInstance(Body* b, D3DXMATRIX offsetM, D3DXCOLOR c) : bodyRef(b), offsetMatrix(offsetM) ,color(c) {}

void DMeshInstance::UpdateState() {

  translation = bodyRef->position.toDxVector();
  rotation = bodyRef->rotation;

}

DMeshClass::DMeshClass(int m, unsigned int maxDMeshCoun) : meshIndex(m), maxDMeshCount(maxDMeshCoun), curDMeshCount(0) {

  aabb = direct3D->static_Meshes[m]->aabb;

}

DMeshClass::~DMeshClass() {

  dMeshes.clear();

}

void DMeshClass::newDMesh(Body *b, D3DXMATRIX offsetMatrix, D3DXCOLOR color) {

    if(curDMeshCount < maxDMeshCount) {
      dMeshes.push_back(DMeshInstance(b, offsetMatrix, color));
      curDMeshCount++;
    }

}

void DMeshClass::updateDMeshs() {

  std::list<DMeshInstance>::iterator it;
  for(it = dMeshes.begin(); it != dMeshes.end(); it++)
    (*it).UpdateState();

}

void DMeshClass::streamDMeshs() {

  D3DXMATRIX matrix, matrix2, trans, rot;
  //float *pointer;

  std::list<DMeshInstance>::iterator it;
  unsigned int i(0);
  for(it = dMeshes.begin(); it != dMeshes.end(); it++,i++) {

    (*it).UpdateState();

  }

  D3DXVECTOR4 light_vector = -camera->getViewDirection();

  //light_vector.x = 1;
  //light_vector.z = 1;
  //light_vector.y = 1;
  light_vector.w = 0.0;
  D3DXVec4Normalize(&light_vector, &light_vector);


  for(it = dMeshes.begin(); it != dMeshes.end(); it++) {

    D3DXMatrixTranslation(&trans, (*it).translation.x, (*it).translation.y, (*it).translation.z);

    rot = (*it).bodyRef->rotation;

    matrix2 = (*it).offsetMatrix * rot * trans* direct3D->viewProjMatrix;

    direct3D->constant_tables["vs_dot"]->SetMatrix(direct3D->device, direct3D->constant_handles["vs_dot_worldViewProj"], &matrix2);
    direct3D->constant_tables["vs_dot"]->SetVector(direct3D->device, direct3D->constant_handles["vs_dot_color"], (D3DXVECTOR4*)&(*it).color);
    direct3D->constant_tables["vs_dot"]->SetVector(direct3D->device, direct3D->constant_handles["vs_dot_light_position"], (D3DXVECTOR4*)&light_vector);

    direct3D->streamStaticMesh(meshIndex);
  }


}


void DMeshInstance::getAABB(AABB& a) {

  D3DXMATRIX trans, world;
  D3DXMatrixTranslation(&trans, translation.x, translation.y, translation.z);
  world = offsetMatrix * rotation * trans;

  D3DXVec3TransformCoord(&a.max_, &a.max_, &world);
  D3DXVec3TransformCoord(&a.min_, &a.min_, &world);

}


void DMeshManager::newDMesh(unsigned int m, Body *b, D3DXMATRIX offsetMatrix, D3DXCOLOR color) {

  dmeshClasses[m]->newDMesh(b, offsetMatrix, color);

}
void DMeshManager::registerDMeshClass(unsigned int m) {

  dmeshClasses[m] = new DMeshClass(m, 100);

}
void DMeshManager::streamDMeshClasses() {

  std::map<unsigned int, DMeshClass*>::iterator it;
  for( it= dmeshClasses.begin(); it!= dmeshClasses.end(); it++) {
    (*it).second->streamDMeshs();
  }

}
void DMeshManager::updateDMeshClasses() {

  std::map<unsigned int, DMeshClass*>::iterator it;
  for( it= dmeshClasses.begin(); it!= dmeshClasses.end(); it++) {
    (*it).second->updateDMeshs();
  }

}

void DMeshManager::Reset() {

  std::map<unsigned int, DMeshClass*>::iterator it;
  for(it =dmeshClasses.begin(); it != dmeshClasses.end(); it++) {

    delete (*it).second;
  }
  dmeshClasses.clear();
}
