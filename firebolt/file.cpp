#include "file.h"

#include "direct3d.h"
#include "dmesh.h"
#include "mesh.h"
#include "oob.h"

#include <cfloat>

SceneReader sceneReader;


void SceneReader::readScene(char* filename) {

  meshCount =0;

  file.open(filename, std::ios::binary);

  unsigned int rootNodesCount;
  file.read((char*)&rootNodesCount, sizeof(unsigned int));
  Node *n = new Node[rootNodesCount];

  for(unsigned int i= 0; i< rootNodesCount; i++) {
    n[i].parent= 0;
    readNode(&n[i]);
  }

  direct3D->meshBaseIndex += meshCount;

  file.close();

  for(unsigned int i= 0; i< rootNodesCount; i++) {
    createObjects(&n[i]);
  }

  delete[] n;

}

void SceneReader::createObjects(Node *n) {

//  Body * b = bodyManager->AddBody( new Body( mMATRIX (n->staticMatrix) , n->inertiaTensor ) );
  
  Body * b = bodyManager->NewBody ( mMATRIX(n->staticMatrix), n->inertiaTensor) ;

  n->bodyRef =b;



  if(n->frozen) {

    float veryLargeFloat = FLT_MAX;
    float verySmallFloat = FLT_MIN;

    b->immovable=true;
  //  b->mass = veryLargeFloat;
  //  mMATRIX &it = b->bodyInertiaTensor;
  //  mMATRIX &itinv = b->bodyInertiaTensorInverse;

    //it(0, 0) = veryLargeFloat;
    //it(1, 1) = veryLargeFloat;
    //it(2, 2) = veryLargeFloat;

    //mMatrixInverse(&itinv,0,&it);

    //itinv(0, 0) = verySmallFloat;
    //itinv(1, 1) = verySmallFloat;
    //itinv(2, 2) = verySmallFloat;

    }
  else
    b->immovable=false;

  if (n->meshIndex != -1) {

    if (!n->obb) {

      Mesh *m = direct3D->static_Meshes[n->meshIndex];
      D3DXVECTOR3 v = (m->aabb->max_ - m->aabb->min_) * 0.5f;
      n->obb = new OBB(b, v.x, v.y, v.z);

    //  n->obb->translation = &b->position;
    //  n->obb->bodyRef = b;

    //  b->polyRef = n->obb;
      n->obb->rotation = &b->rotation;

      n->obb->offset = mMATRIX(n->offsetMatrix);
      collisionManager->AddCollisionAvatar(n->obb);


    }
    else {

    //  n->obb->translation = &b->position;
      //n->obb->bodyRef = b;
    //  b->polyRef = n->obb;

      n->obb->rotation = &b->rotation;

      n->obb->offset = mMATRIX(n->offsetMatrix);
      collisionManager->AddCollisionAvatar(n->obb);
    }
    dmeshManager.newDMesh(n->meshIndex, b, n->offsetMatrix, n->color);

  }

  if (n->parent)
    collisionManager->constraints.push_back(bilateralConstraint(n->obb, n->parent->obb));

  for(unsigned int i= 0; i< n->childrenCount; i++) {
    createObjects(n->children[i]);
  }

}



void SceneReader::readNode(Node *n) {

//  unsigned int frozen;
  file.read((char*)&n->frozen, sizeof(unsigned int));

  DWORD color;
  file.read((char*)&color, sizeof(DWORD));
  n->color.r = ((int) GetRValue(color)) / 255.0f;
  n->color.g = ((int) GetGValue(color)) / 255.0f;
  n->color.b = ((int) GetBValue(color)) / 255.0f;
  n->color.a = 1.0f;
  
  file.read((char*)&n->childrenCount, sizeof(int));
  n->children = new Node*[n->childrenCount];

  unsigned int oldMesh;
  file.read((char*)&oldMesh, sizeof(unsigned int));

//  unsigned int hasGeometry;
//  file.read((char*)&hasGeometry, sizeof(unsigned int));


  int ID;
  file.read((char*)&ID, sizeof(int));
    
  if(ID == 1) { //it's a box

    //create obb right away
    float length, width, height;
    file.read((char*)&length, sizeof(float));
    file.read((char*)&width, sizeof(float));
    file.read((char*)&height, sizeof(float));
  //  n->obb = new OBB(width/2, length/2, height/2);
    n->obb = 0;
    memset(&n->inertiaTensor, 0, sizeof(mMATRIX));
    n->inertiaTensor(0, 0) = (height * height + length * length) / 12.0f;
    n->inertiaTensor(1, 1) = (height * height + width * width) / 12.0f;
    n->inertiaTensor(2, 2) = (width * width + length * length) / 12.0f;

  }
  else
    n->obb = 0;


  if (!oldMesh) {

  //  if (hasGeometry) {

      readMesh();
      meshCount++;
  //}
  }
  
  file.read((char*)&n->meshIndex, sizeof(int));
  
  if (n->meshIndex != -1)
    n->meshIndex += direct3D->meshBaseIndex;  

  //check for animation  
  int anim;
//  int type;
//  int time;
  file.read((char*)&anim, sizeof(int));

  //deleted for now
  if (anim) {  animationCount++; } 
  
  //read static matrix
  else {
    
    D3DXVECTOR3 pos;
    float *m = new float[16];
    file.read((char*)m, sizeof(float)*3); m+=4;
    file.read((char*)m, sizeof(float)*3); m+=4;
    file.read((char*)m, sizeof(float)*3); m+=4;
    file.read((char*)&pos, sizeof(float)*3); m-=12;
    m[3] = m[7] = m[11] = 0;
    m[15] = 1;
    m[14] = -m[14];

    D3DXQUATERNION q;
    D3DXQuaternionRotationMatrix(&q, (D3DXMATRIX*)m);
    q.w = -q.w;
    q.z = -q.z;
    D3DXMatrixRotationQuaternion((D3DXMATRIX*)m, &q);

    m[12] = pos.x;
    m[13] = pos.y;
    m[14] = -pos.z;
    m[3] = m[7] = m[11] = 0;
    m[15] = 1;
    D3DXMATRIX world(m);
    n->worldMatrix = world;
    D3DXMATRIX parentWorldInverse;
    if(n->parent) {
      D3DXMatrixInverse(&parentWorldInverse, 0, &n->parent->worldMatrix);
      world *= parentWorldInverse;
      n->staticMatrix = world * n->parent->staticMatrix;
    }
    else {
      n->staticMatrix = world;
    }
  

    delete[] m;

  }

  //read offset matrix
  D3DXVECTOR3 offset_scale;
  D3DXQUATERNION offset_scale_axis;
  D3DXQUATERNION offset_rotation;
  D3DXVECTOR3 offset_translation;

  file.read((char*)&offset_translation, sizeof(float)*3);
  file.read((char*)&offset_rotation, sizeof(float)*4);
  file.read((char*)&offset_scale, sizeof(float)*3);
  file.read((char*)&offset_scale_axis, sizeof(float)*4);

  //offset_rotation.w = -offset_rotation.w;
  //offset_rotation.z= -offset_rotation.z;
  offset_translation.z = -offset_translation.z; 

  D3DXMATRIX o_scale;
  D3DXMATRIX o_rotation;
  D3DXMATRIX o_translation;

  D3DXMatrixScaling(&o_scale, offset_scale.x, offset_scale.y, offset_scale.z);
  D3DXMatrixRotationQuaternion(&o_rotation, &offset_rotation);
  D3DXMatrixTranslation(&o_translation, offset_translation.x, offset_translation.y, offset_translation.z);

  n->offsetMatrix = o_rotation*o_translation;

  for(unsigned int i =0; i<n->childrenCount; i++) {

    n->children[i] = new Node; 
    n->children[i]->parent = n;
    readNode(n->children[i]);



  }



}


void SceneReader::readMesh() {

  Mesh *m = new Mesh;

  file.read((char*)&m->vertex_count, sizeof(int));
  file.read((char*)&m->face_count, sizeof(int));


  m->vertices = new vertex[m->vertex_count];
  memset(m->vertices , 0, sizeof(vertex) *m->vertex_count);
  m->faces = new face[m->face_count];

  //read vertices
  float crap;
  for(unsigned int i=0; i<m->vertex_count; i++) {

    file.read((char*)&m->vertices[i].v[0], sizeof(float));
    file.read((char*)&m->vertices[i].v[1], sizeof(float));
    file.read((char*)&m->vertices[i].v[2], sizeof(float));

    file.read((char*)&crap, sizeof(float));
    file.read((char*)&crap, sizeof(float));
    
  //  m->vertices[i].uv[1] = m->vertices[i].uv[1];
    m->vertices[i].v[2] = - m->vertices[i].v[2];

  }

  //read indices
  unsigned int temp;
  for(unsigned int i=0; i<m->face_count; i++) {

    file.read((char*)&temp, sizeof(unsigned int));
    m->faces[i].i[0] = temp;
    file.read((char*)&temp, sizeof(unsigned int));
    m->faces[i].i[2] = temp;
    file.read((char*)&temp, sizeof(unsigned int));
    m->faces[i].i[1] = temp;

  }
  
  //setup vertex_info
  vertex_info *v_info = new vertex_info[m->vertex_count];
  memset(v_info, 0, sizeof(vertex_info) * m->vertex_count);

  for(unsigned int i=0; i<m->face_count; i++) {

    v_info[m->faces[i].i[0]].num_adjacent_faces++;
    v_info[m->faces[i].i[1]].num_adjacent_faces++;
    v_info[m->faces[i].i[2]].num_adjacent_faces++;

  }

  for(unsigned int i=0; i<m->vertex_count; i++)
    v_info[i].adjacent_faces = new unsigned short[v_info[i].num_adjacent_faces];

  for(unsigned short i=0; i<m->face_count; i++) {

    *(v_info[m->faces[i].i[0]].adjacent_faces) = i;
    v_info[m->faces[i].i[0]].adjacent_faces++;
    *(v_info[m->faces[i].i[1]].adjacent_faces) = i;
    v_info[m->faces[i].i[1]].adjacent_faces++;
    *(v_info[m->faces[i].i[2]].adjacent_faces) = i;
    v_info[m->faces[i].i[2]].adjacent_faces++;
  }

  for(unsigned int i=0; i<m->vertex_count; i++)
    v_info[i].adjacent_faces -= v_info[i].num_adjacent_faces;


  
  //each face has its own set of vertices
  vertex *V = new vertex[m->face_count*3];
  memset(V, 0, sizeof(vertex) *m->face_count*3);

  m->vertex_count = m->face_count*3;
  vertex_info *V_info = new vertex_info[m->face_count*3];

  for(unsigned int i=0; i<m->face_count; i++) {

    V[(i*3)] = m->v(i,0);
    V[(i*3)+1] = m->v(i,1);
    V[(i*3)+2] = m->v(i,2);

    V_info[(i*3)] = v_info[m->faces[i].i[0]];
    V_info[(i*3)+1] =  v_info[m->faces[i].i[1]];
    V_info[(i*3)+2] = v_info[m->faces[i].i[2]];

    m->faces[i].i[0] = (i*3) ;
    m->faces[i].i[1] = (i*3) +1;
    m->faces[i].i[2] = (i*3) +2;

  }


  float b = 0.4f;

  delete [] m->vertices;
  m->vertices = V;


  /*for(int i=0; i<m->face_count; i++) {

    m->v(i,0).uv[0] = 0.0f;
    m->v(i,0).uv[1] = 0.0f;
    m->v(i,1).uv[0] = 0.0f;
    m->v(i,1).uv[1] = 1.0f;
    m->v(i,2).uv[0] = 1.0f;
    m->v(i,2).uv[1] = 1.0f;
  }*/

  //m->setup_per_face_normals();
  //m->setup_tangent_basis();

  for(unsigned int i=0; i<m->face_count; i++) {

    for(unsigned int j=0; j<V_info[(i*3)].num_adjacent_faces; j++) {

      if(D3DXVec3Dot(&m->face_normal(i), &m->face_normal(V_info[(i*3)].adjacent_faces[j])) > b)
        //V_info[(i*3)].adjacent_faces[j] = -1;
        reinterpret_cast<D3DXVECTOR3&>(m->vertices[(i*3)].n[0]) += m->face_normal(V_info[(i*3)].adjacent_faces[j]);

    }

    for(unsigned int j=0; j<V_info[(i*3)+1].num_adjacent_faces; j++) {
      if(D3DXVec3Dot(&m->face_normal(i), &m->face_normal(V_info[(i*3)+1].adjacent_faces[j])) > b)
      //  V_info[(i*3)+1].adjacent_faces[j] = -1;
        reinterpret_cast<D3DXVECTOR3&>(m->vertices[(i*3)+1].n[0]) += m->face_normal(V_info[(i*3)+1].adjacent_faces[j]);
    
    }

    for(unsigned int j=0; j<V_info[(i*3)+2].num_adjacent_faces; j++) {
      if(D3DXVec3Dot(&m->face_normal(i), &m->face_normal(V_info[(i*3)+2].adjacent_faces[j])) > b)
      //  V_info[(i*3)+2].adjacent_faces[j] = -1;
        reinterpret_cast<D3DXVECTOR3&>(m->vertices[(i*3)+2].n[0]) += m->face_normal(V_info[(i*3)+2].adjacent_faces[j]);
  
    }

  }

  for(unsigned int i=0; i<m->face_count*3; i++) {
    D3DXVec3Normalize(reinterpret_cast<D3DXVECTOR3*>(m->vertices[i].n), reinterpret_cast<D3DXVECTOR3*>(m->vertices[i].n));
  }


  delete[] v_info;
  delete[] V_info;

  m->calcAABB();
  m->calcBoundingSphereRadius();

  direct3D->static_Meshes.push_back(m);
  dmeshManager.registerDMeshClass(meshCount + direct3D->meshBaseIndex);
  //m->CPA=0;

}
