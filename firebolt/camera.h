#ifndef CAMERA_H
#define CAMERA_H

#include <cmath>
#include <d3dx9.h>

#include "mvector.h"


class Viewport;

class Camera {

  friend class Viewport;

public:

  Camera(float fieldOfView, float aspectRatio, float nearClippingPlane, float farClippingPlane);

  void moveAlongLocalAxisX(float distance);
  void moveAlongLocalAxisY(float distance);
  void moveAlongLocalAxisZ(float distance);

  void moveAlongWorldAxisX(float distance);
  void moveAlongWorldAxisY(float distance);
  void moveAlongWorldAxisZ(float distance);

  void rotatateAroundLocalAxisX(float angle);
  void rotatateAroundLocalAxisY(float angle);
  void rotatateAroundLocalAxisZ(float angle);

  void rotatateAroundWorldAxisX(float angle);
  void rotatateAroundWorldAxisY(float angle);
  void rotatateAroundWorldAxisZ(float angle);


  void setTranslation(const D3DXVECTOR4 &newTranslation) { translation = newTranslation; }
  D3DXVECTOR4 getTranslation() { return translation; }

  void setRotation(const D3DXQUATERNION &newRotation) { rotation = newRotation; updateViewMatrixWithNewRotation();}
  D3DXQUATERNION getRotation() { return rotation; }

  void setViewMatrix(const D3DXMATRIX& newViewMatrix) { viewMatrix = newViewMatrix; }
  void getViewMatrix(D3DXMATRIX&);

  void getProjectionMatrix(D3DXMATRIX& p) { p = projectionMatrix; }

  D3DXVECTOR3 getViewDirection();

private:

  void updateViewMatrixWithNewRotation();

  float fieldOfView;
  float aspectRatio;

  float nearClippingPlane;
  float farClippingPlane;

  D3DXVECTOR4 translation;
  D3DXQUATERNION rotation;

  D3DXMATRIX viewMatrix;
  D3DXMATRIX projectionMatrix;

};


class Viewport {

public:

  Viewport (unsigned int x, unsigned int y, unsigned int width, unsigned int height) : xOffset(x), yOffset(y), width(width), height(height) {}
  mVECTOR2 inverse(float, float);
  mVECTOR2 inverse2(float, float);

  Camera *activeCamera;

private:

  unsigned int xOffset, yOffset;
  unsigned int width, height;


};

extern Camera *camera;


#endif
