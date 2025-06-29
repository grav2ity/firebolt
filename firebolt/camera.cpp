#include "Camera.h"


Camera *camera;


Camera::Camera(float fieldOfView, float aspectRatio, float nearClippingPlane, float farClippingPlane) :
  fieldOfView(fieldOfView), aspectRatio(aspectRatio), nearClippingPlane(nearClippingPlane), farClippingPlane(farClippingPlane)
{

  translation = D3DXVECTOR4(0.0f, 0.0f, -100.0f, 1.0f);
  rotation = D3DXQUATERNION(0.0f,0.0f,0.0f,1.0f);

  updateViewMatrixWithNewRotation();
  viewMatrix(3,3) = 1.0f;

  D3DXMatrixPerspectiveFovLH(
    &projectionMatrix,
        fieldOfView,
        aspectRatio,
        nearClippingPlane,
        farClippingPlane);

}

void Camera::getViewMatrix(D3DXMATRIX& matrix) {

  D3DXMATRIX translationMatrix;
  D3DXMatrixTranslation(&translationMatrix, translation.x, translation.y, translation.z);
  matrix = viewMatrix *translationMatrix;
  D3DXMatrixInverse(&matrix, NULL, &matrix);

}

void Camera::moveAlongLocalAxisX(float distance) {

  translation.x += viewMatrix._11 * distance;
  translation.y += viewMatrix._12 * distance;
  translation.z += viewMatrix._13 * distance;

}

void Camera::moveAlongLocalAxisY(float distance) {

  translation.x += viewMatrix._21 * distance;
  translation.y += viewMatrix._22 * distance;
  translation.z += viewMatrix._23 * distance;

}

void Camera::moveAlongLocalAxisZ(float distance) {

  translation.x += viewMatrix._31 * distance;
  translation.y += viewMatrix._32 * distance;
  translation.z += viewMatrix._33 * distance;

}

D3DXVECTOR3 Camera::getViewDirection() {

  D3DXVECTOR3 view(0.0f, 0.0f, 1.0f);
  D3DXVec3TransformNormal(&view, &view, &viewMatrix);
  return view;

}

void Camera::rotatateAroundWorldAxisX(float angle) {

  D3DXQUATERNION rotate(sin(angle/2), 0.0f, 0.0f, cos(angle/2));
  rotation *= rotate;
  updateViewMatrixWithNewRotation();


}

void Camera::rotatateAroundWorldAxisY(float angle) {

  D3DXQUATERNION rotate(0.0f, sin(angle/2.0f), 0.0f, cos(angle/2.0f));
  rotation *= rotate;
  updateViewMatrixWithNewRotation();

}

void Camera::rotatateAroundWorldAxisZ(float angle) {

  D3DXQUATERNION rotate(0.0f, 0.0f, sin(angle/2), cos(angle/2));
  rotation *= rotate;
  updateViewMatrixWithNewRotation();

}

void Camera::rotatateAroundLocalAxisX(float angle) {

  D3DXQUATERNION rotate(sin(angle/2), 0,0, cos(angle/2) );
  rotation = rotate *rotation;
  D3DXQuaternionNormalize(&rotation, &rotation);
  updateViewMatrixWithNewRotation();

}

void Camera::rotatateAroundLocalAxisY(float angle) {

  D3DXQUATERNION rotate(0, sin(angle/2),0, cos(angle/2) );
  rotation = rotate *rotation;
  D3DXQuaternionNormalize(&rotation, &rotation);
  updateViewMatrixWithNewRotation();

}

void Camera::rotatateAroundLocalAxisZ(float angle) {

  D3DXQUATERNION rotate( 0,  0,  sin(angle / 2), cos(angle / 2));
  rotation = rotate *rotation;
  D3DXQuaternionNormalize(&rotation, &rotation);
  updateViewMatrixWithNewRotation();

}

void Camera::updateViewMatrixWithNewRotation() {

  D3DXMatrixRotationQuaternion(&viewMatrix, &rotation);

}



mVECTOR2 Viewport::inverse(float x, float y) {

  mVECTOR2 r;
  r.x = (2*x - 2*xOffset - width) / width;
  r.y = ( -2*y + 2*yOffset + height) / height;
  r.x /= activeCamera->projectionMatrix(0,0);
  r.y /= activeCamera->projectionMatrix(1,1);
  return r;
}

mVECTOR2 Viewport::inverse2(float x, float y) {

  mVECTOR2 r;
  r.x = (2*x - 2*xOffset - width) / width;
  r.y = ( -2*y + 2*yOffset + height) / height;
  return r;
}
