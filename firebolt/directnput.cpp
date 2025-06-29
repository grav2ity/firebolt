#include "directinput.h"

#include "application.h"
#include "camera.h"

DInput *dInput;

extern double inertiaTensorMultiplier;


unsigned int framesToAdvance;

const float mouseSensitivity = 0.01f;

DInput::DInput(HINSTANCE h, HWND w) {

  memset(pressedKeys, 0, KeyNumber);
  memset(pressedKeysThisFrame, 0, KeyNumber);

  DIPROPDWORD prop = { { sizeof(DIPROPDWORD), sizeof(DIPROPHEADER), 0, DIPH_DEVICE, }, 250 };

  DICREATE( h, DIRECTINPUT_VERSION, IID_IDirectInput8, (void**)&Interface, NULL);

  Interface->CreateDevice( GUID_SysKeyboard, &Keyboard, NULL);
  Keyboard->SetDataFormat(&c_dfDIKeyboard);
  Keyboard->SetCooperativeLevel(w, DISCL_EXCLUSIVE | DISCL_BACKGROUND );
  Keyboard->SetProperty(DIPROP_BUFFERSIZE, &prop.diph);
  Keyboard->Acquire();

  Interface->CreateDevice( GUID_SysMouse, &Mouse, NULL);
  Mouse->SetDataFormat(&c_dfDIMouse);
  Mouse->SetCooperativeLevel(w, DISCL_NONEXCLUSIVE  | DISCL_BACKGROUND);
  Mouse->Acquire();

}


void DInput::Frame() {

  DIDEVICEOBJECTDATA keys[250];
  DWORD k = 250;

  DIMOUSESTATE mstate;
  Mouse->GetDeviceState(sizeof(DIMOUSESTATE), (LPVOID) &mstate);

  if(mstate.rgbButtons[1]) {

    camera->rotatateAroundLocalAxisY(mstate.lX * mouseSensitivity);
    camera->rotatateAroundLocalAxisX(mstate.lY * mouseSensitivity);

  }

  Keyboard->GetDeviceData(sizeof(DIDEVICEOBJECTDATA),keys,&k,0);

  memset(pressedKeysThisFrame, 0, sizeof(bool)*KeyNumber);


  for (unsigned int i = 0; i < k; i++){

    if (!keys[i].dwOfs)
      continue;
    if (keys[i].dwData)  {

      if (!pressedKeys[keys[i].dwOfs])
        pressedKeysThisFrame[keys[i].dwOfs] = true;

      pressedKeys[keys[i].dwOfs] = true;
    }
    else
      pressedKeys[keys[i].dwOfs] = false;

  }

  if (pressedKeys[DIK_W])
    camera->moveAlongLocalAxisZ(1.0f);
  if (pressedKeys[DIK_S])
    camera->moveAlongLocalAxisZ(-1.0f);
  if (pressedKeys[DIK_A])
    camera->moveAlongLocalAxisX(-1.0f);
  if (pressedKeys[DIK_D])
    camera->moveAlongLocalAxisX(1.0f);
  if (pressedKeys[DIK_E])
    camera->moveAlongLocalAxisY(-1.0f);
  if (pressedKeys[DIK_R])
    camera->moveAlongLocalAxisY(1.0f);


  if (pressedKeysThisFrame[DIK_P]) {

    if(framesToAdvance>0)
      framesToAdvance =0;
    else
      framesToAdvance =1;

  }

    if (pressedKeysThisFrame[DIK_1]) {

      //inertiaTensorMultiplier = 1;
      //  winApplication->LoadScene("firebolt\\1.scn");
      //  camera->setTranslation(D3DXVECTOR4(0.0f,50.0f,-150.0f,1.0f));
      //  camera->setRotation(D3DXQUATERNION(0.0f,0.0f,0.0f,1.0f));
      }


    if (pressedKeysThisFrame[DIK_2]) {


        inertiaTensorMultiplier=1;
        winApplication->LoadScene("firebolt\\2.scn");
        camera->setTranslation(D3DXVECTOR4(-70.0f,4.0f,0.7f,1.0f));
        camera->setRotation(D3DXQUATERNION(0.0f,0.5f,0.0f,0.8f));


    }

    if (pressedKeysThisFrame[DIK_3]) {

        inertiaTensorMultiplier=1;
        winApplication->LoadScene("firebolt\\3.scn");
        camera->setTranslation(D3DXVECTOR4(-150.0f,-120.0f,-330.0f,1.0f));
        camera->setRotation(D3DXQUATERNION(0.0f, 0.0f, 0.0f, 1.0f));

    }

    if (pressedKeysThisFrame[DIK_4]) {

        inertiaTensorMultiplier=100;
        winApplication->LoadScene("firebolt\\4.scn");
        camera->setTranslation(D3DXVECTOR4(-3.0f,-100.0f,-250.0f,1.0f));
        camera->setRotation(D3DXQUATERNION(0.0f, 0.0f, 0.0f, 1.0f));

    }

    if (pressedKeysThisFrame[DIK_5]) {

        inertiaTensorMultiplier=100;
        winApplication->LoadScene("firebolt\\5.scn");
        camera->setTranslation(D3DXVECTOR4(-3.0f,-100.0f,-250.0f,1.0f));
        camera->setRotation(D3DXQUATERNION(0.0f, 0.0f, 0.0f, 1.0f));

    }

    if (pressedKeysThisFrame[DIK_6]) {

        inertiaTensorMultiplier = 100;
        winApplication->LoadScene("firebolt\\6.scn");
        camera->setTranslation(D3DXVECTOR4(-110.0f,65.0f,-80.0f,1.0f));
        camera->setRotation(D3DXQUATERNION(0.3f,0.3f,0.0f,0.8f));

    }

    if (pressedKeysThisFrame[DIK_7]) {

        inertiaTensorMultiplier=10;
        winApplication->LoadScene("firebolt\\7.scn");
        camera->setTranslation(D3DXVECTOR4(-110.0f,65.0f,-80.0f,1.0f));
        camera->setRotation(D3DXQUATERNION(0.3f,0.3f,0.0f,0.8f));

    }

}
