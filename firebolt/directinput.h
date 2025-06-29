#ifndef DIRECTINPUT_H
#define DIRECTINPUT_H

#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>

#include <windows.h>


#define DIINTERFACE IDirectInput8
#define DICREATE DirectInput8Create
#define DIDEVICE IDirectInputDevice8


class DInput  {

public:

  DInput(HINSTANCE, HWND);

  void Frame();

private:

  DIINTERFACE* Interface;
  DIDEVICE* Keyboard;
  DIDEVICE* Mouse;

  static const int KeyNumber = 256;

  bool pressedKeys[KeyNumber];
  bool pressedKeysThisFrame[KeyNumber];


};

extern DInput *dInput;


#endif
