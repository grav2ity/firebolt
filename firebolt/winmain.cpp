#include "application.h"
#include "timer.h"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR szCmdLine, int iShowCmd) {

  winApplication = new WinApplication(hInstance, iShowCmd);

  MSG msg;
  ZeroMemory(&msg, sizeof(MSG));

  while(msg.message != WM_QUIT) {

    if(PeekMessage(&msg, 0, 0, 0, PM_REMOVE)) {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }
    else {
      winApplication->Frame(timer->timeDelta());
    }

  }

  return (int)msg.wParam;

}
