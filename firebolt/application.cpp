#include "application.h"

#include "body.h"
#include "direct3d.h"
#define DIRECTINPUT_VERSION 0x0800
#include "directinput.h"
#include "dmesh.h"
#include "file.h"
#include "timer.h"


extern unsigned int framesToAdvance;
extern double inertiaTensorMultiplier;

WinApplication *winApplication;
LogicTimer *logicTimer;

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

WinApplication::WinApplication(HINSTANCE hInstance, int iCmdShow) {

  WNDCLASS wc;
  wc.style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS;
  wc.lpfnWndProc = WndProc;
  wc.cbClsExtra = 0;
  wc.cbWndExtra = 0;
  wc.hInstance = hInstance;
  wc.hIcon = ::LoadIcon(0, IDI_APPLICATION);
  wc.hCursor = ::LoadCursor(0, IDC_ARROW);
  wc.hbrBackground = NULL;
  wc.lpszMenuName = NULL;
  wc.lpszClassName = L"firebolt";

  RegisterClass(&wc);

  hWnd = CreateWindow(
    L"firebolt",
    L"firebolt",
    WS_OVERLAPPEDWINDOW,
    CW_USEDEFAULT,
    CW_USEDEFAULT,
    CW_USEDEFAULT,
    CW_USEDEFAULT,
    0,
    0,
    hInstance,
    0);

  ShowWindow(hWnd, iCmdShow);
  UpdateWindow(hWnd);


  timer = new Timer();
  logicTimer = new LogicTimer(120);
  dInput = new DInput(hInstance, hWnd);
  collisionManager = new CollisionManager();
  bodyManager = new BodyManager();
  direct3D = new Direct3D(hWnd);


  inertiaTensorMultiplier = 1;
  winApplication->LoadScene("firebolt\\2.scn");
  camera->setTranslation(D3DXVECTOR4(-70.0f, 4.0f, 0.7f, 1.0f));
  camera->setRotation(D3DXQUATERNION(0.0f, 0.5f, 0.0f, 0.8f));

}

WinApplication::~WinApplication() {

  delete direct3D;
  delete bodyManager;
  delete collisionManager;
  delete dInput;
  delete logicTimer;
  delete timer;

}

void WinApplication::LoadScene(char* filename) {

  UnloadScene();
  sceneReader.readScene(filename);
  collisionManager->BuildConstraintGroups();
  direct3D->SetupStaticBuffers();

}

void WinApplication::UnloadScene() {

  bodyManager->Reset();
  collisionManager->Reset();
  dmeshManager.Reset();
  direct3D->ReleaseStaticBuffers();

}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {

  switch (message) {

    case WM_DESTROY :
      PostQuitMessage(0);
      break;

    case WM_KEYDOWN :
      switch (wParam) {

      case VK_ESCAPE:
        DestroyWindow(hWnd);
        break;

      default:
        ;
      }

    case WM_MOUSEMOVE:
      direct3D->setMouseWindowX((LOWORD(lParam)));
      direct3D->setMouseWindowY((HIWORD(lParam)));
      break;

    default :
      return DefWindowProc(hWnd, message, wParam, lParam);

  }

  return 0;

}

void WinApplication::Frame(double timeDelta) {

  logicTimer->Frame(timeDelta);

}


LogicTimer::LogicTimer(unsigned int logicFreq) :
  logicFrequency(logicFreq), logicFrameDuration(1.0 / (double)(logicFreq)), sinceLastLogicFrame(0.0),
  frameNumber(0), logicFrameNumber(0)
{}



void LogicTimer::Frame(double timeDelta) {

  if ((sinceLastLogicFrame + timeDelta) >= logicFrameDuration) {

    sinceLastLogicFrame = (sinceLastLogicFrame + timeDelta) - logicFrameDuration;

    dInput->Frame();

    if (framesToAdvance > 0) {

      bodyManager->Frame(logicFrameDuration);
      collisionManager->Frame(logicFrameDuration);
      direct3D->Frame(logicFrameDuration);

    }
    else
      direct3D->Frame(logicFrameDuration);

  }
  else
    sinceLastLogicFrame += timeDelta;

}
