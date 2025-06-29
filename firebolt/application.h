#ifndef APPLICATION_H
#define APPLICATION_H

#include <windows.h>


class WinApplication {

public:

  WinApplication(HINSTANCE, int);
  ~WinApplication();

  void Frame(double);

  void LoadScene(char* fileName);
  void UnloadScene();

private:

  HINSTANCE hInstance;
  HWND hWnd;

};


class LogicTimer {

public:

  LogicTimer(unsigned int logicFreq);

  void Frame(double);

private:

  unsigned __int64 frameNumber;
  unsigned __int64 logicFrameNumber;

  unsigned int logicFrequency;
  double logicFrameDuration;
  double sinceLastLogicFrame;
};

extern WinApplication *winApplication;
extern LogicTimer *logicTimer;


#endif
