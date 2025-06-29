#ifndef TIMER_H
#define TIMER_H

#include <windows.h>

#include <map>


class Timer {

public:

  Timer();
  double time();
  double timeDelta();

  double windowsTime();
  void reset();

  void startCycle(char*, double);
  double cycle(char*);

private:

  _int64 frequency;
  _int64 start;
  double ticksToSecond;

  _int64 temp;

  double lastFrameTime;

  class cycle_ {

  public:

    double start;
    double duration;

  };

  std::map<char*, cycle_*> cycles;

};


extern Timer* timer;


#endif

