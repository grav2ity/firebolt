#include "timer.h"


Timer* timer;


Timer::Timer() {

  QueryPerformanceFrequency((LARGE_INTEGER*)&frequency);
  QueryPerformanceCounter((LARGE_INTEGER*)&start);
  ticksToSecond = (double)1.0 / (double)frequency;

  lastFrameTime = time();

}

double Timer::time() {

  QueryPerformanceCounter((LARGE_INTEGER*)&temp);
  return ((double(temp - start)) * ticksToSecond);

}

double Timer::timeDelta() {

  double t = time() - lastFrameTime;
  lastFrameTime = time();
  return t;

}

void Timer::reset() {

  lastFrameTime = time();

}

double Timer::windowsTime() {

  QueryPerformanceCounter((LARGE_INTEGER*)&temp);
  return (float(temp) * ticksToSecond);

}

void  Timer::startCycle(char* name, double duration) {

  cycles[name] = new cycle_();
  cycles[name]->start = time();
  cycles[name]->duration = duration;

}

double Timer::cycle(char *name) {

  int n = (int)((time() - cycles[name]->start) / cycles[name]->duration);
  double r = (time() - cycles[name]->start) - n * cycles[name]->duration;
  return r / cycles[name]->duration;

}

