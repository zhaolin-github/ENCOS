#include "example_soem/demo_timing.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <errno.h>
#include <time.h>
#endif

enum {
  kNanosecondsPerSecond = 1000000000L,
  kNanosecondsPerMicrosecond = 1000L,
};

DemoTimePoint demo_time_now(void) {
  DemoTimePoint result;
#ifdef _WIN32
  LARGE_INTEGER counter;
  LARGE_INTEGER frequency;
  int64_t whole_seconds;
  int64_t remainder;
  QueryPerformanceCounter(&counter);
  QueryPerformanceFrequency(&frequency);
  whole_seconds = counter.QuadPart / frequency.QuadPart;
  remainder = counter.QuadPart % frequency.QuadPart;
  result.nanoseconds =
      (whole_seconds * kNanosecondsPerSecond) +
      ((remainder * kNanosecondsPerSecond) / frequency.QuadPart);
#else
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  result.nanoseconds =
      ((int64_t)now.tv_sec * kNanosecondsPerSecond) + (int64_t)now.tv_nsec;
#endif
  return result;
}

void demo_time_add_us(DemoTimePoint *time_point, long period_us) {
  if (time_point == 0 || period_us <= 0) {
    return;
  }

  time_point->nanoseconds += (int64_t)period_us * kNanosecondsPerMicrosecond;
}

/**
 * @brief 等待直到下一个周期
 * @param[in,out] next_cycle 下一个周期的时间点，会被更新为下一个周期
 * @param[in] period_us 周期时长（微秒）
 */
void demo_wait_until_next_cycle(DemoTimePoint *next_cycle, long period_us) {
  if (next_cycle == 0 || period_us <= 0) {
    return;
  }

  demo_time_add_us(next_cycle, period_us);

  while (1) {
    const DemoTimePoint now = demo_time_now();
    const int64_t remaining_ns = next_cycle->nanoseconds - now.nanoseconds;
    if (remaining_ns <= 0) {
      return;
    }

#ifdef _WIN32
    if (remaining_ns >= 2000000) {
      Sleep((DWORD)(remaining_ns / 1000000));
    } else {
      Sleep(0);
    }
#else
    {
      struct timespec remaining;
      remaining.tv_sec = (time_t)(remaining_ns / kNanosecondsPerSecond);
      remaining.tv_nsec = (long)(remaining_ns % kNanosecondsPerSecond);
      while (nanosleep(&remaining, &remaining) != 0 && errno == EINTR) {
      }
    }
#endif
  }
}
