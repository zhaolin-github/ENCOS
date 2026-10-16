#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 跨平台单调时钟时间点（纳秒） */
typedef struct {
  int64_t nanoseconds;
} DemoTimePoint;

/** @brief 获取当前单调时钟时间 */
DemoTimePoint demo_time_now(void);

/** @brief 将时间点增加指定的微秒数 */
void demo_time_add_us(DemoTimePoint *time_point, long period_us);

/**
 * @brief 等待直到下一个周期
 * @param[in,out] next_cycle 下一个周期的时间点，会被更新为下一个周期
 * @param[in] period_us 周期时长（微秒）
 */
void demo_wait_until_next_cycle(DemoTimePoint *next_cycle, long period_us);

#ifdef __cplusplus
}
#endif
