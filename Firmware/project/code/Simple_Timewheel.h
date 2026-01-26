#ifndef SIMPLE_TIMEWHEEL_H
#define SIMPLE_TIMEWHEEL_H

#include <stdint.h>
#include <stdio.h>

#define MAX_TASKS 10

// 1. 先定义类型
typedef void (*TaskFunc)(void);

typedef struct {
    TaskFunc func;     // 任务函数指针
    uint16_t interval; // 任务周期
    uint16_t timer;    // 倒计时器
} TimeWheel_Task;

// 2. 用 extern 声明变量，实际定义放在 .c 文件中
extern TimeWheel_Task task_list[MAX_TASKS];
extern volatile uint8_t tick_flag;

// 3. 函数声明
void add_task(uint16_t ms, void (*f)(void));
void time_wheel_run(void);

#endif