#ifndef TIME_WHEEL_H
#define TIME_WHEEL_H

#define WHEEL_SIZE    8    // 时间轮槽数
#define MAX_TASKS     16   // 最大同时存在的任务数
#define TICK_MS       1  // 滴答周期

typedef void (*task_cb)(void* arg);

// 任务节点
typedef struct TaskNode {
    uint8_t  priority;     // 优先级
    uint32_t rounds;       // 剩余圈数
    task_cb  callback;
    void* arg;
    int16_t  next;         // 使用数组下标作为指针，-1 表示 NULL
} TaskNode;

// 静态时间轮结构
typedef struct {
    TaskNode task_pool[MAX_TASKS]; // 静态任务池
    int16_t  free_head;            // 空闲链表头
    int16_t  slots[WHEEL_SIZE];    // 时间轮槽位，存储任务下标
    uint16_t current_pos;
} TimeWheel;

void wheel_init(TimeWheel* wheel);
int add_task(TimeWheel* wheel, uint32_t delay_ms, uint8_t priority, task_cb cb, void* arg);
void wheel_tick(TimeWheel* wheel);
#endif