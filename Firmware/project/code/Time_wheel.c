#include <stdio.h>
#include <stdint.h>
#include "Time_wheel.h"



// 1. 初始化时间轮
void wheel_init(TimeWheel* wheel) {
    wheel->current_pos = 0;
    wheel->free_head = 0;

    // 初始化槽位为空
    for (int i = 0; i < WHEEL_SIZE; i++) wheel->slots[i] = -1;

    // 初始化空闲链表：将所有节点串起来
    for (int i = 0; i < MAX_TASKS - 1; i++) {
        wheel->task_pool[i].next = i + 1;
    }
    wheel->task_pool[MAX_TASKS - 1].next = -1; // 最后一个节点
}

// 2. 添加任务
int add_task(TimeWheel* wheel, uint32_t delay_ms, uint8_t priority, task_cb cb, void* arg) {
    if (wheel->free_head == -1) return -1; // 内存池已满

    // 从空闲链表取出一个节点
    int16_t new_idx = wheel->free_head;
    wheel->free_head = wheel->task_pool[new_idx].next;

    // 配置任务
    uint32_t ticks = delay_ms / TICK_MS;
    wheel->task_pool[new_idx].rounds = ticks / WHEEL_SIZE;
    wheel->task_pool[new_idx].priority = priority;
    wheel->task_pool[new_idx].callback = cb;
    wheel->task_pool[new_idx].arg = arg;

    int slot_idx = (wheel->current_pos + ticks) % WHEEL_SIZE;

    // 优先级排序插入 (降序)
    int16_t* p_curr = &wheel->slots[slot_idx];
    while (*p_curr != -1 && wheel->task_pool[*p_curr].priority >= priority) {
        p_curr = &wheel->task_pool[*p_curr].next;
    }
    
    wheel->task_pool[new_idx].next = *p_curr;
    *p_curr = new_idx;

    return 0;
}

// 3. 时间轮轮询
void wheel_tick(TimeWheel* wheel) {
    int16_t* p_curr = &wheel->slots[wheel->current_pos];
    
    while (*p_curr != -1) {
        TaskNode* task = &wheel->task_pool[*p_curr];

        if (task->rounds == 0) {
            // 执行任务
            if (task->callback) task->callback(task->arg);

            // 释放节点：从当前槽位移除，还给空闲链表
            int16_t next_idx = task->next;
            task->next = wheel->free_head;
            wheel->free_head = *p_curr;
            *p_curr = next_idx;
        } else {
            task->rounds--;
            p_curr = &task->next;
        }
    }

    wheel->current_pos = (wheel->current_pos + 1) % WHEEL_SIZE;
}
