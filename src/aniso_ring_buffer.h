#ifndef _ANISO_RING_BUFFER_H
#define _ANISO_RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>

// 通用环形缓冲区结构
struct aniso_ring_buffer_t
{
    int16_t head;      // 头指针，指向最新数据
    uint16_t count;    // 当前数据数量
    uint16_t capacity; // 缓冲区容量
    size_t elem_size;  // 元素大小
    void *buffer;      // 数据缓冲区指针
};

// 环形缓冲区操作函数声明
void aniso_ring_buffer_init(struct aniso_ring_buffer_t *rb, void *buffer, uint16_t capacity, size_t elem_size);
void aniso_ring_buffer_push(struct aniso_ring_buffer_t *rb, const void *data);
int8_t aniso_ring_buffer_pick(struct aniso_ring_buffer_t *rb, int16_t index, void *data);
int8_t aniso_ring_buffer_update(struct aniso_ring_buffer_t *rb, int16_t index, const void *data);
int16_t aniso_ring_buffer_size(const struct aniso_ring_buffer_t *rb);
uint16_t aniso_ring_buffer_capacity(const struct aniso_ring_buffer_t *rb);

#endif // _ANISO_RING_BUFFER_H