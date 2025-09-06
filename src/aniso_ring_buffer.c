#include "aniso_ring_buffer.h"
#include <string.h> //memcpy

// 初始化环形缓冲区
void aniso_ring_buffer_init(struct aniso_ring_buffer_t *rb, void *buffer, uint16_t capacity, size_t elem_size)
{
    rb->head = -1;
    rb->count = 0;
    rb->capacity = capacity;
    rb->elem_size = elem_size;
    rb->buffer = buffer;
}

// 向环形缓冲区添加数据
void aniso_ring_buffer_push(struct aniso_ring_buffer_t *rb, const void *data)
{
    rb->head = (rb->head + 1) % rb->capacity;
    memcpy((char *)rb->buffer + rb->head * rb->elem_size, data, rb->elem_size);
    if (rb->count < rb->capacity)
        rb->count++;
}

// 从环形缓冲区获取数据（仅get，不pop）
int8_t aniso_ring_buffer_pick(struct aniso_ring_buffer_t *rb, int16_t index, void *data)
{
    if (index >= rb->count || index < 0)
        return -1; // 索引超出范围

    int actual_index = (rb->head - index + rb->capacity) % rb->capacity;
    memcpy(data, (char *)rb->buffer + actual_index * rb->elem_size, rb->elem_size);
    return 0;
}

// 更新环形缓冲区中指定位置的数据
int8_t aniso_ring_buffer_update(struct aniso_ring_buffer_t *rb, int16_t index, const void *data)
{
    if (index >= rb->count || index < 0)
    {
        return -1; // 索引超出范围
    }
    int actual_index = (rb->head - index + rb->capacity) % rb->capacity;
    memcpy((char *)rb->buffer + actual_index * rb->elem_size, data, rb->elem_size);
    return 0;
}

// 获取环形缓冲区当前元素数量
int16_t aniso_ring_buffer_size(const struct aniso_ring_buffer_t *rb)
{
    return rb->count;
}

// 获取环形缓冲区容量
uint16_t aniso_ring_buffer_capacity(const struct aniso_ring_buffer_t *rb)
{
    return rb->capacity;
}