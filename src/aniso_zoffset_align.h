#ifndef ANISO_ZOFFSET_ALIGN_H
#define ANISO_ZOFFSET_ALIGN_H

#include <math.h>              // sqrtf
#include <stdint.h>
#include "autoconf.h"          // CONFIG_CLOCK_FREQ
#include "board/irq.h" // irq_disable
#include "board/gpio.h" // gpio_out_write
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "spicmds.h" // spidev_transfer
#include <stdint.h>
#include <string.h>
#include "aniso_ring_buffer.h" // aniso_ring_buffer_t
#include "aniso_filter.h"      // aniso_kalman_filter_t
#include "stm32/debug/debug.h"
#include "stm32/debug/printf.h"
#include "trsync.h" // trsync_do_trigger

#define ADS131M08_DATA 0x00
#define ADS131M08_CONFIG 0x01
#define ADS131M08_MODE 0x02
#define ADS131M08_CLOCK 0x03
#define ADS131M08_GAIN1 0x04
#define ADS131M08_GAIN2 0x05
#define ADS131M08_CFG 0x06
#define ADS131M08_THRSHLD_MSB 0x07
#define ADS131M08_THRSHLD_LSB 0x08

#define ADS131M08_CH0_CFG 0x09
#define ADS131M08_CH1_CFG 0x0E
#define ADS131M08_CH2_CFG 0x13
#define ADS131M08_CH3_CFG 0x18
#define ADS131M08_CH4_CFG 0x1D
#define ADS131M08_CH5_CFG 0x22
#define ADS131M08_CH6_CFG 0x27
#define ADS131M08_CH7_CFG 0x2C

#define ADS131M08_MODE_HR 0x02  // High-resolution mode
#define ADS131M08_MODE_LP 0x01 // Low-power mode
#define ADS131M08_MODE_VLP 0x00 // Very-low-power mode

#define ADS131M08_GAIN_1 0x00
#define ADS131M08_GAIN_2 0x01
#define ADS131M08_GAIN_4 0x02
#define ADS131M08_GAIN_8 0x03
#define ADS131M08_GAIN_16 0x04
#define ADS131M08_GAIN_32 0x05
#define ADS131M08_GAIN_64 0x06
#define ADS131M08_GAIN_128 0x07

#define ADS131M08_OSR_128 32000
#define ADS131M08_OSR_245 16000
#define ADS131M08_OSR_512 8000
#define ADS131M08_OSR_1024 4000
#define ADS131M08_OSR_2048 2000
#define ADS131M08_OSR_4096 1000
#define ADS131M08_OSR_8192 500
#define ADS131M08_OSR_16384 250

// 状态机状态定义
enum aniso_zoffset_align_t
{
    ANISO_ZOFFSET_STATE_IDLE,      // 待机阶段
    ANISO_ZOFFSET_STATE_INIT,      // 初始阶段
    ANISO_ZOFFSET_STATE_PREHEAT,   // 预热阶段
    ANISO_ZOFFSET_STATE_CHECK,     // 检查阶段
    ANISO_ZOFFSET_STATE_STOP,      // 停止阶段
    ANISO_ZOFFSET_STATE_MOVING,    // Z轴移动阶段
    ANISO_ZOFFSET_STATE_TRIGGERED  // 触发停止阶段
};

struct ads131m08_adc {
    struct timer timer;
    uint32_t rest_ticks;
    uint32_t default_rest_ticks; // Store default sampling interval
    struct gpio_in data_ready;
    struct spidev_s *spi;
    uint8_t pending_flag, data_count;
    uint8_t enabled_channel; // Bitmask for enabled channels
    struct sensor_bulk sb;
    struct trsync *ts;
};

// 单次采样数据内容
struct aniso_ads131m08_data_t
{
    uint8_t trigger; // 触发标志
    int32_t step;    // 电机步数
    uint32_t tick;   // 时间戳
    float raw_val;   // adc原始值
    float flt_val;   // adc滤波值
};

// 全局状态变量
extern uint8_t state_zoffset_flag;
extern float trigger_position;
extern int8_t z_moving_direction;
extern uint8_t trigger_detected;

// 函数声明
void command_start_zoffset_align(uint32_t *args);
void command_stop_zoffset_align(uint32_t *args);
void command_get_zoffset_position(uint32_t *args);
uint8_t check_trigger(void);
void handle_trigger_stop(void);

#endif // ANISO_ZOFFSET_ALIGN_H
