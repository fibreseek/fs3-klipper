#ifndef _ANISO_BED_MESH_H
#define _ANISO_BED_MESH_H

#include <stdint.h>

// 状态机状态定义
enum aniso_state_machine_t
{
    ANISO_STATE_IDLE,      // 待机阶段
    ANISO_STATE_INIT,      // 初始阶段
    ANISO_STATE_PREHEAT,   // 预热阶段
    ANISO_STATE_SAMPLE,    // 采样阶段
    ANISO_STATE_STOP,      // 停止阶段
    ANISO_STATE_PLACEHOLD, // 缺省占位（自动切换）
};

// 单次采样数据内容
struct aniso_adc_data_t
{
    uint8_t trigger; // 触发标志
    int32_t step;    // 电机步数
    uint32_t tick;   // 时间戳
    float raw_val;   // adc原始值
    float flt_val;   // adc滤波值
};

// 外部函数传递
extern int32_t get_step_count_z(void);

// 函数声明
void calc_sample_freq(uint32_t tick);
void command_setup_aniso_bed_mesh(uint32_t *args);
void command_config_aniso_bed_mesh(uint32_t *args);
uint8_t aniso_adc_sample(enum aniso_state_machine_t smf);
uint8_t get_trigger1(void);
uint32_t get_critical3(void);

// 出于封装性考虑，将变量声明放在源文件中

#endif // _ANISO_BED_MESH_H