// readme
// 20250911:
// 1. 使用STM32H7的ADC3作为电压采集转换，理论精度12位，转换结果合法取值[0, 4095]
// 2. gpio_adc_sample 手动开启一次转换，每次转换后需等待20us才覆盖采样结果
//    暂时处理为提前一个周期触发转换，下一周期再通过gpio_adc_read读取数据
//    导致问题：
//    (1) 数据至少晚了一个周期
//    (2) 读取频率理论上限 freq = (1s) / (20us)
//    解决方案：
//    (1) 开启连续转换，尝试过了，会导致mcu繁忙，与上位机通信超时
//    (2) 连续转换+DMA，或单次转换+定时器，后续有嵌入式能力强的同事，可以尝试
//
// 20251013:
// 1. 暂时使用遍历法计算最值，单个采样周期，缓存只出入各一组数据，理论上存在更高效的最值计算
// 2. 期望的预热阶段是刚好映射电机加速段
//*************************************************************************

#include <math.h>              // sqrtf
#include <string.h>            // memcpy memset
#include "autoconf.h"          // CONFIG_CLOCK_FREQ
#include "board/gpio.h"        // gpio_adc_read
#include "board/misc.h"        // timer_read_time
#include "command.h"           // DECL_CONSTANT_STR
#include "aniso_ring_buffer.h" // aniso_ring_buffer_t
#include "aniso_filter.h"      // aniso_kalman_filter_t
#include "aniso_bed_mesh.h"
#include "board/io.h"         // writel
#include "board/usb_cdc.h"    // usb_notify_ep0
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "stm32/internal.h"   // GPIO
#include "sched.h"            // DECL_INIT
#include "stm32/gpio.h"
#include "stm32/debug/debug.h"
#include "stm32/debug/printf.h"

// 变量声明
#define BUFFER_VALUE_CNT 120                          // 采样缓存的容量
#define ALIGN_VALUE_CNT 60                            // 触发判断所需的最少缓存数量
#define CRITICAL_VALUE_CNT 30                         // 回溯临界拐点的容量
#define SAMPLE_INTERVAL_US 200                        // 采样间隔200微秒
#define SAMPLE_FREQUENCY_HZ (1000000/SAMPLE_INTERVAL_US) // 采样频率5000Hz
#define TRIGGER_COUNT_MAX 7                       // 触发计数最大值

static uint8_t log_flag = 0;                              // 0 数据上报关闭；1 数据上报开启
static int32_t sample_step_acc;                       // 加速段对应电机微步数
static int32_t sample_step_start;                     // 起始电机微步数
static uint32_t sample_cycle_now;                     // 当前采样周期次数
static uint32_t critical_tick = 1;                    // 临界拐点的时间戳
static enum aniso_state_machine_t state_machine_flag; // 状态机标志位
static float sample_freq = 1.0f;                      // 采样频率
static float sample_val_mean;                         // 采样均值
static struct gpio_adc gPF;                           // adc引脚通道
static struct gpio_out triggerPF;                     // 触发指示引脚
static struct aniso_kalman_filter_t kmf;              // 卡尔曼滤波器
static struct aniso_iir_filter_t bsf;                 // 陷波(带阻)滤波器
// static struct aniso_iir_filter_t hpf;                             // 高通滤波器
static struct aniso_iir_filter_t lpf;                 // 低通滤波器
static struct aniso_adc_data_t sample_datas_ar[BUFFER_VALUE_CNT]; // 采样缓存array
static struct aniso_ring_buffer_t sample_datas_rb;                // 采样缓存ring_buffer
static struct timer sample_timer;                     // 采样定时器
volatile float adc_val_min = INFINITY; // adc触发的负向阈值
volatile float adc_val_max = -INFINITY; // adc触发的正向阈值
volatile uint16_t trigger1_count = 0;
volatile int16_t printed_count = -1;

// 采样频率计算
void calc_sample_freq(uint32_t tick)
{
    // 参考自clocksync.py clock_to_print_time()
    // sample_time = sample_tick/mcu_freq
    // sample_freq = 1 / sample_time = mcu_freq / sample_tick
    sample_freq = 1.0f;
    if (tick != 0)
    {
        sample_freq = (float)CONFIG_CLOCK_FREQ / tick;
        if (sample_freq <= 0.0f)
            sample_freq = 1.0f;
    }
}

// 触发判断 阈值法
uint8_t get_trigger1(void)
{
    struct aniso_adc_data_t temp_data;
    for (int16_t i = ALIGN_VALUE_CNT; i < BUFFER_VALUE_CNT; i++) // 计算最值（缓存的一半旧数据）
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data);
        adc_val_max = fmaxf(adc_val_max, temp_data.flt_val);
        adc_val_min = fminf(adc_val_min, temp_data.flt_val);
    }
    float max_factor = 0.1f;
    float min_factor = 0.1f;
    float val_max = adc_val_max + adc_val_max*max_factor;
    float val_min = adc_val_min - adc_val_min*min_factor;

    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data); // 本周期最新数据
    if (temp_data.flt_val > val_max ||
        temp_data.flt_val < val_min) // 超出动态阈值，才算本周期有触发
    {
        return 1;
    }
    return  0;
}

// 临界回溯 线性拟合法
uint32_t get_critical3(void)
{
    struct aniso_adc_data_t temp_data;
    aniso_ring_buffer_pick(&sample_datas_rb, CRITICAL_VALUE_CNT - 1, &temp_data); // 最旧缓存数据
    float ye = temp_data.flt_val;                                               // y终止拟合值，对应x=end
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data);                    // 最新缓存数据
    float ys = temp_data.flt_val;                                               // y起始拟合值，对应x=0
    float k = (ye - ys) / (CRITICAL_VALUE_CNT - 1);                             // 斜率k=(yend-ystart)/(xend-xstart)
    critical_tick = temp_data.tick;                                             // 默认返回最新时间戳

    // 拐点检测 遍历找到纵向距离偏差最大点
    float err_max = 0.0f;
    for (int16_t i = 1; i < CRITICAL_VALUE_CNT; i++) // 首尾两点偏差一定是0，不必遍历
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data);
        float yi = ys + i * k;                     // y期望
        float err = fabsf(temp_data.flt_val - yi); // y实际与y期望的偏差
        if (err_max < err)                         // 偏差最大的点，即为拐点
        {
            err_max = err;
            critical_tick = temp_data.tick;
        }
    }

    return critical_tick;
}

// 定时器事件处理函数
static uint_fast8_t
sample_adc_event(struct timer *timer)
{
    if (state_machine_flag != ANISO_STATE_IDLE)
    {
        aniso_adc_sample(ANISO_STATE_PLACEHOLD);
    }
    
    // 重新调度定时器，固定间隔采样
    timer->waketime += timer_from_us(SAMPLE_INTERVAL_US);
    return SF_RESCHEDULE;
}

// shutdown清理函数
void
aniso_bed_mesh_shutdown(void)
{
    sched_del_timer(&sample_timer);
}
DECL_SHUTDOWN(aniso_bed_mesh_shutdown);

// 采样任务状态机
uint8_t aniso_adc_sample(enum aniso_state_machine_t smf)
{
    struct aniso_adc_data_t sample_data = {0};
    // c语言不支持参数默认值，通过placehold来区分强制赋值和默认赋值
    if (smf != ANISO_STATE_PLACEHOLD)
        state_machine_flag = smf;

    switch (state_machine_flag)
    {
    // 初始化阶段：状态复位
    case ANISO_STATE_INIT:
    {
        sample_val_mean = 0.0f;
        sample_cycle_now = 0;
        // 过滤触发后，产生振荡的异常数据，以300ms振荡为例，0.5*(1000*1000/SAMPLE_INTERVAL_US)=2500个采样点
        sample_step_acc = 0.5*SAMPLE_FREQUENCY_HZ;
        sample_freq = SAMPLE_FREQUENCY_HZ;
        sample_step_start = 0;
        critical_tick = 1;
        gpio_adc_sample(gPF); // 启动下一周期adc采样转换
        memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
        aniso_ring_buffer_init(&sample_datas_rb, sample_datas_ar, BUFFER_VALUE_CNT, sizeof(struct aniso_adc_data_t));
        aniso_kalman_filter_init(&kmf, 0.2f, 0.03f, 0.2f);
        aniso_iir_filter_init(&bsf, ANISO_FILTER_BAND_STOP, 50.0f, 10.0f, sample_freq);
        // aniso_iir_filter_init(&hpf, ANISO_FILTER_HIGH_PASS, 5.0f, 0.707f, sample_freq);
        aniso_iir_filter_init(&lpf, ANISO_FILTER_LOW_PASS, 3000.0f, 0.707f, sample_freq);
        state_machine_flag = ANISO_STATE_PREHEAT;
        adc_val_min = INFINITY;
        adc_val_max = -INFINITY;
        trigger1_count = 0;
    }
    break;

    // 预热阶段：对应寻位运动的加速段，计算直流分量（采样均值）
    case ANISO_STATE_PREHEAT:
    {
        sample_step_start++;
        sample_data.raw_val = (float)gpio_adc_read(gPF); // 读取上一周期adc转换数据
        gpio_adc_sample(gPF);                            // 启动下一周期adc采样转换
        sample_cycle_now++;
        sample_val_mean += (sample_data.raw_val - sample_val_mean) / sample_cycle_now; // 在线计算历史均值
        sample_data.raw_val -= sample_val_mean;                // 去除直流分量
        if (sample_step_start >= sample_step_acc)
        {
            sample_data.flt_val = sample_data.raw_val;                                     // 数据预填充，避免下阶段起始无数据判断
            aniso_ring_buffer_push(&sample_datas_rb, &sample_data);  
            adc_val_max = fmaxf(adc_val_max, sample_data.flt_val);
            adc_val_min = fminf(adc_val_min, sample_data.flt_val);                      // 压入缓存
            if (aniso_ring_buffer_size(&sample_datas_rb) >= BUFFER_VALUE_CNT)
            {
                sample_step_acc = 0;
                sample_step_start = 0;
                state_machine_flag = ANISO_STATE_SAMPLE;
            }
        }
    }
    break;

    // 采样阶段：对应寻位运动的匀速段，判断触发标志
    case ANISO_STATE_SAMPLE:
    {
        sample_data.raw_val = (float)gpio_adc_read(gPF) - sample_val_mean; // 读取上一周期adc转换数据（同时去除直流分量）
        gpio_adc_sample(gPF);                                       // 启动下一周期adc采样转换
        // // sample_data.flt_val = aniso_iir_filter_process(&hpf, sample_data.raw_val);
        sample_data.flt_val = aniso_iir_filter_process(&lpf, sample_data.raw_val);
        sample_data.flt_val = aniso_iir_filter_process(&bsf, sample_data.flt_val);
        sample_data.flt_val = aniso_kalman_filter_process(&kmf, sample_data.flt_val);
        aniso_ring_buffer_push(&sample_datas_rb, &sample_data); // 更新采样缓存（必须先push，判断trigger需要读缓存）

        sample_data.trigger = get_trigger1();                // 判断触发标志
        if (sample_data.trigger == 1)
        {
            aniso_ring_buffer_update(&sample_datas_rb, 0, &sample_data);
            trigger1_count++;
        }
        else {
            trigger1_count = 0;
        }

        if (trigger1_count > TRIGGER_COUNT_MAX) {
            trigger1_count = 0;
            gpio_out_write_now(triggerPF, 0);
            state_machine_flag = ANISO_STATE_STOP;
        }
    }
    break;

    // 停止阶段
    case ANISO_STATE_STOP:
    {
        gpio_out_write_now(triggerPF, 1);
        memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
        state_machine_flag = ANISO_STATE_IDLE;
        sched_del_timer(&sample_timer);
        printed_count = 0;
    }
    break;

    default:
        break;
    }

    return sample_data.trigger;
}

void sample_task(void)
{
    if (state_machine_flag == ANISO_STATE_STOP)
    {        
        if (printed_count >= 0 && printed_count < BUFFER_VALUE_CNT) {
            struct aniso_adc_data_t temp_data;
            aniso_ring_buffer_pick(&sample_datas_rb, printed_count, &temp_data);
            printf_("AEI_FLT=%d RAW_VAL=%d max=%d min=%d\r\n", (int32_t)(temp_data.flt_val * 1000), (int32_t)(temp_data.raw_val * 1000), (int32_t)(adc_val_max*1000), (int32_t)(adc_val_min*1000));
            printed_count++;
        }

        if (printed_count >= BUFFER_VALUE_CNT) {
            printed_count = -1; // 打印完成，重置为-1，避免重复打印
            memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
            state_machine_flag = ANISO_STATE_IDLE;
        }
    }
}
// DECL_TASK(sample_task);

void command_config_aniso_bed_mesh(uint32_t *args)
{
    // uint8_t goid = (uint8_t)args[0];
    uint32_t adc_pin = args[1];
    uint32_t trigger_pin = args[2];
    gPF = gpio_adc_setup(adc_pin);
    triggerPF = gpio_out_setup(trigger_pin, 1);
    sample_timer.func = sample_adc_event;

    state_machine_flag = ANISO_STATE_IDLE;
    // printf_("aniso_bed_mesh configuring oid=%u op=%u adc_pin=%u trigger_pin=%u\r\n", goid, 0, adc_pin, trigger_pin);
}
DECL_COMMAND(command_config_aniso_bed_mesh, "config_aniso_bed_mesh oid=%c adc_pin=%u trigger_pin=%u");

void command_setup_aniso_bed_mesh(uint32_t *args)
{
    uint8_t goid = (uint8_t)args[0];
    switch (args[1]) // 功能码
    {
    case 1: // 设置工作模式
        printf_("setup_aniso_bed_mesh oid=%u\r\n", goid);
        gpio_out_write_now(triggerPF, 1);
        state_machine_flag = ANISO_STATE_INIT;
        memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
        trigger1_count = 0;
        sched_del_timer(&sample_timer);
        sample_timer.waketime = timer_read_time() + timer_from_us(SAMPLE_INTERVAL_US);
        sched_add_timer(&sample_timer);
        break;

    case 2: // 获取寻位运动的加速段对应的电机微步数
        sample_step_acc = args[2];
        break;

    case 3: // 设置广播标志位
        log_flag = (uint8_t)args[2];
        break;
    }
}
DECL_COMMAND(command_setup_aniso_bed_mesh, "setup_aniso_bed_mesh oid=%u op=%u val=%u");
