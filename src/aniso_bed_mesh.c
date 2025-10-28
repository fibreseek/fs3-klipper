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
#define BUFFER_VALUE_CNT 100                          // 采样缓存的容量
#define ALIGN_VALUE_CNT 50                            // 触发判断所需的最少缓存数量
#define ALIGN_VALUE_CNT_TRIGGER2 50                 // 触发判断所需的最少缓存数量
#define CRITICAL_VALUE_CNT 30                         // 回溯临界拐点的容量
#define SAMPLE_INTERVAL_US 200                         // 采样间隔200微秒

static uint8_t log_flag = 0;                              // 0 数据上报关闭；1 数据上报开启
static uint8_t sample_mode_flag = 0;                  // 0 adc采样+触发判断；1 adc采样(调试用)
static int32_t sample_step_acc;                       // 加速段对应电机微步数
static int32_t sample_step_start;                     // 起始电机微步数
static uint32_t sample_cycle_now;                     // 当前采样周期次数
static uint32_t critical_tick = 1;                    // 临界拐点的时间戳
static enum aniso_state_machine_t state_machine_flag; // 状态机标志位
static float sample_freq = 1.0f;                      // 采样频率
static float sample_val_mean;                         // 采样均值
static struct gpio_adc gPF;                           // adc引脚通道
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
    float val_max = adc_val_max, val_min = adc_val_min;
    float max_factor = 1.1f;
    float min_factor = 1.1f;

    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data); // 本周期最新数据
    if (temp_data.flt_val > val_max*max_factor ||
        temp_data.flt_val < val_min*min_factor) // 超出动态阈值，才算本周期有触发
    {
        return 1;
    }

    return  0;
}

// 触发判断 斜率法
uint8_t get_trigger2(void)
{
    // 1.检测末尾3点是否有序递增或递减
    struct aniso_adc_data_t temp_data1, temp_data2, temp_data3, temp_data4, temp_data5;
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data1);
    aniso_ring_buffer_pick(&sample_datas_rb, 1, &temp_data2);
    aniso_ring_buffer_pick(&sample_datas_rb, 2, &temp_data3);
    aniso_ring_buffer_pick(&sample_datas_rb, 3, &temp_data4);
    aniso_ring_buffer_pick(&sample_datas_rb, 4, &temp_data5);

    uint8_t flag; // 单调性标志位
    if (temp_data1.flt_val > temp_data2.flt_val && temp_data2.flt_val > temp_data3.flt_val && temp_data3.flt_val > temp_data4.flt_val && temp_data4.flt_val > temp_data5.flt_val)
        flag = 1; // 单调递增
    else if (temp_data1.flt_val < temp_data2.flt_val && temp_data2.flt_val < temp_data3.flt_val && temp_data3.flt_val < temp_data4.flt_val && temp_data4.flt_val < temp_data5.flt_val)
        flag = 0; // 单调递减
    else
        return 0; // 非单调变化

    // 2.检测末尾3点是否均大于或小于其它任意点
    uint16_t i = 0;
    uint16_t icount = ALIGN_VALUE_CNT_TRIGGER2;
    float val_max = adc_val_max, val_min = adc_val_min;

    // 3.对数据进行归一化，方便处理
    float val_range = val_max - val_min;
    uint16_t pass_cnt_now = 0, pass_cnt_max = icount * 0.8; // 允许最多10%缓存点通不过斜率检查
    float angele_threshold = 1.0f; // 触发斜率阈值，tan(45°)=1.0，tan(40°)=0.8391，tan(75°)=3.73，tan(80°)=5.67
    if (!flag) pass_cnt_max = icount * 0.5; // 递减时，允许更多点通不过斜率检查
    if (!flag) angele_threshold = 0.75f; // 递减时，斜率阈值放宽一些
    for (i = icount - 1; i > 0; i--)                       // 数据越旧，斜率越小，倒序可提高筛选效率
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data2);
        float norm_y = fabsf(temp_data1.flt_val - temp_data2.flt_val) / val_range; // tan角的对边归一化
        float norm_x = (float)i / (icount - 1); // tan角的临边归一化

        // 4.比较各点针对最后1点的斜率是否超过35度
        if (norm_y / norm_x > angele_threshold) // tan = 对边/临边，tan(45°)=1.0，tan(40°)=0.8391，tan(75°)=3.73，tan(80°)=5.67
        {
            pass_cnt_now++;
        }
    }

    return (pass_cnt_now >= pass_cnt_max) ? 1 : 0;
}

// 临界回溯 趋势逆反法
uint32_t get_critical1(void)
{
    struct aniso_adc_data_t temp_data1, temp_data2;
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data1);                    // 最新缓存数据
    aniso_ring_buffer_pick(&sample_datas_rb, CRITICAL_VALUE_CNT - 1, &temp_data2); // 最旧缓存数据

    critical_tick = temp_data1.tick;
    uint8_t flag = (temp_data1.flt_val >= temp_data2.flt_val) ? 1 : 0; // 1整体趋势↑ 0整体趋势↓

    for (int16_t i = 1; i < CRITICAL_VALUE_CNT; i++)
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data2);

        // 若数据整体趋势和局部趋势反向，即为拐点
        if ((flag == 1 && temp_data1.flt_val < temp_data2.flt_val) ||
            (flag == 0 && temp_data1.flt_val > temp_data2.flt_val))
        {
            critical_tick = temp_data1.tick;
            break;
        }

        temp_data1 = temp_data2;
    }

    return critical_tick;
}

// 临界回溯 三角翻转法
uint32_t get_critical2(void)
{
    // 1 趋势统一化
    int16_t i, j;
    struct aniso_adc_data_t temp_data1, temp_data2;
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data1);                    // 最新缓存数据
    aniso_ring_buffer_pick(&sample_datas_rb, CRITICAL_VALUE_CNT - 1, &temp_data2); // 最旧缓存数据
    if (temp_data1.flt_val < temp_data2.flt_val)                                 // 若整体趋势↓
    {
        for (i = 0; i < CRITICAL_VALUE_CNT; i++)
        { // 翻转整体趋势
            aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data2);
            temp_data2.flt_val = -temp_data2.flt_val;
            aniso_ring_buffer_update(&sample_datas_rb, i, &temp_data2);
        }
    }

    // 2 数据归一化
    float val_max, val_min, val_range;
    val_max = val_min = temp_data1.flt_val;
    for (i = 1; i < CRITICAL_VALUE_CNT; i++)
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data2);
        val_max = fmaxf(val_max, temp_data2.flt_val);
        val_min = fminf(val_min, temp_data2.flt_val);
    }
    val_range = val_max - val_min;

    float norm_y[CRITICAL_VALUE_CNT];
    float norm_x[CRITICAL_VALUE_CNT];
    for (i = 0; i < CRITICAL_VALUE_CNT; i++)
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data2);
        norm_y[i] = (temp_data2.flt_val - val_min) / val_range; // yi'=(yi-ymin)/(ymax-ymin)
        norm_x[i] = (float)i / (CRITICAL_VALUE_CNT - 1);        // xi'=(xi-xmin)/(xmax-xmin)
    }

    // 3 夹角计算
    float delta_y = fabsf(norm_y[0] - norm_y[CRITICAL_VALUE_CNT - 1]); // 对边长度（归一化）
    float delta_x = fabsf(norm_x[0] - norm_x[CRITICAL_VALUE_CNT - 1]); // 临边长度（归一化）
    float delta_l = sqrtf(delta_x * delta_x + delta_y * delta_y);    // 斜边长度（勾股定理）
    float sin_q = delta_y / delta_l;                                 // 首末点夹角sin
    float cos_q = delta_x / delta_l;                                 // 首末点夹角cos

    // 4 拐点检测
    val_min = norm_y[0];
    for (j = 0, i = 1; i < CRITICAL_VALUE_CNT; i++)
    { // 以首点(norm_x[0],norm_y[0])为旋转中心，其余点逆时针旋转q角
        float rotate_y = sin_q * (norm_x[i] - norm_x[0]) +
                         cos_q * (norm_y[i] - norm_y[0]) +
                         norm_y[0];
        // 找出最（小）值，即为拐点
        if (val_min > rotate_y)
        {
            val_min = rotate_y;
            j = i;
        }
    }
    aniso_ring_buffer_pick(&sample_datas_rb, j, &temp_data2);
    critical_tick = temp_data2.tick;

    return critical_tick;
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
    // if (state_machine_flag != ANISO_STATE_IDLE)
    {
        aniso_adc_sample(ANISO_STATE_PLACEHOLD);
    }

    // uint16_t sample_adc_value = gpio_adc_read(gPF); // 读取上一周期adc转换数据
    // gpio_adc_sample(gPF);                            // 启动下一周期adc采样转换
    // printf_("sample_adc_value: %d\r\n", sample_adc_value);
    
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

    if (sample_mode_flag == 0)
    {
        switch (state_machine_flag)
        {
        // 初始化阶段：状态复位
        case ANISO_STATE_INIT:
            sample_val_mean = 0.0f;
            sample_cycle_now = 0;
            // sample_step_start = get_step_count_z();
            // 过滤触发后，产生振荡的异常数据，以50ms振荡为例，0.05*(1000*1000/SAMPLE_INTERVAL_US)=500个采样点
            sample_step_acc = 0.05*(1000*1000/SAMPLE_INTERVAL_US);
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

            // printf_("START AEI ADC_RANGE\r\n");
            // printf_("START AEI ADC_RANGE: sample_step_acc= %d sample_step_start= %d\r\n", sample_step_acc, sample_step_start);
            break;

        // 预热阶段：对应寻位运动的加速段，计算直流分量（采样均值）
        case ANISO_STATE_PREHEAT:
            // sample_data.step = get_step_count_z() - sample_step_start;
            sample_step_start++;
            sample_data.raw_val = (float)gpio_adc_read(gPF); // 读取上一周期adc转换数据
            gpio_adc_sample(gPF);                            // 启动下一周期adc采样转换
            sample_cycle_now++;
            sample_val_mean += (sample_data.raw_val - sample_val_mean) / sample_cycle_now; // 在线计算历史均值
            sample_data.raw_val -= sample_val_mean;                                        // 去除直流分量
            // if (abs(sample_data.step) >= sample_step_acc)
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
                    // printf_("ANISO_STATE_SAMPLE start\r\n");
                }
                // printf_("ANISO_STATE_PREHEAT %.3f\r\n", sample_data.flt_val);

            }
            break;

        // 采样阶段：对应寻位运动的匀速段，判断触发标志
        case ANISO_STATE_SAMPLE:
            sample_data.tick = timer_read_time();
            // sample_data.step = get_step_count_z() - sample_step_start;
            sample_data.raw_val = gpio_adc_read(gPF) - sample_val_mean; // 读取上一周期adc转换数据（同时去除直流分量）
            gpio_adc_sample(gPF);                                       // 启动下一周期adc采样转换
            // sample_data.flt_val = aniso_iir_filter_process(&hpf, sample_data.raw_val);
            sample_data.flt_val = aniso_iir_filter_process(&lpf, sample_data.raw_val);
            sample_data.flt_val = aniso_iir_filter_process(&bsf, sample_data.flt_val);
            sample_data.flt_val = aniso_kalman_filter_process(&kmf, sample_data.flt_val);
            aniso_ring_buffer_push(&sample_datas_rb, &sample_data); // 更新采样缓存（必须先push，判断trigger需要读缓存）

            struct aniso_adc_data_t maxmin_data;
            aniso_ring_buffer_pick(&sample_datas_rb, ALIGN_VALUE_CNT, &maxmin_data);
            adc_val_max = fmaxf(adc_val_max, maxmin_data.flt_val);
            adc_val_min = fminf(adc_val_min, maxmin_data.flt_val);
            sample_data.trigger = get_trigger1();                   // 判断触发标志
            if (sample_data.trigger == 1)
            {
                // printf_("ANISO_STATE_SAMPLE triggered 0\r\n");
                aniso_ring_buffer_update(&sample_datas_rb, 0, &sample_data);
                trigger1_count++;
            }
            else {
                trigger1_count = 0;
            }

            if (trigger1_count > 4) {
                // printf_("ANISO_STATE_SAMPLE triggered 1\r\n");
                trigger1_count = 0;
                gpio_out_setup(GPIO('B', 2), 0);
                gpio_out_setup(GPIO('G', 7), 0);
                state_machine_flag = ANISO_STATE_STOP;
            }
            break;

        // 停止阶段
        case ANISO_STATE_STOP:
            gpio_out_setup(GPIO('B', 2), 1);
            gpio_out_setup(GPIO('G', 7), 1);
            memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
            sample_data.tick = critical_tick; // 记录临界点时间戳
            // state_machine_flag = ANISO_STATE_IDLE;
            sample_mode_flag = 0;
            state_machine_flag = ANISO_STATE_INIT;
            break;

        default:
            break;
        }
    }
    else
    {
        switch (state_machine_flag)
        {
        // 初始阶段
        case ANISO_STATE_INIT:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z();
            gpio_adc_sample(gPF); // 启动下一周期adc采样转换
            state_machine_flag = ANISO_STATE_SAMPLE;
            break;

        // 采样阶段
        case ANISO_STATE_SAMPLE:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z();
            sample_data.raw_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
            gpio_adc_sample(gPF);                     // 启动下一周期adc采样转换
            break;

        // 停止阶段
        case ANISO_STATE_STOP:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z();
            sample_data.raw_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
            state_machine_flag = ANISO_STATE_IDLE;
            break;

        default:
            break;
        }
    }

    if (log_flag) // aniso_endstop_information上报
        sendf("aei flag=%u tick=%u step=%i raw=%i flt=%i trg=%c",
              state_machine_flag, sample_data.tick, sample_data.step,
              (int32_t)(sample_data.raw_val * 1000), (int32_t)(sample_data.flt_val * 1000),
              sample_data.trigger);

    return sample_data.trigger;
}

void sample_task(void)
{
    printf_("aniso_bed_mesh state_machine_flag=%u\r\n", (uint16_t)state_machine_flag);
}
// DECL_TASK(sample_task);

void command_config_aniso_bed_mesh(uint32_t *args)
{
    uint8_t goid = (uint8_t)args[0];
    uint32_t pin = args[1];
    gPF = gpio_adc_setup(pin);
    gpio_out_setup(GPIO('B', 2), 1);

    sample_mode_flag = 0;
    state_machine_flag = ANISO_STATE_INIT;
    
    // 初始化并启动定时器
    sched_del_timer(&sample_timer);
    sample_timer.func = sample_adc_event;
    sample_timer.waketime = timer_read_time() + timer_from_us(SAMPLE_INTERVAL_US);
    sched_add_timer(&sample_timer);
    
    printf_("aniso_bed_mesh oid=%u op=%u val=%u\r\n", goid, 0, pin);
}
DECL_COMMAND(command_config_aniso_bed_mesh, "config_aniso_bed_mesh oid=%c pin=%u");

void command_setup_aniso_bed_mesh(uint32_t *args)
{
    uint8_t goid = (uint8_t)args[0];
    switch (args[1]) // 功能码
    {
    case 1: // 设置工作模式
        // sample_mode_flag = (uint8_t)args[2];
        sample_mode_flag = 0;
        state_machine_flag = ANISO_STATE_INIT;
        gpio_out_setup(GPIO('B', 2), 1);
        gpio_out_setup(GPIO('G', 7), 1);
        printf_("setup_aniso_bed_mesh oid=%u\r\n", goid);
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
