// readme (20250911):
// 1. 使用STM32H7的ADC3作为电压采集转换，理论精度12位，转换结果合法取值[0, 4095]
// 2. gpio_adc_sample 手动开启一次转换，每次转换后需等待20us才覆盖采样结果
//    暂时处理为提前一个周期触发转换，下一周期再通过gpio_adc_read读取数据
//    导致问题：
//    (1) 数据至少晚了一个周期
//    (2) 读取频率理论上限 freq = (1s) / (20us)
//    解决方案：
//    (1) 开启连续转换，尝试过了，会导致mcu繁忙，与上位机通信超时
//    (2) 连续转换+DMA，或单次转换+定时器，后续有嵌入式能力强的同事，可以尝试
//*************************************************************************

#include <string.h>           // memcpy memset
#include "autoconf.h"         // CONFIG_MACH_STM32F446
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/io.h"         // writel
#include "board/usb_cdc.h"    // usb_notify_ep0
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "board/misc.h"       // timer_read_time
#include "command.h"          // DECL_CONSTANT_STR
#include "internal.h"         // GPIO
#include "sched.h"            // DECL_INIT
#include "gpio.h"
#include "debug/debug.h"
#include "aniso_ring_buffer.h" // aniso_ring_buffer_t
#include "aniso_filter.h"      // aniso_kalman_filter_t

extern uint8_t get_step_dir_z(void);
extern int32_t get_step_count_z(void);

#define PROBE_CNT_MAX 5       // 单点复探次数
#define STEP_ALIGN_INIT 10000 // 初始值为10000，表示未计算对齐步数
#define BUFFER_VALUE_CNT 200  // 采样缓存的容量
#define ALIGN_VALUE_CNT 100   // 用于adc数据对齐的缓存元素数量
#define ABS(x) ((x) < 0 ? -(x) : (x))

// 状态机状态定义
enum aniso_state_machine_t
{
    ANISO_STATE_IDLE = 0,      // 待机阶段
    ANISO_STATE_INIT = 1,      // 初始阶段
    ANISO_STATE_START = 2,     // 起步阶段（step_start放在1阶段会报错）
    ANISO_STATE_WAIT = 3,      // 等待阶段（0 ~ 1200 step)
    ANISO_STATE_PREHEAT = 4,   // 预热阶段（1200 ~ 2400 step）
    ANISO_STATE_SAMPLE = 5,    // 采样阶段 (2400+ step)
    ANISO_STATE_STOP = 6,      // 停止阶段
    ANISO_STATE_PLACEHOLD = 7, // 缺省占位（从当前阶段，按条件自动切换）
};

// 单次采样数据内容
struct aniso_adc_data_t
{
    uint8_t trigger;  // 触发标志位
    uint32_t adc_val; // adc采样值
    int32_t step;     // 电机步数
    uint32_t tick;    // 时间戳(下位机时钟)
    float flt_val;    // adc滤波值
};

struct gpio_adc gPF;             // adc引脚通道
struct aniso_kalman_filter_t kf; // 卡尔曼滤波器
struct aniso_iir_filter_t bsf;   // 陷波(带阻)滤波器
// struct aniso_iir_filter_t hpf;                             // 高通滤波器
struct aniso_iir_filter_t lpf;                             // 低通滤波器
uint32_t step_align_ar[PROBE_CNT_MAX];                     // step补偿缓存array
struct aniso_ring_buffer_t step_align_rb;                  // step补偿缓存ring_buffer
struct aniso_adc_data_t sample_datas_ar[BUFFER_VALUE_CNT]; // adc采样缓存array
struct aniso_ring_buffer_t sample_datas_rb;                // adc采样缓存ring_buffer

static float sample_freq = 1.0f;                                         // adc采样频率
static uint8_t sample_mode_flag = 0;                                     // 0 adc采样+触发判断；1仅adc采样(调试用)
static int32_t step_start = 0;                                           // 起始时刻的电机步进
static uint32_t zmicrosteps = 8;                                         // 默认微步数为8
static enum aniso_state_machine_t state_machine_flag = ANISO_STATE_IDLE; // 状态机标志

volatile float adc_val_min; // adc触发的负向阈值
volatile float adc_val_max; // adc触发的正向阈值
volatile uint8_t step_align_flag = 0;
volatile uint32_t step_align_now; // 本次电机步数补偿值

const uint8_t log_flag = 0; // 采样数据上报标志位，1周期上报，0关闭上报

void calc_sample_freq(uint32_t tick)
{
    // 参考自clocksync.py clock_to_print_time()
    // sample_time = sample_tick/mcu_freq
    // sample_freq = 1 / sample_time = mcu_freq / sample_tick
    //  STM32H723主频400mhz，也即400000000hz
    sample_freq = 400000000.0f / (float)tick;
    }

void step_align_rb_init(void)
{ // 按当前的机制，上电后step_align_rb只能被初始化1次，多于1次会导致数据错误
    static uint8_t step_inited = 0;
    if (step_inited == 0)
    {
        aniso_ring_buffer_init(&step_align_rb, step_align_ar, PROBE_CNT_MAX, sizeof(uint32_t));
        step_inited = 1;
    }
}

uint32_t get_zmicrosteps_val(uint32_t step)
{ // 正常情况下，zmicrosteps总是8的正整数倍，可以简化计算
    // 位移量 = log2(zmicrosteps/8) = log2(zmicrosteps) - log2(8) = log2(zmicrosteps) - 3
    uint32_t shift = __builtin_ctz(zmicrosteps) - 3;
    return step << shift;
}

uint32_t get_step_align_val(void)
{
    if (step_align_now != STEP_ALIGN_INIT)
        return step_align_now;
    // flag用作缓存元素数量
    int16_t flag = aniso_ring_buffer_size(&sample_datas_rb);
    if (flag == 0) // 缓存为空
        return 0;

    step_align_now = 0; // 先赋个初值，避免后续分类有遗漏
    struct aniso_adc_data_t temp_data0, temp_data1, temp_data2;
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data0); // 提取最新缓存
    if (temp_data0.trigger != 1)
        return step_align_now;

    flag = 0; // flag用作符号标志位，正向阈值触发取1，负向阈值触发取-1
    if (temp_data0.flt_val > adc_val_max)
        flag = 1;
    else if (temp_data0.flt_val < adc_val_min)
        flag = -1;

    if (flag != 0) // 有触发
    {
        for (int16_t i = 5; i < BUFFER_VALUE_CNT - 1; i++) // 为了去抖，i从5开始
        {
            aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data1);
            aniso_ring_buffer_pick(&sample_datas_rb, i + 1, &temp_data2);

            // 根据flag的值来判断比较方向
            if ((flag == 1 && temp_data1.flt_val < temp_data2.flt_val) ||
                (flag == -1 && temp_data1.flt_val > temp_data2.flt_val))
            {
                step_align_now = temp_data0.step - temp_data1.step;
                // printf_("%d\r\n", i);
                break;
            }
        }
    }

    // 使用临时变量避免volatile限定符警告
    uint32_t step_align_copy = step_align_now;
    aniso_ring_buffer_push(&step_align_rb, &step_align_copy);
    return step_align_now;
}

int32_t get_step_align_vals(void)
{
    int32_t res = 0;
    uint32_t step = 0;

    // 从最新的数据开始，依次压缩5个步数对齐值
    for (int16_t i = 0; i < PROBE_CNT_MAX; i++)
    {
        aniso_ring_buffer_pick(&step_align_rb, i, &step);
        // 每个值只保留低6位(0-63)，然后放到结果的相应位置
        // 第0个值占位0-5位，第1个值占位6-11位，以此类推
        res |= ((uint32_t)(step & 0x3F)) << (i * 6);
    }

    return res;
}

void aniso_bed_mesh_cmd(uint32_t *args)
{
    uint8_t goid = (uint8_t)args[0];
    uint32_t op = args[1];
    uint32_t val = args[2];

    switch (op) // 功能码
    {
    case 2: // 设置adc通道参数
        gPF = gpio_adc_setup(val);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, 0);
        break;

    case 3: // 设置z轴电机细分数
        zmicrosteps = (val > 256) ? 256 : ((val < 8) ? 8 : val);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, 0);
        break;

    case 4: // 输出adc采样数据缓存
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 30000 + adc_val_min);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 40000 + adc_val_max);

        printf_("min=%d, max=%d\r\n", adc_val_min, adc_val_max);

        val = val > 3 ? 3 : val; // 取值合法性检查，val ∈ [0~3]，避免索引越界
        struct aniso_adc_data_t temp_data;
        for (int16_t i = 0, j = BUFFER_VALUE_CNT / 4, k; i < j; i++)
        { // BUFFER_VALUE_CNT太大，一次性print会导致下位机通信超时，需要分批打印
            k = i + val * j;
            aniso_ring_buffer_pick(&sample_datas_rb, k, &temp_data);
            printf_("%u,%d,%u,%u,%.3f\r\n", temp_data.adc_val, temp_data.step,
                    temp_data.trigger, temp_data.tick, temp_data.flt_val);
        }
        break;

    case 7: // 清除多次步数补偿值缓存
        memset(step_align_ar, 0, sizeof(step_align_ar));
        aniso_ring_buffer_init(&step_align_rb, step_align_ar, PROBE_CNT_MAX, sizeof(uint32_t));
        step_align_flag = val;
        // printf_("step_align_flag=%d\r\n", step_align_flag);
        break;

    case 8: // 反馈多次步数补偿值
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 8, get_step_align_vals());
        break;

    case 9: // val=1，启用adc采样+周期上报的模式；val=0，启用adc采样+触发判断的模式
        sample_mode_flag = val;
        state_machine_flag = ANISO_STATE_IDLE;
        break;
    }
}

// 触发判断 阈值法
uint8_t get_trigger1(int32_t adc_val_delta)
{
    struct aniso_adc_data_t temp_data;
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data);

    if (temp_data.flt_val > adc_val_max + adc_val_delta ||
        temp_data.flt_val < adc_val_min - adc_val_delta)
        return 1;
    else
        return 0;
}

// 触发判断 斜率法
uint8_t get_trigger2(int32_t adc_val_delta)
{
    // 1.检测末尾3点是否有序递增或递减
    struct aniso_adc_data_t temp_data1, temp_data2, temp_data3;
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data1);
    aniso_ring_buffer_pick(&sample_datas_rb, 1, &temp_data2);
    aniso_ring_buffer_pick(&sample_datas_rb, 2, &temp_data3);

    uint8_t flag; // 单调性标志位
    if (temp_data1.flt_val > temp_data2.flt_val && temp_data2.flt_val > temp_data3.flt_val)
        flag = 1; // 单调递增
    else if (temp_data1.flt_val < temp_data2.flt_val && temp_data2.flt_val < temp_data3.flt_val)
        flag = 0; // 单调递减
    else
        return 0; // 非单调变化

    // 2.检测末尾3点是否均大于或小于其它任意点
    uint16_t i = 0;
    float val_min = adc_val_min;
    float val_max = adc_val_max;
    for (i = 3; i < ALIGN_VALUE_CNT; i++)
    {//此时temp_data2的含义另作他用
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data2);
        if (val_min > temp_data2.flt_val)
            val_min = temp_data2.flt_val;
        if (val_max < temp_data2.flt_val)
            val_max = temp_data2.flt_val;
    }

    if (flag==1) // 单调递增
    {
        if (temp_data3.flt_val < val_max) // 末尾三个点不都大于其他任意值
            return 0;
        val_max = temp_data1.flt_val;
    }
    else // 单调递减
    {
        if (temp_data3.flt_val > val_min) // 末尾三个点不都小于其他任意值
            return 0;
        val_min = temp_data1.flt_val;
    }

    // 3.对数据进行归一化，方便处理
    float val_range = val_max - val_min;
    for (i = 1; i < ALIGN_VALUE_CNT; i++)
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data2);
        float norm_y; // tan角的对边归一化
        if (flag==1)
            norm_y = (temp_data1.flt_val - temp_data2.flt_val) / val_range;
        else
            norm_y = (temp_data2.flt_val - temp_data1.flt_val) / val_range;
        float norm_x = (float)i / (ALIGN_VALUE_CNT-1); // tan角的临边归一化

        // 4.比较各点针对最后1点的斜率是否超过45度
        if (norm_y / norm_x < 0.8391f) // tan = 对边/临边，tan(45°)=1
            return 0;
    }

    return 1;
}

uint8_t aniso_adc_sample(int smf)
{
    uint8_t res = 0; // 函数返回值赋初值
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
            adc_val_min = 4095.0f;
            adc_val_max = 0.0f;
            // memset(step_align_ar, 0, sizeof(step_align_ar));
            memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
            // aniso_ring_buffer_init(&step_align_rb, step_align_ar, PROBE_CNT_MAX, sizeof(uint32_t));
            step_align_rb_init();
            aniso_ring_buffer_init(&sample_datas_rb, sample_datas_ar, BUFFER_VALUE_CNT, sizeof(struct aniso_adc_data_t));
            aniso_kalman_filter_init(&kf, 0.2f, 0.03f, 0.2f);
            aniso_iir_filter_init(&bsf, ANISO_FILTER_BAND_STOP, 50.0f, 10.0f, sample_freq);
            // aniso_iir_filter_init(&hpf, ANISO_FILTER_HIGH_PASS, 5.0f, 0.707f, sample_freq);
            aniso_iir_filter_init(&lpf, ANISO_FILTER_LOW_PASS, 3000.0f, 0.707f, sample_freq);
            gpio_adc_sample(gPF); // 提前启动adc采样转换
            step_align_now = STEP_ALIGN_INIT;
            state_machine_flag = ANISO_STATE_START;
            break;

        // 起步阶段：获取步数初始值
        case ANISO_STATE_START:
            step_start = get_step_count_z();
            state_machine_flag = ANISO_STATE_WAIT;
            break;

        // 等待阶段：前1200步，认为不稳定
        case ANISO_STATE_WAIT:
            sample_data.step = get_step_count_z() - step_start;
            if (ABS(sample_data.step) > get_zmicrosteps_val(1200))
            {
                sample_data.adc_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
                gpio_adc_sample(gPF);                     // 启动下一周期adc采样转换
                aniso_kalman_filter_preheat(&kf, (float)sample_data.adc_val);
                aniso_iir_filter_preheat(&bsf, (float)sample_data.adc_val);
                // aniso_iir_filter_preheat(&hpf, (float)sample_data.adc_val);
                aniso_iir_filter_preheat(&lpf, (float)sample_data.adc_val);
                state_machine_flag = ANISO_STATE_PREHEAT;
            }
            break;

        // 预热阶段：1200~2400步，更新采样缓存，动态计算adc触发阈值，不判断触发标志
        case ANISO_STATE_PREHEAT:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z() - step_start;
            sample_data.trigger = 0;
            sample_data.adc_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
            gpio_adc_sample(gPF);                     // 启动下一周期adc采样转换
            // sample_data.flt_val = aniso_iir_filter_process(&hpf, (float)sample_data.adc_val);
            sample_data.flt_val = aniso_iir_filter_process(&lpf, (float)sample_data.adc_val);
            sample_data.flt_val = aniso_iir_filter_process(&bsf, sample_data.flt_val);
            sample_data.flt_val = aniso_kalman_filter_process(&kf, sample_data.flt_val);
            aniso_ring_buffer_push(&sample_datas_rb, &sample_data); // 更新采样缓存

            if (ABS(sample_data.step) > get_zmicrosteps_val(2400))
                state_machine_flag = ANISO_STATE_SAMPLE;

            // 缓存达到需求容量后，取第ALIGN_VALUE_CNT个元素，与历史阈值比较并更新
            if (aniso_ring_buffer_size(&sample_datas_rb) > ALIGN_VALUE_CNT)
            {
                aniso_ring_buffer_pick(&sample_datas_rb, ALIGN_VALUE_CNT, &sample_data);
                if (sample_data.flt_val > adc_val_max)
                    adc_val_max = sample_data.flt_val;
                if (sample_data.flt_val < adc_val_min)
                    adc_val_min = sample_data.flt_val;
                aniso_ring_buffer_pick(&sample_datas_rb, 0, &sample_data); // 为了上报变更回最新数据~
            }
            break;

        // 采样阶段：2400+步，更新采样缓存，动态计算adc触发阈值，判断触发标志
        case ANISO_STATE_SAMPLE:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z() - step_start;
            sample_data.trigger = 0;
            sample_data.adc_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
            gpio_adc_sample(gPF);                     // 启动下一周期adc采样转换
            // sample_data.flt_val = aniso_iir_filter_process(&hpf, (float)sample_data.adc_val);
            sample_data.flt_val = aniso_iir_filter_process(&lpf, (float)sample_data.adc_val);
            sample_data.flt_val = aniso_iir_filter_process(&bsf, sample_data.flt_val);
            sample_data.flt_val = aniso_kalman_filter_process(&kf, sample_data.flt_val);
            aniso_ring_buffer_push(&sample_datas_rb, &sample_data); // 更新采样缓存

            int32_t adc_val_delta = (adc_val_max - adc_val_min) * 30 / 100;
            // 缓存达到需求容量后，取第ALIGN_VALUE_CNT个元素，与历史阈值比较并更新
            if (aniso_ring_buffer_size(&sample_datas_rb) > ALIGN_VALUE_CNT)
            {
                aniso_ring_buffer_pick(&sample_datas_rb, ALIGN_VALUE_CNT, &sample_data);
                if (sample_data.flt_val > adc_val_max)
                    adc_val_max = sample_data.flt_val;
                if (sample_data.flt_val < adc_val_min)
                    adc_val_min = sample_data.flt_val;
                aniso_ring_buffer_pick(&sample_datas_rb, 0, &sample_data);
            }

            // --------触发标志判断--------
            if (get_trigger2(adc_val_delta) == 1)
            {
                sample_data.trigger = 1;
                aniso_ring_buffer_update(&sample_datas_rb, 0, &sample_data);
                res = 1;
            }
            break;

        // 停止阶段：更新停止状态
        case ANISO_STATE_STOP:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z() - step_start;
            sample_data.trigger = 1;
            sample_data.adc_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
            // sample_data.flt_val = aniso_iir_filter_process(&hpf, (float)sample_data.adc_val);
            sample_data.flt_val = aniso_iir_filter_process(&lpf, (float)sample_data.adc_val);
            sample_data.flt_val = aniso_iir_filter_process(&bsf, sample_data.flt_val);
            sample_data.flt_val = aniso_kalman_filter_process(&kf, sample_data.flt_val);
            aniso_ring_buffer_push(&sample_datas_rb, &sample_data); // 更新采样缓存
            aniso_kalman_filter_reset(&kf);
            aniso_iir_filter_reset(&bsf);
            // aniso_iir_filter_reset(&hpf);
            aniso_iir_filter_reset(&lpf);
            state_machine_flag = ANISO_STATE_IDLE;
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
            sample_data.step = 0;
            step_align_now = STEP_ALIGN_INIT;
            state_machine_flag = ANISO_STATE_START;
            break;

        // 起步阶段
        case ANISO_STATE_START:
            sample_data.tick = timer_read_time();
            sample_data.step = 0;
            step_start = get_step_count_z();
            gpio_adc_sample(gPF); // 启动下一周期adc采样转换
            state_machine_flag = ANISO_STATE_SAMPLE;
            break;

        // 采样阶段
        case ANISO_STATE_SAMPLE:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z() - step_start;
            sample_data.adc_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
            gpio_adc_sample(gPF);                     // 启动下一周期adc采样转换
            break;

        // 停止阶段
        case ANISO_STATE_STOP:
            sample_data.tick = timer_read_time();
            sample_data.step = get_step_count_z() - step_start;
            sample_data.adc_val = gpio_adc_read(gPF); // 读取上一周期adc转换数据
            state_machine_flag = ANISO_STATE_IDLE;

        default:
            break;
        }
    }

    if (log_flag) // aniso_endstop_information上报
        sendf("aei flag=%u tick=%u step=%i adc=%u flt=%i trg=%u",
              state_machine_flag, sample_data.tick, sample_data.step, sample_data.adc_val, (int32_t)sample_data.flt_val, sample_data.trigger);

    return res;
}

DECL_COMMAND(aniso_bed_mesh_cmd, "aniso_bed_mesh_cmd oid=%u op=%u val=%u");