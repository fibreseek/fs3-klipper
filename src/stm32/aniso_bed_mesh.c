#include <string.h> // NULL
#include "autoconf.h" // CONFIG_MACH_STM32F446
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/io.h" // writel
#include "board/usb_cdc.h" // usb_notify_ep0
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // GPIO
#include "sched.h" // DECL_INIT
#include "gpio.h"

/*! Returns the minimum value out of two values */
#define MIN(x, y)                       ((x) < (y) ? (x) : (y))

/*! Returns the maximum value out of two values */
#define MAX(x, y)                       ((x) > (y) ? (x) : (y))
#define ABS(x) ((x) < 0 ? -(x) : (x))
#define UABS_DIFF(a, b)  (( (a) > (b) ) ? ((a) - (b)) : ((b) - (a)))
#define USE_PROPORTION  1
struct gpio_adc pin[4]; 
uint32_t adc_val[4] = {0};
volatile uint32_t adc_val_min_t[4] = {0};
volatile uint32_t adc_val_max_t[4] = {0};

static uint8_t goid = 0;
static uint8_t startb = 0;
struct gpio_adc gPF[4];
#define ADC_MAX_OFFSET  300
#define ADC_MIN(a, b) ((a) > (b) ? ((a) - (b)) : (a))
#define ADC_MAX(a, b) (((a) + (b)) > 4095 ? (a) : ((a) + (b)))

static uint32_t ticks = 0;

#if USE_PROPORTION
static uint32_t adc_val_pre[4] = {0};
static uint32_t delta = 120;
int32_t val_delta = 0;
#else
static uint32_t g_adc_trigge = 0;
#endif
static uint32_t zmicrosteps = 8; // 默认微步数为8
static int32_t Z_STEP = 0xFFFFFFFF;
extern int32_t get_step_count_z(void);
extern uint8_t get_step_dir_z(void);
uint8_t gpio_in_read_pre1(void);

extern uint32_t ticks_info1;
extern uint32_t ticks_info2;

volatile uint32_t step_align = 0; // 用于补偿步进计数
uint32_t step_align_array[3] = {0, 0, 0}; // 用于存储对齐步数
uint8_t step_align_count = 0;
volatile uint8_t align_flag = 0;
#define STEP_ALIGN_INIT_VAL 10000 // 初始值为10000，表示未计算对齐步数
static uint8_t adc_flag = 0;
static uint8_t align_start_flag = 0; // 用于标记是否开始对齐
static int32_t align_start_step = 0; // 用于记录对齐开始时的步进计数
static int8_t trigge_count = 0; // 用于记录触发次数
#define MAX_TRIGGE_COUNT    5 // 最大触发次数
#define MAX_TRIGGE_NUM      5 // 最大触发消抖次数
#define ADC_MIN_TRIGGE  500 // 最小触发值
#define ADC_MAX_TRIGGE  3600 // 最大触发值

struct adc_buff {
    uint16_t adc_val; // ADC采样值
    int32_t step; // 活动打印时间
    uint8_t trigger;
};
#define VALUE_COUNT 300
#define ADC_ALIGN_COUNT 50 
struct adc_buff adc_data[4][VALUE_COUNT];

void init_step_align(void)
{
    step_align = STEP_ALIGN_INIT_VAL;
    memset(adc_data, 0, sizeof(adc_data));
    align_start_flag = 1;
    trigge_count = 0;
    //memset(step_align_array,0,sizeof(step_align_array));
    printf_("init_step_align\r\n");
}

int32_t gen_step_align_data(void) 
{
    // 限制输入最大为1023（10位）
    step_align_array[0] &= 0x3FF; // 0b0000001111111111
    step_align_array[1] &= 0x3FF;
    step_align_array[2] &= 0x3FF;

    return (step_align_array[0] << 20) | (step_align_array[1] << 10) | step_align_array[2];
}


void insert_at_front(int value) {
    // 先将数组中已有的值往后移动
    for (int i = 3 - 1; i > 0; i--) {
        step_align_array[i] = step_align_array[i - 1];
    }
    // 将新值插入到第一个位置
    step_align_array[0] = value;
   //printf_("add:%d\r\n",step_align_count++);
}

uint32_t get_step_align(void)
{
    uint8_t index = 0;
    uint8_t count = 0;
    uint16_t offset = 0;
    
    if(step_align != STEP_ALIGN_INIT_VAL) 
    {
        return step_align;
    }

    for(index = 0; index < 4; index++)
    {
        if(adc_data[index][0].trigger == 1)
        {
            break;
        }
    }

    if(index >= 4) 
    {
        // 没有触发的ADC数据
        step_align = 0;
        insert_at_front(step_align);
        return step_align;
    }

    if(step_align == STEP_ALIGN_INIT_VAL) 
    {
        //step_align
        if(adc_data[index][0].adc_val > adc_val_max_t[index])
        {
            for (int i = 0; i < VALUE_COUNT; i++)
            {
                if((adc_data[index][i].adc_val < adc_data[index][i+1].adc_val) && adc_data[index][i].adc_val < adc_val_max_t[index])
                {
                    step_align = adc_data[index][0].step - adc_data[index][i].step;
                    insert_at_front(step_align);
                    return step_align;
                }
            }
        }
        else if(adc_data[index][0].adc_val < adc_val_min_t[index])
        {
            for (int i = 0; i < VALUE_COUNT; i++)
            {
                if((adc_data[index][i].adc_val > adc_data[index][i+1].adc_val) && adc_data[index][i].adc_val > adc_val_min_t[index])
                {
                    step_align = adc_data[index][0].step - adc_data[index][i].step;
                    insert_at_front(step_align);
                    return step_align;
                }
            }
        }
        else
        {
            step_align = 0;
            insert_at_front(step_align);
        }
    }
    return step_align;
}

void insert_adc_data_to_group(uint16_t index, struct adc_buff new_data) {
    // 从后往前复制，避免数据覆盖
    uint16_t max = 0;
    uint16_t min = 4096;

    for (int i = VALUE_COUNT-1; i > 0; i--) {
        adc_data[index][i] = adc_data[index][i - 1];

    }
    adc_data[index][0] = new_data;

    for(int i = ADC_ALIGN_COUNT; i < VALUE_COUNT; i++)
    {
        if(adc_data[index][i].adc_val > max)
        {
            max = adc_data[index][i].adc_val;
        }
        if(adc_data[index][i].adc_val < min && adc_data[index][i].adc_val != 0)
        {
            min = adc_data[index][i].adc_val;
        }
    }
    adc_val_min_t[index] = min;
    adc_val_max_t[index] = max;

}


void aniso_bed_mesh_cmd(uint32_t *args)
{
    //uint32_t delay;
    uint32_t op;
    uint32_t val;
    uint16_t i=0;
    uint8_t res = 0;
    uint8_t index = 0;
    //static uint8_t proportion = 0;

    goid = args[0];
    op = args[1];
    val = args[2];

    if(op == 2)   
    {
        for(i = 0; i < 4; i++)
        {
            adc_val_min_t[i] = 4095;
            adc_val_max_t[i] = 0;
        }

        delta = val;
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, res);
    }
    else if(op == 3)   
    {
        startb = 0;
        zmicrosteps = val; // 更新微步数
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, 0);
        for(i = 0; i < 4; i++)
        {
            
        }
    }
    else if(op == 4)   
    {
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 30000+adc_val_min_t[val]);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 40000+adc_val_max_t[val]);

        for(i=0;i<4;i++)
        {
            if(adc_data[i][0].trigger == 1)
            {
                index = i;
            }
        }

        printf_("index=%d,min=%d,max=%d\r\n", index, adc_val_min_t[index], adc_val_max_t[index]);
        for(i=0;i<300;i++)
        {
            printf_("%d\t%d\t%d\t\n", adc_data[index][i].adc_val, adc_data[index][i].step, adc_data[index][i].trigger);
        }
    }
    else if(op == 7)
    {
        //printf_("aniso_bed_mesh oid=%d op=%d val=%i\r\n", goid, 7, val);
        memset(step_align_array,0,sizeof(step_align_array));
        align_flag = val;
        step_align_count = 0;
    }   
    else if(op == 8)
    {
        //printf_("aniso_bed_mesh oid=%d op=%d val=%u\r\n", goid, 8, gen_step_align_data());
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 8, gen_step_align_data());
    }
	//sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, val);
}

uint32_t get_zmicrosteps_value(uint32_t step)
{
    switch(zmicrosteps)
    {
        case 8:
            return step;              // 8微步，相当于 step << 0
        case 16:
            return step << 1;         // 16微步，step * 2
        case 32:
            return step << 2;         // 32微步，step * 4
        case 64:        
            return step << 3;         // 64微步，step * 8
        case 128:
            return step << 4;         // 128微步，step * 16
        case 256:   
            return step << 5;         // 256微步，step * 32
        default:
            return step;              // fallback，避免意外情况
    }
}

uint8_t gpio_in_read_pre1(void)
{
    uint16_t val = 0;
    uint8_t i;
    //uint8_t z_val= 0;
    int32_t step; // = get_step_count_z() - Z_STEP;
    //int32_t delta_half = 0; // = get_step_count_z() - Z_STEP;
    static uint32_t adc_count = 0;
    struct adc_buff adc_buff_data = {0, 0, 0};
    //static uint8_t trigge_count = 0;
    static uint8_t vib_count[4] = {0};

    if(align_start_flag == 1)
    {
        align_start_flag = 0;
        ticks = 0;
        for(i = 0; i < 4; i++)
        {
            adc_val_min_t[i] = 4095;
            adc_val_max_t[i] = 0;
            vib_count[i] = 0;
        }
        trigge_count = 0;
        align_start_step = get_step_count_z();
        return 0;
    }
    
    if(trigge_count > 1)
    {
        //printf_("1");
        trigge_count --;
        return 1; // 如果已经触发过，则直接返回
    }
    else if(trigge_count == 1)
    {
        // 如果只触发过一次，则检查是否需要继续触发
        //printf_("2");
        return 0;
    }

    for(i = 0; i < 4; i++)
    {
        if(gpio_adc_sample(gPF[i]) == 0)
        {
            val = gpio_adc_read(gPF[i]);
            step = get_step_count_z() - align_start_step;

            if(val > ADC_MAX_TRIGGE || val < ADC_MIN_TRIGGE)
            {
                adc_buff_data.trigger = 3;
                insert_adc_data_to_group(i,adc_buff_data);
                trigge_count = MAX_TRIGGE_COUNT;
                return 1;
            }


            if(ABS(step) <= get_zmicrosteps_value(800))
            {
                //return 0;
            }
            else if(ABS(step) > get_zmicrosteps_value(800) && ABS(step) <get_zmicrosteps_value(2800))
            {
                adc_buff_data.adc_val = val;
                adc_buff_data.step = step;
                adc_buff_data.trigger = 0;
                insert_adc_data_to_group(i, adc_buff_data);
                
                //return 0;
            }
            else
            {
                val_delta = (adc_val_max_t[i] -adc_val_min_t[i])*delta/100;
                adc_buff_data.adc_val = val;
                adc_buff_data.step = step;
                adc_buff_data.trigger = 0;
                insert_adc_data_to_group(i,adc_buff_data);

                val_delta = MAX(val_delta, 60); // 确保val_delta至少为60
                if(val > adc_val_max_t[i]+val_delta||val < adc_val_min_t[i]-val_delta)
                {
                    vib_count[i] = vib_count[i]+1;     
                    if(vib_count[i]>=MAX_TRIGGE_NUM)   
                    {
                        adc_buff_data.trigger = 1;
                        insert_adc_data_to_group(i,adc_buff_data);
                        trigge_count = MAX_TRIGGE_COUNT;
                        return 1;
                    }      
                }
                else
                {
                    vib_count[i] = 0;
                }
            }
        }  
    }     
    return 0; 
}

void adc3_gpio_init(void)
{
    gPF[0] = gpio_adc_setup(0x55);
    gPF[1] = gpio_adc_setup(0x56);
    gPF[2] = gpio_adc_setup(0x59);
    gPF[3] = gpio_adc_setup(0x5A);
}

DECL_COMMAND(aniso_bed_mesh_cmd,"aniso_bed_mesh_cmd oid=%u op=%u val=%u");
//DECL_TASK(bed_mesh_task);
DECL_INIT(adc3_gpio_init);