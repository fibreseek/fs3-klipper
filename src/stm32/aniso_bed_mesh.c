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
uint32_t adc_val_min[4] = {0};
uint32_t adc_val_max[4] = {0};
uint32_t adc_val_min_t[4] = {0};
uint32_t adc_val_max_t[4] = {0};

static uint8_t goid = 0;
static uint8_t startb = 0;
struct gpio_adc gPF[4];
#define ADC_MAX_OFFSET  300
#define ADC_MIN(a, b) ((a) > (b) ? ((a) - (b)) : (a))
#define ADC_MAX(a, b) (((a) + (b)) > 4095 ? (a) : ((a) + (b)))

static uint32_t ticks = 0;

#if USE_PROPORTION
static uint32_t adc_val_pre[4] = {0};
static uint32_t delta = 0;
int32_t val_delta = 0;
#else
static uint32_t g_adc_trigge = 0;
#endif

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
//static uint32_t val1=0,val2=0,val3=0,val4=0,val5=0,val6=0,val7=0,val8=0,val9=0,val10=0;
// static uint32_t val_max[4] = {0};
// static uint32_t val_min[4] = {0};
// static uint32_t index_array[100];
//static uint16_t index_offset = 1;
static uint8_t trigge_flag[4] = {0, 0, 0, 0};
static uint8_t adc_flag = 0;
static uint8_t align_start_flag = 0; // 用于标记是否开始对齐
static int32_t align_start_step = 0; // 用于记录对齐开始时的步进计数
static int8_t trigge_count = 0; // 用于记录触发次数
#define MAX_TRIGGE_COUNT 5 // 最大触发次数
typedef struct {
    uint32_t data[50];
    uint8_t count;  // 当前已有数据个数（最大到20）
} U8Buffer;

U8Buffer my_buf = {0};


struct adc_buff {
    uint16_t adc_val; // ADC采样值
    int32_t step; // 活动打印时间
    uint8_t trigger;
};
#define VALUE_COUNT 100
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
        //printf_("1:%d\n", step_align);
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
                    for (int j = i + 1; j < VALUE_COUNT && j< i + 10 && count < 3; j++)
                    {
                        if(adc_data[index][j].adc_val > adc_val_max_t[index])
                        {
                            count++;
                            offset = j;
                        }
                        else
                        {
                            count = 0; // 重置计数
                        }
                    }
                    if(count >= 3)
                    {
                        for(int k=offset; k < VALUE_COUNT; k++)
                        {
                            if((adc_data[index][k].adc_val < adc_data[index][k+1].adc_val) && adc_data[index][k].adc_val < adc_val_max_t[index])
                            {
                                step_align = adc_data[index][0].step - adc_data[index][k].step;
                                insert_at_front(step_align);
                                //printf_("4:%d\n", step_align);
                                return step_align;
                            }
                        }
                    }
                    step_align = adc_data[index][0].step - adc_data[index][i].step;
                    insert_at_front(step_align);
                    //printf_("2:%d\n", step_align);
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

                    for (int j = i + 1; j < VALUE_COUNT && j< i + 10 && count < 3; j++)
                    {
                        if(adc_data[index][j].adc_val < adc_val_min_t[index])
                        {
                            count++;
                            offset = j;
                        }
                        else
                        {
                            count = 0; // 重置计数
                        }
                    }
                    if(count >= 3)
                    {
                        for(int k=offset; k < VALUE_COUNT; k++)
                        {
                            if((adc_data[index][k].adc_val > adc_data[index][k+1].adc_val) && adc_data[index][k].adc_val > adc_val_min_t[index])
                            {
                                step_align = adc_data[index][0].step - adc_data[index][k].step;
                                insert_at_front(step_align);
                                //printf_("4:%d\n", step_align);
                                return step_align;
                            }
                        }
                    }


                    step_align = adc_data[index][0].step - adc_data[index][i].step;
                    insert_at_front(step_align);
                    //printf_("3: %d\n", step_align);
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
    for (int i = VALUE_COUNT-1; i > 0; i--) {
        adc_data[index][i] = adc_data[index][i - 1];
    }
    adc_data[index][0] = new_data;
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

    if(op == 1)  
    {
        gPF[0] = gpio_adc_setup(val&0xFF);
        val = (val >> 8);
        gPF[1] = gpio_adc_setup(val&0xFF);
        val = (val >> 8);
        gPF[2] = gpio_adc_setup(val&0xFF);
        val = (val >> 8);
        gPF[3] = gpio_adc_setup(val&0xFF);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, 0);
    }
    else if(op == 2)   
    {
        for(i = 0; i < 4; i++)
        {
            adc_val_min[i] = 4095;
            adc_val_max[i] = 0;
            adc_val_min_t[i] = 4095;
            adc_val_max_t[i] = 0;
            adc_val_pre[i] = 0;
            trigge_flag[i] =0;
        }
        //val1=val2=0;
        ticks = 0;
        startb = 1; 
        adc_flag = 0;
        #if USE_PROPORTION
            delta = val;
            my_buf.count = 0;
            memset(my_buf.data, 0, sizeof(my_buf.data));
        #else
            g_adc_trigge = val;
        #endif
        Z_STEP = 0xFFFFFFFF;
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, res);
    }
    else if(op == 3)   
    {
        startb = 0;
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, op, 0);
        for(i = 0; i < 4; i++)
        {
            printf_("adc_val_min[%d]=%d, adc_val_max[%d]=%d\r\n", i, adc_val_min[i], i, adc_val_max[i]);
        }
    }
    else if(op == 4)   
    {
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 10000+adc_val_min[val]);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 20000+adc_val_max[val]);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 30000+adc_val_min_t[val]);
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, 40000+adc_val_max_t[val]);
        // if(val == 0)
        // {
        //      memset(index_array, 0, sizeof(index_array));
        //     index_offset = 1;
        // }
        while(i < 30 )
        { 
            sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 4, my_buf.data[i]);
            i++;
        }
        for(i=0;i<4;i++)
        {
            if(adc_data[i][0].trigger == 1)
            {
                index = i;
            }
        }

        printf_("min=%d,max=%d\r\n", adc_val_min_t[index], adc_val_max_t[index]);
        for(i=val*100;i<val*100 + 100;i++)
        {
            printf_("index=%d, adc_val=%d, step=%d, trigger=%d\n", index, adc_data[index][i].adc_val, adc_data[index][i].step, adc_data[index][i].trigger);
        }
    }
    else if(op == 5)   
    {
        Z_STEP = get_step_count_z();
        ticks = 0;
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 5, Z_STEP);
    }
    else if(op == 6)   
    {
        Z_STEP = get_step_count_z();
        ticks = 0;
        sendf("aniso_bed_mesh oid=%c op=%c val=%i", goid, 6, Z_STEP);
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


void bed_mesh_task(void)
{
    //uint32_t ticks;
    uint32_t val = 0;
    uint32_t i = 0;

    if (startb != 0)
    {
        for(i = 0; i < 4; i++)
        {
            if(gpio_adc_sample(gPF[i]) == 0)
            {
                val = gpio_adc_read(gPF[i]);
                adc_val_min[i] = MIN(adc_val_min[i], val);
                adc_val_max[i] = MAX(adc_val_max[i], val);
            }
        }
    }  
    else
    {
        //gpio_in_read_pre1();
    }  
}
// timer_read_time
// timer_is_before

/**
 * 判断两个ticks中哪一个在前
 * 
 * @param tick1 第一个ticks值
 * @param tick2 第二个ticks值
 * @return 1 如果tick1在前，0 如果tick2在前
 */
int tick_is_earlier(uint32_t tick1, uint32_t tick2) {
    // 如果tick2比tick1大，且差值小于0x80000000，则tick1在前
    if (tick2 > tick1 && (tick2 - tick1) < 0x80000000) {
        return 1; // tick1在前
    }
    // 如果tick2比tick1小，或者差值大于0x80000000，则tick2在前
    return 0; // tick2在前
}

void u8buffer_push(uint32_t value)
{
    if(my_buf.count < 50) {
        // 还没满，直接追加
        my_buf.data[my_buf.count++] = value;
    } else {
        // 满了，所有数据前移一位
        memmove(&my_buf.data[0], &my_buf.data[1], 50 - 1);
        my_buf.data[50 - 1] = value;
    }
}

// void add_index(uint32_t val)
// {
//     // if(index_array[index_offset-1] == val)
//     // {
//     //     return; // 如果当前值和上一个值相同，则不添加
//     // }
//     if(index_offset >=30)
//     {
//         return; // 如果索引数组已满，则不添加
//     }
//     index_array[index_offset] = val;
//     index_offset++;
    
// }
//8weibu  800 2000
//64weibu  6400 16000

uint8_t gpio_in_read_pre1(void)
{
    uint16_t val = 0;
    uint8_t i;
    //uint8_t z_val= 0;
    int32_t step; // = get_step_count_z() - Z_STEP;
    //int32_t delta_half = 0; // = get_step_count_z() - Z_STEP;
    static uint32_t adc_count = 0;
    //static uint8_t trigge_count = 0;

    if(align_start_flag == 1)
    {
        align_start_flag = 0;
        ticks = 0;
        for(i = 0; i < 4; i++)
        {
            adc_val_pre[i] = 0;
            if(adc_flag == 0)
            {
                adc_val_min_t[i] = 4095;
                adc_val_max_t[i] = 0;
            }
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
    //printf_("3");
    // if(ticks != 0)
    // {
    //     if(tick_is_earlier(ticks  + timer_from_us(500), timer_read_time()) == 0)
    //     {
    //         return 1;
    //     }
    // }
    // ticks = 0;

    for(i = 0; i < 4; i++)
    {
        if(gpio_adc_sample(gPF[i]) == 0)
        {
            val = gpio_adc_read(gPF[i]);
            step = get_step_count_z() - align_start_step;
            if(ABS(step) <= 1200)
            {
                if(val > (ADC_MAX(adc_val_max[i],ADC_MAX_OFFSET))||val < (ADC_MIN(adc_val_min[i],ADC_MAX_OFFSET)))
                {
                    //ticks = timer_read_time();
                    trigge_count = MAX_TRIGGE_COUNT;
                    Z_STEP = get_step_count_z();
                    u8buffer_push(3);
                    return 1;
                }
                return 0;
            }
            else if(ABS(step) > 1200 && ABS(step) <2400)
            {
                if(adc_flag == 0)
                {
                    adc_val_min_t[i] = MIN(adc_val_min_t[i], val);
                    adc_val_max_t[i] = MAX(adc_val_max_t[i], val);
                }
                insert_adc_data_to_group(i, (struct adc_buff){.adc_val = val, .step = get_step_count_z(),. trigger = 2});
                adc_val_pre[i] = val;
                
                if(val > (ADC_MAX(adc_val_max[i],ADC_MAX_OFFSET))||val < (ADC_MIN(adc_val_min[i],ADC_MAX_OFFSET)))
                {
                    //ticks = timer_read_time();
                    //u8buffer_push(3);
                    trigge_count = MAX_TRIGGE_COUNT;
                    Z_STEP = get_step_count_z();
                    return 1;
                }
                continue;
            }
            else
            {
                adc_flag = 1;
                val_delta = (adc_val_max_t[i] -adc_val_min_t[i])*delta/100;
                //delta_half = (val_delta*1.3); 
                insert_adc_data_to_group(i, (struct adc_buff){.adc_val = val, .step = get_step_count_z(),. trigger = 0});
                if(Z_STEP == 0xFFFFFFFF)
                {
                    if(val > (ADC_MAX(adc_val_max[i],ADC_MAX_OFFSET))||val < (ADC_MIN(adc_val_min[i],ADC_MAX_OFFSET)))
                    {
                        //ticks = timer_read_time();
                        trigge_count = MAX_TRIGGE_COUNT;
                        Z_STEP = get_step_count_z();
                        //u8buffer_push(3);
                        return 1;
                    }
                }
                else
                {
                    if(val > (ADC_MAX(adc_val_pre[i],val_delta))||val < (ADC_MIN(adc_val_pre[i],val_delta)))
                    {
                        insert_adc_data_to_group(i, (struct adc_buff){.adc_val = val, .step = get_step_count_z(),. trigger = 1});
                        //u8buffer_push(1);
                        //ticks = timer_read_time();
                        trigge_count = MAX_TRIGGE_COUNT;
                        return 1;
                    }
                }
            }
        }  
    }     
    return 0; 
}

DECL_COMMAND(aniso_bed_mesh_cmd,"aniso_bed_mesh_cmd oid=%u op=%u val=%u");
DECL_TASK(bed_mesh_task);
//DECL_INIT(adc3_gpio_init);