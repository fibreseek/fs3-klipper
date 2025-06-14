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
#define ADC_MAX_OFFSET  250
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

//static uint32_t val1=0,val2=0,val3=0,val4=0,val5=0,val6=0,val7=0,val8=0,val9=0,val10=0;
// static uint32_t val_max[4] = {0};
// static uint32_t val_min[4] = {0};
// static uint32_t index_array[100];
//static uint16_t index_offset = 1;
static uint8_t trigge_flag[4] = {0, 0, 0, 0};
static uint8_t adc_flag = 0;

typedef struct {
    uint32_t data[50];
    uint8_t count;  // 当前已有数据个数（最大到20）
} U8Buffer;

U8Buffer my_buf = {0};

void aniso_bed_mesh_cmd(uint32_t *args)
{
    //uint32_t delay;
    uint32_t op;
    uint32_t val;
    uint8_t i=0;
    uint8_t res = 0;
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
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, op, 0);
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
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, op, res);
    }
    else if(op == 3)   
    {
        startb = 0;
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, op, 0);
    }
    else if(op == 4)   
    {
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 10000+adc_val_min[val]);
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 20000+adc_val_max[val]);
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 30000+adc_val_min_t[val]);
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 40000+adc_val_max_t[val]);
        // if(val == 0)
        // {
        //      memset(index_array, 0, sizeof(index_array));
        //     index_offset = 1;
        // }
        while(i < 30 )
        { 
            sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, my_buf.data[i]);
            i++;
        }
        // sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 10000+val1);
        // sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 20000+val2);
        // sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 50000+val3);
        // sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 10000+val_max[val]);
        // sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 4, 20000+val_min[val]);


    }
    else if(op == 5)   
    {
        Z_STEP = get_step_count_z();
        ticks = 0;
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 5, Z_STEP);
    }
    else if(op == 6)   
    {
        Z_STEP = get_step_count_z();
        ticks = 0;
        sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, 6, Z_STEP);
    }

   
	//sendf("aniso_bed_mesh oid=%c op=%c val=%u", goid, op, val);
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

    if(get_step_dir_z() == 0)
    {
        ticks = 0;
        for(i = 0; i < 4; i++)
        {
            adc_val_pre[i] = 0;
            trigge_flag[i] =0;
            if(adc_flag == 0)
            {
                adc_val_min_t[i] = 4095;
                adc_val_max_t[i] = 0;
            }
        }
        return 0;
        adc_count = 0; // 如果Z轴方向为0，则重置采样计数器
    }
    else
    {
        adc_count++;
        if(adc_count < 5)
        {
            return 0; // 如果还没到达采样周期，则不处理
        }
    }

    if(ticks != 0)
    {
        if(tick_is_earlier(ticks  + timer_from_us(500), timer_read_time()) == 0)
        {
            return 1;
        }
    }
    ticks = 0;

    for(i = 0; i < 4; i++)
    {
        if(gpio_adc_sample(gPF[i]) == 0)
        {
            val = gpio_adc_read(gPF[i]);

            if((Z_STEP != 0xFFFFFFFF))
            {
                step = get_step_count_z() - Z_STEP;
                
                if(ABS(step) > 800 && ABS(step) <2000)
                {
                    if(adc_flag == 0)
                    {
                        adc_val_min_t[i] = MIN(adc_val_min_t[i], val);
                        adc_val_max_t[i] = MAX(adc_val_max_t[i], val);
                    }
                    else
                    {
                        if(val > adc_val_max[i]+200||val <adc_val_min[i]-200)
                        {
                            ticks = timer_read_time();
                            u8buffer_push(2);
                            u8buffer_push(ABS(step));
                            u8buffer_push(val);
                            u8buffer_push(adc_val_max[i]+200);
                            u8buffer_push(adc_val_min[i]-200);
                            return 1;
                        }
                    }

                    adc_val_pre[i] = val;
                    continue;
                }
                else if(ABS(step) < 800)
                {
                    adc_flag = 1;
                    val_delta = (adc_val_max_t[i] -adc_val_min_t[i])*delta/100;
                    //delta_half = (val_delta*1.3); 

                    if(val > (adc_val_pre[i] + val_delta) || val < (adc_val_pre[i]-val_delta))
                    {
                        ticks = timer_read_time();
                        u8buffer_push(1);
                        return 1;
                    }
                    // if(val > (adc_val_pre[i]+val_delta) || val < (adc_val_pre[i]-val_delta))
                    // {
                    //     trigge_flag[i] = 1;
                    //     if(trigge_flag[1] + trigge_flag[2] + trigge_flag[3] + trigge_flag[0] > 1)
                    //     {
                    //         ticks = timer_read_time();
                    //         u8buffer_push(4);
                    //         return 1;
                    //     }
                    // }
                    // else
                    // {
                    //     trigge_flag[i] = 0;
                    // }
                }
                else
                {
                    return 0;
                }

            }
            else
            {
                if(val > adc_val_max[i]+200||val <adc_val_min[i]-200)
                {
                    ticks = timer_read_time();
                    u8buffer_push(3);
                    return 1;
                }
            }
        }
    }

    return 0;
}

DECL_COMMAND(aniso_bed_mesh_cmd,"aniso_bed_mesh_cmd oid=%u op=%u val=%u");
DECL_TASK(bed_mesh_task);
//DECL_INIT(adc3_gpio_init);