// Support for ADS131M08 8-Channel ADC Chip
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "aniso_zoffset_align.h"
#include "ads131m0x.h"
#include "stepper.h" // stepper_event


#define BUFFER_VALUE_CNT 200                          // 采样缓存的容量
#define ALIGN_VALUE_CNT 10                            // 触发判断所需的最少缓存数量
#define SAMPLE_INTERVAL_US (1000*1000)                        // 采样间隔200微秒

static float sample_freq;                             // 采样频率
static float sample_val_mean;                         // 采样均值
static uint32_t sample_init_counts;                     // 当前采样周期次数
static struct aniso_ads131m08_data_t sample_datas_ar[BUFFER_VALUE_CNT]; // 采样缓存array
static struct aniso_ring_buffer_t sample_datas_rb;                  // 采样缓存ring_buffer
static struct aniso_kalman_filter_t kmf;              // 卡尔曼滤波器
static struct aniso_moving_average_filter_t maf;      // 滑动均值滤波器
static uint32_t sample_count = 0;                          // 采样总次数
static uint32_t sample_count_now = 0;                      // 当前采样次数

static float ads131_adc_val_min = INFINITY; // 初始为最大值，便于后续取最小值
static float ads131_adc_val_max = -INFINITY;         // 初始为最小值，便于后续取最大值

// Global variables for Z offset alignment
uint8_t state_zoffset_flag = ANISO_ZOFFSET_STATE_IDLE;
float trigger_position = 0.0f;
uint8_t trigger_detected = 0;
struct trsync *zoffset_trsync = NULL;
uint8_t trigger_reason = 1;
uint8_t error_reason = 2;


// Function prototypes
void command_config_zoffset_align(uint32_t *args);
void ads131m08_read_adc(struct ads131m08_adc *ads131m08, uint8_t oid);

// Flag types
enum {
    FLAG_PENDING = 1 << 0
};

#define BYTES_PER_SAMPLE 4
#define CHANNELS_PER_DEVICE 8
#define WORDS_PER_FRAME 10  // STATUS + 8 channels + CRC
#define BYTES_PER_FRAME (WORDS_PER_FRAME * 3)  // 24-bit words (保持帧结构不变)

static struct task_wake wake_zoffset_task;

/****************************************************************
 * ADS131M08 Sensor Support
 ****************************************************************/

int8_t
is_adc_data_ready(struct ads131m08_adc *ads131m08) {
    return gpio_in_read(ads131m08->data_ready) == 0;
}

// 定时器检测ads131数据是否ready,启动task读取数据
// 检查是否触发
static uint_fast8_t
aniso_zoffset_align_event(struct timer *timer)
{
    struct ads131m08_adc *ads131m08 = container_of(timer, struct ads131m08_adc,
                                                   timer);
    uint32_t rest_ticks = ads131m08->rest_ticks;

    if (is_adc_data_ready(ads131m08)) {
        ads131m08->pending_flag = 1;
        sched_wake_task(&wake_zoffset_task);
    }
    ads131m08->timer.waketime += timer_from_us(rest_ticks);
    return SF_RESCHEDULE;
}

uint8_t check_trigger(void)
{
    if (aniso_ring_buffer_size(&sample_datas_rb) < BUFFER_VALUE_CNT)
    {
        return 0;
    }
    // struct aniso_ads131m08_data_t temp_data;
    // aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data); // 本周期最新数据
    // if (temp_data.flt_val > 1000 ||
    //     temp_data.flt_val < -1000) // 超出动态阈值，才算本周期有触发
    // {   
    //     printf_("Triggered: adc_val= %u\r\n", temp_data.flt_val);
    //     return 1;
    // }

    // float temp_range = (ads131_adc_val_max - ads131_adc_val_min) * 1.5f;
    // struct aniso_ads131m08_data_t temp_data0, temp_data1, temp_data2, temp_data3, temp_data4;
    // // aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data0);
    // // aniso_ring_buffer_pick(&sample_datas_rb, 1, &temp_data1);
    // // aniso_ring_buffer_pick(&sample_datas_rb, 2, &temp_data2);
    // // aniso_ring_buffer_pick(&sample_datas_rb, 3, &temp_data3);
    // // aniso_ring_buffer_pick(&sample_datas_rb, 4, &temp_data4);

    // uint8_t trigger_count = 0;
    // for (int16_t i = 0; i < 6; i++)
    // {
    //     aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data0);
    //     if (temp_data0.flt_val > temp_range || temp_data0.flt_val < -temp_range) {
    //         printf_("Triggered: adc_val= %.3f, temp_range= %.3f\r\n", temp_data0.flt_val, temp_range);
    //         trigger_count++;
    //     }
    // }

    // if (trigger_count >= 4) {
    //     printf_("Triggered count: %u\r\n", trigger_count);
    //     return 1;
    // }

    ads131_adc_val_min = INFINITY;
    ads131_adc_val_max = -INFINITY;
    struct aniso_ads131m08_data_t temp_data;
    for (int16_t i = 80; i < BUFFER_VALUE_CNT; i++) // 计算最值（缓存的一半旧数据）
    {
        aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data);
        if (temp_data.flt_val < ads131_adc_val_min)
            ads131_adc_val_min = temp_data.flt_val;
        if (temp_data.flt_val > ads131_adc_val_max)
            ads131_adc_val_max = temp_data.flt_val;
    }

    float max_factor = 0.3f;
    float min_factor = 0.3f;
    float val_max = ads131_adc_val_max + ads131_adc_val_max*max_factor;
    float val_min = ads131_adc_val_min - ads131_adc_val_min*min_factor;
    aniso_ring_buffer_pick(&sample_datas_rb, 0, &temp_data); // 本周期最新数据
    if (temp_data.flt_val > val_max ||
        temp_data.flt_val < val_min) // 超出动态阈值，才算本周期有触发
    {   
        printf_("Triggered: adc_val= %u, val_max= %.3f, val_min= %.3f\r\n", temp_data.flt_val, val_max, val_min);
        return 1;
    }
    return  0;
}

// // Add a measurement to the buffer
// static void
// add_sample(struct ads131m08_adc *ads131m08, uint8_t oid, 
//            uint_fast32_t counts, uint8_t channel)
// {
//     // Store channel number in upper bits of timestamp
//     uint32_t sample = counts | ((uint32_t)channel << 24);
//     // printf_("ADS131M08 Channel %u Sample: 0x%08X\n", channel, sample);
    
//     ads131m08->sb.data[ads131m08->sb.data_count] = (sample >> 0) & 0xFF;
//     ads131m08->sb.data[ads131m08->sb.data_count + 1] = (sample >> 8) & 0xFF;
//     ads131m08->sb.data[ads131m08->sb.data_count + 2] = (sample >> 16) & 0xFF;
//     ads131m08->sb.data[ads131m08->sb.data_count + 3] = (sample >> 24) & 0xFF;
//     ads131m08->sb.data_count += BYTES_PER_SAMPLE;

//     // printf_("ADS131M08 sb.data @%u: %02X %02X %02X %02X\n",
//     //     ads131m08->sb.data_count,
//     //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count]),
//     //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count+1]),
//     //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count+2]),
//     //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count+3]));

//     if ((ads131m08->sb.data_count + BYTES_PER_SAMPLE) >
//             ARRAY_SIZE(ads131m08->sb.data)) {
//         sensor_bulk_report(&ads131m08->sb, oid);
//     }
// }

// ADS131M08 ADC query - optimized for single channel processing
void
aniso_zoffset_align_read_adc(struct ads131m08_adc *ads131m08, uint8_t oid)
{
    struct aniso_ads131m08_data_t sample_data = {0};
    // Send NULL command (0x0000) to read conversion data
    uint8_t tx_frame[BYTES_PER_FRAME] = {0};
    uint8_t rx_frame[BYTES_PER_FRAME] = {0};
    
    // Send NULL command and read response frame
    // Copy tx data to rx buffer for transfer
    memcpy(rx_frame, tx_frame, BYTES_PER_FRAME);
    spidev_transfer(ads131m08->spi, 1, BYTES_PER_FRAME, rx_frame);
    ads131m08->pending_flag = 0;
    barrier();

    // 零基通道索引 ch (0..7)
    int ch = ads131m08->enabled_channel - 1;
    int byte_offset = 3 + ch * 3; // 使用 rx_frame[byte_offset], [byte_offset+1], [byte_offset+2]

    // Optimized parsing for single channel (channel 0 only)
    // Skip STATUS word (first 3 bytes) and process only channel 0
    // if (ads131m08->enabled_channel & 0x02) { // Check if channel 0 is enabled
        // int byte_offset = 6; // Skip STATUS, channel 1 starts at byte 6
        
        // Extract 20-bit data from response for channel 0 only
        // Use only 20 bits: bits 23-4 from the original 24-bit data (remove lower 4 bits)
        uint32_t counts = ((uint32_t)rx_frame[byte_offset] << 16)
                        | ((uint32_t)rx_frame[byte_offset + 1] << 8)
                        | ((uint32_t)rx_frame[byte_offset + 2]);

        sample_data.raw_val = (float)counts;
        sample_data.flt_val = sample_data.raw_val;
        aniso_ring_buffer_push(&sample_datas_rb, &sample_data);
        printf_("Sample counts = %u, raw = %.3f, flt = %.3f\n", 
            counts, sample_data.raw_val, sample_data.flt_val);

        // // counts = counts / 100.0;
        // if (sample_init_counts >= ALIGN_VALUE_CNT) {
        //     // printf_("Sample cycle now: %u, mean = %.3f\n", sample_init_counts, sample_val_mean);
        //     // sample_data.raw_val = (float)(counts - sample_val_mean);
        //     sample_data.raw_val = (float)counts;
            
        //     // 使用滑动均值滤波处理数据
        //     float ma_filtered_val = aniso_moving_average_filter_process_remove_outliers(&maf, counts);
        //     sample_data.flt_val = ma_filtered_val;
        //     // 然后使用卡尔曼滤波进一步平滑
        //     // sample_data.flt_val = aniso_kalman_filter_process(&kmf, ma_filtered_val);

        //     // struct aniso_ads131m08_data_t temp_data;
        //     // aniso_ring_buffer_pick(&sample_datas_rb, ALIGN_VALUE_CNT, &temp_data);
        //     // if (temp_data.flt_val > ads131_adc_val_max)
        //     //     ads131_adc_val_max = temp_data.flt_val;
        //     // if (temp_data.flt_val < ads131_adc_val_min)
        //     //     ads131_adc_val_min = temp_data.flt_val;
            
        //     aniso_ring_buffer_push(&sample_datas_rb, &sample_data);
        //     printf_("Sample counts = %u, flt = %.3f, max-min = %.3f, mean = %.3f\n", 
        //         counts, sample_data.flt_val, ads131_adc_val_max-ads131_adc_val_min, sample_val_mean);
        // }
        // else {
        //     sample_data.raw_val = (float)counts;
        //     float ma_filtered_val = aniso_moving_average_filter_process_remove_outliers(&maf, counts);
        //     sample_data.flt_val = ma_filtered_val;
        //     aniso_ring_buffer_push(&sample_datas_rb, &sample_data);

        //     sample_init_counts++;
        //     sample_val_mean += (ma_filtered_val - sample_val_mean) / sample_init_counts; // 在线计算历史均值

        //     // if (sample_data.flt_val > ads131_adc_val_max)
        //     //     ads131_adc_val_max = sample_data.flt_val;
        //     // if (sample_data.flt_val < ads131_adc_val_min)
        //     //     ads131_adc_val_min = sample_data.flt_val;

        //     printf_("Init Sample counts = %u, raw val = %.3f, flt val = %.3f, mean = %.3f\n", 
        //         counts, sample_data.raw_val, sample_data.flt_val, sample_val_mean);
        // }
}

// Send RESET command (0x0011) to ADS131M08 with frame padding
void
aniso_zoffset_align_reset_chip(struct ads131m08_adc *ads131m08)
{
    if (!ads131m08 || !ads131m08->spi) {
        printf_("aniso_zoffset_align_reset_chip: invalid ads or spi\n");
        return;
    }

    const uint16_t RESET_CMD = 0x0011;
    uint8_t tx_frame[BYTES_PER_FRAME] = {0};
    uint8_t rx_frame[BYTES_PER_FRAME];

    tx_frame[0] = (uint8_t)((RESET_CMD >> 8) & 0xFF);
    tx_frame[1] = (uint8_t)(RESET_CMD & 0xFF);

    memcpy(rx_frame, tx_frame, BYTES_PER_FRAME);
    spidev_transfer(ads131m08->spi, 1, BYTES_PER_FRAME, rx_frame);

    ads131m08->pending_flag = 0;
    barrier();
}

// Helper to write a 16-bit register to ADS131M08.
// ADS131M08 requires WREG command format: [0x60 | (reg_addr >> 2)][(reg_addr & 0x03) << 6 | 0x00][MSB][LSB]
static void
ads131m08_write_reg(struct ads131m08_adc *ads, uint8_t reg_addr, uint16_t value)
{
    /* ADS131M08 WREG command format: 4 bytes total */
    uint8_t tx_frame[4] = {0};
    uint8_t rx_frame[4] = {0};

    // WREG command format: [0x60 | (reg_addr >> 2)][(reg_addr & 0x03) << 6 | 0x00][MSB][LSB]
    tx_frame[0] = (uint8_t)(OPCODE_WREG | ((reg_addr >> 2) & 0x0F));  // Command + upper 4 bits of address
    tx_frame[1] = (uint8_t)(((reg_addr & 0x03) << 6) | 0x00);         // Lower 2 bits of address + 0x00
    tx_frame[2] = (uint8_t)((value >> 8) & 0xFF);                     // Register value MSB
    tx_frame[3] = (uint8_t)(value & 0xFF);                            // Register value LSB

    /* Copy tx into rx buffer for in-place SPI transfer APIs that overwrite the buffer */
    memcpy(rx_frame, tx_frame, sizeof(tx_frame));
    spidev_transfer(ads->spi, 1, sizeof(tx_frame), rx_frame);

    ads->pending_flag = 0;
    barrier();
}

// Configure ADS131M08 registers (converted from the provided Python logic).
// osr: one of OSR_* (0-7)
// power_mode: one of PWR_* (0-2)
// channel_gains: array[8] with gain codes (0..7)
void
aniso_zoffset_align_setup_chip(struct ads131m08_adc *ads, uint32_t osr,
                               uint8_t power_mode, const uint8_t channel_gains[8])
{
    // Reset chip first
    aniso_zoffset_align_reset_chip(ads);

    // MODE register: default with 24-bit word length (0x0510)
    // 0x0510 = 24-bit word length, timeout enabled, default DRDY settings
    uint16_t mode_reg = 0x0510u;
    ads131m08_write_reg(ads, ADS131M08_MODE, mode_reg);

    // CLOCK register: enable channels based on enabled_channel bitmask
    // OSR bits [4:2], PWR bits [1:0], RESERVED bits [7:5]
    uint16_t clock_reg = 0;
    
    // Set channel enable bits based on enabled_channel bitmask
    if (ads->enabled_channel & 0x01) clock_reg |= CLOCK_CH0_EN_ENABLED;
    if (ads->enabled_channel & 0x02) clock_reg |= CLOCK_CH1_EN_ENABLED;
    if (ads->enabled_channel & 0x04) clock_reg |= CLOCK_CH2_EN_ENABLED;
    if (ads->enabled_channel & 0x08) clock_reg |= CLOCK_CH3_EN_ENABLED;
    if (ads->enabled_channel & 0x10) clock_reg |= CLOCK_CH4_EN_ENABLED;
    if (ads->enabled_channel & 0x20) clock_reg |= CLOCK_CH5_EN_ENABLED;
    if (ads->enabled_channel & 0x40) clock_reg |= CLOCK_CH6_EN_ENABLED;
    if (ads->enabled_channel & 0x80) clock_reg |= CLOCK_CH7_EN_ENABLED;
    
    // Set OSR (bits [4:2]) and PWR (bits [1:0])
    clock_reg |= (uint16_t)((osr & 0x07) << 2);
    clock_reg |= (uint16_t)(power_mode & 0x03);
    
    ads131m08_write_reg(ads, ADS131M08_CLOCK, clock_reg);

    // GAIN1 packs gains for channels 0..3 (4 nibbles, 3-bit each)
    // Each gain occupies 3 bits with reserved bit between them
    uint16_t gain1_reg = 0;
    for (int i = 0; i < 4; ++i) {
        gain1_reg |= (uint16_t)((channel_gains[i] & 0x07) << (i * 4));
    }
    ads131m08_write_reg(ads, ADS131M08_GAIN1, gain1_reg);

    // GAIN2 packs gains for channels 4..7
    uint16_t gain2_reg = 0;
    for (int i = 4; i < 8; ++i) {
        gain2_reg |= (uint16_t)((channel_gains[i] & 0x07) << ((i - 4) * 4));
    }
    ads131m08_write_reg(ads, ADS131M08_GAIN2, gain2_reg);

    // Default channel configuration: normal input, no phase shift -> 0x0000
    const uint8_t ch_regs[8] = {
        ADS131M08_CH0_CFG, ADS131M08_CH1_CFG, ADS131M08_CH2_CFG, ADS131M08_CH3_CFG,
        ADS131M08_CH4_CFG, ADS131M08_CH5_CFG, ADS131M08_CH6_CFG, ADS131M08_CH7_CFG
    };
    for (int i = 0; i < 8; ++i) {
        ads131m08_write_reg(ads, ch_regs[i], 0x0000u);
    }

    printf_("ADS131M08: Chip configuration complete (osr=%u power=%u clock=0x%04X)\n",
            (unsigned)osr, (unsigned)power_mode, clock_reg);
}

// Configure trsync for Z offset alignment
void
command_config_zoffset_trsync(uint32_t *args)
{
    uint8_t trsync_oid = args[0];
    
    // Lookup the trsync object
    zoffset_trsync = trsync_oid_lookup(trsync_oid);
    
    printf_("Z offset alignment: Configured trsync oid=%u\n", trsync_oid);
}
DECL_COMMAND(command_config_zoffset_trsync, "config_zoffset_trsync oid=%c");

// Create an ads131m08 sensor
void
command_config_zoffset_align(uint32_t *args)
{
    ads131_adc_val_min = INFINITY;
    ads131_adc_val_max = -INFINITY;
    sample_init_counts = 0;
    sample_val_mean = 0.0f;
    memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
    aniso_ring_buffer_init(&sample_datas_rb, sample_datas_ar, BUFFER_VALUE_CNT, sizeof(struct aniso_ads131m08_data_t));
    // aniso_kalman_filter_init(&kmf, 0.2f, 0.03f, 0.2f);
    aniso_moving_average_filter_init(&maf, 20); // 初始化滑动均值滤波器，窗口大小为10
    // aniso_iir_filter_init(&bsf, ANISO_FILTER_BAND_STOP, 50.0f, 10.0f, sample_freq);
    // aniso_iir_filter_init(&lpf, ANISO_FILTER_LOW_PASS, 3000.0f, 0.707f, sample_freq);

    uint8_t oid = args[0];
    uint8_t spi_oid = args[1];
    // uint32_t data_ready_pin = args[2];
    uint8_t enabled_channel = args[3];
    uint8_t channel_gain = args[4];
    uint8_t power_mode = args[5];
    uint32_t sps = args[6] * 1000;
    uint32_t osr = args[7];

    struct ads131m08_adc *ads131m08 = oid_alloc(oid, command_config_zoffset_align, sizeof(*ads131m08));
    ads131m08->timer.func = aniso_zoffset_align_event;
    ads131m08->pending_flag = 0;
    ads131m08->spi = spidev_oid_lookup(spi_oid);
    ads131m08->data_ready = gpio_in_setup(args[2], 0);
    ads131m08->enabled_channel = enabled_channel; // Bitmask of enabled channels
    ads131m08->rest_ticks = sps;
    ads131m08->default_rest_ticks = sps; // Store default sampling interval

    // Map user-provided channel_gain (1,2,4,8,16,32,64,128) to ADS131M08 gain code (0..7)
    // 1->0, 2->1, 4->2, 8->3, 16->4, 32->5, 64->6, 128->7
    uint8_t gain_code;
    switch (channel_gain) {
    case 1:   gain_code = ADS131M08_GAIN_1; break;
    case 2:   gain_code = ADS131M08_GAIN_2; break;
    case 4:   gain_code = ADS131M08_GAIN_4; break;
    case 8:   gain_code = ADS131M08_GAIN_8; break;
    case 16:  gain_code = ADS131M08_GAIN_16; break;
    case 32:  gain_code = ADS131M08_GAIN_32; break;
    case 64:  gain_code = ADS131M08_GAIN_64; break;
    case 128: gain_code = ADS131M08_GAIN_128; break;
    // If caller passed an index 0..7, accept it directly; otherwise default to code 1 (gain x2)
    default:
        gain_code = ADS131M08_GAIN_1;
        break;
    }

    // Use the mapped gain_code for all channels (adjust if per-channel control is needed)
    uint8_t channel_gains[8] = {
        gain_code, gain_code, gain_code, gain_code,
        gain_code, gain_code, gain_code, gain_code
    };

    aniso_zoffset_align_setup_chip(ads131m08, osr, power_mode, channel_gains);

    // Start new measurements
    // irq_disable();
    // ads131m08->timer.waketime = timer_read_time() + timer_from_us(ads131m08->rest_ticks);
    // sched_add_timer(&ads131m08->timer);
    // irq_enable();
    printf_("Configured zoffset align oid=%u spi_oid=%u drdy_pin=%u enable_channel=0x%02X\n",
            oid, spi_oid, args[2], ads131m08->enabled_channel);
}
DECL_COMMAND(command_config_zoffset_align, "config_zoffset_align oid=%c spi_oid=%c "
    "data_ready_pin=%u enable_channel=%c channel_gain=%c power_mode=%c sps=%u osr=%u");

// start/stop capturing ADC data
void
command_query_zoffset_align(uint32_t *args)
{
    uint8_t oid = args[0];
    // printf_("command_query_zoffset_align called: oid=%u rest_ticks=%u\r\n", oid, args[1]);
    struct ads131m08_adc *ads131m08 = oid_lookup(oid, command_config_zoffset_align);
    sched_del_timer(&ads131m08->timer);
    ads131m08->pending_flag = 0;
    ads131m08->rest_ticks = args[1];
    ads131m08->default_rest_ticks = args[1]; // Store default sampling interval
    sample_freq = (float)CONFIG_CLOCK_FREQ / ads131m08->rest_ticks;
    if (!ads131m08->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    irq_disable();
    ads131m08->timer.waketime = timer_read_time() + ads131m08->rest_ticks;
    sched_add_timer(&ads131m08->timer);
    irq_enable();
    printf_("Started ADS131M08 measurements oid=%u rest_ticks=%u\n",
            oid, ads131m08->rest_ticks);
}
DECL_COMMAND(command_query_zoffset_align, "query_zoffset_align oid=%c rest_ticks=%u");

void
command_query_zoffset_align_status(const uint32_t *args)
{
    printf_("command_query_zoffset_align_status called: oid=%u\r\n", args[0]);
    uint8_t oid = args[0];
    struct ads131m08_adc *ads131m08 = oid_lookup(oid, command_config_zoffset_align);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = is_adc_data_ready(ads131m08);
    irq_enable();
    
    // Optimized pending bytes calculation for single channel (channel 0 only)
    uint8_t pending_bytes = (is_data_ready && (ads131m08->enabled_channel & 0x01)) 
                          ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&ads131m08->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_zoffset_align_status, "query_zoffset_align_status oid=%c");

// Handle trigger stop - called when trigger is detected
void
handle_trigger_stop(void)
{
    printf_("handle_trigger_stop called: %d\r\n", state_zoffset_flag);
    if (state_zoffset_flag == ANISO_ZOFFSET_STATE_MOVING) {
        // Get current Z position (this would need to be implemented with stepper position tracking)
        // For now, we'll use a placeholder - in real implementation this should get actual Z position
        trigger_position = 0.0f; // Placeholder - should be replaced with actual Z position query
        
        trigger_detected = 1;
        
        // Use trsync_do_trigger to actually stop the Z axis motor
        if (zoffset_trsync != NULL) {
            printf_("Z offset alignment: Trigger detected, stopping Z axis at position %.3f using trsync\n", trigger_position);
            trsync_do_trigger(zoffset_trsync, trigger_reason);
        } else {
            printf_("Z offset alignment: Trigger detected, but no trsync configured. Position: %.3f\n", trigger_position);
        }
        
        // Set state to triggered after calling trsync_do_trigger
        state_zoffset_flag = ANISO_ZOFFSET_STATE_TRIGGERED;
    }
}

// Start Z offset alignment process
void
command_start_zoffset_align(uint32_t *args)
{    
    // if (state_zoffset_flag != ANISO_ZOFFSET_STATE_IDLE) {
    //     printf_("Z offset alignment: Cannot start, already in state %d\n", state_zoffset_flag);
    //     return;
    // }
    
    trigger_detected = 0;
    trigger_position = 0.0f;
    state_zoffset_flag = ANISO_ZOFFSET_STATE_MOVING;
    ads131_adc_val_min = INFINITY;
    ads131_adc_val_max = -INFINITY;
    sample_init_counts = 0;
    sample_val_mean = 0.0f;
    memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
    aniso_ring_buffer_init(&sample_datas_rb, sample_datas_ar, BUFFER_VALUE_CNT, sizeof(struct aniso_ads131m08_data_t));
    aniso_moving_average_filter_reset(&maf);

    uint8_t oid;
    struct ads131m08_adc *ads131m08;
    foreach_oid(oid, ads131m08, command_config_zoffset_align) {
        sched_del_timer(&ads131m08->timer);
        irq_disable();
        ads131m08->timer.waketime = timer_read_time() + timer_from_us(ads131m08->rest_ticks);
        sched_add_timer(&ads131m08->timer);
        irq_enable();
    }
    
    sendf("aniso_zoffset_align_started oid=%c", oid);
    printf_("Z offset alignment: Started moving check triggered\n");
    
    // Start Z axis movement (this would need to be implemented with actual stepper control)
    // For now, we just set the state and let the trigger detection handle stopping
}
DECL_COMMAND(command_start_zoffset_align, "start_zoffset_align oid=%c");

// Stop Z offset alignment process
void
command_stop_zoffset_align(uint32_t *args)
{
    if (state_zoffset_flag == ANISO_ZOFFSET_STATE_MOVING) {
        state_zoffset_flag = ANISO_ZOFFSET_STATE_IDLE;
        printf_("Z offset alignment: Stopped by command\n");
    } else {
        printf_("Z offset alignment: Not in moving state, current state %d\n", state_zoffset_flag);
    }
}
DECL_COMMAND(command_stop_zoffset_align, "stop_zoffset_align");

// Get Z offset position after trigger
void
command_check_zoffset_align_triggered(uint32_t *args)
{
    uint8_t oid = args[0];
    // struct ads131m08_adc *ads131m08 = oid_lookup(oid, command_config_zoffset_align);

    // sendf("aniso_zoffset_align_result oid=%c triggered=%c", oid, state_zoffset_flag == ANISO_ZOFFSET_STATE_TRIGGERED ? 1 : 0);
    if (state_zoffset_flag == ANISO_ZOFFSET_STATE_TRIGGERED) {
        sendf("aniso_zoffset_align_result oid=%c triggered=%c", oid, 1);
        state_zoffset_flag = ANISO_ZOFFSET_STATE_IDLE; // Reset state after reporting
        printf_("Z offset alignment triggered\n");
    } else {
        sendf("aniso_zoffset_align_result oid=%c triggered=%c", oid, 0);
        printf_("Z offset alignment: Not triggered yet, current state %d\n", state_zoffset_flag);
    }
}
DECL_COMMAND(command_check_zoffset_align_triggered, "check_zoffset_align_triggered oid=%c");

void
command_start_get_adc_value(uint32_t *args)
{
    uint8_t timems = args[1];

    trigger_detected = 0;
    trigger_position = 0.0f;
    state_zoffset_flag = ANISO_ZOFFSET_STATE_MOVING;
    ads131_adc_val_min = INFINITY;
    ads131_adc_val_max = -INFINITY;
    sample_init_counts = 0;
    sample_val_mean = 0.0f;
    memset(sample_datas_ar, 0, sizeof(sample_datas_ar));
    aniso_ring_buffer_init(&sample_datas_rb, sample_datas_ar, BUFFER_VALUE_CNT, sizeof(struct aniso_ads131m08_data_t));
    aniso_moving_average_filter_reset(&maf);

    uint8_t oid;
    struct ads131m08_adc *ads131m08;
    foreach_oid(oid, ads131m08, command_config_zoffset_align) {
        sched_del_timer(&ads131m08->timer);
        irq_disable();
        ads131m08->timer.waketime = timer_read_time() + timer_from_us(ads131m08->rest_ticks);
        sched_add_timer(&ads131m08->timer);
        irq_enable();
        sample_count = timems * 1000 / (ads131m08->rest_ticks); // 计算总采样次数
        sample_count_now = 0;
    }

    // printf_("Started ADS131M08 measurements for adc value oid=%u count=%u\n",
    //         oid, sample_count);
}
DECL_COMMAND(command_start_get_adc_value, "start_get_adc_value oid=%c timems=%u");

// Background task that performs measurements
void
aniso_zoffset_align_capture_task(void)
{
    if (!sched_check_wake(&wake_zoffset_task) || state_zoffset_flag != ANISO_ZOFFSET_STATE_MOVING)
        return;
    // printf_("aniso_zoffset_align_capture_task running\r\n");

    uint8_t oid;
    struct ads131m08_adc *ads131m08;
    foreach_oid(oid, ads131m08, command_config_zoffset_align) {
        if (ads131m08->pending_flag) {
            // printf_("aniso_zoffset_align_capture_task running 1111\r\n");
            aniso_zoffset_align_read_adc(ads131m08, oid);
        }

        uint32_t buffer_counts = aniso_ring_buffer_size(&sample_datas_rb);
        if (buffer_counts >= sample_count || buffer_counts >= BUFFER_VALUE_CNT) 
        {    
            irq_disable();
            sched_del_timer(&ads131m08->timer);
            irq_enable();

            // Calculate average of buffer excluding one min and one max value
            struct aniso_ads131m08_data_t temp_data;
            float sum = 0.0f;
            float min_val = INFINITY;
            float max_val = -INFINITY;

            for (uint32_t i = 0; i < buffer_counts; ++i) {
                aniso_ring_buffer_pick(&sample_datas_rb, i, &temp_data);
                float v = temp_data.flt_val;
                sum += v;
                if (v < min_val) min_val = v;
                if (v > max_val) max_val = v;
            }

            uint32_t result = 0;
            result = (sum - min_val - max_val) / (float)(buffer_counts - 2);
            sendf("aniso_zoffset_align_adc_value oid=%c adc_value=%u", oid, result);

            printf_("Sample counts = %u, flt = %u\n", buffer_counts, result);
        }

        // if (check_trigger()) {
        //     // Trigger detected: stop further timer wakeups but ensure one final read is processed.
        //     // Remove the timer from scheduler so it won't be rescheduled.
        //     irq_disable();
        //     sched_del_timer(&ads131m08->timer);
        //     irq_enable();

        //     state_zoffset_flag = ANISO_ZOFFSET_STATE_TRIGGERED;

        //     // If we're in moving state and trigger is detected, handle the stop
        //     // if (state_zoffset_flag == ANISO_ZOFFSET_STATE_MOVING) {
        //     //     handle_trigger_stop();
        //     // }
        //     printf_("aniso_zoffset_align_event: trigger detected, timer stopped\n");
        // }
    }
}
DECL_TASK(aniso_zoffset_align_capture_task);
