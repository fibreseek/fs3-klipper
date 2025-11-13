// Support for ADS131M08 8-Channel ADC Chip
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

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
#include "stm32/debug/debug.h"
#include "stm32/debug/printf.h"

struct ads131m08_adc {
    struct timer timer;
    uint32_t rest_ticks;
    uint32_t default_rest_ticks; // Store default sampling interval
    struct gpio_in data_ready;
    struct spidev_s *spi;
    uint8_t pending_flag, data_count;
    uint8_t enabled_channels; // Bitmask for enabled channels
    struct sensor_bulk sb;
    uint16_t overflow_history[8]; // History of overflows for rate adjustment
    uint8_t history_index; // Index for overflow history
};

// Function prototypes
void command_config_ads131m08(uint32_t *args);
void ads131m08_read_adc(struct ads131m08_adc *ads131m08, uint8_t oid);

// Flag types
enum {
    FLAG_PENDING = 1 << 0
};

#define BYTES_PER_SAMPLE 4
#define CHANNELS_PER_DEVICE 8
#define WORDS_PER_FRAME 10  // STATUS + 8 channels + CRC
#define BYTES_PER_FRAME (WORDS_PER_FRAME * 3)  // 24-bit words

static struct task_wake wake_ads131m08;

/****************************************************************
 * ADS131M08 Sensor Support
 ****************************************************************/

int8_t
ads131m08_is_data_ready(struct ads131m08_adc *ads131m08) {
    return gpio_in_read(ads131m08->data_ready) == 0;
}

// Event handler that wakes wake_ads131m08() periodically
static uint_fast8_t
ads131m08_event(struct timer *timer)
{
    // uint8_t oid;
    // struct ads131m08_adc *ads131m08 = container_of(timer, struct ads131m08_adc, timer);

    // uint32_t rest_ticks = ads131m08->rest_ticks;
    // foreach_oid(oid, ads131m08, command_config_ads131m08) {
    //     if (ads131m08_is_data_ready(ads131m08)) {
    //         ads131m08_read_adc(ads131m08, oid);
    //     }
    // }

    struct ads131m08_adc *ads131m08 = container_of(timer, struct ads131m08_adc,
                                                   timer);
    uint32_t rest_ticks = ads131m08->rest_ticks;

    if (ads131m08->pending_flag) {
        ads131m08->sb.possible_overflows++;
        rest_ticks *= 4;
    } else if (ads131m08_is_data_ready(ads131m08)) {
        ads131m08->pending_flag = 1;
        sched_wake_task(&wake_ads131m08);
        rest_ticks *= 20;
    }
    ads131m08->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

// Add a measurement to the buffer
static void
add_sample(struct ads131m08_adc *ads131m08, uint8_t oid, 
           uint_fast32_t counts, uint8_t channel)
{
    // Store channel number in upper bits of timestamp
    uint32_t sample = counts | ((uint32_t)channel << 24);
    // printf_("ADS131M08 Channel %u Sample: 0x%08X\n", channel, sample);
    
    ads131m08->sb.data[ads131m08->sb.data_count] = (sample >> 0) & 0xFF;
    ads131m08->sb.data[ads131m08->sb.data_count + 1] = (sample >> 8) & 0xFF;
    ads131m08->sb.data[ads131m08->sb.data_count + 2] = (sample >> 16) & 0xFF;
    ads131m08->sb.data[ads131m08->sb.data_count + 3] = (sample >> 24) & 0xFF;
    ads131m08->sb.data_count += BYTES_PER_SAMPLE;

    // printf_("ADS131M08 sb.data @%u: %02X %02X %02X %02X\n",
    //     ads131m08->sb.data_count,
    //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count]),
    //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count+1]),
    //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count+2]),
    //     (unsigned)(ads131m08->sb.data[ads131m08->sb.data_count+3]));

    if ((ads131m08->sb.data_count + BYTES_PER_SAMPLE) >
            ARRAY_SIZE(ads131m08->sb.data)) {
        sensor_bulk_report(&ads131m08->sb, oid);
    }
}

// ADS131M08 ADC query - optimized for single channel processing
void
ads131m08_read_adc(struct ads131m08_adc *ads131m08, uint8_t oid)
{
    // Send NULL command (0x0000) to read conversion data
    uint8_t tx_frame[BYTES_PER_FRAME] = {0};
    uint8_t rx_frame[BYTES_PER_FRAME] = {0};
    
    // Send NULL command and read response frame
    // Copy tx data to rx buffer for transfer
    memcpy(rx_frame, tx_frame, BYTES_PER_FRAME);
    spidev_transfer(ads131m08->spi, 1, BYTES_PER_FRAME, rx_frame);
    ads131m08->pending_flag = 0;
    barrier();

    // Optimized parsing for single channel (channel 0 only)
    // Skip STATUS word (first 3 bytes) and process only channel 0
    if (ads131m08->enabled_channels & 0x01) { // Check if channel 0 is enabled
        int byte_offset = 3; // Skip STATUS, channel 0 starts at byte 3
        
        // Extract 24-bit data from response for channel 0 only
        uint32_t counts = ((uint32_t)rx_frame[byte_offset] << 16)
                        | ((uint32_t)rx_frame[byte_offset + 1] << 8)
                        | ((uint32_t)rx_frame[byte_offset + 2]);

        // Extend 2's complement 24 bits to 32bits
        if (counts & 0x800000)
            counts |= 0xFF000000;
        // printf_("ADS131M08 Channel 0 Raw Counts: 0x%06X Extended Counts: 0x%08X\n",
        //         ((uint32_t)rx_frame[byte_offset] << 16)
        //       | ((uint32_t)rx_frame[byte_offset + 1] << 8)
        //       | ((uint32_t)rx_frame[byte_offset + 2]), counts);
        
        add_sample(ads131m08, oid, counts, 0);
    }    
}

// Create an ads131m08 sensor
void
command_config_ads131m08(uint32_t *args)
{
    struct ads131m08_adc *ads131m08 = oid_alloc(args[0]
                , command_config_ads131m08, sizeof(*ads131m08));
    ads131m08->timer.func = ads131m08_event;
    ads131m08->pending_flag = 0;
    ads131m08->spi = spidev_oid_lookup(args[1]);
    ads131m08->data_ready = gpio_in_setup(args[2], 0);
    ads131m08->enabled_channels = args[3]; // Bitmask of enabled channels
    ads131m08->history_index = 0;
    // Initialize overflow history to zero
    for (int i = 0; i < 8; i++) {
        ads131m08->overflow_history[i] = 0;
    }
    printf_("Configured ADS131M08 oid=%u spi_oid=%u drdy_pin=%u enabled_channels=0x%02X\n",
            args[0], args[1], args[2], ads131m08->enabled_channels);
}
DECL_COMMAND(command_config_ads131m08, "config_ads131m08 oid=%c"
    " spi_oid=%c data_ready_pin=%u enabled_channels=%c");

// start/stop capturing ADC data
void
command_query_ads131m08(uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m08_adc *ads131m08 = oid_lookup(oid, command_config_ads131m08);
    sched_del_timer(&ads131m08->timer);
    ads131m08->pending_flag = 0;
    ads131m08->rest_ticks = args[1];
    ads131m08->default_rest_ticks = args[1]; // Store default sampling interval
    if (!ads131m08->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    sensor_bulk_reset(&ads131m08->sb);
    irq_disable();
    ads131m08->timer.waketime = timer_read_time() + ads131m08->rest_ticks;
    sched_add_timer(&ads131m08->timer);
    irq_enable();
    printf_("Started ADS131M08 measurements oid=%u rest_ticks=%u\n",
            oid, ads131m08->rest_ticks);
}
DECL_COMMAND(command_query_ads131m08, "query_ads131m08 oid=%c rest_ticks=%u");

void
command_query_ads131m08_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m08_adc *ads131m08 = oid_lookup(oid, command_config_ads131m08);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = ads131m08_is_data_ready(ads131m08);
    irq_enable();
    
    // Optimized pending bytes calculation for single channel (channel 0 only)
    uint8_t pending_bytes = (is_data_ready && (ads131m08->enabled_channels & 0x01)) 
                          ? BYTES_PER_SAMPLE : 0;
    
    sensor_bulk_status(&ads131m08->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_ads131m08_status, "query_ads131m08_status oid=%c");

// Background task that performs measurements
void
ads131m08_capture_task(void)
{
    if (!sched_check_wake(&wake_ads131m08))
        return;
    
    uint8_t oid;
    struct ads131m08_adc *ads131m08;
    foreach_oid(oid, ads131m08, command_config_ads131m08) {
        if (ads131m08->pending_flag) {
            ads131m08_read_adc(ads131m08, oid);
        }
    }
}
DECL_TASK(ads131m08_capture_task);
