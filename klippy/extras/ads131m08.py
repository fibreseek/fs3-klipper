# ADS131M08 Support
#
# Copyright (C) 2024 Klipper Developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bulk_sensor, bus

# ADS131M08 Register Addresses
ID = 0x00
STATUS = 0x01
MODE = 0x02
CLOCK = 0x03
GAIN1 = 0x04
GAIN2 = 0x05
CFG = 0x06
THRSHLD_MSB = 0x07
THRSHLD_LSB = 0x08

# Channel Configuration Registers
CH0_CFG = 0x09
CH1_CFG = 0x0E
CH2_CFG = 0x13
CH3_CFG = 0x18
CH4_CFG = 0x1D
CH5_CFG = 0x22
CH6_CFG = 0x27
CH7_CFG = 0x2C

# Commands
NULL_CMD = 0x0000
RESET_CMD = 0x0011
STANDBY_CMD = 0x0022
WAKEUP_CMD = 0x0033
LOCK_CMD = 0x0555
UNLOCK_CMD = 0x0666
UPDATE_INTERVAL = 0.10

# Power Modes
PWR_HR = 0x02  # High-resolution mode
PWR_LP = 0x01  # Low-power mode
PWR_VLP = 0x00  # Very-low-power mode

# OSR Settings
OSR_128 = 0x00
OSR_256 = 0x01
OSR_512 = 0x02
OSR_1024 = 0x03
OSR_2048 = 0x04
OSR_4096 = 0x05
OSR_8192 = 0x06
OSR_16384 = 0x07

# Gain Settings
GAIN_1 = 0x00
GAIN_2 = 0x01
GAIN_4 = 0x02
GAIN_8 = 0x03
GAIN_16 = 0x04
GAIN_32 = 0x05
GAIN_64 = 0x06
GAIN_128 = 0x07

# Data Rates for 8.192 MHz CLKIN
DATA_RATES = {
    OSR_128: 32000,
    OSR_256: 16000,
    OSR_512: 8000,
    OSR_1024: 4000,
    OSR_2048: 2000,
    OSR_4096: 1000,
    OSR_8192: 500,
    OSR_16384: 250
}

class ADS131M08:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        # SPI Configuration
        spi_speed = config.getint('spi_speed', 1000000, minval=100000)

        # SPI setup - use correct MCU SPI initialization (like ADS1220)
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=spi_speed)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        logging.info("ADS131M08 init: oid=%d on MCU %s", self.oid, mcu.get_name())

        # Data Ready (DRDY) Pin
        drdy_pin = config.get('data_ready_pin')
        ppins = printer.lookup_object('pins')
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin['pin']
        drdy_pin_mcu = drdy_ppin['chip']
        if drdy_pin_mcu != self.mcu:
            raise config.error("ADS131M08 config error: SPI communication and"
                               " data_ready_pin must be on the same MCU")
        
        # Configuration parameters
        self.osr = config.getchoice('osr', {
            '128': OSR_128, '256': OSR_256, '512': OSR_512, '1024': OSR_1024,
            '2048': OSR_2048, '4096': OSR_4096, '8192': OSR_8192, '16384': OSR_16384
        }, default='16384')
        
        self.power_mode = config.getchoice('power_mode', {
            'high_resolution': PWR_HR,
            'low_power': PWR_LP,
            'very_low_power': PWR_VLP
        }, default='high_resolution')
        
        # Channel configuration
        # Use ConfigWrapper.getchoice with a default so we don't access
        # internal fileconfig attributes (config.file doesn't exist).
        self.channel_gains = {}
        gain_choices = {
            '1': GAIN_1, '2': GAIN_2, '4': GAIN_4, '8': GAIN_8,
            '16': GAIN_16, '32': GAIN_32, '64': GAIN_64, '128': GAIN_128
        }
        for i in range(8):
            gain_key = 'channel_%d_gain' % i
            # getchoice will return the mapped gain; default to '1' if
            # option not present in the config section.
            gain = config.getchoice(gain_key, gain_choices, default='1')
            self.channel_gains[i] = gain
        
        # Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, oid=self.oid)
        
        # Calculate samples per second
        self.sps = DATA_RATES[self.osr]
        
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2  # 10% update interval
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        
        # Command Configuration
        self.query_ads131m08_cmd = None
        # Use the spi oid instead of re-looking up the CS pin. This mirrors
        # other drivers (eg. ADS1220) and avoids double-registration of the
        # pin in the pins registry.
        # MCU side expects: config_ads131m08 oid=%c spi_oid=%c data_ready_pin=%u enabled_channels=%c
        enabled_channels = 0x01  # enable only channel 0
        # data_ready_pin is a pin identifier string (e.g. 'toolhead:PB7')
        data_ready_param = self.data_ready_pin or 0
        mcu.add_config_cmd(
            "config_ads131m08 oid=%d spi_oid=%d data_ready_pin=%s enabled_channels=%d"
            % (self.oid, self.spi.get_oid(), data_ready_param, enabled_channels))
        
        mcu.register_config_callback(self._build_config)
        
        logging.info("ADS131M08: Initialized %s with SPS=%d", self.name, self.sps)

    def _build_config(self):
        self.query_ads131m08_cmd = self.mcu.lookup_command(
            "query_ads131m08 oid=%c rest_ticks=%u")
        self.ffreader.setup_query_command("query_ads131m08_status oid=%c",
                                          oid=self.oid,
                                          cq=self.mcu.alloc_command_queue())

    def reset_chip(self):
        """Reset the ADS131M08 chip"""
        # Send RESET command
        # Send the reset as a sequence of bytes (MSB, LSB) followed by
        # padding so the MCU side receives a full frame. Passing an int
        # here caused msgproto to attempt to call len() on the int.
        self._spi_transfer([RESET_CMD >> 8, RESET_CMD & 0xFF] + [0] * 16)
        # Wait for reset to complete
        self.printer.get_reactor().pause(self.printer.get_reactor().monotonic() + 0.1)

    def setup_chip(self):
        """Configure ADS131M08 registers"""
        # Reset chip first
        self.reset_chip()
        
        # Configure MODE register
        mode_reg = 0x0510  # Default with 24-bit word length
        self._write_reg(MODE, [(mode_reg >> 8) & 0xFF, mode_reg & 0xFF])
        
        # Configure CLOCK register
        clock_reg = (0xFF & ~0x07) | (self.osr & 0x07)  # Enable all channels
        clock_reg = (clock_reg & ~0x03) | (self.power_mode & 0x03)
        self._write_reg(CLOCK, [(clock_reg >> 8) & 0xFF, clock_reg & 0xFF])
        
        # Configure GAIN registers
        gain1_reg = 0
        gain2_reg = 0
        for i in range(4):
            gain1_reg |= (self.channel_gains[i] & 0x07) << (i * 4)
        for i in range(4, 8):
            gain2_reg |= (self.channel_gains[i] & 0x07) << ((i-4) * 4)
        
        self._write_reg(GAIN1, [(gain1_reg >> 8) & 0xFF, gain1_reg & 0xFF])
        self._write_reg(GAIN2, [(gain2_reg >> 8) & 0xFF, gain2_reg & 0xFF])
        
        # Configure channel settings (default: normal input, no phase shift)
        for i, ch_reg in enumerate([CH0_CFG, CH1_CFG, CH2_CFG, CH3_CFG, 
                                   CH4_CFG, CH5_CFG, CH6_CFG, CH7_CFG]):
            self._write_reg(ch_reg, [0x00, 0x00])  # Default configuration
        
        logging.info("ADS131M08: Chip configuration complete")

    def _spi_transfer(self, data):
        """Perform SPI transfer"""
        # Use spi_transfer and pass the data list directly. The previous
        # implementation wrapped the data in another list which produced
        # nested lists being formatted as ints and raised
        # "'list' object cannot be interpreted as an integer".
        return self.spi.spi_transfer(data)

    def _read_reg(self, reg_addr):
        """Read register"""
        cmd = [0x20 | (reg_addr & 0x1F), 0x00]  # RREG command for 1 register
        # Perform a spi_transfer which returns a dict containing the
        # response bytes under the 'response' key (see other drivers such
        # as ADS1220.read_reg). Convert to bytearray and extract the
        # requested register byte (3rd byte in the returned frame).
        params = self._spi_transfer(cmd + [0] * 18)  # Full frame
        if params is None:
            return None
        resp = bytearray(params['response'])
        return resp[2]  # Register data is in the 3rd byte

    def _write_reg(self, reg_addr, data):
        """Write register"""
        cmd = [0x40 | (reg_addr & 0x1F), 0x00]  # WREG command for 1 register
        self._spi_transfer(cmd + data + [0] * 16)  # Full frame

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    def get_range(self):
        """Return the ADC range for saturation detection"""
        return -0x800000, 0x7FFFFF  # 24-bit signed range

    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def _convert_samples(self, samples):
        """Convert raw ADC samples to proper format"""
        adc_factor = 1. / (1 << 23)
        count = 0
        for ptime, val in samples:
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    def _start_measurements(self):
        """Start continuous measurements"""
        logging.info("ADS131M08: Starting measurements for %s", self.name)
        
        self.reset_chip()
        # Setup chip configuration
        self.setup_chip()
        
        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_ads131m08_cmd.send([self.oid, rest_ticks])
        
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        """Stop measurements"""
        if self.printer.is_shutdown():
            return
        
        # Halt bulk reading
        self.query_ads131m08_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("ADS131M08: Finished measurements for %s", self.name)

    def _process_batch(self, eventtime):
        """Process a batch of samples"""
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        
        overflows = self.ffreader.get_last_overflows()
        # ADS131M08 doesn't report errors in the same way as HX71x
        
        return {'data': samples, 'errors': 0, 'overflows': overflows}


def load_config(config):
    return ADS131M08(config)

def load_config_prefix(config):
    return load_config(config)

# Sensor type for registration in load_cell.py
ADS131M08_SENSOR_TYPE = {
    'ads131m08': ADS131M08
}
