# ADS131M08 Support
#
# Copyright (C) 2024 Klipper Developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus
import mcu
from mcu import TriggerDispatch
from . import probe
from math import fabs

# Power Modes
PWR_HR = 0x02  # High-resolution mode
PWR_LP = 0x01  # Low-power mode
PWR_VLP = 0x00  # Very-low-power mode

class aniso_zoffset_align:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        # SPI Configuration
        spi_speed = config.getint('spi_speed', 1000000, minval=100000)

        # SPI setup - use correct MCU SPI initialization (like ADS1220)
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=spi_speed)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        logging.info("aniso_zoffset_align init: oid=%d on MCU %s", self.oid, mcu.get_name())

        # Create TriggerDispatch for managing trsync objects
        self._dispatch = TriggerDispatch(self.mcu)
        
        # Use LookupZSteppers to find all Z axis steppers
        self.lookup_zsteppers = probe.LookupZSteppers(config, self._dispatch.add_stepper)
        
        self.trsync_oid = None
        # Data Ready (DRDY) Pin
        drdy_pin = config.get('data_ready_pin')
        ppins = printer.lookup_object('pins')
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin['pin']
        drdy_pin_mcu = drdy_ppin['chip']
        if drdy_pin_mcu != self.mcu:
            raise config.error("aniso_zoffset_align config error: SPI communication and"
                               " data_ready_pin must be on the same MCU")
        
        # 噪声比例设置 128,256,512,1024,2048,4096,8192,16384
        self.osr = config.getint('osr', default=16384)

        # power_mode: high_resolution, low_power, very_low_power
        self.power_mode = config.get('power_mode', default='high_resolution')

        # Calculate samples per second
        self.sps = config.getint('sps', default=500)

        # Enable channel 0~7
        self.enable_channel = config.getint('enable_channel', default=2)

        # Global channel gain 1,2,4,8,16,32,64,128
        self.channel_gain = config.getint('channel_gain', default=128)
        
        # Command Configuration
        self.query_zoffset_align_cmd = None
        self.start_zoffset_align_cmd = None
        self.stop_zoffset_align_cmd = None
        self.check_zoffset_align_triggered_cmd = None
        self.start_get_adc_value_cmd = None
        self.zoffset_align_triggered = False
        self.wait_check_result = False
        self.wait_check_started = False
        self.wait_get_adc_value = False
        self.current_adc_value = 0.0
        
        # Z axis stepper and trsync references
        self.z_stepper = None
        self.trsync = None
        
        # Register G-code commands
        self.gcode = printer.lookup_object('gcode')
        self.gcode.register_command("ANISO_ZOFFSET_ALIGN_START", self.cmd_ANISO_ZOFFSET_ALIGN_START,
                              desc=self.cmd_ANISO_ZOFFSET_ALIGN_START_help)
        self.gcode.register_command("ANISO_ZOFFSET_CHECK_TRIGGER", self.cmd_ANISO_ZOFFSET_CHECK_TRIGGER,
                              desc=self.cmd_ANISO_ZOFFSET_CHECK_TRIGGER_help)
        self.gcode.register_command("ANISO_ZOFFSET_ALIGN_STOP", self.cmd_ANISO_ZOFFSET_ALIGN_STOP,
                              desc=self.cmd_ANISO_ZOFFSET_ALIGN_STOP_help)
        self.mcu.register_response(self.handle_zoffset_align_check_result, "aniso_zoffset_align_result", self.oid)
        self.mcu.register_response(self.handle_zoffset_align_check_started, "aniso_zoffset_align_started", self.oid)
        self.mcu.register_response(self.handle_zoffset_align_adc_value, "aniso_zoffset_align_adc_value", self.oid)
        # gcode.register_command("ANISO_ZOFFSET_ALIGN_QUERY", self.cmd_ANISO_ZOFFSET_ALIGN_QUERY,
        #                       desc=self.cmd_ANISO_ZOFFSET_ALIGN_QUERY_help)
        
        # Register event handlers
        printer.register_event_handler("klippy:connect", self._handle_connect)
        
        mcu.register_config_callback(self._build_config)
        
        logging.info("aniso_zoffset_align: Initialized %s with SPS=%d OSR=%d", self.name, self.sps, self.osr)

    def getPowerMode(self, power_mode):
        """Change power mode of ADS131M08"""
        if power_mode == 'high_resolution':
            return PWR_HR
        elif power_mode == 'low_power':
            return PWR_LP
        elif power_mode == 'very_low_power':
            return PWR_VLP
        else:
            raise ValueError("Invalid power mode: %s" % power_mode)

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    def _handle_connect(self):
        """Handle klippy connect event - get Z stepper and trsync references"""
        logging.info("aniso_zoffset_align: Handling klippy:connect event")
        
        # Get trsync_oid from trigger dispatch (already created in __init__)
        trsync_oid = self._dispatch.get_oid()
        z_stepper_oid = self.lookup_zsteppers.get_z_stepper_oid()

        # Send trsync_oid to MCU for configuration
        self.config_zoffset_trsync_cmd.send([trsync_oid])
        logging.info("aniso_zoffset_align: Configured trsync_oid=%d for Z stepper_oid=%d", trsync_oid, z_stepper_oid)

    def _build_config(self):
        data_ready_param = self.data_ready_pin or 0
        enabled_channels = 0x02  # enable only channel 0
        self.mcu.add_config_cmd(
            "config_zoffset_align oid=%d spi_oid=%d data_ready_pin=%s enable_channel=%d channel_gain=%d power_mode=%d sps=%d osr=%d"
            % (self.oid, self.spi.get_oid(), data_ready_param, self.enable_channel, self.channel_gain, self.getPowerMode(self.power_mode), self.sps, self.osr))

        # Initialize command objects
        self.query_zoffset_align_cmd = self.mcu.lookup_command(
            "query_zoffset_align oid=%c rest_ticks=%u")
        self.start_zoffset_align_cmd = self.mcu.lookup_command(
            "start_zoffset_align oid=%c")
        self.stop_zoffset_align_cmd = self.mcu.lookup_command(
            "stop_zoffset_align")
        self.check_zoffset_align_triggered_cmd = self.mcu.lookup_command(
            "check_zoffset_align_triggered oid=%c")
        self.config_zoffset_trsync_cmd = self.mcu.lookup_command(
            "config_zoffset_trsync oid=%c")
        self.start_get_adc_value_cmd = self.mcu.lookup_command(
            "start_get_adc_value oid=%c timems=%u")
        logging.info("aniso_zoffset_align: Configuration built and commands initialized")

    cmd_ANISO_ZOFFSET_ALIGN_START_help = "Start Z-axis alignment movement"
    def cmd_ANISO_ZOFFSET_ALIGN_START(self, gcmd):
        """Start Z-axis alignment movement"""

        curtime = self.printer.get_reactor().monotonic()
        toolhead = self.printer.lookup_object('toolhead')
        status = toolhead.get_status(curtime)['homed_axes']
        self.gcode.run_script_from_command("G28\n")
        self.gcode.run_script_from_command("G1 Z%.3f F600\n" % (40))
        self.gcode.run_script_from_command("M400\n")

        # if status != 'xyz':
        #     raise self.printer.command_error("Must home before calibration.")
        # else:
        self.gcode.run_script_from_command("T0\n")
        self.gcode.run_script_from_command("G1 X%.3f Y%.3f F6000\n" % (217, 131))
        self.gcode.run_script_from_command("G1 Z%.3f F300\n" % (23))
        self.gcode.run_script_from_command("M400\n")
        toolhead.wait_moves()
        self.probe_test(gcmd)
        self.gcode.run_script_from_command("M114\n")
        left_x, left_y, left_z, left_e = toolhead.get_position()

        self.gcode.run_script_from_command("G1 Z%.3f F600\n" % (40))
        self.gcode.run_script_from_command("T1\n")
        self.gcode.run_script_from_command("G1 X%.3f Y%.3f F6000\n" % (175, 131))
        self.gcode.run_script_from_command("G1 Z%.3f F300\n" % (25))
        self.gcode.run_script_from_command("M400\n")
        toolhead.wait_moves()
        self.probe_test(gcmd)
        self.gcode.run_script_from_command("M114\n")
        right_x, right_y, right_z, right_e = toolhead.get_position()

        gcmd.respond_info("ANISO_ZOFFSET_ALIGN: Left Z=%.3f mm, Right Z=%.3f mm, Offset=%.3f mm" % (left_z, right_z, right_z - left_z))
    
    def probe_test(self, gcmd):
        move_speed = 200 # mm/min
        move_step = -0.01 # mm
        move_distance = 4.0 # mm
        move_total_distance = 0.0
        self.current_adc_value = 0.0
        toolhead = self.printer.lookup_object('toolhead')

        for i in range(2):
            gcmd.respond_info("ANISO_ZOFFSET_ALIGN: Starting probe test iteration %d" % (i))
            if i == 0: move_step = -0.05
            if i == 1: move_step = -0.01
            move_distance = 4.0 # mm

            self.start_get_adc_value_cmd.send([self.oid, 50])  # Get ADC value for 50 ms
            self.wait_get_adc_value = False
            start_time = self.printer.get_reactor().monotonic()
            timeout = 2.0
            while (not self.wait_get_adc_value) and (self.printer.get_reactor().monotonic() - start_time) < timeout:
                self.printer.get_reactor().pause(0.1)
            init_adc_value = self.current_adc_value

            self.gcode.run_script_from_command("G91\n")
            while(move_total_distance < move_distance):
                if fabs(self.current_adc_value - init_adc_value) > 1000.0:
                    self.gcode.respond_info("cmd_XYZ_ALIGN: Z-offset trigger detected at distance %.3f mm" % move_total_distance)
                    break

                # Move Z axis upwards
                self.gcode.run_script_from_command("G1 Z%.3f F%d\n" % (move_step, move_speed))
                self.gcode.run_script_from_command("M400\n")
                toolhead.wait_moves()

                self.start_get_adc_value_cmd.send([self.oid, 10])  # Get ADC value for 10 ms
                self.wait_get_adc_value = False
                start_time = self.printer.get_reactor().monotonic()
                timeout = 2.0
                while (not self.wait_get_adc_value) and (self.printer.get_reactor().monotonic() - start_time) < timeout:
                    self.printer.get_reactor().pause(0.1)
                move_total_distance += fabs(move_step)
                # self.printer.get_reactor().pause(0.1)
            self.gcode.run_script_from_command("G1 Z0.2 F300\n")
            self.gcode.run_script_from_command("M400\n")
            self.gcode.run_script_from_command("G90\n")
            gcmd.respond_info("ANISO_ZOFFSET_ALIGN: Probe test iteration %d completed" % (i))
    
    cmd_ANISO_ZOFFSET_CHECK_TRIGGER_help = "Check Z-offset alignment trigger status"
    def cmd_ANISO_ZOFFSET_CHECK_TRIGGER(self, gcmd):
        """Check Z-offset alignment trigger status"""
        self.check_zoffset_align_triggered_cmd.send([self.oid])
        self.wait_check_result = False

        start_time = self.printer.get_reactor().monotonic()
        timeout = 1.0
        while (not self.wait_check_result) and (self.printer.get_reactor().monotonic() - start_time) < timeout:
            self.printer.get_reactor().pause(0.1)

        if self.zoffset_align_triggered:
            gcmd.respond_info("ANISO_ZOFFSET_CHECK_TRIGGER: Trigger is ACTIVE")
        else:
            gcmd.respond_info("ANISO_ZOFFSET_CHECK_TRIGGER: Trigger is INACTIVE")

    cmd_ANISO_ZOFFSET_ALIGN_STOP_help = "Stop Z-axis alignment movement"
    def cmd_ANISO_ZOFFSET_ALIGN_STOP(self, gcmd):
        """Stop Z-axis alignment movement"""
        # Send command to MCU to stop Z-axis alignment
        self.stop_zoffset_align_cmd.send([])
        
        # Also stop any ongoing movement
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.motor_off()
        
        gcmd.respond_info("Z-axis alignment stopped")
    
    def handle_zoffset_align_check_result(self, params):
        """Handle response from MCU about Z-offset alignment trigger status"""
        oid = params['oid']
        triggered = params['triggered']
        self.zoffset_align_triggered = triggered
        self.wait_check_result = True

        gcmd.respond_info("ANISO_ZOFFSET_ALIGN: Trigger status received: %s" % ("ACTIVE" if triggered else "INACTIVE"))
        
        # if triggered:
        #     logging.info("aniso_zoffset_align: Trigger detected")
        # else:
        #     logging.info("aniso_zoffset_align: No trigger detected")
    
    def handle_zoffset_align_check_started(self, params):
        """Handle response from MCU confirming start of Z-offset alignment"""
        self.wait_check_started = True
        logging.info("aniso_zoffset_align: Z-offset alignment started")

    def handle_zoffset_align_adc_value(self, params):
        """Handle ADC value response from MCU"""
        oid = params['oid']
        adc_val = params['adc_value']
        self.current_adc_value = adc_val
        self.wait_get_adc_value = True
        logging.info("aniso_zoffset_align: ADC Value Received - Value: %.3f", adc_val)

def load_config(config):
    return aniso_zoffset_align(config)

def load_config_prefix(config):
    return load_config(config)
