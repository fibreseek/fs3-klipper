# Extruder Stall Detector Module
#
# Copyright (C) 2025  AI Assistant
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, collections
from . import tmc

class ExtruderStallDetector:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Read config
        self.extruder_name = config.get('extruder')
        self.sample_interval = config.getfloat('sample_interval', 0.1, above=0.0)
        self.stall_threshold = config.getfloat('stall_threshold', 0.5, minval=0.0, maxval=1.0)
        self.sg_threshold = config.getint('sg_threshold', 20, minval=0, maxval=1023)
        self.sg_drop_rate = config.getfloat('sg_drop_rate', 0.3, minval=0.0, maxval=1.0)
        self.filter_window = config.getint('filter_window', 5, minval=1)
        self.enable_stallguard = config.getboolean('enable_stallguard', False)
        self.enable_sg_monitor = config.getboolean('enable_sg_monitor', True)
        
        # Internal state
        self.extruder = None
        self.tmc_driver = None
        self.tmc_fields = None
        self.is_printing = False
        self.enabled = True
        self.poll_timer = None
        
        # Stall detection state
        self.sg_history = collections.deque(maxlen=self.filter_window)
        self.last_sg_value = 0
        self.stall_detected = False
        self.stall_count = 0
        self.consecutive_stalls = 0
        
        # Register commands and event handlers
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler("filament_clog:check_start", self._handle_check_start)
        self.printer.register_event_handler("filament_clog:check_stop", self._handle_check_stop)
        # self.printer.register_event_handler('idle_timeout:printing', 
        #                                    self._handle_printing)
        # self.printer.register_event_handler('idle_timeout:ready', 
        #                                    self._handle_not_printing)
        # self.printer.register_event_handler('idle_timeout:idle', 
        #                                    self._handle_not_printing)
        
        # Register G-code commands
        self.gcode.register_mux_command(
            "SET_EXTRUDER_STALL_DETECTION", "EXTRUDER", self.name,
            self.cmd_SET_EXTRUDER_STALL_DETECTION,
            desc=self.cmd_SET_EXTRUDER_STALL_DETECTION_help)
        self.gcode.register_mux_command(
            "QUERY_EXTRUDER_STALL", "EXTRUDER", self.name,
            self.cmd_QUERY_EXTRUDER_STALL,
            desc=self.cmd_QUERY_EXTRUDER_STALL_help)
        self.gcode.register_mux_command(
            "TEST_STALL_DETECTION", "EXTRUDER", self.name,
            self.cmd_TEST_STALL_DETECTION,
            desc=self.cmd_TEST_STALL_DETECTION_help)
    
    def _handle_ready(self):
        # Get extruder and TMC driver objects
        self.extruder = self.printer.lookup_object(self.extruder_name)
        
        # Try to find TMC driver for this extruder
        # In Klipper, TMC drivers are named like "tmc2240 extruder"
        driver_name = None
        for obj_name, obj in self.printer.objects.items():
            if obj_name.startswith('tmc') and self.extruder_name in obj_name:
                driver_name = obj_name
                break
        
        if driver_name:
            self.tmc_driver = self.printer.lookup_object(driver_name)
            if hasattr(self.tmc_driver, 'fields'):
                self.tmc_fields = self.tmc_driver.fields
            elif hasattr(self.tmc_driver, 'get_fields'):
                self.tmc_fields = self.tmc_driver.get_fields()
        
        if not self.tmc_fields:
            logging.error("ExtruderStallDetector %s: Could not find TMC driver for extruder %s",
                         self.name, self.extruder_name)
            return
        
        logging.info("ExtruderStallDetector %s: Initialized for extruder %s with TMC driver",
                    self.name, self.extruder_name)
    
    def _handle_printing(self, print_time):
        logging.info("ExtruderStallDetector %s: Printing started, enabling stall detection polling",
                     self.name)
        self.is_printing = True
        if self.enabled and not self.poll_timer:
            self.poll_timer = self.reactor.register_timer(
                self._poll_stall_status, self.reactor.NOW)
    
    def _handle_not_printing(self, print_time):
        logging.info("ExtruderStallDetector %s: Not printing, stopping stall detection polling",
                     self.name)
        self.is_printing = False
        if self.poll_timer:
            self.reactor.unregister_timer(self.poll_timer)
            self.poll_timer = None
    
    def _handle_check_start(self):
        self._handle_printing(self.reactor.monotonic())
    
    def _handle_check_stop(self):
        self._handle_not_printing(self.reactor.monotonic())

    def _get_drv_status(self):
        """Read DRV_STATUS register from TMC driver"""
        try:
            if hasattr(self.tmc_driver, 'mcu_tmc'):
                drv_status = self.tmc_driver.mcu_tmc.get_register("DRV_STATUS")
                return drv_status
            elif hasattr(self.tmc_driver, 'get_register'):
                return self.tmc_driver.get_register("DRV_STATUS")
        except Exception as e:
            logging.error("ExtruderStallDetector %s: Failed to read DRV_STATUS: %s",
                         self.name, str(e))
        return None
    
    def _poll_stall_status(self, eventtime):
        """Periodic stall status polling"""
        if not self.enabled or not self.is_printing:
            return eventtime + self.sample_interval
        
        drv_status = self._get_drv_status()
        if drv_status is None:
            return eventtime + self.sample_interval
        
        # Extract fields from DRV_STATUS
        sg_result = self.tmc_fields.get_field("sg_result", drv_status, "DRV_STATUS")
        stallguard = self.tmc_fields.get_field("stallguard", drv_status, "DRV_STATUS")
        cs_actual = self.tmc_fields.get_field("cs_actual", drv_status, "DRV_STATUS")
        stealth = self.tmc_fields.get_field("stealth", drv_status, "DRV_STATUS")
        stst = self.tmc_fields.get_field("stst", drv_status, "DRV_STATUS")
        logging.info("ExtruderStallDetector %s: DRV_STATUS read: sg_result=%d, stallguard=%d, "
                      "cs_actual=%d, stealth=%d, stst=%d", self.name, sg_result, stallguard, cs_actual, stealth, stst)
        
        # Update history
        self.sg_history.append(sg_result)
        self.last_sg_value = sg_result
        
        # Check for stall
        stall_detected = self._check_stall(sg_result, stallguard, eventtime)
        
        if stall_detected and not self.stall_detected:
            # 本轮检测触发一次事件后，立刻清空状态，进入下一轮检测
            self.stall_detected = True
            self.stall_count += 1
            self.consecutive_stalls += 1
            self._send_stall_event(eventtime, sg_result, stallguard)
            # 重置为“未触发”状态，下一轮从头开始判定
            self.stall_detected = False
            self.consecutive_stalls = 0
        elif not stall_detected and self.stall_detected:
            self.stall_detected = False
            self.consecutive_stalls = 0
        
        return eventtime + self.sample_interval
    
    def _check_stall(self, sg_result, stallguard, eventtime):
        """Check if stall is detected based on multiple criteria"""
        stall_detected = False
        
        # 1. Check stallguard flag (hardware detection)
        if self.enable_stallguard and stallguard:
            logging.info("ExtruderStallDetector %s: StallGuard flag detected at sg_result=%d",
                        self.name, sg_result)
            stall_detected = True
        
        # 2. Check sg_result absolute threshold
        if self.enable_sg_monitor and sg_result < self.sg_threshold:
            logging.info("ExtruderStallDetector %s: sg_result below threshold: %d < %d",
                        self.name, sg_result, self.sg_threshold)
            stall_detected = True
        
        # 3. Check sg_result drop rate (if we have history)
        if self.enable_sg_monitor and len(self.sg_history) >= 2:
            recent_values = list(self.sg_history)
            if len(recent_values) >= 2:
                drop = recent_values[-2] - recent_values[-1]
                drop_rate = drop / max(recent_values[-2], 1)
                
                if drop_rate > self.sg_drop_rate and recent_values[-1] < recent_values[-2]:
                    logging.info("ExtruderStallDetector %s: Rapid sg_result drop detected: %.2f",
                                self.name, drop_rate)
                    stall_detected = True
        
        # # 4. Check for pattern: decrease then increase (user's original idea)
        # if self.enable_sg_monitor and len(self.sg_history) >= 3:
        #     values = list(self.sg_history)
        #     if values[-3] > values[-2] and values[-2] < values[-1]:
        #         # Found a valley pattern
        #         valley_depth = min(values[-3] - values[-2], values[-1] - values[-2])
        #         if valley_depth > 5:  # Significant valley
        #             logging.info("ExtruderStallDetector %s: Valley pattern detected (depth=%d)",
        #                         self.name, valley_depth)
        #             stall_detected = True
        
        return stall_detected
    
    def _send_stall_event(self, eventtime, sg_result, stallguard):
        """Send stall detection event"""
        logging.warning("ExtruderStallDetector %s: STALL DETECTED! sg_result=%d, stallguard=%d",
                       self.name, sg_result, stallguard)

        self.printer.send_event("extruder_stall_detector:stall_detected", self.extruder_name)
        
        # # Send event for other modules to handle
        # self.printer.send_event("extruder_stall:stall_detected", 
        #                        self.extruder_name, sg_result, stallguard)
        
        # Optionally run a G-code script
        # This could be configured to pause print, retract, etc.
    
    def get_status(self, eventtime):
        """Return current status for API queries"""
        return {
            'enabled': self.enabled,
            'stall_detected': self.stall_detected,
            'stall_count': self.stall_count,
            'consecutive_stalls': self.consecutive_stalls,
            'last_sg_value': self.last_sg_value,
            'sg_history': list(self.sg_history),
            'is_printing': self.is_printing,
            'extruder': self.extruder_name
        }
    
    # G-code command implementations
    cmd_SET_EXTRUDER_STALL_DETECTION_help = (
        "Configure extruder stall detection parameters"
    )
    
    def cmd_SET_EXTRUDER_STALL_DETECTION(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.enabled = bool(enable)
            gcmd.respond_info("Extruder stall detection %s for %s" % 
                            ("enabled" if self.enabled else "disabled", self.name))
        
        sample_interval = gcmd.get_float('SAMPLE_INTERVAL', None, above=0.0)
        if sample_interval is not None:
            self.sample_interval = sample_interval
            gcmd.respond_info("Sample interval set to %.3f seconds for %s" % 
                            (sample_interval, self.name))
        
        sg_threshold = gcmd.get_int('SG_THRESHOLD', None, minval=0, maxval=1023)
        if sg_threshold is not None:
            self.sg_threshold = sg_threshold
            gcmd.respond_info("SG threshold set to %d for %s" % (sg_threshold, self.name))
        
        stall_threshold = gcmd.get_float('STALL_THRESHOLD', None, minval=0.0, maxval=1.0)
        if stall_threshold is not None:
            self.stall_threshold = stall_threshold
            gcmd.respond_info("Stall threshold set to %.2f for %s" % 
                            (stall_threshold, self.name))
    
    cmd_QUERY_EXTRUDER_STALL_help = (
        "Query current extruder stall detection status"
    )
    
    def cmd_QUERY_EXTRUDER_STALL(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        msg = [
            "Extruder Stall Detection Status for %s:" % self.name,
            "  Enabled: %s" % ("Yes" if status['enabled'] else "No"),
            "  Stall Detected: %s" % ("Yes" if status['stall_detected'] else "No"),
            "  Total Stall Count: %d" % status['stall_count'],
            "  Consecutive Stalls: %d" % status['consecutive_stalls'],
            "  Last SG Result: %d" % status['last_sg_value'],
            "  SG History: %s" % str(status['sg_history']),
            "  Is Printing: %s" % ("Yes" if status['is_printing'] else "No"),
            "  Extruder: %s" % status['extruder']
        ]
        gcmd.respond_info("\n".join(msg))
    
    cmd_TEST_STALL_DETECTION_help = (
        "Test stall detection by simulating stall conditions"
    )
    
    def cmd_TEST_STALL_DETECTION(self, gcmd):
        test_type = gcmd.get('TEST_TYPE', 'status').lower()
        
        if test_type == 'status':
            # Just read and display current DRV_STATUS
            drv_status = self._get_drv_status()
            if drv_status is None:
                gcmd.respond_info("Could not read DRV_STATUS register")
                return
            
            sg_result = self.tmc_fields.get_field("sg_result", drv_status, "DRV_STATUS")
            stallguard = self.tmc_fields.get_field("stallguard", drv_status, "DRV_STATUS")
            cs_actual = self.tmc_fields.get_field("cs_actual", drv_status, "DRV_STATUS")
            stealth = self.tmc_fields.get_field("stealth", drv_status, "DRV_STATUS")
            stst = self.tmc_fields.get_field("stst", drv_status, "DRV_STATUS")
            
            msg = [
                "TMC DRV_STATUS Register Test:",
                "  sg_result: %d" % sg_result,
                "  stallguard: %d" % stallguard,
                "  cs_actual: %d" % cs_actual,
                "  stealth: %d" % stealth,
                "  stst: %d" % stst,
                "  Raw value: 0x%08x" % drv_status
            ]
            gcmd.respond_info("\n".join(msg))
        
        elif test_type == 'simulate':
            # Simulate a stall event
            gcmd.respond_info("Simulating stall detection event...")
            self._handle_printing(self.reactor.monotonic())  # Set printing state to trigger polling
            # gcmd.respond_info("Stall event simulation complete")
        
        else:
            gcmd.respond_info("Unknown test type. Use 'status' or 'simulate'")

def load_config_prefix(config):
    return ExtruderStallDetector(config)
