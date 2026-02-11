# Filament Motion Sensor Module
#
# Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor

CHECK_RUNOUT_TIMEOUT = 2.0  # seconds

class EncoderSensor:
    def __init__(self, config):
        # Read config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        switch_pin = config.get('switch_pin')
        self.extruder_name = config.get('extruder')
        self.detection_length = config.getfloat(
                'detection_length', 7., above=0.)
        self.default_detection_length = self.detection_length
        # Configure pins
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_buttons([switch_pin], self.encoder_event)
        # Get printer objects
        self.reactor = self.printer.get_reactor()
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.get_status = self.runout_helper.get_status
        self.extruder = None
        self.estimated_print_time = None
        self.extruder_pos = 0.0
        self.trigger_count = 0
        self.trigger_count_max = 3
        self.check_filament_clog_standby = False
        # Initialise internal state
        self.filament_runout_pos = None
        self.last_move_time = 0.0
        self.delayed_check_timer = None
        # Register commands and event handlers
        self.printer.register_event_handler('klippy:ready',
                self._handle_ready)
        self.printer.register_event_handler('idle_timeout:printing',
                self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready',
                self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle',
                self._handle_not_printing)
        self.printer.register_event_handler("gcode:move_extruder",
                self._handle_move_extruder)
        self.printer.register_event_handler("virtual_sdcard:print_start", self._handle_print_file_start)
        self.printer.register_event_handler("filament_clog:check_start", self._handle_check_start)
        self.printer.register_event_handler("filament_clog:check_stop", self._handle_check_stop)
    def _update_filament_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.filament_runout_pos = (
                self._get_extruder_pos(eventtime) +
                self.detection_length)
    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = (
                self.printer.lookup_object('mcu').estimated_print_time)
        self._update_filament_runout_pos()
        self._extruder_pos_update_timer = self.reactor.register_timer(
                self._extruder_pos_update_event)
    def _handle_print_file_start(self):
        self.extruder_pos = 0.0
        self.trigger_count = 0
        self._update_filament_runout_pos()
        self.reactor.update_timer(self._extruder_pos_update_timer, self.reactor.NOW)
    def _handle_printing(self, print_time):
        if not self.check_filament_clog_standby:
            self.reactor.update_timer(self._extruder_pos_update_timer,
                    self.reactor.NOW)
    def _handle_not_printing(self, print_time):
        if not self.check_filament_clog_standby:
            self.reactor.update_timer(self._extruder_pos_update_timer,
                    self.reactor.NEVER)
    def _handle_check_stop(self):
        self.check_filament_clog_standby = False
        self.detection_length = self.default_detection_length
        self.trigger_count_max = 3
        self.reactor.update_timer(self._extruder_pos_update_timer, self.reactor.NEVER)
        logging.info("Stop blockage inspection: extruder_name=%s", self.extruder_name)

    def _handle_check_start(self):
        self.check_filament_clog_standby = True
        self.detection_length = 1
        self.trigger_count_max = 1
        self.extruder_pos = 0.0
        self.trigger_count = 0
        self._update_filament_runout_pos()
        self.reactor.update_timer(self._extruder_pos_update_timer, self.reactor.NOW)
        logging.info("Start blockage inspection: extruder_name=%s", self.extruder_name)
        
    def _get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        # print_time = self.estimated_print_time(eventtime)
        # return self.extruder.find_past_position(print_time)
        return self.extruder_pos

    def _handle_move_extruder(self, move_extruder, move_pos, absolute_mode):
        if move_extruder == self.extruder_name:
            if absolute_mode:
                self.extruder_pos = move_pos
            elif move_pos > 0:
                self.extruder_pos += move_pos
            # logging.info("_handle_move_extruder: extruder_name=%s, extruder_pos=%.3f, "
            #         "filament_runout_pos=%.3f, absolute_mode=%d", self.extruder_name,
            #         self.extruder_pos, self.filament_runout_pos, absolute_mode)

        if self.check_filament_clog_standby:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.wait_moves()
            self._check_clog()

    def _check_clog(self, extruder_pos=None):
        if extruder_pos is None:
            extruder_pos = self.extruder_pos
        # Check for filament runout
        triggered = (extruder_pos > self.filament_runout_pos)
        if triggered:
            self.trigger_count += 1
            logging.info("Filament motion sensor triggered (delayed): extruder_name=%s, extruder_pos=%.3f, "
                    "filament_runout_pos=%.3f, trigger_count=%d", self.extruder_name,
                    extruder_pos, self.filament_runout_pos, self.trigger_count)
        else: 
            self.trigger_count = 0
        
        if self.trigger_count > self.trigger_count_max:
            self.trigger_count = 0
            self.printer.send_event("filament_motion_sensor:filament_clog", self.extruder_name)
            logging.info("Filament runout event processed (delayed): extruder_name=%s, extruder_pos=%.3f, "
                    "filament_runout_pos=%.3f", self.extruder_name,
                    extruder_pos, self.filament_runout_pos)
            return True
        return False
        
    def _extruder_pos_update_event(self, eventtime):
        extruder_pos = self._get_extruder_pos(eventtime)
        if self._check_clog(extruder_pos):
            self.runout_helper.note_filament_present(eventtime, False)
        else:
            self.runout_helper.note_filament_present(eventtime, True)
        return eventtime + CHECK_RUNOUT_TIMEOUT

    def encoder_event(self, eventtime, state):
        if self.extruder is not None:
            self._update_filament_runout_pos(eventtime)
            # Check for filament insertion
            # Filament is always assumed to be present on an encoder event
            self.trigger_count = 0
            self.runout_helper.note_filament_present(eventtime, True)
            # logging.info("Encoder event detected: extruder_name=%s, extruder_pos=%.3f, "
            #         "filament_runout_pos=%.3f, state=%d", self.extruder_name,
            #         self._get_extruder_pos(eventtime),
            #         self.filament_runout_pos, state)

def load_config_prefix(config):
    return EncoderSensor(config)
