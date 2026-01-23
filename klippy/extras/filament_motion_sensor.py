# Filament Motion Sensor Module
#
# Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor

CHECK_RUNOUT_TIMEOUT = 2.0

class EncoderSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        switch_pin = config.get('switch_pin')
        self.extruder_name = config.get('extruder')
        self.detection_length = config.getfloat('detection_length', 20., above=0.)

        self.pulse_per_mm   = config.getfloat('encoder_pulses_per_mm', 1.57, above=0.)
        self.encoder_pos    = 0.0        
        self.last_move_dir  = 1           
        self.dir_expire     = 0             
        self.last_pulse_t   = 0           
        self.last_move_t    = 0             
        self.max_pulse_interval = 1.5 * self.detection_length / 0.1

        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_buttons([switch_pin], self.encoder_event)

        self.reactor = self.printer.get_reactor()
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.get_status = self.runout_helper.get_status

        self.extruder = None
        self.estimated_print_time = None
        self.extruder_pos = 0.0    
        self.trigger_count = 0
        self.filament_runout_pos = None

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('idle_timeout:printing', self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready', self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle', self._handle_not_printing)
        self.printer.register_event_handler("gcode:move_extruder", self._handle_move_extruder)
        self.printer.register_event_handler("virtual_sdcard:print_start", self._handle_print_file_start)

    def _update_filament_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.filament_runout_pos = self.encoder_pos + self.detection_length

    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = self.printer.lookup_object('mcu').estimated_print_time
        self._update_filament_runout_pos()
        self._timer = self.reactor.register_timer(self._timer_handler)

    def _handle_print_file_start(self):
        self.extruder_pos = 0.0
        self.encoder_pos  = 0.0
        self.trigger_count = 0
        self._update_filament_runout_pos()
        self.reactor.update_timer(self._timer, self.reactor.NOW)

    def _handle_printing(self, pt): self.reactor.update_timer(self._timer, self.reactor.NOW)
    def _handle_not_printing(self, pt): self.reactor.update_timer(self._timer, self.reactor.NEVER)

    def _handle_move_extruder(self, move_extruder, move_pos, absolute_mode):
        if move_extruder == self.extruder_name:
            if move_pos > 0:
                if absolute_mode:
                    self.extruder_pos = move_pos
                else:
                    self.extruder_pos += move_pos
            # logging.info("_handle_move_extruder: extruder_name=%s, extruder_pos=%.3f, "
            #         "filament_runout_pos=%.3f, absolute_mode=%d", self.extruder_name,
            #         self.extruder_pos, self.filament_runout_pos, absolute_mode)
    def _extruder_pos_update_event(self, eventtime):
        extruder_pos = self._get_extruder_pos(eventtime)
        # Check for filament runout
        triggered = (extruder_pos > self.filament_runout_pos)
        if triggered:
            self.trigger_count += 1
            logging.info("Runout detected: enc_pos=%.3f  threshold=%.3f  count=%d",
                         self.encoder_pos, self.filament_runout_pos, self.trigger_count)
        else:
            self.trigger_count = 0

        if self.trigger_count > 3:
            self.trigger_count = 0
            self.runout_helper.note_filament_present(eventtime, False)
        else:
            self.runout_helper.note_filament_present(eventtime, True)
        return eventtime + CHECK_RUNOUT_TIMEOUT

def load_config_prefix(config):
    return EncoderSensor(config)