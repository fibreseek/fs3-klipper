# Filament Motion Sensor Module
#
# Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor

CHECK_RUNOUT_TIMEOUT = .50
# 回调间隔超过该值说明主机反应堆被阻塞过：此期间 toolhead 仍在按队列挤出，
# 而 encoder_event 只能等反应堆恢复后才被投递，两者不同步会吃掉检测余量
STALL_THRESHOLD = CHECK_RUNOUT_TIMEOUT * 2.

class EncoderSensor:
    def __init__(self, config):
        # Read config
        self.printer = config.get_printer()
        switch_pin = config.get('switch_pin')
        self.extruder_name = config.get('extruder')
        self.detection_length = config.getfloat(
                'detection_length', 7., above=0.)
        # Configure pins
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_buttons([switch_pin], self.encoder_event)
        # Get printer objects
        self.reactor = self.printer.get_reactor()
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.sensor_name = self.runout_helper.name
        self.extruder = None
        self.estimated_print_time = None
        # Initialise internal state
        self.filament_runout_pos = None
        self.last_check_time = None
        self.runout_pending = False
        self.last_encoder_pos = None
        # Register commands and event handlers
        self.printer.register_event_handler('klippy:ready',
                self._handle_ready)
        self.printer.register_event_handler('virtual_sdcard:move_start',
                self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready',
                self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle',
                self._handle_not_printing)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            "SET_FILAMENT_MOTION_DETECTION_LENGTH", "SENSOR",
            self.sensor_name, self.cmd_SET_FILAMENT_MOTION_DETECTION_LENGTH,
            desc=self.cmd_SET_FILAMENT_MOTION_DETECTION_LENGTH_help)
    def get_status(self, eventtime):
        status = self.runout_helper.get_status(eventtime)
        status['detection_length'] = self.detection_length
        return status
    cmd_SET_FILAMENT_MOTION_DETECTION_LENGTH_help = (
        "Set filament motion sensor detection_length at runtime")
    def cmd_SET_FILAMENT_MOTION_DETECTION_LENGTH(self, gcmd):
        length = gcmd.get_float('LENGTH', above=0.)
        self.detection_length = length
        self.runout_pending = False
        self._update_filament_runout_pos()
        gcmd.respond_info(
            "Filament motion sensor %s: detection_length set to %.3f mm"
            % (self.sensor_name, length))
    def _update_filament_runout_pos(self, eventtime=None, monotonic=False):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        extruder_pos = self._get_extruder_pos(eventtime)
        runout_pos = extruder_pos + self.detection_length
        if monotonic and self.filament_runout_pos is not None:
            # 积压的编码器事件带着偏旧的 eventtime 被延迟投递时，不允许把阈值往回拉
            runout_pos = max(runout_pos, self.filament_runout_pos)
        self.filament_runout_pos = runout_pos
        return extruder_pos
    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = (
                self.printer.lookup_object('mcu').estimated_print_time)
        self._update_filament_runout_pos()
        self._extruder_pos_update_timer = self.reactor.register_timer(
                self._extruder_pos_update_event)
    def _handle_printing(self, print_time):
        self.last_check_time = None
        self.runout_pending = False
        self.last_encoder_pos = None
        self._update_filament_runout_pos()
        self.runout_helper.note_filament_present(self.reactor.NOW, True)
        self.reactor.update_timer(self._extruder_pos_update_timer,
                self.reactor.NOW)
        logging.info(f"{self.extruder_name}: Printing move start, Filament runout position: {self.filament_runout_pos}, Extruder position: {self._get_extruder_pos()}")
    def _handle_not_printing(self, print_time):
        self.last_check_time = None
        self.runout_pending = False
        self.reactor.update_timer(self._extruder_pos_update_timer,
                self.reactor.NEVER)
    def _get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)
    def _extruder_pos_update_event(self, eventtime):
        extruder_pos = self._get_extruder_pos(eventtime)
        last_check_time = self.last_check_time
        self.last_check_time = eventtime
        check_interval = 0.
        if last_check_time is not None:
            check_interval = eventtime - last_check_time
        if check_interval > STALL_THRESHOLD:
            # 阻塞期间的挤出量不能计入堵料判定
            self.runout_pending = False
            self._update_filament_runout_pos(eventtime, monotonic=True)
            logging.info(f"{self.extruder_name}: Host stalled {check_interval - CHECK_RUNOUT_TIMEOUT:.3f}s, Extruder position: {extruder_pos}, Filament runout position extended to: {self.filament_runout_pos}")
            return eventtime + CHECK_RUNOUT_TIMEOUT
        if extruder_pos < self.filament_runout_pos:
            self.runout_pending = False
            self.runout_helper.note_filament_present(eventtime, True)
            return eventtime + CHECK_RUNOUT_TIMEOUT
        if not self.runout_pending:
            # 宽限一轮，等待可能仍排在反应堆队列里的 encoder_event
            self.runout_pending = True
            logging.info(f"{self.extruder_name}: Extruder position: {extruder_pos}, Filament runout position: {self.filament_runout_pos}, awaiting confirmation")
            return eventtime + CHECK_RUNOUT_TIMEOUT
        logging.info(f"{self.extruder_name}: Extruder position: {extruder_pos}, Filament runout position: {self.filament_runout_pos}")
        self.runout_helper.note_filament_present(eventtime, False)
        return eventtime + CHECK_RUNOUT_TIMEOUT
    def encoder_event(self, eventtime, state):
        if self.extruder is not None:
            self.runout_pending = False
            extruder_pos = self._update_filament_runout_pos(eventtime,
                    monotonic=True)
            # Check for filament insertion
            # Filament is always assumed to be present on an encoder event
            self.runout_helper.note_filament_present(eventtime, True)
            last_encoder_pos = self.last_encoder_pos
            self.last_encoder_pos = extruder_pos
            if last_encoder_pos is None:
                last_encoder_pos = extruder_pos
            logging.info(f"{self.extruder_name}: Encoder event, Extruder position: {extruder_pos}, Moved since last event: {extruder_pos - last_encoder_pos:.3f}, Filament runout position: {self.filament_runout_pos}")

def load_config_prefix(config):
    return EncoderSensor(config)
