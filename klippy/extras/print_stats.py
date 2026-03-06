# Virtual SDCard print stat tracking
#
# Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class PrintStats:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode_move = printer.load_object(config, 'gcode_move')
        self.reactor = printer.get_reactor()
        # Track individual extruder usage
        self.e_filament_used = 0.  # extruder1 (E)
        self.u_filament_used = 0.  # extruder (U) 
        self.v_filament_used = 0.  # extruder2 (V)
        self.last_e_pos = 0.
        self.last_u_pos = 0.
        self.last_v_pos = 0.
        self.current_extruder = None
        self.reset()
        # Register commands
        self.gcode = printer.lookup_object('gcode')
        self.gcode.register_command(
            "SET_PRINT_STATS_INFO", self.cmd_SET_PRINT_STATS_INFO,
            desc=self.cmd_SET_PRINT_STATS_INFO_help)
            
        # Delay toolhead lookup until it's actually needed
        self._toolhead = None
        # Register event handler for extruder activation
        printer.register_event_handler("extruder:activate_extruder",
                                      self._handle_activate_extruder)
        printer.register_event_handler("gcode:move_extruder",
                self._handle_move_extruder)
    
    def _handle_move_extruder(self, move_extruder, move_pos, absolute_mode):
        self.current_extruder = move_extruder
        if self.current_extruder == 'extruder':
            if absolute_mode:
                self.u_filament_used = move_pos
            else:
                self.u_filament_used += move_pos
        elif self.current_extruder == 'extruder1':
            if absolute_mode:
                self.e_filament_used = move_pos
            else:
                self.e_filament_used += move_pos
        elif self.current_extruder == 'extruder2':
            if absolute_mode:
                self.v_filament_used = move_pos
            else:
                self.v_filament_used += move_pos

    def _handle_activate_extruder(self):
        # Get the current extruder name
        if self._toolhead is None:
            self._toolhead = self.printer.lookup_object('toolhead')
        extruder = self._toolhead.get_extruder()
        self.current_extruder = extruder.get_name()
        # self.printer.get_logger().info(f"Activated extruder: {self.current_extruder}")

        # Update last positions for all extruders
        eventtime = self.reactor.monotonic()
        gc_status = self.gcode_move.get_status(eventtime)
        cur_epos = gc_status['position'].e
        
        if self.current_extruder == 'extruder':
            self.last_u_pos = cur_epos
        elif self.current_extruder == 'extruder1':
            self.last_e_pos = cur_epos
        elif self.current_extruder == 'extruder2':
            self.last_v_pos = cur_epos
    
    def _update_filament_usage(self, eventtime):
        gc_status = self.gcode_move.get_status(eventtime)
        cur_epos = gc_status['position'].e
        extrude_factor = gc_status['extrude_factor']
        
        # Calculate total filament used (backward compatibility)
        delta = (cur_epos - self.last_epos) / extrude_factor
        self.filament_used += delta
        self.last_epos = cur_epos
        
        # Update individual extruder usage based on current extruder
        if self.current_extruder == 'extruder':
            u_delta = (cur_epos - self.last_u_pos) / extrude_factor
            self.u_filament_used += u_delta
            self.last_u_pos = cur_epos
        elif self.current_extruder == 'extruder1':
            e_delta = (cur_epos - self.last_e_pos) / extrude_factor
            self.e_filament_used += e_delta
            self.last_e_pos = cur_epos
        elif self.current_extruder == 'extruder2':
            v_delta = (cur_epos - self.last_v_pos) / extrude_factor
            self.v_filament_used += v_delta
            self.last_v_pos = cur_epos
        
    def set_current_file(self, filename):
        self.reset()
        self.filename = filename
    def note_start(self):
        curtime = self.reactor.monotonic()
        if self.print_start_time is None:
            self.print_start_time = curtime
        elif self.last_pause_time is not None:
            # Update pause time duration
            pause_duration = curtime - self.last_pause_time
            self.prev_pause_duration += pause_duration
            self.last_pause_time = None
        # Reset last e-position
        gc_status = self.gcode_move.get_status(curtime)
        self.last_epos = gc_status['position'].e
        self.state = "printing"
        self.error_message = ""
    def note_pause(self):
        if self.last_pause_time is None:
            curtime = self.reactor.monotonic()
            self.last_pause_time = curtime
            # update filament usage
            # self._update_filament_usage(curtime)
        if self.state != "error":
            self.state = "paused"
    def note_complete(self):
        self._note_finish("complete")
    def note_error(self, message):
        self._note_finish("error", message)
    def note_cancel(self):
        self._note_finish("cancelled")
    def _note_finish(self, state, error_message = ""):
        if self.print_start_time is None:
            return
        self.state = state
        self.error_message = error_message
        eventtime = self.reactor.monotonic()
        self.total_duration = eventtime - self.print_start_time
        if self.filament_used < 0.0000001:
            # No positive extusion detected during print
            self.init_duration = self.total_duration - \
                self.prev_pause_duration
        self.print_start_time = None
    cmd_SET_PRINT_STATS_INFO_help = "Pass slicer info like layer act and " \
                                    "total to klipper"
    def cmd_SET_PRINT_STATS_INFO(self, gcmd):
        total_layer = gcmd.get_int("TOTAL_LAYER", self.info_total_layer, \
                                   minval=0)
        current_layer = gcmd.get_int("CURRENT_LAYER", self.info_current_layer, \
                                     minval=0)
        if total_layer == 0:
            self.info_total_layer = None
            self.info_current_layer = None
        elif total_layer != self.info_total_layer:
            self.info_total_layer = total_layer
            self.info_current_layer = 0

        if self.info_total_layer is not None and \
                current_layer is not None and \
                current_layer != self.info_current_layer:
            self.info_current_layer = min(current_layer, self.info_total_layer)
    def reset(self):
        self.filename = self.error_message = ""
        self.state = "standby"
        self.prev_pause_duration = self.last_epos = 0.
        self.filament_used = self.total_duration = 0.
        self.e_filament_used = self.u_filament_used = self.v_filament_used = 0.
        self.last_e_pos = self.last_u_pos = self.last_v_pos = 0.
        self.current_extruder = None
        self.print_start_time = self.last_pause_time = None
        self.init_duration = 0.
        self.info_total_layer = None
        self.info_current_layer = None
    def get_status(self, eventtime):
        time_paused = self.prev_pause_duration
        if self.print_start_time is not None:
            if self.last_pause_time is not None:
                # Calculate the total time spent paused during the print
                time_paused += eventtime - self.last_pause_time
            # else:
            #     # Accumulate filament if not paused
            #     self._update_filament_usage(eventtime)
            self.total_duration = eventtime - self.print_start_time
            if self.filament_used < 0.0000001:
                # Track duration prior to extrusion
                self.init_duration = self.total_duration - time_paused
        print_duration = self.total_duration - self.init_duration - time_paused
        return {
            'filename': self.filename,
            'total_duration': self.total_duration,
            'print_duration': print_duration,
            'filament_used': self.filament_used,
            'e_filament_used': self.e_filament_used,
            'u_filament_used': self.u_filament_used,
            'v_filament_used': self.v_filament_used,
            'state': self.state,
            'message': self.error_message,
            'info': {'total_layer': self.info_total_layer,
                     'current_layer': self.info_current_layer}
        }

def load_config(config):
    return PrintStats(config)
