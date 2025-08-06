# G-Code G1 movement commands (and associated coordinate manipulation)
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class GCodeMove:
    def __init__(self, config):
        self.config = config
        self.mode = -1
        self.proportion = -1
        self.printer = printer = config.get_printer()
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        printer.register_event_handler("toolhead:set_position",
                                       self.reset_last_position)
        printer.register_event_handler("toolhead:manual_move",
                                       self.reset_last_position)
        printer.register_event_handler("gcode:command_error",
                                       self.reset_last_position)
        printer.register_event_handler("extruder:activate_extruder",
                                       self._handle_activate_extruder)
        printer.register_event_handler("homing:home_rails_end",
                                       self._handle_home_rails_end)
        self.is_printer_ready = False
        # Register g-code commands
        gcode = printer.lookup_object('gcode')
        handlers = [
            'G1', 'G20', 'G21',
            'M82', 'M83', 'G90', 'G91', 'G92', 'M220', 'M221',
            'SET_GCODE_OFFSET', 'SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE',
        ]
        for cmd in handlers:
            func = getattr(self, 'cmd_' + cmd)
            desc = getattr(self, 'cmd_' + cmd + '_help', None)
            gcode.register_command(cmd, func, False, desc)
        gcode.register_command('G0', self.cmd_G1)
        gcode.register_command('M114', self.cmd_M114, True)
        gcode.register_command('GET_POSITION', self.cmd_GET_POSITION, True,
                               desc=self.cmd_GET_POSITION_help)
        self.Coord = gcode.Coord
        # G-Code coordinate manipulation
        self.absolute_coord = self.absolute_extrude = True
        self.base_position = [0.0, 0.0, 0.0, 0.0]
        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.homing_position = [0.0, 0.0, 0.0, 0.0]
        # G-Code extruder E U V position
        # TODO 更好的实现方式是在extruder类中加入挤出机的基点坐标和上一次的坐标
        self.extruder_base_position = [0.0, 0.0, 0.0]
        self.extruder_last_position = [0.0, 0.0, 0.0]
        self.speed = 25.
        self.speed_factor = 1. / 60.
        self.extrude_factor = 1.
        # G-Code state
        self.saved_states = {}
        self.move_transform = self.move_with_transform = None
        self.position_with_transform = (lambda: [0., 0., 0., 0.])
        self.printer.register_event_handler('idle_timeout:idle', self._handle_not_printing)

    # idle Timeout handler
    def _handle_not_printing(self, print_time):
        logging.info("G-Code move: idle_timeout:idle")
        self.mode = -1
        self.proportion = -1
    def _handle_ready(self):
        self.is_printer_ready = True
        if self.move_transform is None:
            toolhead = self.printer.lookup_object('toolhead')
            self.move_with_transform = toolhead.move
            self.position_with_transform = toolhead.get_position
        self.reset_last_position()
    def _handle_shutdown(self):
        if not self.is_printer_ready:
            return
        self.is_printer_ready = False
        logging.info("gcode state: absolute_coord=%s absolute_extrude=%s"
                     " base_position=%s last_position=%s homing_position=%s"
                     " speed_factor=%s extrude_factor=%s speed=%s",
                     self.absolute_coord, self.absolute_extrude,
                     self.base_position, self.last_position,
                     self.homing_position, self.speed_factor,
                     self.extrude_factor, self.speed)
    def _handle_activate_extruder(self):
        self.reset_last_position()
        self.extrude_factor = 1.
        self.base_position[3] = self.last_position[3]
    def _handle_home_rails_end(self, homing_state, rails):
        self.reset_last_position()
        for axis in homing_state.get_axes():
            self.base_position[axis] = self.homing_position[axis]
    def set_move_transform(self, transform, force=False):
        if self.move_transform is not None and not force:
            raise self.printer.config_error(
                "G-Code move transform already specified")
        old_transform = self.move_transform
        if old_transform is None:
            old_transform = self.printer.lookup_object('toolhead', None)
        self.move_transform = transform
        self.move_with_transform = transform.move
        self.position_with_transform = transform.get_position
        return old_transform
    def _get_gcode_position(self):
        p = [lp - bp for lp, bp in zip(self.last_position, self.base_position)]
        p[3] /= self.extrude_factor
        return p
    def _get_gcode_speed(self):
        return self.speed / self.speed_factor
    def _get_gcode_speed_override(self):
        return self.speed_factor * 60.
    def get_status(self, eventtime=None):
        move_position = self._get_gcode_position()
        return {
            'speed_factor': self._get_gcode_speed_override(),
            'speed': self._get_gcode_speed(),
            'extrude_factor': self.extrude_factor,
            'absolute_coordinates': self.absolute_coord,
            'absolute_extrude': self.absolute_extrude,
            'homing_origin': self.Coord(*self.homing_position),
            'position': self.Coord(*self.last_position),
            'gcode_position': self.Coord(*move_position),
        }
    def reset_last_position(self):
        if self.is_printer_ready:
            self.last_position = self.position_with_transform()
    
    # G-Code movement commands
    # extruder carbon fibre
    # extruder1 plastics
    # extruder2 carbon fibre adhesive
    # mode 0:E, 1:UV, 2:U, 3:V
    def aniso_extruder_switch(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        params = gcmd.get_command_parameters()
        #curr_extruder = self.printer.lookup_object('toolhead').get_extruder()
        extruder = self.printer.lookup_object('extruder')
        extruder1 = self.printer.lookup_object('extruder1')
        extruder2 = self.printer.lookup_object('extruder2')
        
        if 'E' in params:
            self.mode = 0
            self.proportion = 0
        elif 'U' in params:
            if self.absolute_extrude:
                logging.info("absolute extrude")
            
            if 'V' in params:
                if self.mode != 1:
                    gcode.run_script_from_command("ACTIVATE_EXTRUDER EXTRUDER=extruder")
                    gcode.run_script_from_command("SYNC_EXTRUDER_MOTION EXTRUDER=extruder2 MOTION_QUEUE=extruder")
                    self.mode = 1

                U_rotation_distance = self.config.getsection('extruder').getfloat('rotation_distance')
                V_rotation_distance = self.config.getsection('extruder2').getfloat('rotation_distance')
                # 因为gcode文件中的P参数是错误的，所以这里先忽略它。
                '''
                if 'P' in params:
                    p =  float(params['P'])
                    if self.proportion != p:
                        gcode.run_script_from_command("SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=extruder2 DISTANCE=%f" % (V_rotation_distance / p))
                        self.proportion = p
                else:
                '''
                p = self.proportion
                incrument_u = 0.0
                incrument_v = 0.0
                if self.absolute_extrude:
                    incrument_u = float(params['U']) + self.extruder_base_position[1] - self.extruder_last_position[1]
                    incrument_v = float(params['V']) + self.extruder_base_position[2] - self.extruder_last_position[2]
                else:
                    incrument_u = float(params['U'])
                    incrument_v = float(params['V'])

                logging.debug("incrument_u: %f, incrument_v: %f", incrument_u, incrument_v)
                if abs(incrument_u) < 0.0000001 or abs(incrument_v) < 0.0000001:
                    logging.warning("U or V increment is too small, do not set proportion")
                else:
                    p = incrument_v / incrument_u

                if self.proportion != p:
                    gcode.run_script_from_command("SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=extruder2 DISTANCE=%f" % (V_rotation_distance / p))
            else:
                if self.mode != 2:
                    gcode.run_script_from_command("ACTIVATE_EXTRUDER EXTRUDER=extruder")
                    gcode.run_script_from_command("SYNC_EXTRUDER_MOTION EXTRUDER=extruder2 MOTION_QUEUE=extruder2")
                    self.proportion = 0
                    self.mode = 2
        elif 'V' in params: 
            if self.mode != 3:
                if self.absolute_extrude:
                    logging.info("absolute extrude")

                gcode.run_script_from_command("ACTIVATE_EXTRUDER EXTRUDER=extruder2")
                V_rotation_distance = self.config.getsection('extruder2').getfloat('rotation_distance')
                gcode.run_script_from_command("SYNC_EXTRUDER_MOTION EXTRUDER=extruder2 MOTION_QUEUE=extruder2")
                gcode.run_script_from_command("SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=extruder2 DISTANCE=%f" % (V_rotation_distance))
                self.mode = 3
                self.proportion = 0

    def cmd_G1(self, gcmd):
        # Move
        self.aniso_extruder_switch(gcmd)
        params = gcmd.get_command_parameters()
        try:
            for pos, axis in enumerate('XYZ'):
                if axis in params:
                    val = float(params[axis])
                    if self.absolute_coord:
                        # value relative to base coordinate position
                        self.last_position[pos] = val + self.base_position[pos]
                    else:
                        # value relative to position of last move
                        self.last_position[pos] += val

            for pos, axis in enumerate('EUV'):
                if axis in params:
                    val = float(params[axis]) * self.extrude_factor
                    if self.absolute_extrude:
                        # value relative to base coordinate position
                        self.extruder_last_position[pos] = val + self.extruder_base_position[pos]
                    else:
                        # value relative to position of last move
                        self.extruder_last_position[pos] += val

            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                self.speed = gcode_speed * self.speed_factor
        except ValueError as e:
            raise gcmd.error("Unable to parse move '%s'"
                             % (gcmd.get_commandline(),))
        
        if self.mode == 0:
            # E mode
            self.last_position[3] = self.extruder_last_position[0]
        elif self.mode == 1:
            # UV mode
            self.last_position[3] = self.extruder_last_position[1]
            # 因为在UV模式下，U挤出机内部的last_position在Move对象被加入梯形运动队列后会被更新，而V挤出机不会被更新，
            # 所以这里需要手动更新V挤出机的last_position。
            # 更好的做法是将U V解绑。
            extruder2 = self.printer.lookup_object('extruder2')
            extruder2.last_position = self.extruder_last_position[2]
        elif self.mode == 2:
            # U mode
            self.last_position[3] = self.extruder_last_position[1]
        elif self.mode == 3:
            # V mode
            self.last_position[3] = self.extruder_last_position[2]

        self.move_with_transform(self.last_position, self.speed)
    # G-Code coordinate manipulation
    def cmd_G20(self, gcmd):
        # Set units to inches
        raise gcmd.error('Machine does not support G20 (inches) command')
    def cmd_G21(self, gcmd):
        # Set units to millimeters
        pass
    def cmd_M82(self, gcmd):
        # Use absolute distances for extrusion
        self.absolute_extrude = True
    def cmd_M83(self, gcmd):
        # Use relative distances for extrusion
        self.absolute_extrude = False
    def cmd_G90(self, gcmd):
        # Use absolute coordinates
        self.absolute_coord = True
    def cmd_G91(self, gcmd):
        # Use relative coordinates
        self.absolute_coord = False

    def cmd_G92(self, gcmd):
        # Set position

        offsets_space = [ gcmd.get_float(a, None) for a in 'XYZ' ]
        for i, offset in enumerate(offsets_space):
            if offset is not None:
                self.base_position[i] = self.last_position[i] - offset

        offsets_extrude = [ gcmd.get_float(a, None) for a in 'EUV' ]
        for i, offset in enumerate(offsets_extrude):
            if offset is not None:
                offset *= self.extrude_factor
                self.base_position[3] = self.last_position[3] - offset
                self.extruder_base_position[i] = self.extruder_last_position[i] - offset     

        if offsets_space == [None, None, None] and offsets_extrude == [None, None, None]:
            self.base_position = list(self.last_position)
            # Reset the extruder base position
            self.extruder_base_position = list(self.extruder_last_position)
            logging.info("G92: Reset base position to last position, extruder_base_position:%s", self.extruder_base_position)

    def cmd_M114(self, gcmd):
        # Get Current Position
        p = self._get_gcode_position()
        gcmd.respond_raw("X:%.3f Y:%.3f Z:%.3f E:%.3f" % tuple(p))
    def cmd_M220(self, gcmd):
        # Set speed factor override percentage
        value = gcmd.get_float('S', 100., above=0.) / (60. * 100.)
        self.speed = self._get_gcode_speed() * value
        self.speed_factor = value
    def cmd_M221(self, gcmd):
        # Set extrude factor override percentage
        new_extrude_factor = gcmd.get_float('S', 100., above=0.) / 100.
        last_e_pos = self.last_position[3]
        e_value = (last_e_pos - self.base_position[3]) / self.extrude_factor
        self.base_position[3] = last_e_pos - e_value * new_extrude_factor
        self.extrude_factor = new_extrude_factor
    cmd_SET_GCODE_OFFSET_help = "Set a virtual offset to g-code positions"
    def cmd_SET_GCODE_OFFSET(self, gcmd):
        move_delta = [0., 0., 0., 0.]
        for pos, axis in enumerate('XYZE'):
            offset = gcmd.get_float(axis, None)
            if offset is None:
                offset = gcmd.get_float(axis + '_ADJUST', None)
                if offset is None:
                    continue
                offset += self.homing_position[pos]
            delta = offset - self.homing_position[pos]
            move_delta[pos] = delta
            self.base_position[pos] += delta
            self.homing_position[pos] = offset
        # Move the toolhead the given offset if requested
        if gcmd.get_int('MOVE', 0):
            speed = gcmd.get_float('MOVE_SPEED', self.speed, above=0.)
            for pos, delta in enumerate(move_delta):
                self.last_position[pos] += delta
            self.move_with_transform(self.last_position, speed)
    cmd_SAVE_GCODE_STATE_help = "Save G-Code coordinate state"
    def cmd_SAVE_GCODE_STATE(self, gcmd):
        state_name = gcmd.get('NAME', 'default')
        self.saved_states[state_name] = {
            'absolute_coord': self.absolute_coord,
            'absolute_extrude': self.absolute_extrude,
            'base_position': list(self.base_position),
            'last_position': list(self.last_position),
            'homing_position': list(self.homing_position),
            'speed': self.speed, 'speed_factor': self.speed_factor,
            'extrude_factor': self.extrude_factor,
        }
    cmd_RESTORE_GCODE_STATE_help = "Restore a previously saved G-Code state"
    def cmd_RESTORE_GCODE_STATE(self, gcmd):
        state_name = gcmd.get('NAME', 'default')
        state = self.saved_states.get(state_name)
        if state is None:
            raise gcmd.error("Unknown g-code state: %s" % (state_name,))
        # Restore state
        self.absolute_coord = state['absolute_coord']
        self.absolute_extrude = state['absolute_extrude']
        self.base_position = list(state['base_position'])
        self.homing_position = list(state['homing_position'])
        self.speed = state['speed']
        self.speed_factor = state['speed_factor']
        self.extrude_factor = state['extrude_factor']
        # Restore the relative E position
        e_diff = self.last_position[3] - state['last_position'][3]
        self.base_position[3] += e_diff
        # Move the toolhead back if requested
        if gcmd.get_int('MOVE', 0):
            speed = gcmd.get_float('MOVE_SPEED', self.speed, above=0.)
            self.last_position[:3] = state['last_position'][:3]
            self.move_with_transform(self.last_position, speed)
    cmd_GET_POSITION_help = (
        "Return information on the current location of the toolhead")
    def cmd_GET_POSITION(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            raise gcmd.error("Printer not ready")
        kin = toolhead.get_kinematics()
        steppers = kin.get_steppers()
        mcu_pos = " ".join(["%s:%d" % (s.get_name(), s.get_mcu_position())
                            for s in steppers])
        cinfo = [(s.get_name(), s.get_commanded_position()) for s in steppers]
        stepper_pos = " ".join(["%s:%.6f" % (a, v) for a, v in cinfo])
        kinfo = zip("XYZ", kin.calc_position(dict(cinfo)))
        kin_pos = " ".join(["%s:%.6f" % (a, v) for a, v in kinfo])
        toolhead_pos = " ".join(["%s:%.6f" % (a, v) for a, v in zip(
            "XYZE", toolhead.get_position())])
        gcode_pos = " ".join(["%s:%.6f"  % (a, v)
                              for a, v in zip("XYZE", self.last_position)])
        base_pos = " ".join(["%s:%.6f"  % (a, v)
                             for a, v in zip("XYZE", self.base_position)])
        homing_pos = " ".join(["%s:%.6f"  % (a, v)
                               for a, v in zip("XYZ", self.homing_position)])
        gcmd.respond_info("mcu: %s\n"
                          "stepper: %s\n"
                          "kinematic: %s\n"
                          "toolhead: %s\n"
                          "gcode: %s\n"
                          "gcode base: %s\n"
                          "gcode homing: %s"
                          % (mcu_pos, stepper_pos, kin_pos, toolhead_pos,
                             gcode_pos, base_pos, homing_pos))

def load_config(config):
    return GCodeMove(config)
