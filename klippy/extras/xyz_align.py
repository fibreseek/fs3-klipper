import logging
import configparser
import os

class xyz_align:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.session_x = config.getsection('stepper_x')
        self.session_y = config.getsection('stepper_y')
        self.session_z = config.getsection('stepper_z')
        self.gcode = self.printer.lookup_object('gcode')
        self.t0_offset_x = config.getfloat('t0_offset_x')
        self.t0_offset_y = config.getfloat('t0_offset_y')
        self.t1_offset_x = config.getfloat('t1_offset_x')
        self.t1_offset_y = config.getfloat('t1_offset_y')
        self.left_move_offset = config.getfloat('left_move_offset', 12.0)
        self.right_move_offset = config.getfloat('right_move_offset', 10.0)
        self.z_press_offset = config.getfloat('z_press_offset', 0.7)
        self.speed = config.getfloat('speed', 3000)
        self.align_trigger_timeout = config.getfloat(
            'align_trigger_timeout', 3.0)
        self.is_first_align = True
        self.offset_z = 10.0
        self.trigger_value = 1.0
        self.trigger_step_x = 0
        self.trigger_step_y = 0
        self.trigger_step_z = 0
        self.wait_check_result = False
        self.ini_path = os.path.expanduser("~/printer_data/config/xyz_offset.ini")
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(config.get('pin'))
        self.mcu = pin_params['chip']
        self.oid = self.mcu.create_oid()
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self.build_config)

        self.printer.add_object('xyz_align', self)
        self.gcode.register_command(
            'XYZ_ALIGN_NEW', self.cmd_XYZ_ALIGN_NEW,
            desc=self.cmd_XYZ_ALIGN_NEW_help)

        self.mcu.register_response(
            self.handle_trigger_result, "xyz_align_trigger", self.oid)
        self.printer.register_event_handler(
            "klippy:ready", self.xyz_init_offset)

    def xyz_init_offset(self):
        config = configparser.ConfigParser()

        try:
            if os.path.exists(self.ini_path):
                config.read(self.ini_path)

                if 't0_offset' in config:
                    self.t0_offset_x = config.getfloat('t0_offset', 'x_offset')
                    self.t0_offset_y = config.getfloat('t0_offset', 'y_offset')
                    self.left_move_offset = config.getfloat(
                        't0_offset', 'move_offset')

                if 't1_offset' in config:
                    self.t1_offset_x = config.getfloat('t1_offset', 'x_offset')
                    self.t1_offset_y = config.getfloat('t1_offset', 'y_offset')
                    self.right_move_offset = config.getfloat(
                        't1_offset', 'move_offset')

                self.is_first_align = False
                logging.info(
                    "Loaded xyz_offset.ini value: T0(%.3f,%.3f) T1(%.3f,%.3f)"
                    % (self.t0_offset_x, self.t0_offset_y,
                       self.t1_offset_x, self.t1_offset_y))
            else:
                logging.info("xyz_offset.ini file not found, using default values")

        except Exception as e:
            logging.error("Failed to load xyz_offset.ini: %s" % str(e))
            self.gcode.respond_info(
                "Warning: Failed to load xyz_offset.ini, using default values")

        logging.info("init xyz_offset.ini file end")

    def build_config(self):
        self.mcu.add_config_cmd("align_start oid=%d val=%d" % (self.oid, 0))
        self.align_start_cmd = self.mcu.lookup_command(
            "align_start oid=%c val=%c", cq=self.cmd_queue)

    def xyz_update_position(self, x0, y0, x1, y1):
        config = configparser.ConfigParser()

        os.makedirs(os.path.dirname(self.ini_path), exist_ok=True)
        config.read(self.ini_path)

        if 't0_offset' not in config:
            config['t0_offset'] = {}
        if 't1_offset' not in config:
            config['t1_offset'] = {}

        config['t0_offset']['x_offset'] = str(x0)
        config['t0_offset']['y_offset'] = str(y0)
        config['t0_offset']['move_offset'] = str(self.left_move_offset)
        config['t1_offset']['x_offset'] = str(x1)
        config['t1_offset']['y_offset'] = str(y1)
        config['t1_offset']['move_offset'] = str(self.right_move_offset)
        logging.info(
            "Updated xyz_offset.ini: T0(%.3f,%.3f) T1(%.3f,%.3f)"
            % (x0, y0, x1, y1))

        try:
            with open(self.ini_path, 'w') as configfile:
                config.write(configfile)
            self.gcode.respond_info("Update xyz offset config file success")
        except Exception as e:
            self.gcode.respond_info(
                "Update xyz offset config file failed: %s" % str(e))
            logging.error("Failed to update xyz_offset.ini: %s" % str(e))

    def handle_trigger_result(self, params):
        res = params['val']
        if res == 0 or res == 1:
            self.trigger_value = res
            self.wait_check_result = True
            self.trigger_step_x = params['x0']
            self.trigger_step_y = params['y0']
            self.trigger_step_z = params['z0']
            self.gcode.respond_info(
                "handle_trigger_result: res = %d step_x = %d step_y = %d step_z = %d"
                % (res, self.trigger_step_x, self.trigger_step_y,
                   self.trigger_step_z))

    def _send_align_trigger_query(self):
        self.wait_check_result = False
        self.align_start_cmd.send([self.oid, 2])

    def _wait_align_trigger_response(self, context):
        reactor = self.printer.get_reactor()
        start_time = reactor.monotonic()
        while (not self.wait_check_result) and (
                reactor.monotonic() - start_time) < self.align_trigger_timeout:
            reactor.pause(0.05)
        if not self.wait_check_result:
            msg = ("xyz_align: MCU xyz_align_trigger timeout after %.1fs [%s]"
                   % (self.align_trigger_timeout, context))
            logging.error(msg)
            self.gcode.respond_info("[NOZZLE_OFFSET_CALIBRATION] %s" % (msg,))
            return False
        return True

    def get_z_press_val_new(self, t, r=2):
        toolhead = self.printer.lookup_object('toolhead')
        trigger_pos = toolhead.get_position()
        move_distance = 0
        move_step = -0.05
        self.gcode.run_script_from_command("G1 Z%f F180\n" % (5+5*t))
        self.gcode.run_script_from_command("M400\n")
        self.gcode.run_script_from_command("G91\n")
        for i in range(r):
            if i == 0:
                move_step = -0.05
            if i == 1:
                self.gcode.run_script_from_command("G1 Z0.5 F80\n")
                self.gcode.run_script_from_command("M400\n")
                move_step = -0.01
            move_distance = 0
            self.trigger_value = 1
            while move_distance <= (10 - t*3):
                self.gcode.run_script_from_command("G1 Z%f F80\n" % (move_step))
                self.gcode.run_script_from_command("M400\n")
                self._send_align_trigger_query()
                if not self._wait_align_trigger_response(
                        "get_z_press_val_new t=%d" % (t,)):
                    return None

                if self.trigger_value == 0:
                    toolhead.wait_moves()
                    trigger_pos = toolhead.get_position()
                    break
                move_distance += move_step

        self.gcode.run_script_from_command("G90\n")
        self.gcode.run_script_from_command("M114\n")
        self.offset_z_press = trigger_pos[2] - self.z_press_offset
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN: offset_z_press = %f, self.offset_z = %f, move_distance = %f"
            % (self.offset_z_press, self.offset_z, move_distance))
        return trigger_pos[2]

    def get_x_press_val_new(self, dir, r=2):
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN: get_x_press_val_new dir = %d" % (dir))
        toolhead = self.printer.lookup_object('toolhead')
        trigger_pos = toolhead.get_position()
        move_distance = 0
        move_step = 0.5*dir
        self.gcode.run_script_from_command(
            "G1 Z%f F180\n" % (self.offset_z_press))
        self.gcode.run_script_from_command("M400\n")

        for i in range(r):
            if i == 0:
                move_step = 0.5*dir
            if i == 1:
                self.gcode.run_script_from_command("G91\n")
                self.gcode.run_script_from_command("G1 Z3 F300\n")
                self.gcode.run_script_from_command("G1 X%f F800\n" % (-1.2*dir))
                self.gcode.run_script_from_command("G90\n")
                self.gcode.run_script_from_command(
                    "G1 Z%f F180\n" % (self.offset_z_press))
                self.gcode.run_script_from_command("M400\n")
                move_step = 0.01*dir
            move_distance = 0
            self.trigger_value = 0
            self.gcode.run_script_from_command("G91\n")
            while move_distance <= 15:
                self.gcode.run_script_from_command(
                    "G1 X%f F%f\n" % (move_step, self.speed))
                self.gcode.run_script_from_command("M400\n")
                self._send_align_trigger_query()
                if not self._wait_align_trigger_response(
                        "get_x_press_val_new dir=%d" % (dir,)):
                    return None

                if self.trigger_value == 1:
                    toolhead.wait_moves()
                    trigger_pos = toolhead.get_position()
                    break
                move_distance += move_step
            self.gcode.run_script_from_command("G90\n")

        self.gcode.run_script_from_command("M114\n")
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN: move_distance = %f" % (move_distance))
        return trigger_pos[0]

    def get_y_press_val_new(self, dir, r=2):
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN: get_y_press_val_new dir = %d" % (dir))
        toolhead = self.printer.lookup_object('toolhead')
        trigger_pos = toolhead.get_position()
        move_distance = 0
        move_step = 0.5*dir
        self.gcode.run_script_from_command(
            "G1 Z%f F180\n" % (self.offset_z_press))
        self.gcode.run_script_from_command("M400\n")

        for i in range(r):
            if i == 0:
                move_step = 0.5*dir
            if i == 1:
                self.gcode.run_script_from_command("G91\n")
                self.gcode.run_script_from_command("G1 Z3 F300\n")
                self.gcode.run_script_from_command("G1 Y%f F800\n" % (-1.2*dir))
                self.gcode.run_script_from_command("G90\n")
                self.gcode.run_script_from_command(
                    "G1 Z%f F180\n" % (self.offset_z_press))
                self.gcode.run_script_from_command("M400\n")
                move_step = 0.01*dir
            move_distance = 0
            self.trigger_value = 0
            self.gcode.run_script_from_command("G91\n")
            while move_distance <= 15:
                self.gcode.run_script_from_command(
                    "G1 Y%f F%f\n" % (move_step, self.speed))
                self.gcode.run_script_from_command("M400\n")
                self._send_align_trigger_query()
                if not self._wait_align_trigger_response(
                        "get_y_press_val_new dir=%d" % (dir,)):
                    return None

                if self.trigger_value == 1:
                    toolhead.wait_moves()
                    trigger_pos = toolhead.get_position()
                    break
                move_distance += move_step
            self.gcode.run_script_from_command("G90\n")

        self.gcode.run_script_from_command("M114\n")
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN: move_distance = %f" % (move_distance))
        return trigger_pos[1]

    def get_t0_center_position(self, range=2):
        trigger_z0 = self.get_z_press_val_new(0, range)
        if trigger_z0 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 Z probe MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 0:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 Z trigger failed, aborting.")
            return -1, -1, -1
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW: trigger_z0 = %f" % (trigger_z0))

        self.gcode.run_script_from_command("G1 Z13 F600\n")
        trigger_t0_x0 = self.get_x_press_val_new(-1, range)
        if trigger_t0_x0 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 X- MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 X- trigger failed, aborting.")
            return -1, -1, -1

        self.gcode.run_script_from_command("G1 Z12 F600\n")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F3000\n" % (self.t0_offset_x, self.t0_offset_y))
        trigger_t0_x1 = self.get_x_press_val_new(1, range)
        if trigger_t0_x1 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 X+ MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 X+ trigger failed, aborting.")
            return -1, -1, -1

        self.gcode.run_script_from_command("G1 Z12 F600\n")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F3000\n" % (self.t0_offset_x, self.t0_offset_y))
        trigger_t0_y0 = self.get_y_press_val_new(-1, range)
        if trigger_t0_y0 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 Y- MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 Y- trigger failed, aborting.")
            return -1, -1, -1

        self.gcode.run_script_from_command("G1 Z12 F600\n")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F3000\n" % (self.t0_offset_x, self.t0_offset_y))
        trigger_t0_y1 = self.get_y_press_val_new(1, range)
        if trigger_t0_y1 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 Y+ MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T0 Y+ trigger failed, aborting.")
            return -1, -1, -1

        center_x0 = trigger_t0_x0 + abs(trigger_t0_x0 - trigger_t0_x1) / 2
        center_y0 = trigger_t0_y0 + abs(trigger_t0_y0 - trigger_t0_y1) / 2
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW: trigger_z0 = %f, trigger_t0_x0 = %f, trigger_t0_x1 = %f, trigger_t0_y0 = %f, trigger_t0_y1 = %f, center_x0 = %f, center_y0 = %f"
            % (trigger_z0, trigger_t0_x0, trigger_t0_x1, trigger_t0_y0,
               trigger_t0_y1, center_x0, center_y0))
        return center_x0, center_y0, trigger_z0

    def get_t1_center_position(self, range=2):
        trigger_z1 = self.get_z_press_val_new(1, range)
        if trigger_z1 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 Z probe MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 0:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 Z trigger failed, aborting.")
            return -1, -1, -1
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW: trigger_z1 = %f" % (trigger_z1))

        self.gcode.run_script_from_command("G1 Z12 F600\n")
        trigger_t1_x0 = self.get_x_press_val_new(-1, range)
        if trigger_t1_x0 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 X- MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 X- trigger failed, aborting.")
            return -1, -1, -1

        self.gcode.run_script_from_command("G1 Z12 F600\n")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F3000\n" % (self.t1_offset_x, self.t1_offset_y))
        trigger_t1_x1 = self.get_x_press_val_new(1, range)
        if trigger_t1_x1 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 X+ MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 X+ trigger failed, aborting.")
            return -1, -1, -1

        self.gcode.run_script_from_command("G1 Z12 F600\n")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F3000\n" % (self.t1_offset_x, self.t1_offset_y))
        trigger_t1_y0 = self.get_y_press_val_new(-1, range)
        if trigger_t1_y0 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 Y- MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 Y- trigger failed, aborting.")
            return -1, -1, -1

        self.gcode.run_script_from_command("G1 Z12 F600\n")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F3000\n" % (self.t1_offset_x, self.t1_offset_y))
        trigger_t1_y1 = self.get_y_press_val_new(1, range)
        if trigger_t1_y1 is None:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 Y+ MCU timeout, aborting.")
            return -1, -1, -1
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] T1 Y+ trigger failed, aborting.")
            return -1, -1, -1

        center_x1 = trigger_t1_x0 + abs(trigger_t1_x0 - trigger_t1_x1) / 2
        center_y1 = trigger_t1_y0 + abs(trigger_t1_y0 - trigger_t1_y1) / 2
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW: trigger_z1 = %f, trigger_t1_x0 = %f, trigger_t1_x1 = %f, trigger_t1_y0 = %f, trigger_t1_y1 = %f, center_x1 = %f, center_y1 = %f"
            % (trigger_z1, trigger_t1_x0, trigger_t1_x1, trigger_t1_y0,
               trigger_t1_y1, center_x1, center_y1))
        return center_x1, center_y1, trigger_z1

    def get_trigger_steps(self, center_x, center_y, trigger_press_z):
        trigger_step_z0 = 0
        init_step_x0 = 0
        init_step_y0 = 0
        trigger_step_x0 = 0
        trigger_step_x1 = 0
        trigger_step_y0 = 0
        trigger_step_y1 = 0

        self.gcode.run_script_from_command(
            "G90\n"
            "G1 Z%.3f F600\n"
            "G1 X%.3f Y%.3f F3000\n"
            "G1 Z%.3f F180\n"
            % (self.offset_z, center_x, center_y, trigger_press_z))

        self.gcode.run_script_from_command("G91\n")
        self.gcode.run_script_from_command("G1 Z2 F120\n")
        self.gcode.run_script_from_command("G90\n")
        self.gcode.run_script_from_command("M400\n")
        self._send_align_trigger_query()
        if not self._wait_align_trigger_response(
                "XYZ_ALIGN_NEW get_trigger_steps Z"):
            return None
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T0 Z trigger failed, aborting.")
            return None
        trigger_step_z0 = self.trigger_step_z
        init_step_x0 = self.trigger_step_x
        init_step_y0 = self.trigger_step_y
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW_3: trigger_step_z0 = %d, init_step_x0 = %d, init_step_y0 = %d"
            % (trigger_step_z0, init_step_x0, init_step_y0))

        self.gcode.run_script_from_command(
            "G90\n"
            "G1 Z%.3f F600\n"
            "G1 X%.3f Y%.3f F3000\n"
            "G1 Z%.3f F180\n"
            % (self.offset_z, center_x, center_y, trigger_press_z))

        self.gcode.run_script_from_command("G91\n")
        self.gcode.run_script_from_command("G1 X8 F%f\n" % (self.speed))
        self.gcode.run_script_from_command("M400\n")
        self.gcode.run_script_from_command("G90\n")
        self._send_align_trigger_query()
        if not self._wait_align_trigger_response(
                "XYZ_ALIGN_NEW get_trigger_steps X-"):
            return None
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T0 X- trigger failed, aborting.")
            return None
        trigger_step_x0 = self.trigger_step_x

        self.gcode.run_script_from_command(
            "G90\n"
            "G1 Z%.3f F600\n"
            "G1 X%.3f Y%.3f F3000\n"
            "G1 Z%.3f F180\n"
            % (self.offset_z, center_x, center_y, trigger_press_z))

        self.gcode.run_script_from_command("G91\n")
        self.gcode.run_script_from_command("G1 X-8 F%f\n" % (self.speed))
        self.gcode.run_script_from_command("M400\n")
        self.gcode.run_script_from_command("G90\n")
        self._send_align_trigger_query()
        if not self._wait_align_trigger_response(
                "XYZ_ALIGN_NEW get_trigger_steps X+"):
            return None
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T0 X+ trigger failed, aborting.")
            return None
        trigger_step_x1 = self.trigger_step_x

        self.gcode.run_script_from_command(
            "G90\n"
            "G1 Z%.3f F600\n"
            "G1 X%.3f Y%.3f F3000\n"
            "G1 Z%.3f F180\n"
            % (self.offset_z, center_x, center_y, trigger_press_z))

        self.gcode.run_script_from_command("G91\n")
        self.gcode.run_script_from_command("G1 Y8 F%f\n" % (self.speed))
        self.gcode.run_script_from_command("M400\n")
        self.gcode.run_script_from_command("G90\n")
        self._send_align_trigger_query()
        if not self._wait_align_trigger_response(
                "XYZ_ALIGN_NEW get_trigger_steps Y-"):
            return None
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T0 Y- trigger failed, aborting.")
            return None
        trigger_step_y0 = self.trigger_step_y

        self.gcode.run_script_from_command(
            "G90\n"
            "G1 Z%.3f F600\n"
            "G1 X%.3f Y%.3f F3000\n"
            "G1 Z%.3f F180\n"
            % (self.offset_z, center_x, center_y, trigger_press_z))

        self.gcode.run_script_from_command("G91\n")
        self.gcode.run_script_from_command("G1 Y-8 F%f\n" % (self.speed))
        self.gcode.run_script_from_command("M400\n")
        self.gcode.run_script_from_command("G90\n")
        self._send_align_trigger_query()
        if not self._wait_align_trigger_response(
                "XYZ_ALIGN_NEW get_trigger_steps Y+"):
            return None
        if self.trigger_value != 1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T0 Y+ trigger failed, aborting.")
            return None
        trigger_step_y1 = self.trigger_step_y

        return (trigger_step_z0, init_step_x0, init_step_y0, trigger_step_x0,
                trigger_step_x1, trigger_step_y0, trigger_step_y1)

    cmd_XYZ_ALIGN_NEW_help = "This is a aniso customG-code command new"

    def cmd_XYZ_ALIGN_NEW(self, gcmd):
        x_rotation_distance = self.session_x.getfloat('rotation_distance')
        x_microsteps = self.session_x.getfloat('microsteps')
        per_step_distance_x = x_rotation_distance/(x_microsteps*200)

        y_rotation_distance = self.session_y.getfloat('rotation_distance')
        y_microsteps = self.session_y.getfloat('microsteps')
        per_step_distance_y = y_rotation_distance/(y_microsteps*200)

        z_rotation_distance = self.session_z.getfloat('rotation_distance')
        z_microsteps = self.session_z.getfloat('microsteps')
        per_step_distance_z = z_rotation_distance/(z_microsteps*200)

        self.gcode.run_script_from_command(
            "SAVE_GCODE_STATE NAME=pre_offset_state\n"
            "G90\n"
            "G1 Z%.3f F600\n"
            "T0\n"
            "G1 X%.3f Y%.3f F6000\n"
            % (self.offset_z, self.t0_offset_x, self.t0_offset_y))

        center_x0, center_y0, trigger_z0 = self.get_t0_center_position(1)
        if center_x0 == -1 and center_y0 == -1 and trigger_z0 == -1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T0 center init failed, aborting.")
            return
        self.t0_offset_x = center_x0
        self.t0_offset_y = center_y0
        self.offset_z = 10

        results = []
        for _ in range(3):
            result = self.get_trigger_steps(
                self.t0_offset_x, self.t0_offset_y, self.offset_z_press)
            if result is None:
                self.gcode.respond_info(
                    "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T0 trigger failed, aborting.")
                self.gcode.run_script_from_command(
                    "RESTORE_GCODE_STATE NAME=pre_offset_state\n")
                return
            results.append(result)

        median_results = tuple(sorted(result) for result in zip(*results))
        (trigger_step_z, init_step_x0, init_step_y0, trigger_step_x0,
         trigger_step_x1, trigger_step_y0, trigger_step_y1) = map(
            lambda x: x[len(x) // 2], median_results)
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW_3 T0: trigger_step_z = %d, init_step_x0 = %d, init_step_y0 = %d, trigger_step_x0 = %d, trigger_step_x1 = %d, trigger_step_y0 = %d, trigger_step_y1 = %d"
            % (trigger_step_z, init_step_x0, init_step_y0, trigger_step_x0,
               trigger_step_x1, trigger_step_y0, trigger_step_y1))

        step_x0 = ((abs(init_step_x0 - trigger_step_x0)
                   - abs(trigger_step_x1 - init_step_x0)) / 2)
        center_x0 = self.t0_offset_x + step_x0*per_step_distance_x

        step_y0 = ((abs(init_step_y0 - trigger_step_y0)
                   - abs(trigger_step_y1 - init_step_y0)) / 2)
        center_y0 = self.t0_offset_y + step_y0*per_step_distance_y

        trigger_z0 = trigger_step_z*per_step_distance_z
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW_3 T0:center_x0 = %f, center_y0 = %f, trigger_z0 = %f"
            % (center_x0, center_y0, trigger_z0))

        self.gcode.run_script_from_command("G1 Z10 F600\n")
        self.gcode.run_script_from_command("T1 NO_OFFSET=1\n")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F6000\n" % (self.t1_offset_x, self.t1_offset_y))

        center_x1, center_y1, trigger_z1 = self.get_t1_center_position(1)
        if center_x1 == -1 and center_y1 == -1 and trigger_z1 == -1:
            self.gcode.respond_info(
                "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T1 center init failed, aborting.")
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=pre_offset_state\n")
            return
        self.t1_offset_x = center_x1
        self.t1_offset_y = center_y1
        self.offset_z = 12

        results = []
        for _ in range(3):
            result = self.get_trigger_steps(
                self.t1_offset_x, self.t1_offset_y, self.offset_z_press)
            if result is None:
                self.gcode.respond_info(
                    "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 T1 trigger failed, aborting.")
                self.gcode.run_script_from_command(
                    "RESTORE_GCODE_STATE NAME=pre_offset_state\n")
                return
            results.append(result)

        median_results = tuple(sorted(result) for result in zip(*results))
        (trigger_step_z, init_step_x0, init_step_y0, trigger_step_x0,
         trigger_step_x1, trigger_step_y0, trigger_step_y1) = map(
            lambda x: x[len(x) // 2], median_results)
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW_3 T1: trigger_step_z = %d, init_step_x0 = %d, init_step_y0 = %d, trigger_step_x0 = %d, trigger_step_x1 = %d, trigger_step_y0 = %d, trigger_step_y1 = %d"
            % (trigger_step_z, init_step_x0, init_step_y0, trigger_step_x0,
               trigger_step_x1, trigger_step_y0, trigger_step_y1))

        step_x1 = ((abs(init_step_x0 - trigger_step_x0)
                   - abs(trigger_step_x1 - init_step_x0)) / 2)
        center_x1 = self.t1_offset_x + step_x1*per_step_distance_x

        step_y1 = ((abs(init_step_y0 - trigger_step_y0)
                   - abs(trigger_step_y1 - init_step_y0)) / 2)
        center_y1 = self.t1_offset_y + step_y1*per_step_distance_y

        trigger_z1 = trigger_step_z*per_step_distance_z
        self.gcode.respond_info(
            "cmd_XYZ_ALIGN_NEW_3 T1:center_x1 = %f, center_y1 = %f, trigger_z1 = %f"
            % (center_x1, center_y1, trigger_z1))

        x_offset = center_x1 - center_x0
        y_offset = center_y1 - center_y0
        z_offset = abs(trigger_z1 - trigger_z0)
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=xoffset VALUE=%f" % (x_offset))
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=yoffset VALUE=%f" % (y_offset))
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=zoffset VALUE=%f" % (z_offset))
        self.gcode.respond_info(
            "[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=1 x_offset=%f y_offset=%f z_offset=%f"
            % (x_offset, y_offset, z_offset))

        self.t0_offset_x = center_x0
        self.t0_offset_y = center_y0
        self.t1_offset_x = center_x1
        self.t1_offset_y = center_y1
        self.xyz_update_position(
            self.t0_offset_x, self.t0_offset_y,
            self.t1_offset_x, self.t1_offset_y)

        self.is_first_align = False
        self.gcode.run_script_from_command("T0\n")
        self.gcode.run_script_from_command("MOVE_TO_BRUSH_STATION\n")
        self.gcode.run_script_from_command(
            "RESTORE_GCODE_STATE NAME=pre_offset_state\n")

def load_config(config):
    return xyz_align(config)
