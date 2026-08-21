# Helper code for implementing homing operations
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4
# Z归位后探针校验：探点结果相对 z_offset 的最大允许偏差(mm)
HOMING_PROBE_DEVIATION_MAX = 0.05
# Z归位误触发/未触发时的最大重试次数
Z_HOMING_RETRIES = 3

# Return a completion that completes when all completions in a list complete
def multi_complete(printer, completions):
    if len(completions) == 1:
        return completions[0]
    # Build completion that waits for all completions
    reactor = printer.get_reactor()
    cp = reactor.register_callback(lambda e: [c.wait() for c in completions])
    # If any completion indicates an error, then exit main completion early
    for c in completions:
        reactor.register_callback(
            lambda e, c=c: cp.complete(1) if c.wait() else 0)
    return cp

# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.start_cmd_pos = stepper.mcu_to_commanded_position(self.start_pos)
        self.halt_pos = self.trig_pos = None
    def note_home_end(self, trigger_time):
        self.halt_pos = self.stepper.get_mcu_position()
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)
    def verify_no_probe_skew(self, haltpos):
        new_start_pos = self.stepper.get_mcu_position(self.start_cmd_pos)
        if new_start_pos != self.start_pos:
            logging.warning(
                "Stepper '%s' position skew after probe: pos %d now %d",
                self.stepper.get_name(), self.start_pos, new_start_pos)

# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        self.printer = printer
        self.endstops = endstops
        if toolhead is None:
            toolhead = printer.lookup_object('toolhead')
        self.toolhead = toolhead
        self.stepper_positions = []
    def get_mcu_endstops(self):
        return [es for es, name in self.endstops]
    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):
        startpos = self.toolhead.get_position()
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max([(abs(s.calc_position_from_coord(startpos)
                              - s.calc_position_from_coord(movepos))
                          / s.get_step_dist())
                         for s in mcu_endstop.get_steppers()])
        if max_steps <= 0.:
            return .001
        return move_t / max_steps
    def calc_toolhead_pos(self, kin_spos, offsets):
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        return list(kin.calc_position(kin_spos))[:3] + thpos[3:]
    def homing_move(self, movepos, speed, probe_pos=False,
                    triggered=True, check_triggered=True, aniso_pos=None):
        # Notify start of homing/probing move
        self.printer.send_event("homing:homing_move_begin", self)
        aniso_bed_mesh = self.printer.lookup_object('aniso_bed_mesh')
        # Note start location
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        self.stepper_positions = [ StepperPosition(s, name)
                                   for es, name in self.endstops
                                   for s in es.get_steppers() ]
        # Start endstop checking
        print_time = self.toolhead.get_last_move_time()
        endstop_triggers = []
        for mcu_endstop, name in self.endstops:
            rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
            sample_time = ENDSTOP_SAMPLE_TIME #sample_time默认值
            sample_count = ENDSTOP_SAMPLE_COUNT #sample_count默认值
            if name=='z' or name=='probe': #压电采样须在home_start与运动开始前启动
                acc_dist = speed**2/2/self.toolhead.get_kinematics().max_z_accel
                acc_step = (acc_dist / mcu_endstop.get_steppers()[0].get_step_dist())
                aniso_bed_mesh.update_uart_cmd.send(
                    [aniso_bed_mesh.oid, 2, math.ceil(acc_step)])
                aniso_bed_mesh.update_uart_cmd.send(
                    [aniso_bed_mesh.oid, 1, 0])
                sample_time = rest_time
                sample_count = 1
            wait = mcu_endstop.home_start(print_time, sample_time,
                                          sample_count, rest_time,
                                          triggered=triggered)
            endstop_triggers.append(wait)
        all_endstop_trigger = multi_complete(self.printer, endstop_triggers)
        self.toolhead.dwell(HOMING_START_DELAY)
        # Issue move
        error = None
        try:
            self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
        except self.printer.command_error as e:
            error = "Error during homing move: %s" % (str(e),)
        # Wait for endstops to trigger
        trigger_times = {}
        move_end_print_time = self.toolhead.get_last_move_time()
        for mcu_endstop, name in self.endstops:
            try:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
            except self.printer.command_error as e:
                if error is None:
                    error = "Error during homing %s: %s" % (name, str(e))
                continue
            if trigger_time > 0.:
                trigger_times[name] = trigger_time
            elif check_triggered and error is None:
                error = "No trigger on %s after full movement" % (name,)
                # if probe_pos: #若是PROBE命令，忽略报错，人为赋值一个异常结果
                #     trigger_times[name] = 0
                #     error = None              
        # Determine stepper halt positions
        self.toolhead.flush_step_generation()
        for sp in self.stepper_positions:
            tt = trigger_times.get(sp.endstop_name, move_end_print_time)
            sp.note_home_end(tt)
        if probe_pos:
            halt_steps = {sp.stepper_name: sp.halt_pos - sp.start_pos
                          for sp in self.stepper_positions}
            trig_steps = {sp.stepper_name: sp.trig_pos - sp.start_pos
                          for sp in self.stepper_positions}
            haltpos = trigpos = self.calc_toolhead_pos(kin_spos, trig_steps)
            if trig_steps != halt_steps:
                haltpos = self.calc_toolhead_pos(kin_spos, halt_steps)
            self.toolhead.set_position(haltpos)
            for sp in self.stepper_positions:
                sp.verify_no_probe_skew(haltpos)
        else:
            # 仅针对z轴
            if aniso_pos != None:
                movepos[2] = aniso_pos[0] #复原触发位置
                kin.limits[2] = (aniso_pos[1], kin.limits[2][1]) #复原限位
            haltpos = trigpos = movepos
            over_steps = {sp.stepper_name: sp.halt_pos - sp.trig_pos
                          for sp in self.stepper_positions}
            if any(over_steps.values()):
                self.toolhead.set_position(movepos)
                halt_kin_spos = {s.get_name(): s.get_commanded_position()
                                 for s in kin.get_steppers()}
                haltpos = self.calc_toolhead_pos(halt_kin_spos, over_steps)
            self.toolhead.set_position(haltpos)
        # Signal homing/probing move complete
        try:
            self.printer.send_event("homing:homing_move_end", self)
        except self.printer.command_error as e:
            if error is None:
                error = str(e)
        if error is not None:
            raise self.printer.command_error(error)
        # 更新停止与触发的z轴位置偏差
        probe = self.printer.lookup_object('probe')
        probe.set_trigger_offset(haltpos[2] - trigpos[2])
        return trigpos
    def check_no_movement(self):
        if self.printer.get_start_args().get('debuginput') is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                return sp.endstop_name
        return None

# State tracking of homing requests
class Homing:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}
        self.gcode = printer.lookup_object("gcode")
    def set_axes(self, axes):
        self.changed_axes = axes
    def get_axes(self):
        return self.changed_axes
    def get_trigger_position(self, stepper_name):
        return self.trigger_mcu_pos[stepper_name]
    def set_stepper_adjustment(self, stepper_name, adjustment):
        self.adjust_pos[stepper_name] = adjustment
    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        return thcoord
    def set_homed_position(self, pos):
        self.gcode.respond_info("set_homed_position pos:%d" % pos)  
        self.toolhead.set_position(self._fill_coord(pos))
    @staticmethod
    def _is_z_homing_retryable_error(msg):
        if "after full movement" in msg:
            return ("No trigger on z" in msg
                    or "No trigger on probe" in msg)
        if "still triggered after retract" in msg:
            return ("Endstop z " in msg
                    or "Endstop probe " in msg)
        return False
    def _prepare_z_homing_retry(self, rails, hi, forcepos, attempt,
                                homing_travel):
        aniso = self.printer.lookup_object('aniso_bed_mesh')
        aniso.stop_sampling()
        kin = self.toolhead.get_kinematics()
        kin.limits[2] = rails[0].get_range()
        lift_z = max(hi.retract_dist * 2., 8.)
        curpos = self.toolhead.get_position()
        self.gcode.respond_info(
            "Z homing retry %d/%d: cur Z=%.3f lift %.1f mm"
            % (attempt, Z_HOMING_RETRIES, curpos[2], lift_z))
        curpos[2] += lift_z
        self.toolhead.move(curpos, hi.retract_speed)
        self.gcode.run_script_from_command("M400\nG4 P200")
        filled_force = self._fill_coord(forcepos)
        lifted_z = self.toolhead.get_position()[2]
        retry_force_z = max(filled_force[2], lifted_z + homing_travel)
        retry_forcepos = list(forcepos)
        retry_forcepos[2] = retry_force_z
        self.gcode.respond_info(
            "Z homing retry %d/%d: forcepos Z=%.3f lifted Z=%.3f"
            " travel=%.3f"
            % (attempt, Z_HOMING_RETRIES, retry_force_z, lifted_z,
               homing_travel))
        logging.info(
            "Z homing retry %d/%d: lifted Z=%.3f forcepos Z=%.3f"
            " travel=%.3f",
            attempt, Z_HOMING_RETRIES, lifted_z, retry_force_z, homing_travel)
        return retry_forcepos
    def _home_rails_once(self, rails, forcepos, movepos, force_axes,
                         attempt=0):
        homing_axes = "".join(["xyz"[i] for i in force_axes])
        startpos = self._fill_coord(forcepos)
        homepos = self._fill_coord(movepos)
        is_z_homing = 2 in force_axes
        if is_z_homing:
            self.gcode.respond_info(
                "Z homing attempt %d: forcepos Z=%.3f homepos Z=%.3f"
                % (attempt + 1, startpos[2], homepos[2]))
            logging.info(
                "Z homing attempt %d: forcepos Z=%.3f homepos Z=%.3f",
                attempt + 1, startpos[2], homepos[2])
        self.toolhead.set_position(startpos, homing_axes=homing_axes)
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        hmove = HomingMove(self.printer, endstops)
        hmove.homing_move(homepos, hi.speed)
        if hi.retract_dist:
            homing_cycles = 2 if is_z_homing else 1
            for cycle in range(homing_cycles):
                startpos = self._fill_coord(forcepos)
                homepos = self._fill_coord(movepos)
                axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
                move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
                retract_r = min(1., hi.retract_dist / move_d)
                retractpos = [hp - ad * retract_r
                            for hp, ad in zip(homepos, axes_d)]
                self.toolhead.move(retractpos, hi.retract_speed)
                if is_z_homing:
                    self.gcode.run_script_from_command("M400\nG4 P100")
                for stepper in ('stepper_y', 'stepper_x'):
                    if self.printer.lookup_object(
                            'tmc2240 %s' % stepper, None) is not None:
                        self.gcode.run_script_from_command(
                            "SET_TMC_FIELD STEPPER=%s FIELD=drv_err VALUE=1"
                            % stepper)
                startpos = [rp - ad * retract_r
                            for rp, ad in zip(retractpos, axes_d)]
                self.toolhead.set_position(startpos)
                anisopos = None
                if is_z_homing:
                    kin = self.toolhead.get_kinematics()
                    anisopos = (homepos[2], kin.limits[2][0])
                    homepos[2] = -kin.limits[2][1]
                    kin.limits[2] = (homepos[2] - 0.1, kin.limits[2][1])
                hmove = HomingMove(self.printer, endstops)
                hmove.homing_move(homepos, hi.second_homing_speed,
                                  aniso_pos=anisopos)
                if hmove.check_no_movement() is not None:
                    raise self.printer.command_error(
                        "Endstop %s still triggered after retract"
                        % (hmove.check_no_movement(),))
        return hmove
    def home_rails(self, rails, forcepos, movepos):
        # Notify of upcoming homing operation
        self.printer.send_event("homing:home_rails_begin", self, rails)
        force_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        is_z_homing = 2 in force_axes
        hi = rails[0].get_homing_info()
        hmove = None
        if is_z_homing:
            filled_force = self._fill_coord(forcepos)
            filled_home = self._fill_coord(movepos)
            homing_travel = abs(filled_force[2] - filled_home[2])
            max_attempts = Z_HOMING_RETRIES + 1
            self.gcode.respond_info(
                "Z homing start: travel=%.3f max_attempts=%d"
                % (homing_travel, max_attempts))
            logging.info(
                "Z homing start: travel=%.3f max_attempts=%d",
                homing_travel, max_attempts)
            cur_forcepos = forcepos
            for attempt in range(max_attempts):
                try:
                    hmove = self._home_rails_once(
                        rails, cur_forcepos, movepos, force_axes, attempt)
                    break
                except self.printer.command_error as e:
                    err = str(e)
                    if self.printer.is_shutdown():
                        raise
                    if (attempt >= max_attempts - 1
                            or not self._is_z_homing_retryable_error(err)):
                        raise
                    self.gcode.respond_info(
                        "Z homing failed (%s), will retry" % (err,))
                    cur_forcepos = self._prepare_z_homing_retry(
                        rails, hi, forcepos, attempt + 1, homing_travel)
        else:
            hmove = self._home_rails_once(
                rails, forcepos, movepos, force_axes)

        # Signal home operation complete
        self.toolhead.flush_step_generation()
        self.trigger_mcu_pos = {sp.stepper_name: sp.trig_pos
                                for sp in hmove.stepper_positions}
        self.adjust_pos = {}
        self.printer.send_event("homing:home_rails_end", self, rails)
        if any(self.adjust_pos.values()):
            # Apply any homing offsets
            kin = self.toolhead.get_kinematics()
            homepos = self.toolhead.get_position()
            kin_spos = {s.get_name(): (s.get_commanded_position()
                                       + self.adjust_pos.get(s.get_name(), 0.))
                        for s in kin.get_steppers()}
            newpos = kin.calc_position(kin_spos)
            for axis in force_axes:
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)

class PrinterHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Register g-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('G28', self.cmd_G28, desc=self.cmd_G28_help)
    def manual_home(self, toolhead, endstops, pos, speed,
                    triggered, check_triggered):
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            hmove.homing_move(pos, speed, triggered=triggered,
                              check_triggered=check_triggered)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise
    def probing_move(self, mcu_probe, pos, speed):
        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown")
            raise
        if hmove.check_no_movement() is not None:
            raise self.printer.command_error(
                "The homing sensor was triggered before the Z-axis moved.")
        return epos
    def _get_z_rail(self, kin):
        if hasattr(kin, 'get_rails'):
            return kin.get_rails()[2]
        return kin.rails[2]
    def _prepare_z_rehome_from_probe_fail(self, kin):
        aniso = self.printer.lookup_object('aniso_bed_mesh')
        aniso.stop_sampling()
        z_rail = self._get_z_rail(kin)
        kin.limits[2] = z_rail.get_range()
        hi = z_rail.get_homing_info()
        lift_z = max(hi.retract_dist * 2., 8.)
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        curpos = toolhead.get_position()
        gcode.respond_info(
            "Z re-home after probe check fail: cur Z=%.3f lift %.1f mm"
            % (curpos[2], lift_z))
        logging.info(
            "Z re-home after probe check fail: cur Z=%.3f lift %.1f mm",
            curpos[2], lift_z)
        curpos[2] += lift_z
        toolhead.move(curpos, hi.retract_speed)
        gcode.run_script_from_command("M400\nG4 P200")
    def _rehome_z_axis(self, kin):
        kin.clear_homing_state("z")
        z_homing_state = Homing(self.printer)
        z_homing_state.set_axes([2])
        kin.home(z_homing_state)
    def _z_homing_probe_check(self, gcode):
        probe = self.printer.lookup_object('probe')
        toolhead = self.printer.lookup_object('toolhead')
        params = probe.get_probe_params()
        lift_z = params['sample_retract_dist'] + 1.0
        gcode.run_script_from_command(
            "G1 Z%.3f F600\nM400\nG4 P200\n" % (lift_z,))
        print_time = toolhead.get_last_move_time()
        if probe.mcu_probe.query_endstop(print_time):
            gcode.respond_info(
                "Probe still triggered after Z homing, lifting further")
            lift_z = max(lift_z + 4.0, 8.0)
            gcode.run_script_from_command(
                "G1 Z%.3f F600\nM400\nG4 P200\n" % (lift_z,))
        probe_gcmd = gcode.create_gcode_command(
            "PROBE", "PROBE SAMPLES=4 SAMPLES_TOLERANCE_RETRIES=0",
            {'SAMPLES': '4', 'SAMPLES_TOLERANCE_RETRIES': '0'})
        probe_session = probe.start_probe_session(probe_gcmd)
        aniso_bed_mesh = self.printer.lookup_object('aniso_bed_mesh')
        try:
            probe_session.run_probe(probe_gcmd)
            results = probe_session.pull_probed_results()
        finally:
            aniso_bed_mesh.stop_sampling()
            probe_session.end_probe_session()
        if not results:
            raise self.printer.command_error(
                "Homing probe check failed: no probe result")
        probe_result = results[0][2]
        probe.cmd_helper.last_z_result = probe_result
        z_offset = probe.get_offsets()[2]
        probe_delta = probe_result - z_offset
        gcode.respond_info(
            "Z homing probe check: result=%.4f mm, z_offset=%.4f mm,"
            " delta=%.4f mm (limit ±%.2f mm)"
            % (probe_result, z_offset, probe_delta, HOMING_PROBE_DEVIATION_MAX))
        if round(abs(probe_delta), 2) > round(HOMING_PROBE_DEVIATION_MAX, 2):
            raise self.printer.command_error(
                "Z homing check failed: probe result %.2f mm, expected %.2f"
                " ±%.2f mm (delta %.2f mm)"
                % (probe_result, z_offset, HOMING_PROBE_DEVIATION_MAX,
                   probe_delta))
    cmd_G28_help = "Home axes. S1 skips Z homing probe check."
    def cmd_G28(self, gcmd):
        # Move to origin
        axes = []
        for pos, axis in enumerate('XYZ'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [0, 1, 2]
        skip_z_probe_check = gcmd.get_int('S', 0, minval=0, maxval=1)
        homing_state = Homing(self.printer)
        homing_state.set_axes(axes)
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        try:
            kin.home(homing_state)
            if 2 in axes and not skip_z_probe_check:
                gcode = self.printer.lookup_object('gcode')
                max_probe_attempts = Z_HOMING_RETRIES + 1
                for probe_attempt in range(max_probe_attempts):
                    try:
                        self._z_homing_probe_check(gcode)
                        break
                    except self.printer.command_error as e:
                        err = str(e)
                        if self.printer.is_shutdown():
                            raise
                        if (probe_attempt >= max_probe_attempts - 1
                                or not Homing._is_z_homing_retryable_error(
                                    err)):
                            raise
                        gcode.respond_info(
                            "Z homing probe check failed (%s),"
                            " will re-home Z" % (err,))
                        logging.info(
                            "Z homing probe check failed (%s),"
                            " re-home Z attempt %d/%d",
                            err, probe_attempt + 1, Z_HOMING_RETRIES)
                        self._prepare_z_rehome_from_probe_fail(kin)
                        self._rehome_z_axis(kin)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            aniso = self.printer.lookup_object('aniso_bed_mesh')
            aniso.stop_sampling()
            aniso.restore_servo_switch()
            self.printer.lookup_object('stepper_enable').motor_off()
            raise

def load_config(config):
    return PrinterHoming(config)
