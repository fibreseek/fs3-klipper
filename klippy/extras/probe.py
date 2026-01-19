# Z-Probe support
#
# Copyright (C) 2017-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import pins
from . import manual_probe
import os
import sys
import subprocess
import tempfile
import threading
import re

HINT_TIMEOUT = """
If the probe did not move far enough to trigger, then
consider reducing the Z axis minimum position so the probe
can travel further (the Z minimum position can be negative).
"""

# Calculate the average Z from a set of positions
def calc_probe_z_average(positions, method='average'):
    if method != 'median':
        # Use mean average
        count = float(len(positions))
        return [sum([pos[i] for pos in positions]) / count
                for i in range(3)]
    # Use median
    z_sorted = sorted(positions, key=(lambda p: p[2]))
    middle = len(positions) // 2
    if (len(positions) & 1) == 1:
        # odd number of samples
        return list(z_sorted[middle])
    # even number of samples
    return calc_probe_z_average(z_sorted[middle-1:middle+1], 'average')
    
# 根据iqr计算，祛除异常离散点，返回清洗数据组合
def calc_probe_z_iqr(positions, coeff = 0.3):
    # 仅提取z轴数据，并对数据排序
    datas_z = sorted([p[2] for p in positions]) 
    n = len(datas_z)

    # 计算第1和第3四分位数
    q1_pos = (n - 1) * 0.25
    q1_int = int(q1_pos)
    q1_frac = q1_pos - int(q1_pos)
    q1 = datas_z[q1_int] + q1_frac * (datas_z[q1_int + 1] - datas_z[q1_int])

    q3_pos = (n - 1) * 0.75
    q3_int = int(q3_pos)
    q3_frac = q3_pos - int(q3_pos)
    q3 = datas_z[q3_int] + q3_frac * (datas_z[q3_int + 1] - datas_z[q3_int])
    
    # 计算异常值边界
    iqr = q3 - q1
    lower_bound = q1 - coeff * iqr
    upper_bound = q3 + coeff * iqr

    # 返回清洗后数据
    return [p for p in positions if lower_bound <= p[2] <= upper_bound]

# 根据极差和方差/标准差，祛除异常离散点，返回清洗数据组合
def calc_probe_z_variance(positions, threshold=0.05):
    datas = list(positions) # 转换为可修改的列表
    datas_z = [data[2] for data in datas]
    cnt = len(datas_z)
    
    # 循环处理直到极差小于阈值或数组长度小于等于1
    while cnt > 1:
        spread = max(datas_z) - min(datas_z) #当前极差
        if spread <= threshold: # 若极差小于阈值，停止处理
            break
        
        idx = 0 #使集中度最好的数据索引
        cr_min = float('inf') #最小集中度 concentration_rate
        total = sum(datas_z) #计算总和用于均值计算

        # 计算删除第i个元素后的统计信息
        for i in range(cnt):
            mean = (total - datas_z[i]) / (cnt - 1) #均值
            sum_squares = sum(z*z for z in datas_z if z != datas_z[i]) #平方和
            variance = sum_squares / (cnt - 1) - mean * mean #方差
            # std_dev = variance ** 0.5 #标准差
            if cr_min > variance:
                cr_min = variance
                idx = i
        
        # 更新为当前集中度最好的组合
        datas_z = datas_z[:idx] + datas_z[idx+1:]
        datas = datas[:idx] + datas[idx+1:]
        cnt -= 1
    
    return datas

# 根据极差和MAD，祛除异常离散点，返回清洗数据组合
def calc_probe_z_mad(positions, threshold=0.05):
    def calc_median(data):
        n = len(data)
        if n==0:
            return 0
        sorted_data = sorted(data)
        mid = n // 2
        if n % 2 == 0:
            return (sorted_data[mid - 1] + sorted_data[mid]) / 2
        else:
            return sorted_data[mid]
    
    # 转换为可修改的列表
    datas = list(positions)
    datas_z = [data[2] for data in datas]
    cnt = len(datas_z)
    
    # 循环处理直到极差小于阈值或数组长度小于等于1
    while cnt > 1:
        spread = max(datas_z) - min(datas_z) #当前极差
        if spread <= threshold: # 若极差小于阈值，停止处理
            break

        # 计算删除最大值后的MAD
        idx1 = datas_z.index(max(datas_z))
        datas_z1 = datas_z[:idx1] + datas_z[idx1+1:]
        median = calc_median(datas_z1)
        mad1 = calc_median([abs(x - median) for x in datas_z1])

        # 计算删除最小值后的MAD
        idx2 = datas_z.index(min(datas_z))
        datas_z2 = datas_z[:idx2] + datas_z[idx2+1:]
        median = calc_median(datas_z2)
        mad2 = calc_median([abs(x - median) for x in datas_z2])
        
        # 比较集中度，确定删除元素
        if mad1 < mad2: # 删除最大值
            datas_z = datas_z1
            datas = datas[:idx1] + datas[idx1+1:]
        elif mad1 > mad2: # 删除最小值
            datas_z = datas_z2
            datas = datas[:idx2] + datas[idx2+1:]
        else: # mad相等，删除离中位数较远的最值
            median = calc_median(datas_z)
            mad1 = abs(datas_z[idx1] - median) # 最大值与中位数的距离
            mad2 = abs(datas_z[idx2] - median) # 最小值与中位数的距离
            if mad1 > mad2:
                datas_z = datas_z1
                datas = datas[:idx1] + datas[idx1+1:]
            else:
                datas_z = datas_z2
                datas = datas[:idx2] + datas[idx2+1:]
        
        cnt -= 1 #每回合只删除一个元素
    
    return datas

######################################################################
# Probe device implementation helpers
######################################################################

# Helper to implement common probing commands
class ProbeCommandHelper:
    def __init__(self, config, probe, query_endstop=None):
        self.printer = config.get_printer()
        self.probe = probe
        self.query_endstop = query_endstop
        self.name = config.get_name()
        self.trigger_offset = 0.0
        gcode = self.printer.lookup_object('gcode')
        # QUERY_PROBE command
        self.last_state = False
        gcode.register_command('QUERY_PROBE', self.cmd_QUERY_PROBE,
                               desc=self.cmd_QUERY_PROBE_help)
        # PROBE command
        self.last_z_result = 0.
        gcode.register_command('PROBE', self.cmd_PROBE,
                               desc=self.cmd_PROBE_help)
        # PROBE_CALIBRATE command
        self.probe_calibrate_z = 0.
        gcode.register_command('PROBE_CALIBRATE', self.cmd_PROBE_CALIBRATE,
                               desc=self.cmd_PROBE_CALIBRATE_help)
        # Other commands
        gcode.register_command('PROBE_ACCURACY', self.cmd_PROBE_ACCURACY,
                               desc=self.cmd_PROBE_ACCURACY_help)
        gcode.register_command('Z_OFFSET_APPLY_PROBE',
                               self.cmd_Z_OFFSET_APPLY_PROBE,
                               desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)

    def _move(self, coord, speed):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.manual_move(coord, speed)
        # Wait for the retract move to complete before sending commands
        toolhead.wait_moves()

    def set_trigger_offset(self, offset):
        self.trigger_offset = offset

    def get_status(self, eventtime):
        return {'name': self.name,
                'last_query': self.last_state,
                'last_z_result': self.last_z_result,
                'trigger_offset': self.trigger_offset}
    cmd_QUERY_PROBE_help = "Return the status of the z-probe"
    def cmd_QUERY_PROBE(self, gcmd):
        if self.query_endstop is None:
            raise gcmd.error("Probe does not support QUERY_PROBE")
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info("probe: %s" % (["open", "TRIGGERED"][not not res],))
    cmd_PROBE_help = "Probe Z-height at current XY position"
    def cmd_PROBE(self, gcmd):
        pos = run_single_probe(self.probe, gcmd)
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))
        self.last_z_result = pos[2]
    def probe_calibrate_finalize(self, kin_pos):
        if kin_pos is None:
            return
        z_offset = self.probe_calibrate_z - kin_pos[2]
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info(
            "%s: z_offset: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (self.name, z_offset))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'z_offset', "%.3f" % (z_offset,))
    cmd_PROBE_CALIBRATE_help = "Calibrate the probe's z_offset"
    def cmd_PROBE_CALIBRATE(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        params = self.probe.get_probe_params(gcmd)
        # Perform initial probe
        curpos = run_single_probe(self.probe, gcmd)
        # Move away from the bed
        self.probe_calibrate_z = curpos[2]
        curpos[2] += 5.
        self._move(curpos, params['lift_speed'])
        # Move the nozzle over the probe point
        x_offset, y_offset, z_offset = self.probe.get_offsets()
        curpos[0] += x_offset
        curpos[1] += y_offset
        self._move(curpos, params['probe_speed'])
        # Start manual probe
        manual_probe.ManualProbeHelper(self.printer, gcmd,
                                       self.probe_calibrate_finalize)
    cmd_PROBE_ACCURACY_help = "Probe Z-height accuracy at current XY position"
    def cmd_PROBE_ACCURACY(self, gcmd):
        params = self.probe.get_probe_params(gcmd)
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        gcmd.respond_info("PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
                          " (samples=%d retract=%.3f"
                          " speed=%.1f lift_speed=%.1f)\n"
                          % (pos[0], pos[1], pos[2],
                             sample_count, params['sample_retract_dist'],
                             params['probe_speed'], params['lift_speed']))
        # Create dummy gcmd with SAMPLES=1
        fo_params = dict(gcmd.get_command_parameters())
        fo_params['SAMPLES'] = '1'
        gcode = self.printer.lookup_object('gcode')
        fo_gcmd = gcode.create_gcode_command("", "", fo_params)
        # Probe bed sample_count times
        probe_session = self.probe.start_probe_session(fo_gcmd)
        probe_num = 0
        while probe_num < sample_count:
            # Probe position
            probe_session.run_probe(fo_gcmd)
            probe_num += 1
            # # Retract
            # pos = toolhead.get_position()
            # liftpos = [None, None, pos[2] + params['sample_retract_dist']]
            # self._move(liftpos, params['lift_speed'])
        positions = probe_session.pull_probed_results()
        probe_session.end_probe_session()
        # Calculate maximum, minimum and average values
        max_value = max([p[2] for p in positions])
        min_value = min([p[2] for p in positions])
        range_value = max_value - min_value
        avg_value = calc_probe_z_average(positions, 'average')[2]
        median = calc_probe_z_average(positions, 'median')[2]
        # calculate the standard deviation
        deviation_sum = 0
        for i in range(len(positions)):
            deviation_sum += pow(positions[i][2] - avg_value, 2.)
        sigma = (deviation_sum / len(positions)) ** 0.5
        # Show information
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, "
            "average %.6f, median %.6f, standard deviation %.6f" % (
            max_value, min_value, range_value, avg_value, median, sigma))
        try:
            # Prepare temporary input file for analysis script
            # Create a timestamped filename like: probe_samples_20251022_154747.txt
            now = __import__('datetime').datetime.now()
            fname = f"probe_samples_{pos[0]:.1f}_{pos[1]:.1f}_{now.strftime('%Y%m%d_%H%M%S')}.txt"
            tmp_dir = tempfile.gettempdir()
            tmp_path = os.path.join(tmp_dir, fname)
            fd = os.open(tmp_path, os.O_CREAT | os.O_WRONLY, 0o600)
            os.close(fd)
            with open(tmp_path, 'w', encoding='utf-8') as tf:
                for p in positions:
                    # write lines that analyze_probe_data.py expects (contains z=...)
                    tf.write("time=%.6f z=%.6f\n" % (__import__('time').time(), p[2]))
            script = "/home/anisoprint/klipper/scripts/analyze_probe_data.py"
            if not os.path.exists(script):
                gcmd.respond_info(f"Probe analysis script not found: {script}")
            else:
                # Choose a python executable that can import matplotlib.
                candidates = [sys.executable, "python3", "/usr/bin/python3", "/usr/local/bin/python3"]
                chosen_py = None
                for py in candidates:
                    if not py:
                        continue
                    try:
                        test = subprocess.run([py, "-c", "import matplotlib"],
                                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    except FileNotFoundError:
                        continue
                    if test.returncode == 0:
                        chosen_py = py
                        break
                if chosen_py is None:
                    # fallback to system python3; will likely fail if matplotlib is missing
                    chosen_py = "python3"
                    gcmd.respond_info("Probe analysis: could not find python with matplotlib on PATH; using 'python3' (may fail)")

                # Run analysis in a background thread so we don't block the Klipper main loop.
                def _analysis_worker(py_exec, script_path, input_path, gcode_cmd):
                    try:
                        gcode_cmd.respond_info(f"Probe analysis started (background) with: {py_exec}")
                        proc = subprocess.run([py_exec, script_path, input_path],
                                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                text=True, check=False)
                        out = (proc.stdout or '').strip()
                        gcode_cmd.respond_info(f"Probe analysis finished (rc={proc.returncode})")
                        if out:
                            out_lines = [l for l in out.splitlines() if l.strip()]
                            # send up to 50 lines of combined output
                            for line in out_lines[:50]:
                                gcode_cmd.respond_info("ANALYZE: %s" % (line,))
                            # Try to parse any printed paths to generated files (common scripts print 'results/...')
                            pat = re.compile(r'(?P<p>/?(?:\S*/)?results/\S+\.(?:png|txt))')
                            for line in out_lines:
                                m = pat.search(line)
                                if m:
                                    gcode_cmd.respond_info("ANALYZE_FILE: %s" % (m.group('p'),))
                        else:
                            gcode_cmd.respond_info("Probe analysis produced no output")
                    except Exception as e:
                        try:
                            gcode_cmd.respond_info("ANALYZE_ERR: %s" % (str(e),))
                        except:
                            pass
                    finally:
                        # Worker is responsible for removing the temporary input file.
                        try:
                            os.remove(input_path)
                            try:
                                gcode_cmd.respond_info(f"ANALYZE: removed temp file {input_path}")
                            except:
                                pass
                        except Exception:
                            pass

                thr = threading.Thread(target=_analysis_worker, args=(chosen_py, script, tmp_path, gcmd), daemon=True)
                thr.start()
                gcmd.respond_info(f"Probe analysis launched in background with: {chosen_py}")
        except Exception as e:
            try:
                gcmd.respond_info("Probe analysis failed to start: %s" % (str(e),))
            except:
                pass
        
    cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"
    def cmd_Z_OFFSET_APPLY_PROBE(self, gcmd):
        gcode_move = self.printer.lookup_object("gcode_move")
        offset = gcode_move.get_status()['homing_origin'].z
        if offset == 0:
            gcmd.respond_info("Nothing to do: Z Offset is 0")
            return
        z_offset = self.probe.get_offsets()[2]
        new_calibrate = z_offset - offset
        gcmd.respond_info(
            "%s: z_offset: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer."
            % (self.name, new_calibrate))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'z_offset', "%.3f" % (new_calibrate,))

# Helper to lookup the minimum Z position for the printer
def lookup_minimum_z(config):
    if config.has_section('stepper_z'):
        zconfig = config.getsection('stepper_z')
        return zconfig.getfloat('position_min', 0., note_valid=False)
    pconfig = config.getsection('printer')
    return pconfig.getfloat('minimum_z_position', 0., note_valid=False)

# Helper to lookup all the Z axis steppers
class LookupZSteppers:
    def __init__(self, config, add_stepper_cb):
        self.printer = config.get_printer()
        self.add_stepper_cb = add_stepper_cb
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper_cb(stepper)
    
    def get_z_stepper_oid(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                return stepper.get_oid()
        return None

# Homing via probe:z_virtual_endstop
class HomingViaProbeHelper:
    def __init__(self, config, mcu_probe, param_helper):
        self.printer = config.get_printer()
        self.mcu_probe = mcu_probe
        self.param_helper = param_helper
        self.multi_probe_pending = False
        self.z_min_position = lookup_minimum_z(config)
        self.results = []
        LookupZSteppers(config, self.mcu_probe.add_stepper)
        # Register z_virtual_endstop pin
        self.printer.lookup_object('pins').register_chip('probe', self)
        # Register event handlers
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._handle_homing_move_end)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
    def _handle_homing_move_begin(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_prepare(hmove)
    def _handle_homing_move_end(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_finish(hmove)
    def _handle_home_rails_begin(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.mcu_probe.multi_probe_begin()
            self.multi_probe_pending = True
    def _handle_home_rails_end(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.multi_probe_pending and self.mcu_probe in endstops:
            self.multi_probe_pending = False
            self.mcu_probe.multi_probe_end()
    def _handle_command_error(self):
        if self.multi_probe_pending:
            self.multi_probe_pending = False
            try:
                self.mcu_probe.multi_probe_end()
            except:
                logging.exception("Homing multi-probe end")
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'z_virtual_endstop':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe
    # Helper to convert probe based commands to use homing module
    def start_probe_session(self, gcmd):
        self.mcu_probe.multi_probe_begin()
        self.results = []
        return self
    def run_probe(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        pos[2] = self.z_min_position
        speed = self.param_helper.get_probe_params(gcmd)['probe_speed']
        phoming = self.printer.lookup_object('homing')
        self.results.append(phoming.probing_move(self.mcu_probe, pos, speed))
    def pull_probed_results(self):
        res = self.results
        self.results = []
        return res
    def end_probe_session(self):
        self.results = []
        self.mcu_probe.multi_probe_end()

# Helper to read multi-sample parameters from config
class ProbeParameterHelper:
    def __init__(self, config):
        gcode = config.get_printer().lookup_object('gcode')
        self.dummy_gcode_cmd = gcode.create_gcode_command("", "", {})
        # Configurable probing speeds
        self.speed = config.getfloat('speed', 5.0, above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        # Multi-sample support (for improved accuracy)
        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.,
                                                   above=0.)
        atypes = ['median', 'average']
        self.samples_result = config.getchoice('samples_result', atypes,
                                               'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                 minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0,
                                             minval=0)
        # retry到最大次数依旧不通过时，使用iqr评估历史缓存，samples_coeff是iqr边界调整系数，取值[0,∞)，取值越小边界越窄
        self.samples_coeff = config.getfloat('samples_coeff', 0.3, minval=0.0)
    def get_probe_params(self, gcmd=None):
        if gcmd is None:
            gcmd = self.dummy_gcode_cmd
        probe_speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        samples = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        samples_coeff = gcmd.get_float("SAMPLES_COEFF", self.samples_coeff, minval=0.0)
        return {'probe_speed': probe_speed,
                'lift_speed': lift_speed,
                'samples': samples,
                'sample_retract_dist': sample_retract_dist,
                'samples_tolerance': samples_tolerance,
                'samples_tolerance_retries': samples_retries,
                'samples_result': samples_result,
                'samples_coeff': samples_coeff}

# Helper to track multiple probe attempts in a single command
class ProbeSessionHelper:
    def __init__(self, config, param_helper, start_session_cb):
        self.printer = config.get_printer()
        self.param_helper = param_helper
        self.start_session_cb = start_session_cb
        # Session state
        self.hw_probe_session = None
        self.results = []
        # Register event handlers
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
    def _handle_command_error(self):
        if self.hw_probe_session is not None:
            try:
                self.end_probe_session()
            except:
                logging.exception("Multi-probe end")
    def _probe_state_error(self):
        raise self.printer.command_error(
            "Internal probe error - start/end probe session mismatch")
    def start_probe_session(self, gcmd):
        if self.hw_probe_session is not None:
            self._probe_state_error()
        self.hw_probe_session = self.start_session_cb(gcmd)
        self.results = []
        return self
    def end_probe_session(self):
        hw_probe_session = self.hw_probe_session
        if hw_probe_session is None:
            self._probe_state_error()
        self.results = []
        self.hw_probe_session = None
        hw_probe_session.end_probe_session()
    def _probe(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        try:
            self.hw_probe_session.run_probe(gcmd)
            epos = self.hw_probe_session.pull_probed_results()[0]
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
            raise self.printer.command_error(reason)
        # Allow axis_twist_compensation to update results
        self.printer.send_event("probe:update_results", epos)
        return epos[:3]
    def run_probe(self, gcmd):
        if self.hw_probe_session is None:
            self._probe_state_error()
        params = self.param_helper.get_probe_params(gcmd)
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        probexy = toolhead.get_position()[:2]
        cnt = 0
        retries = 0
        positions = []
        sample_count = params['samples']
        sample_cache = [] #采样结果历史缓存初始化
        
        while cnt < sample_count:
            # Probe position
            pos = self._probe(gcmd)
            positions.append(pos)
            cnt += 1
            toolhead.manual_move(
                # probexy + [pos[2] + params['sample_retract_dist']],
                probexy + [params['sample_retract_dist']],
                params['lift_speed'])
            gcode.respond_info("%d: probe at %.3f,%.3f is z=%.6f" 
                               % (cnt, pos[0], pos[1], pos[2]))

            # Wait for the retract move to complete before sending commands
            toolhead.wait_moves()

            # # After move completes, switch aniso_bed_mesh to ADC sampling + trigger mode
            # aniso_bed_mesh = self.printer.lookup_object('aniso_bed_mesh')
            # aniso_bed_mesh.update_uart_cmd.send([aniso_bed_mesh.oid, 1, 0])  # 切换为adc采样+触发判断模式
            # Check samples tolerance
            if cnt == sample_count:
                result = calc_probe_z_mad(positions, params['samples_tolerance'])
                if len(result)*2 >= sample_count: #筛选结果不得少于样本半数
                    positions = result #更新为筛选后的坐标组合
                    break

                #筛选结果少于半数，波动大不稳定，需要重试
                sample_cache.extend(positions) #缓存未通过的采样结果
                if retries >= params['samples_tolerance_retries']: #达到最大尝试次数
                    gcode.respond_info("Probe samples exceed samples_tolerance") #一般通知
                    positions = calc_probe_z_iqr(sample_cache, params['samples_coeff'])
                    if len(positions)*2 < len(sample_cache): #数据波动大，拉起异常报错
                        raise self.printer.command_error("bad samples precision")
                    break
                
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                cnt = 0
                retries += 1
                positions = []
        # Calculate result
        epos = calc_probe_z_average(positions, params['samples_result'])
        # 参与最终特征值计算的z_posistions，以及特征值z_eigen_value
        gcode.respond_info(f"zpos=[{', '.join(f'{x[2]:.6f}' for x in positions)}], zev={epos[2]:.6f}")
        self.results.append(epos)
    def pull_probed_results(self):
        res = self.results
        self.results = []
        return res

# Helper to read the xyz probe offsets from the config
class ProbeOffsetsHelper:
    def __init__(self, config):
        self.x_offset = config.getfloat('x_offset', 0.)
        self.y_offset = config.getfloat('y_offset', 0.)
        self.z_offset = config.getfloat('z_offset')
    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset


######################################################################
# Tools for utilizing the probe
######################################################################

# Helper code that can probe a series of points and report the
# position at each point.
class ProbePointsHelper:
    def __init__(self, config, finalize_callback, default_points=None):
        self.printer = config.get_printer()
        self.finalize_callback = finalize_callback
        self.probe_points = default_points
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        # Read config settings
        if default_points is None or config.get('points', None) is not None:
            self.probe_points = config.getlists('points', seps=(',', '\n'),
                                                parser=float, count=2)
        def_move_z = config.getfloat('horizontal_move_z', 5.)
        self.default_horizontal_move_z = def_move_z
        self.speed = config.getfloat('speed', 50., above=0.)
        self.use_offsets = False
        # Internal probing state
        self.lift_speed = self.speed
        self.probe_offsets = (0., 0., 0.)
        self.manual_results = []
    def minimum_points(self,n):
        if len(self.probe_points) < n:
            raise self.printer.config_error(
                "Need at least %d probe points for %s" % (n, self.name))
    def update_probe_points(self, points, min_points):
        self.probe_points = points
        self.minimum_points(min_points)
    def use_xy_offsets(self, use_offsets):
        self.use_offsets = use_offsets
    def get_lift_speed(self):
        return self.lift_speed
    def _move(self, coord, speed):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.manual_move(coord, speed)
        # Wait for the retract move to complete before sending commands
        toolhead.wait_moves()
    def _raise_tool(self, is_first=False):
        speed = self.lift_speed
        # if is_first:
        #     # Use full speed to first probe position
        #     speed = self.speed
        self._move([None, None, self.horizontal_move_z], speed)
    def _invoke_callback(self, results):
        # Flush lookahead queue
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.get_last_move_time()
        # Invoke callback
        res = self.finalize_callback(self.probe_offsets, results)
        return res != "retry"
    def _move_next(self, probe_num):
        # Move to next XY probe point
        nextpos = list(self.probe_points[probe_num])
        if self.use_offsets:
            nextpos[0] -= self.probe_offsets[0]
            nextpos[1] -= self.probe_offsets[1]
        self._move(nextpos, self.speed)
    def start_probe(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        # Lookup objects
        probe = self.printer.lookup_object('probe', None)
        method = gcmd.get('METHOD', 'automatic').lower()
        def_move_z = self.default_horizontal_move_z
        self.horizontal_move_z = gcmd.get_float('HORIZONTAL_MOVE_Z',
                                                def_move_z)
        if probe is None or method == 'manual':
            # Manual probe
            self.lift_speed = self.speed
            self.probe_offsets = (0., 0., 0.)
            self.manual_results = []
            self._manual_probe_start()
            return
        # Perform automatic probing
        self.lift_speed = probe.get_probe_params(gcmd)['lift_speed']
        self.probe_offsets = probe.get_offsets()
        if self.horizontal_move_z < self.probe_offsets[2]:
            raise gcmd.error("horizontal_move_z can't be less than"
                             " probe's z_offset")
        probe_session = probe.start_probe_session(gcmd)
        probe_num = 0
        while 1:
            self._raise_tool(not probe_num)
            if probe_num >= len(self.probe_points):
                results = probe_session.pull_probed_results()
                done = self._invoke_callback(results)
                if done:
                    break
                # Caller wants a "retry" - restart probing
                probe_num = 0
            self._move_next(probe_num)
            probe_session.run_probe(gcmd)
            probe_num += 1
        probe_session.end_probe_session()
    def _manual_probe_start(self):
        self._raise_tool(not self.manual_results)
        if len(self.manual_results) >= len(self.probe_points):
            done = self._invoke_callback(self.manual_results)
            if done:
                return
            # Caller wants a "retry" - clear results and restart probing
            self.manual_results = []
        self._move_next(len(self.manual_results))
        gcmd = self.gcode.create_gcode_command("", "", {})
        manual_probe.ManualProbeHelper(self.printer, gcmd,
                                       self._manual_probe_finalize)
    def _manual_probe_finalize(self, kin_pos):
        if kin_pos is None:
            return
        self.manual_results.append(kin_pos)
        self._manual_probe_start()

# Helper to obtain a single probe measurement
def run_single_probe(probe, gcmd):
    probe_session = probe.start_probe_session(gcmd)
    probe_session.run_probe(gcmd)
    pos = probe_session.pull_probed_results()[0]
    probe_session.end_probe_session()
    return pos


######################################################################
# Handle [probe] config
######################################################################

# Endstop wrapper that enables probe specific features
class ProbeEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.position_endstop = config.getfloat('z_offset')
        self.stow_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
        # Create an "endstop" object to handle the probe pin
        ppins = self.printer.lookup_object('pins')
        self.mcu_endstop = ppins.setup_pin('endstop', config.get('pin'))
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # multi probes state
        self.multi = 'OFF'
    def _raise_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")
    def _lower_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")
    def multi_probe_begin(self):
        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'
    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self._raise_probe()
        self.multi = 'OFF'
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self._lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self._raise_probe()
    def get_position_endstop(self):
        return self.position_endstop

# Main external probe interface
class PrinterProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mcu_probe = ProbeEndstopWrapper(config)
        self.cmd_helper = ProbeCommandHelper(config, self,
                                             self.mcu_probe.query_endstop)
        self.probe_offsets = ProbeOffsetsHelper(config)
        self.param_helper = ProbeParameterHelper(config)
        self.homing_helper = HomingViaProbeHelper(config, self.mcu_probe,
                                                  self.param_helper)
        self.probe_session = ProbeSessionHelper(
            config, self.param_helper, self.homing_helper.start_probe_session)
    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)
    def get_offsets(self):
        return self.probe_offsets.get_offsets()
    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)
    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)
    def set_trigger_offset(self, offset):
        self.cmd_helper.set_trigger_offset(offset)

def load_config(config):
    return PrinterProbe(config)
