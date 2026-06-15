# Virtual SDCard print stat tracking
#
# Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import os

logger = logging.getLogger(__name__)

# 有效状态集合，与状态机设计保持一致
VALID_STATES = (
    "standby", "precheck", "preprinting", "printing", "pausing", "paused",
    "complete", "cancelled", "load_unload", "calibration", "maintenance",
    "busy", "error"
)

# 内部 note_*、SET_PRINT_STATS_INFO STATE=、SET_PRINT_*_STATE 共用。
# 与状态机流转图一致；paused <-> load_unload 表示暂停时可进退料，结束后可回到 paused。
# printing -> paused 与内部 note_pause 一致；precheck -> printing 对应 note_start 跳过 preprinting 的边界情况。
# paused -> preprinting 用于断电恢复 PREPARE_RECOVER_PRINT_FILE；paused -> standby 用于清除断电记录等。
# precheck/preprinting -> standby/cancelled 供正式打印前中止作业。
STATE_TRANSITIONS = {
    "standby": frozenset({
        "precheck", "paused", "load_unload", "calibration", "maintenance",
        "busy",
    }),
    "precheck": frozenset({
        "preprinting", "printing", "standby", "cancelled", "error",
    }),
    "preprinting": frozenset({
        "printing", "paused", "standby", "cancelled", "error",
    }),
    "printing": frozenset({
        "pausing", "paused", "complete", "cancelled", "error",
    }),
    "pausing": frozenset({"paused", "cancelled", "error"}),
    "paused": frozenset({
        "printing", "preprinting", "standby", "cancelled", "load_unload", "error",
    }),
    "complete": frozenset({"standby"}),
    "cancelled": frozenset({"standby"}),
    "error": frozenset({"standby"}),
    "load_unload": frozenset({"standby", "paused", "error"}),
    "calibration": frozenset({"standby", "error"}),
    "maintenance": frozenset({"standby", "error"}),
    "busy": frozenset({"standby", "error"}),
}

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
        self._busy_timer = None
        self._busy_check_interval = config.getfloat("busy_check_interval", 0.5,
                                                    minval=0.05, maxval=1.0)
        self.reset()
        # Register commands
        self.gcode = printer.lookup_object('gcode')
        # 打印统计相关指令
        self.gcode.register_command(
            "SET_PRINT_STATS_INFO", self.cmd_SET_PRINT_STATS_INFO,
            desc=self.cmd_SET_PRINT_STATS_INFO_help)
        # 预检查阶段状态设置指令（供 Moonraker 调用）
        self.gcode.register_command(
            "SET_PRINT_PRECHECK_STATE", self.cmd_SET_PRINT_PRECHECK_STATE,
            desc=self.cmd_SET_PRINT_PRECHECK_STATE_help)
        # 预备动作阶段状态（加热、归位、清理、调平等），供上位机订阅 print_stats 获取
        self.gcode.register_command(
            "SET_PRINT_PREPRINTING_STATE", self.cmd_SET_PRINT_PREPRINTING_STATE,
            desc=self.cmd_SET_PRINT_PREPRINTING_STATE_help)
        self.gcode.register_command(
            "SET_PRINT_PREP_STAGE", self.cmd_SET_PRINT_PREP_STAGE,
            desc=self.cmd_SET_PRINT_PREP_STAGE_help)
        # 辅助状态设置指令（供 Moonraker/宏 调用）
        for state_name in ("load_unload", "calibration", "maintenance", "busy"):
            self.gcode.register_command(
                "SET_PRINT_" + state_name.upper() + "_STATE",
                self._make_set_state_cmd(state_name),
                desc="Set print_stats state to '%s'" % state_name)
        # Delay toolhead lookup until it's actually needed
        self._toolhead = None
        # Register event handler for extruder activation
        printer.register_event_handler("extruder:activate_extruder",
                                      self._handle_activate_extruder)
        printer.register_event_handler("gcode:move_extruder",
                self._handle_move_extruder)
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("gcode:command_error",
                                       self._handle_fault_standby_aux)
        printer.register_event_handler("klippy:shutdown",
                                       self._handle_fault_standby_aux)

    def _handle_fault_standby_aux(self, *args):
        """G-code 命令错误或 Klippy shutdown 时，任意状态统一回到 standby。"""
        try:
            # Fault path should not be blocked by normal transition rules.
            self.state = "standby"
            self.error_message = ""
            self.prep_stage = None
            self._start_busy_timer()
        except Exception:
            logger.exception("print_stats: fault standby transition failed")

    def _handle_ready(self):
        """注册定时器，用于 standby 下检测 G-code 执行并自动切换 busy"""
        self._start_busy_timer()

    def _start_busy_timer(self):
        """启动 busy 轮询定时器（仅在 standby/busy 时需要）"""
        if self._busy_timer is not None:
            return
        self._busy_timer = self.reactor.register_timer(
            self._check_gcode_busy, self.reactor.NOW)

    def _stop_busy_timer(self):
        """停止 busy 轮询定时器，减少 CPU 占用"""
        if self._busy_timer is None:
            return
        self.reactor.unregister_timer(self._busy_timer)
        self._busy_timer = None

    def _check_gcode_busy(self, eventtime):
        """仅在 standby/busy 间自动切换，不覆盖 load_unload/calibration/maintenance"""
        if self.state not in ("standby", "busy"):
            self._stop_busy_timer()
            return self.reactor.NEVER
        gcode_mutex = self.gcode.get_mutex()
        if gcode_mutex.test():
            self.state = "busy"
        elif self.state == "busy":
            self.state = "standby"
        return eventtime + self._busy_check_interval

    def _make_set_state_cmd(self, state_name):
        def cmd(gcmd):
            self._set_state(state_name, gcmd)
        return cmd

    def _require_allowed_state_transition(self, new_state, gcmd=None):
        old = self.state
        if old == new_state:
            return
        allowed = STATE_TRANSITIONS.get(old)
        if allowed is None or new_state not in allowed:
            msg = "Invalid print_stats state transition %s -> %s" % (old, new_state)
            if gcmd is not None:
                raise gcmd.error(msg)
            logger.error(f"[_require_allowed_state_transition] {msg}")
            raise self.printer.command_error(msg)

    def _set_state(self, state, gcmd):
        """设置辅助状态（load_unload/calibration/maintenance/busy）"""
        self._require_allowed_state_transition(state, gcmd)
        self.state = state
        self.error_message = ""
        self.prep_stage = None
        if state == "busy":
            self._start_busy_timer()
        else:
            self._stop_busy_timer()

    def _handle_move_extruder(self, move_extruder, move_pos, absolute_mode):
        self.current_extruder = move_extruder
        if self.current_extruder == 'extruder':
            if absolute_mode:
                self.v_filament_used = move_pos
            else:
                self.v_filament_used += move_pos
        elif self.current_extruder == 'extruder1':
            if absolute_mode:
                self.e_filament_used = move_pos
            else:
                self.e_filament_used += move_pos
        elif self.current_extruder == 'extruder2':
            if absolute_mode:
                self.u_filament_used = move_pos
            else:
                self.u_filament_used += move_pos
        self.filament_used = self.e_filament_used + self.u_filament_used + self.v_filament_used

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
            self.last_v_pos = cur_epos
        elif self.current_extruder == 'extruder1':
            self.last_e_pos = cur_epos
        elif self.current_extruder == 'extruder2':
            self.last_u_pos = cur_epos
    
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
            v_delta = (cur_epos - self.last_v_pos) / extrude_factor
            self.v_filament_used += v_delta
            self.last_v_pos = cur_epos
        elif self.current_extruder == 'extruder1':
            e_delta = (cur_epos - self.last_e_pos) / extrude_factor
            self.e_filament_used += e_delta
            self.last_e_pos = cur_epos
        elif self.current_extruder == 'extruder2':
            u_delta = (cur_epos - self.last_u_pos) / extrude_factor
            self.u_filament_used += u_delta
            self.last_u_pos = cur_epos
        
    def set_current_file(self, filename):
        # 若当前为 preprinting（宏预备后调用 BASE），只更新 filename，保留 state/prep_stage
        if self.state == "preprinting":
            self.filename = filename
            return
        self.reset()
        self.filename = filename

    def note_precheck(self):
        # 进入打印前预检查阶段
        self._require_allowed_state_transition("precheck")
        self.state = "precheck"
        self.error_message = ""
        self._stop_busy_timer()

    def note_preprinting(self):
        # 进入预备动作阶段（加热、归位、清理、调平等）
        self._require_allowed_state_transition("preprinting")
        self.state = "preprinting"
        self.error_message = ""
    
    def note_pausing(self):
        self._require_allowed_state_transition("pausing")
        self.state = "pausing"
        self.error_message = ""
    
    def note_standby(self):
        self._require_allowed_state_transition("standby")
        self.state = "standby"
        self.error_message = ""

    def note_powerloss_recovery_paused(self, filename=None, gcmd=None):
        """检测到断电续打文件时进入 paused；不经过暂停宏，不修改 last_pause_time。"""
        if filename:
            self.filename = os.path.basename(filename)
        self._require_allowed_state_transition("paused", gcmd)
        self.state = "paused"
        self.error_message = ""

    def note_set_prep_stage(self, stage):
        """仅在 precheck/preprinting 下更新 prep_stage，避免误从 printing/paused 等切换"""
        if self.state not in ("preprinting", "precheck"):
            return
        self.prep_stage = stage

    cmd_SET_PRINT_PRECHECK_STATE_help = (
        "Set print_stats state to 'precheck' before starting the actual print"
    )
    def cmd_SET_PRINT_PRECHECK_STATE(self, gcmd):
        # 该指令通常由 Moonraker 在预检查流程开始时调用
        self.note_precheck()

    cmd_SET_PRINT_PREPRINTING_STATE_help = (
        "Set print_stats state to 'preprinting' (physical prep: heating, homing, etc.)"
    )
    def cmd_SET_PRINT_PREPRINTING_STATE(self, gcmd):
        self.note_preprinting()

    cmd_SET_PRINT_PREP_STAGE_help = (
        "Set prep_stage for preprinting. STAGE=heating_bed|heating_nozzle|homing|cleaning|leveling|ready"
    )
    def cmd_SET_PRINT_PREP_STAGE(self, gcmd):
        stage = gcmd.get("STAGE", "ready")
        self.note_set_prep_stage(stage)

    def restore_print_times(self, saved_total_duration, saved_print_duration):
        """Restore print timing after powerloss recovery.
        Must be called after note_start() has set print_start_time.
        Adjusts print_start_time backward so that total_duration continues
        from the saved value, and sets init_duration so that print_duration
        is correct."""
        if self.print_start_time is None:
            return
        curtime = self.reactor.monotonic()
        self.print_start_time = curtime - saved_total_duration
        self.init_duration = saved_total_duration - saved_print_duration
        self.prev_pause_duration = 0.

    def note_start(self):
        self._require_allowed_state_transition("printing")
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
        # 实际开始执行 G-code 时进入 printing（包括从 precheck/preprinting 过渡）
        self.state = "printing"
        self.prep_stage = None
        self.error_message = ""
    def note_pause(self):
        if self.last_pause_time is None:
            curtime = self.reactor.monotonic()
            self.last_pause_time = curtime
            # update filament usage
            # self._update_filament_usage(curtime)
        if self.state != "error":
            self._require_allowed_state_transition("paused")
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
        self._require_allowed_state_transition(state)
        self.state = state
        self.error_message = error_message
        eventtime = self.reactor.monotonic()
        self.total_duration = eventtime - self.print_start_time
        if self.filament_used < 0.0000001:
            # No positive extusion detected during print
            self.init_duration = self.total_duration - \
                self.prev_pause_duration
        self.print_start_time = None
    cmd_SET_PRINT_STATS_INFO_help = (
        "Pass slicer info like layer and total to klipper. "
        "Optional STATE= to set print state (must follow configured state machine "
        "transitions, e.g. paused may enter load_unload for filament change). "
        "Valid: standby, precheck, preprinting, printing, pausing, paused, complete, "
        "cancelled, load_unload, calibration, maintenance, busy, error."
    )
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

        # 可选的 STATE 参数，仅接受有效状态且须满足状态机允许变迁
        state = gcmd.get("STATE", None)
        if state is not None:
            if state not in VALID_STATES:
                logger.info(f"[_SET_PRINT_STATS_INFO] invalid state {state}")
                raise gcmd.error("Invalid STATE='%s', must be one of: %s" % (
                    state, ", ".join(VALID_STATES)))
            logger.info(f"[_SET_PRINT_STATS_INFO] set state to {state}")
            self._require_allowed_state_transition(state, gcmd)
            self.state = state
            self.error_message = ""
            self.prep_stage = None
            # 与 note_pause 一致：仅设 paused 而未执行 PAUSE 宏时，必须记录 last_pause_time，
            # 否则 get_status 中 time_paused 无法抵消 total_duration，print_duration 会持续增加。
            if state == "paused" and self.last_pause_time is None \
                    and self.print_start_time is not None:
                self.last_pause_time = self.reactor.monotonic()
            if state in ("standby", "busy"):
                self._start_busy_timer()
            else:
                self._stop_busy_timer()
            
    def reset(self):
        self.filename = self.error_message = ""
        self.state = "standby"
        self._start_busy_timer()
        self.prep_stage = None
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
                     'current_layer': self.info_current_layer,
                     'prep_stage': self.prep_stage}
        }

def load_config(config):
    return PrintStats(config)
