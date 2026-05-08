import logging
import time
import collections

class MySerial:
    def __init__(self, config):
        self.spi_device = "/dev/spidev0.0"
        
        # Voltage conversion parameters
        self.VREF = 3.3
        self.GAIN = 128
        self.ADC_MAX = 2**23 - 1  # 8388607 (24-bit signed)
        
        self.sample_interval = 0.2  # seconds (200ms)
        self.auto_baseline_discard_frames = 10
        
        # 判堵：默认用当前单帧与 force_baseline 比较；trigger_sample_count>1 时用最近 N 点中位数。
        # 连续 jam_consecutive_frames 帧超阈锁定。基准仅由 TENSION_UPDATE_BASELINE（主簇 + EMA）更新。
        self.baseline_sample_count = 10
        self.baseline_cluster_eps = config.getint(
            'baseline_cluster_eps', 1200, minval=50, maxval=200000
        )
        self.baseline_cluster_min_points = config.getint(
            'baseline_cluster_min_points', 4, minval=2, maxval=50
        )
        self.baseline_ema_alpha = config.getfloat(
            'baseline_ema_alpha', 0.2, minval=0.01, maxval=1.0
        )
        self.trigger_sample_count = config.getint(
            'trigger_sample_count', 1, minval=1, maxval=10
        )
        self.detection_buffer = collections.deque(maxlen=30)
        
        self.force_baseline = None
        
        # 判堵：力值为有符号 ADC。以 force_baseline 为参考，单向偏离超过 jam_deviation_limit 视为堵料。
        self.jam_deviation_limit = config.getint(
            'jam_deviation_limit', 45000, minval=1000, maxval=10000000
        )
        # 幅值饱和：|力值| 超过此值视为异常（正负对称）
        self.max_normal_value = 0x400000
        
        self.jam_consecutive_frames = config.getint(
            'jam_consecutive_frames', 3, minval=1, maxval=10
        )
        self.consecutive_jam_frames = 0
        # 趋势门控：当超阈后，最近 jam_consecutive_frames 帧需整体继续向更负方向推进。
        # 允许轻微抖动（每步最多 increase_tolerance），但不允许持续回升。
        self.jam_trend_step_tolerance = config.getint(
            'jam_trend_step_tolerance', 2000, minval=0, maxval=1000000
        )
        self.jam_trend_total_tolerance = config.getint(
            'jam_trend_total_tolerance', 3000, minval=0, maxval=1000000
        )
        
        self.current_state = "NORMAL"
        self.jamming_triggered = False
        
        self.value_history = collections.deque(maxlen=500)
        
        self.calibration_log = []

        self.polling_enabled = False
        self.polling_log_enabled = False
        self.jam_detection_enabled = True
        self._poll_sample_counter = 0
        self._auto_baseline_requested = False
        self._auto_baseline_pending_frames = 0
        self._skip_jam_detection = False
        self._baseline_update_running = False
        self._baseline_phase = None
        self._baseline_warmup = []
        self._baseline_samples = []
        
        printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        self.reactor = printer.get_reactor()
        self._polling_timer = self.reactor.register_timer(self._polling_event)
        self._baseline_timer = self.reactor.register_timer(
            self._baseline_update_event
        )
        
        self._register_commands()

    def _register_commands(self):
        self.gcode.register_command('TENSION_SENSOR_POLL', self.cmd_FORCE_SENSOR_POLL,
                                    desc="TENSION_SENSOR_POLL ENABLE=<0|1> [LOGGING=<0|1>] [DETECT=<0|1>]")
        self.gcode.register_command('TENSION_SENSOR_STATUS', self.cmd_FORCE_SENSOR_STATUS)
        self.gcode.register_command('SHOW_CALIBRATION', self.cmd_SHOW_CALIBRATION)
        self.gcode.register_command('GET_FORCE_VALUE', self.cmd_GET_FORCE_VALUE)
        self.gcode.register_command('GET_RAW_FORCE_DATA', self.cmd_GET_RAW_FORCE_DATA)
        self.gcode.register_command('TENSION_SET_THRESHOLD', self.cmd_SET_THRESHOLD,
                                    desc="TENSION_SET_THRESHOLD MIN_DEV=<n>")
        self.gcode.register_command(
            'TENSION_SET_CONSECUTIVE',
            self.cmd_SET_CONSECUTIVE,
            desc="TENSION_SET_CONSECUTIVE FRAMES=<n>",
        )
        self.gcode.register_command(
            'TENSION_SHOW_CONFIG',
            self.cmd_SHOW_CONFIG,
            desc="TENSION_SHOW_CONFIG",
        )
        self.gcode.register_command(
            'TENSION_UPDATE_BASELINE',
            self.cmd_TENSION_UPDATE_BASELINE,
            desc="TENSION_UPDATE_BASELINE: 10 samples, trim outliers/min/max, set baseline (no jam during capture)",
        )
        self.gcode.register_command('TENSION_RESET_LEARNING', self.cmd_RESET_DETECTION_BUFFERS)
        self.gcode.register_command('TENSION_DEBUG', self.cmd_TENSION_DEBUG)
        self.gcode.register_command('TENSION_RESET_ALARM', self.cmd_RESET_ALARM,)
        self.gcode.register_command('TENSION_SHOW_BASELINE', self.cmd_SHOW_BASELINE)
        self.gcode.register_command('TENSION_SET_REFERENCE', self.cmd_REFERENCE_DEPRECATED)
        self.gcode.register_command('TENSION_SHOW_REFERENCE', self.cmd_REFERENCE_DEPRECATED)
        self.gcode.register_command('TENSION_SHOW_LEARNING_HISTORY', self.cmd_REFERENCE_DEPRECATED)

    def adc_to_voltage(self, adc_value):
        return (adc_value / self.ADC_MAX) * (self.VREF / self.GAIN)

    def parse_spi_data(self, data_bytes):
        if len(data_bytes) != 3:
            raise ValueError("Data length error")
        
        value = (data_bytes[0] << 16) | (data_bytes[1] << 8) | data_bytes[2]
        if value > 0x400000:
            return value - 0x800000
        else:
            return value

    def read_spi_once(self):
        try:
            with open(self.spi_device, 'rb') as f:
                data = f.read(3)
                return self.parse_spi_data(data) if len(data) == 3 else None
        except Exception as e:
            logging.error(f"SPI read failed: {e}")
            return None

    def read_raw_spi_bytes(self):
        try:
            with open(self.spi_device, 'rb') as f:
                return f.read(3)
        except Exception as e:
            logging.error(f"SPI read failed: {e}")
            return None

    def _find_primary_cluster(self, values, eps):
        """Find densest 1D cluster where max-min <= eps."""
        ordered = sorted(values)
        best_l, best_r = 0, -1
        l = 0
        for r in range(len(ordered)):
            while ordered[r] - ordered[l] > eps:
                l += 1
            if (r - l) > (best_r - best_l):
                best_l, best_r = l, r
        return ordered[best_l : best_r + 1]

    def _compute_baseline_from_samples(self, samples):
        """主簇 + EMA：异常占比过高则失败，主簇中位值经 EMA 更新基准。"""
        if len(samples) != self.baseline_sample_count:
            return None, f"expected {self.baseline_sample_count} samples, got {len(samples)}", None
        if any(v is None for v in samples):
            return None, "sample read failed", None
        abnormal = [v for v in samples if abs(v) > self.max_normal_value]
        abnormal_ratio = len(abnormal) / float(len(samples))
        if abnormal_ratio > 0.7:
            return None, (
                f"abnormal ratio too high: {len(abnormal)}/{len(samples)} "
                f"({abnormal_ratio*100:.0f}%)"
            ), None
        valid = [v for v in samples if abs(v) <= self.max_normal_value]
        if not valid:
            return None, "no valid samples after abnormal filter", None

        primary = self._find_primary_cluster(valid, self.baseline_cluster_eps)
        if len(primary) < self.baseline_cluster_min_points:
            return None, (
                f"primary cluster too small: {len(primary)} < {self.baseline_cluster_min_points} "
                f"(eps={self.baseline_cluster_eps})"
            ), None

        center = int(round(sorted(primary)[len(primary) // 2]))
        old_baseline = self.force_baseline
        if old_baseline is None:
            baseline = center
        else:
            baseline = int(round(
                self.baseline_ema_alpha * center
                + (1.0 - self.baseline_ema_alpha) * old_baseline
            ))
        details = {
            "cluster_size": len(primary),
            "cluster_center": center,
            "old_baseline": old_baseline,
            "valid_count": len(valid),
            "abnormal_count": len(abnormal),
            "cluster_values": primary,
        }
        return baseline, None, details

    def get_jam_compare_value(self):
        """堵料判据用采样：N=1 为当前帧；N>1 为最近 N 点中位数。"""
        n = self.trigger_sample_count
        if len(self.detection_buffer) < n:
            return None
        tail = list(self.detection_buffer)[-n:]
        if n == 1:
            return tail[0]
        tail.sort()
        return tail[len(tail) // 2]

    def detect_jamming(self, current_value):
        baseline = self.force_baseline
        if baseline is None:
            return False, "Collecting(no baseline)"
        
        sample = self.get_jam_compare_value()
        if sample is None:
            return False, "Collecting"
        
        limit = self.jam_deviation_limit
        delta_fast = sample - baseline
        # 统一判向：仅认“往负方向”偏移为堵料（sample < baseline）。
        fast_deviation = max(0.0, -delta_fast)
        fast_jamming = fast_deviation > limit
        is_jamming = fast_jamming

        reason = (
            f"fast_dev={fast_deviation:.0f}/{limit:.0f} delta_fast={delta_fast:+.0f}, "
            f"val={sample:.0f}, baseline={baseline:.0f}"
        )
        
        return is_jamming, reason

    def _recent_values_keep_getting_more_negative(self):
        """最近 N 帧是否满足“整体向更负方向推进”。
        返回 (ok, detail)。
        """
        n = self.jam_consecutive_frames
        if n <= 1:
            return True, f"trend bypass n={n}"
        if len(self.detection_buffer) < n:
            return False, f"trend collecting {len(self.detection_buffer)}/{n}"
        recent = list(self.detection_buffer)[-n:]
        step_tol = self.jam_trend_step_tolerance
        total_tol = self.jam_trend_total_tolerance

        # 单步不得明显回升：v[i] 不应比上一帧大超过 step_tol（大=更不负）
        for i in range(1, len(recent)):
            if recent[i] > recent[i - 1] + step_tol:
                return False, (
                    f"trend fail step i={i} prev={recent[i-1]} cur={recent[i]} "
                    f"step_tol={step_tol}"
                )

        # 总体应更负：最后一帧不应高于首帧 + total_tol
        if recent[-1] > recent[0] + total_tol:
            return False, (
                f"trend fail total first={recent[0]} last={recent[-1]} "
                f"total_tol={total_tol}"
            )

        return True, (
            f"trend ok first={recent[0]} last={recent[-1]} "
            f"step_tol={step_tol} total_tol={total_tol}"
        )

    def _record_jamming_snapshot(self, reason):
        """堵料锁定时将当前缓冲写入 klipper.log。"""
        try:
            trig = self.get_jam_compare_value()
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            logging.error(
                "tension_sensor jamming snapshot begin time=%s reason=%s",
                ts,
                reason,
            )
            logging.error(
                "tension_sensor jamming meta interval_s=%.3f consecutive_frames=%d "
                "jam_deviation_limit=%s baseline=%s",
                self.sample_interval,
                self.jam_consecutive_frames,
                self.jam_deviation_limit,
                self.force_baseline,
            )
            logging.error(
                "tension_sensor jamming jam_sample=%s (trigger_sample_count=%s)",
                trig,
                self.trigger_sample_count,
            )
            buf = list(self.detection_buffer)
            logging.error(
                "tension_sensor jamming detection_buffer len=%d (idx 0=oldest)",
                len(buf),
            )
            for i, v in enumerate(buf):
                logging.error("tension_sensor jamming detection_buffer[%d]=%s", i, v)
            logging.error("tension_sensor jamming snapshot end")
        except Exception as e:
            logging.error("tension_sensor failed to record jamming snapshot: %s", e)

    def update_detection_state(self, current_value):
        self.value_history.append(current_value)
        
        if self.jamming_triggered:
            return False, "JAMMED", "Permanently jammed"

        # 异常尖峰不参与 jam 判定窗口，避免污染比较值/连续帧逻辑。
        if abs(current_value) > self.max_normal_value:
            self.consecutive_jam_frames = 0
            return False, "NORMAL", (
                f"abnormal sample ignored: val={current_value} "
                f"limit={self.max_normal_value}"
            )

        self.detection_buffer.append(current_value)
        
        if self._skip_jam_detection:
            return False, "NORMAL", "baseline capture (jam check skipped)"
        
        if self.force_baseline is None:
            return False, "COLLECTING", "no baseline (run TENSION_UPDATE_BASELINE)"
        
        if len(self.detection_buffer) < self.trigger_sample_count:
            return False, "COLLECTING", (
                f"{len(self.detection_buffer)}/{self.trigger_sample_count} for jam compare"
            )
        
        is_jamming, reason = self.detect_jamming(current_value)
        
        if is_jamming:
            self.consecutive_jam_frames += 1
            
            if self.consecutive_jam_frames >= self.jam_consecutive_frames:
                trend_ok, trend_reason = self._recent_values_keep_getting_more_negative()
                if not trend_ok:
                    # 超阈但趋势不满足，撤销本次累计，避免“先降后升”被锁死。
                    self.consecutive_jam_frames = 0
                    self.current_state = "NORMAL"
                    return False, "NORMAL", (
                        f"threshold met but trend rejected: {trend_reason}"
                    )
                self.jamming_triggered = True
                self.current_state = "JAMMED"
                logging.error(f"JAMMING DETECTED and LOCKED! {reason}")
                self._record_jamming_snapshot(reason)
                return True, "JAMMED", (
                    f"Jamming after {self.consecutive_jam_frames} consecutive jam frames"
                )
            
            return False, "CHECKING", (
                f"jamming {self.consecutive_jam_frames}/{self.jam_consecutive_frames}"
            )
        else:
            self.consecutive_jam_frames = 0
            return False, "NORMAL", f"ok"

    def _send_jamming_warning(self, details):
        try:
            self.gcode.run_script_from_command(f"M118 FIBER JAMMING DETECTED and LOCKED: {details}")
            self.gcode.run_script_from_command("FIBER_FILAMENT_TRIGGER")
            logging.error(f"JAMMING WARNING SENT: {details}")
        except Exception as e:
            logging.error(f"Failed to send warning: {e}")

    def _notify_info(self, msg):
        """Send async-safe user-facing info."""
        logging.info("tension_sensor: %s", msg)
        try:
            self.gcode.run_script_from_command(f"M118 {msg}")
        except Exception as e:
            logging.error("Failed to notify message '%s': %s", msg, e)

    def cmd_RESET_ALARM(self, gcmd):
        was_jammed = self.jamming_triggered
        self.jamming_triggered = False
        self.consecutive_jam_frames = 0
        self.current_state = "NORMAL"
        self.detection_buffer.clear()
        self.value_history.clear()
        gcmd.respond_info(f"Alarm reset. Previous state: {'JAMMED' if was_jammed else 'NORMAL'}")
        gcmd.respond_info(
            "Detection buffer cleared; jam detection uses existing baseline and "
            f"{self.trigger_sample_count} new sample(s) for jam compare."
        )

    def cmd_GET_RAW_FORCE_DATA(self, gcmd):
        raw = self.read_raw_spi_bytes()
        if raw is None:
            gcmd.respond_info("ERROR: SPI read failed")
            return
        
        hex_str = ' '.join(f'{b:02X}' for b in raw)
        try:
            val = self.parse_spi_data(raw)
            volt = self.adc_to_voltage(val)
            gcmd.respond_info(f"Raw: {hex_str} | Value: {val} | Voltage: {volt:.6f}V")
        except Exception as e:
            gcmd.respond_info(f"Raw: {hex_str} | Parse error: {e}")

    def cmd_GET_FORCE_VALUE(self, gcmd):
        val = self.read_spi_once()
        if val is None:
            gcmd.respond_info("Read failed")
            return
        
        volt = self.adc_to_voltage(val)
        gcmd.respond_info(f"Tension Sensor Value: {val} | Voltage: {volt:.6f}V")

    def cmd_SHOW_CALIBRATION(self, gcmd):
        gcmd.respond_info(f"Calibration entries: {len(self.calibration_log)}")

    def cmd_SET_THRESHOLD(self, gcmd):
        changed = []
        
        min_dev = gcmd.get_int('MIN_DEV', None)
        if min_dev is not None and min_dev >= 1000:
            self.jam_deviation_limit = min_dev
            changed.append(f"MIN_DEV={min_dev}")
        
        gcmd.respond_info(f"Updated: {', '.join(changed) if changed else 'None'}")

    def cmd_SET_CONSECUTIVE(self, gcmd):
        frames = gcmd.get_int('FRAMES', None, minval=1, maxval=100)
        if frames is None:
            gcmd.respond_info(
                f"Usage: TENSION_SET_CONSECUTIVE FRAMES=<1..100> "
                f"(current={self.jam_consecutive_frames})"
            )
            return
        self.jam_consecutive_frames = frames
        self.consecutive_jam_frames = 0
        gcmd.respond_info(
            f"Updated jam_consecutive_frames={self.jam_consecutive_frames}"
        )

    def cmd_SHOW_CONFIG(self, gcmd):
        gcmd.respond_info("=== TENSION SENSOR CONFIG ===")
        gcmd.respond_info(f"sample_interval: {self.sample_interval:.3f}s")
        gcmd.respond_info(f"baseline_sample_count: {self.baseline_sample_count}")
        gcmd.respond_info(f"baseline_cluster_eps: {self.baseline_cluster_eps}")
        gcmd.respond_info(
            f"baseline_cluster_min_points: {self.baseline_cluster_min_points}"
        )
        gcmd.respond_info(f"baseline_ema_alpha: {self.baseline_ema_alpha:.3f}")
        gcmd.respond_info(f"trigger_sample_count: {self.trigger_sample_count}")
        gcmd.respond_info(f"jam_deviation_limit: {self.jam_deviation_limit}")
        gcmd.respond_info(f"jam_consecutive_frames: {self.jam_consecutive_frames}")
        gcmd.respond_info(
            f"jam_trend_step_tolerance: {self.jam_trend_step_tolerance}"
        )
        gcmd.respond_info(
            f"jam_trend_total_tolerance: {self.jam_trend_total_tolerance}"
        )
        gcmd.respond_info(f"max_normal_value: {self.max_normal_value}")
        gcmd.respond_info(f"force_baseline: {self.force_baseline}")

    def _finish_baseline_update(self):
        self._skip_jam_detection = False
        self._baseline_update_running = False
        self._baseline_phase = None
        self.reactor.update_timer(self._baseline_timer, self.reactor.NEVER)

    def _baseline_update_event(self, eventtime):
        if not self._baseline_update_running:
            return self.reactor.NEVER
        try:
            val = self.read_spi_once()
            if val is None:
                if self._baseline_phase == "warmup":
                    step = len(self._baseline_warmup) + 1
                    self._notify_info(
                        f"TENSION_UPDATE_BASELINE ERROR: warmup read failed at "
                        f"{step}/{self.baseline_sample_count}"
                    )
                else:
                    step = len(self._baseline_samples) + 1
                    self._notify_info(
                        f"TENSION_UPDATE_BASELINE ERROR: read failed at "
                        f"{step}/{self.baseline_sample_count}"
                    )
                self._finish_baseline_update()
                return self.reactor.NEVER

            if self._baseline_phase == "warmup":
                self._baseline_warmup.append(val)
                if len(self._baseline_warmup) >= self.baseline_sample_count:
                    self._baseline_phase = "sample"
                return eventtime + self.sample_interval

            self._baseline_samples.append(val)
            if len(self._baseline_samples) < self.baseline_sample_count:
                return eventtime + self.sample_interval

            warmup = list(self._baseline_warmup)
            samples = list(self._baseline_samples)
            baseline, err, details = self._compute_baseline_from_samples(samples)
            if baseline is None:
                self._notify_info(f"TENSION_UPDATE_BASELINE ERROR: {err}")
                self._notify_info(f"warmup_samples={warmup}")
                self._notify_info(f"raw_samples={samples}")
                self._finish_baseline_update()
                return self.reactor.NEVER

            self.force_baseline = baseline
            self.consecutive_jam_frames = 0
            self.detection_buffer.clear()
            if details is None:
                self._notify_info(f"Baseline updated to {baseline}")
            else:
                self._notify_info(
                    f"Baseline updated to {baseline} (center={details['cluster_center']}, "
                    f"cluster_size={details['cluster_size']}, old={details['old_baseline']}, "
                    f"ema_alpha={self.baseline_ema_alpha:.3f})"
                )
            self._notify_info(f"warmup_samples={warmup}")
            self._notify_info(f"raw_samples={samples}")
            if details is not None:
                self._notify_info(
                    f"cluster_values={details['cluster_values']} "
                    f"valid={details['valid_count']} abnormal={details['abnormal_count']}"
                )
        except Exception as e:
            logging.exception("TENSION_UPDATE_BASELINE event error: %s", e)
            self._notify_info(f"TENSION_UPDATE_BASELINE ERROR: {e}")
        self._finish_baseline_update()
        return self.reactor.NEVER

    def _start_baseline_update(self):
        if self._baseline_update_running:
            return False
        self._baseline_update_running = True
        self._skip_jam_detection = True
        self._baseline_phase = "warmup"
        self._baseline_warmup = []
        self._baseline_samples = []
        self.reactor.update_timer(self._baseline_timer, self.reactor.NOW)
        return True

    def cmd_TENSION_UPDATE_BASELINE(self, gcmd):
        if not self._start_baseline_update():
            gcmd.respond_info(
                "TENSION_UPDATE_BASELINE ignored: previous update still running"
            )
            return
        gcmd.respond_info(
            "TENSION_UPDATE_BASELINE started in background timer; "
            "result will be reported via M118"
        )

    def cmd_TENSION_DEBUG(self, gcmd):
        gcmd.respond_info("=== TENSION SENSOR DEBUG ===")
        gcmd.respond_info(f"State: {self.current_state}")
        gcmd.respond_info(f"Jamming Triggered: {self.jamming_triggered}")
        gcmd.respond_info(f"Jam deviation limit: {self.jam_deviation_limit}")
        gcmd.respond_info(f"force_baseline: {self.force_baseline}")
        
        if self.force_baseline is not None and len(self.detection_buffer) >= self.trigger_sample_count:
            trig = self.get_jam_compare_value()
            if trig is not None:
                bl = self.force_baseline
                delta = trig - bl
                fast_dev = max(0.0, -delta)
                gcmd.respond_info(
                    f"jam_compare (N={self.trigger_sample_count})={trig:.0f} "
                    f"fast_dev={fast_dev:.0f} (limit={self.jam_deviation_limit})"
                )
        
        gcmd.respond_info(
            f"Consecutive jam frames: {self.consecutive_jam_frames}/{self.jam_consecutive_frames}"
        )
        
        if len(self.detection_buffer) > 0:
            recent = list(self.detection_buffer)[-min(10, len(self.detection_buffer)) :]
            gcmd.respond_info(f"Recent values: min={min(recent)}, max={max(recent)}, avg={sum(recent)/len(recent):.0f}")

    def cmd_SHOW_BASELINE(self, gcmd):
        if self.force_baseline is None:
            gcmd.respond_info("force_baseline: not set (run TENSION_UPDATE_BASELINE)")
        else:
            gcmd.respond_info(f"force_baseline: {self.force_baseline}")

    def cmd_REFERENCE_DEPRECATED(self, gcmd):
        gcmd.respond_info(
            "Use TENSION_UPDATE_BASELINE to set force_baseline; "
            "TENSION_SHOW_BASELINE to query."
        )

    def cmd_RESET_DETECTION_BUFFERS(self, gcmd):
        self.detection_buffer.clear()
        self.consecutive_jam_frames = 0
        self.jamming_triggered = False
        self.current_state = "NORMAL"
        gcmd.respond_info(
            "Detection buffers reset. Baseline unchanged. "
            f"Need {self.trigger_sample_count} sample(s) for jam compare."
        )

    def cmd_FORCE_SENSOR_STATUS(self, gcmd):
        val = self.read_spi_once()
        if val is None:
            gcmd.respond_info("Read failed")
            return
        
        volt = self.adc_to_voltage(val)
        gcmd.respond_info(f"Current: {val} ({volt:.6f}V)")
        gcmd.respond_info(f"State: {self.current_state} | Jamming Locked: {self.jamming_triggered}")
        gcmd.respond_info(f"Jam detection enabled: {self.jam_detection_enabled}")
        gcmd.respond_info(f"Jam deviation limit: {self.jam_deviation_limit}")
        gcmd.respond_info(f"force_baseline: {self.force_baseline}")
        
        if self.force_baseline is not None and len(self.detection_buffer) >= self.trigger_sample_count:
            trig = self.get_jam_compare_value()
            if trig is not None:
                bl = self.force_baseline
                delta = trig - bl
                fast_dev = max(0.0, -delta)
                gcmd.respond_info(
                    f"baseline={bl:.0f} | jam_compare(N={self.trigger_sample_count})={trig:.0f} "
                    f"| fast_dev={fast_dev:.0f}"
                )

    def cmd_FORCE_SENSOR_POLL(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        logging_enable = gcmd.get_int('LOGGING', 0)
        detect_enable = gcmd.get_int('DETECT', 1)
        
        if enable is None:
            gcmd.respond_info("Usage: TENSION_SENSOR_POLL ENABLE=<0|1> [LOGGING=<0|1>] [DETECT=<0|1>]")
            return
        
        if enable:
            self.jam_detection_enabled = (detect_enable == 1)
            self.polling_log_enabled = (logging_enable == 1)
            
            self.start_polling()
            self._auto_baseline_requested = True
            self._auto_baseline_pending_frames = self.auto_baseline_discard_frames
            mode = "DETECTION" if self.jam_detection_enabled else "LOG_ONLY"
            gcmd.respond_info(
                f"Polling ON ({self.sample_interval*1000:.0f}ms) "
                f"klipper.log={'ON' if self.polling_log_enabled else 'OFF'} "
                f"mode={mode}"
            )
            gcmd.respond_info(
                f"Auto baseline update scheduled after {self.auto_baseline_discard_frames} frames "
                "(startup dirty-data discard)."
            )
        else:
            self.stop_polling()
            gcmd.respond_info("Polling OFF")

    def start_polling(self):
        if self.polling_enabled:
            return
        
        self.detection_buffer.clear()
        self.value_history.clear()
        self.consecutive_jam_frames = 0
        self.jamming_triggered = False
        self.current_state = "NORMAL"
        
        self.polling_enabled = True
        self._poll_sample_counter = 0
        self.reactor.update_timer(self._polling_timer, self.reactor.NOW)
        
        logging.info(
            f"Polling started: {self.sample_interval*1000:.0f}ms; "
            "set baseline with TENSION_UPDATE_BASELINE before jam detection applies"
        )

    def stop_polling(self):
        if not self.polling_enabled:
            return
        
        self.polling_enabled = False
        self._auto_baseline_requested = False
        self._auto_baseline_pending_frames = 0
        self.reactor.update_timer(self._polling_timer, self.reactor.NEVER)
        
        logging.info("Polling stopped")

    def _polling_event(self, eventtime):
        if not self.polling_enabled:
            return self.reactor.NEVER
        # Avoid concurrent SPI reads with baseline capture timer.
        # During baseline update, SPI is sampled exclusively by _baseline_update_event.
        if self._baseline_update_running:
            return eventtime + self.sample_interval
        try:
            val = self.read_spi_once()
            if val is None:
                return eventtime + self.sample_interval

            if self.polling_log_enabled:
                logging.info(
                    "TENSION_SAMPLE value=%s voltage=%.6f detect=%s state=%s",
                    val,
                    self.adc_to_voltage(val),
                    1 if self.jam_detection_enabled else 0,
                    self.current_state,
                )

            if self.jam_detection_enabled:
                changed, state, details = self.update_detection_state(val)
                if changed and state == "JAMMED":
                    self.reactor.register_callback(
                        lambda e: self._send_jamming_warning(details)
                    )
            else:
                self.value_history.append(val)
                changed, state, details = False, "LOG_ONLY", "jam detection disabled"

            self._poll_sample_counter += 1
            # if self._poll_sample_counter % 60 == 0:
            #     logging.info(f"Status: {state} | {details}")
            if self._auto_baseline_requested:
                self._auto_baseline_pending_frames -= 1
                if self._auto_baseline_pending_frames <= 0:
                    self._auto_baseline_requested = False
                    if self._start_baseline_update():
                        self._notify_info(
                            f"Auto baseline update started after "
                            f"{self.auto_baseline_discard_frames} startup frames"
                        )
                    else:
                        logging.info(
                            "Auto baseline update skipped: baseline update already running"
                        )
        except Exception as e:
            logging.error(f"Polling error: {e}")
        return eventtime + self.sample_interval

def load_config(config):
    return MySerial(config)
