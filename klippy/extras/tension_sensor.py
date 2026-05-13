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
        
        # 判堵：过滤异常幅值后，最近 trigger_sample_count 帧中，过半帧满足
        # 仅“往负方向”相对 force_baseline 偏离超过 jam_deviation_limit 计为超阈；正方向不算。
        # 基准仅由 TENSION_UPDATE_BASELINE（主簇 + EMA）更新。
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
            'trigger_sample_count', 6, minval=1, maxval=30
        )
        self.detection_buffer = collections.deque(maxlen=30)
        
        self.force_baseline = None
        
        # 判堵：以 force_baseline 为参考，仅当力值相对基准更负且偏离超过 jam_deviation_limit 视为该帧超阈。
        self.jam_deviation_limit = config.getint(
            'jam_deviation_limit', 45000, minval=1000, maxval=10000000
        )
        # 幅值饱和：|力值| 超过此值视为异常（正负对称）
        self.max_normal_value = 0x400000
        
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
            desc="TENSION_SET_CONSECUTIVE FRAMES=<n> (jam vote window, same as trigger_sample_count)",
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

    def _jam_vote_stats(self):
        """最近 N 帧中超阈帧数。超阈：仅往负方向 (force_baseline - v) > jam_deviation_limit。"""
        baseline = self.force_baseline
        if baseline is None:
            return None
        n = self.trigger_sample_count
        if len(self.detection_buffer) < n:
            return None
        limit = self.jam_deviation_limit
        tail = list(self.detection_buffer)[-n:]
        over = sum(1 for v in tail if (baseline - v) > limit)
        return tail, over, n, baseline, limit

    def detect_jamming(self):
        stats = self._jam_vote_stats()
        if stats is None:
            if self.force_baseline is None:
                return False, "Collecting(no baseline)"
            return False, "Collecting"
        tail, over, n, baseline, limit = stats
        # 过半：严格大于 n/2（例如 N=6 需至少 4 帧超阈）
        is_jamming = over > (n // 2)
        reason = (
            f"jam_vote over={over}/{n} (need>{n//2}) neg_dev_limit={limit:.0f} "
            f"baseline={baseline:.0f} tail_min={min(tail):.0f} tail_max={max(tail):.0f}"
        )
        return is_jamming, reason

    def _record_jamming_snapshot(self, reason):
        """堵料锁定时将当前缓冲写入 klipper.log。"""
        try:
            stats = self._jam_vote_stats()
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            logging.error(
                "tension_sensor jamming snapshot begin time=%s reason=%s",
                ts,
                reason,
            )
            logging.error(
                "tension_sensor jamming meta interval_s=%.3f jam_vote_N=%d "
                "jam_deviation_limit=%s baseline=%s",
                self.sample_interval,
                self.trigger_sample_count,
                self.jam_deviation_limit,
                self.force_baseline,
            )
            if stats is not None:
                tail, over, n, baseline, limit = stats
                logging.error(
                    "tension_sensor jamming jam_vote over=%d/%d limit=%s baseline=%s",
                    over,
                    n,
                    limit,
                    baseline,
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

        # 异常尖峰不参与 jam 判定窗口，避免污染投票窗口。
        if abs(current_value) > self.max_normal_value:
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
                f"{len(self.detection_buffer)}/{self.trigger_sample_count} for jam vote"
            )
        
        is_jamming, reason = self.detect_jamming()
        
        if is_jamming:
            self.jamming_triggered = True
            self.current_state = "JAMMED"
            logging.error(f"JAMMING DETECTED and LOCKED! {reason}")
            self._record_jamming_snapshot(reason)
            return True, "JAMMED", reason
        
        return False, "NORMAL", reason

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
        self.current_state = "NORMAL"
        self.detection_buffer.clear()
        self.value_history.clear()
        gcmd.respond_info(f"Alarm reset. Previous state: {'JAMMED' if was_jammed else 'NORMAL'}")
        gcmd.respond_info(
            "Detection buffer cleared; jam detection uses existing baseline and "
            f"{self.trigger_sample_count} new sample(s) for jam vote window."
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
        frames = gcmd.get_int('FRAMES', None, minval=1, maxval=30)
        if frames is None:
            gcmd.respond_info(
                f"Usage: TENSION_SET_CONSECUTIVE FRAMES=<1..30> "
                f"(jam vote window, current={self.trigger_sample_count})"
            )
            return
        self.trigger_sample_count = frames
        gcmd.respond_info(
            f"Updated trigger_sample_count (jam vote window)={self.trigger_sample_count}"
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
        gcmd.respond_info(
            f"trigger_sample_count (jam vote window N): {self.trigger_sample_count}"
        )
        gcmd.respond_info(f"jam_deviation_limit: {self.jam_deviation_limit}")
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
        
        stats = self._jam_vote_stats()
        if stats is not None:
            tail, over, n, baseline, limit = stats
            need = (n // 2) + 1
            gcmd.respond_info(
                f"jam_vote: over={over}/{n} (majority need>={need}) "
                f"limit={limit:.0f} baseline={baseline:.0f}"
            )
            gcmd.respond_info(
                f"Recent tail neg_dev max: "
                f"{max(max(0, baseline - v) for v in tail):.0f}"
            )
        else:
            gcmd.respond_info("jam_vote: collecting")
        
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
        self.jamming_triggered = False
        self.current_state = "NORMAL"
        gcmd.respond_info(
            "Detection buffers reset. Baseline unchanged. "
            f"Need {self.trigger_sample_count} sample(s) for jam vote."
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
        
        stats = self._jam_vote_stats()
        if stats is not None:
            tail, over, n, baseline, limit = stats
            need = (n // 2) + 1
            gcmd.respond_info(
                f"baseline={baseline:.0f} | jam_vote over={over}/{n} (need>={need}) "
                f"| neg_dev_max={max(max(0, baseline - v) for v in tail):.0f} "
                f"(limit={limit:.0f})"
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
