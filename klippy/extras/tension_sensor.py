import logging
import re
import time
import collections

try:
    import spidev
except ImportError:
    spidev = None

class MySerial:
    def __init__(self, config):
        self.spi_device = config.get('spi_device', '/dev/spidev0.0')
        self._spi_bus, self._spi_dev = self._parse_spi_device(self.spi_device)
        self.spi_mode = config.getint('spi_mode', 0, minval=0, maxval=3)
        self.spi_speed_hz = config.getint('spi_speed_hz', 1000000, minval=10000)
        
        # Voltage conversion parameters
        self.VREF = 3.3
        self.GAIN = 128
        self.ADC_MAX = 2**23 - 1  # 8388607 (24-bit signed)
        
        self.sample_interval = 0.3  # seconds (200ms)
        self.auto_baseline_discard_frames = 10
        
        # 判堵/缠料：过滤异常幅值后，最近 trigger_sample_count 连续帧均超阈，
        # 且相对 force_baseline 的偏离持续增大（相邻帧允许 trend_tolerance 回落）。
        # 判堵：负向偏离 (force_baseline - v) > jam_deviation_limit 且持续增大。
        # 缠料：正向偏离 (v - force_baseline) > tangling_deviation_limit 且持续增大。
        # 基准仅由 TENSION_UPDATE_BASELINE（主簇中位值）更新。
        self.baseline_sample_count = 10
        self.baseline_cluster_eps = config.getint(
            'baseline_cluster_eps', 1200, minval=50, maxval=200000
        )
        self.baseline_cluster_min_points = config.getint(
            'baseline_cluster_min_points', 6, minval=2, maxval=50
        )
        self.trigger_sample_count = config.getint(
            'trigger_sample_count', 4, minval=1, maxval=30
        )
        self.detection_buffer = collections.deque(maxlen=30)
        
        self.force_baseline = None
        
        # 判堵：以 force_baseline 为参考，仅当力值相对基准更负且偏离超过 jam_deviation_limit 视为该帧超阈。
        self.jam_deviation_limit = config.getint(
            'jam_deviation_limit', 45000, minval=1000, maxval=10000000
        )
        # 缠料：判堵镜像逻辑，仅“往正方向”相对 force_baseline 偏离超过 tangling_deviation_limit 计为超阈。
        self.tangling_deviation_limit = config.getint(
            'tangling_deviation_limit', 40000, minval=1000, maxval=10000000
        )
        # 持续趋势判定：相邻帧偏离允许的最大回落量
        self.trend_tolerance = config.getint(
            'trend_tolerance', 1500, minval=0, maxval=10000
        )
        # 幅值饱和：|力值| 超过此值视为异常（正负对称）
        self.max_normal_value = 0x400000
        
        self.current_state = "NORMAL"
        self.jamming_triggered = False
        self.tangling_triggered = False
        
        self.value_history = collections.deque(maxlen=500)
        
        self.calibration_log = []

        self.enabled = config.getboolean('enable', True)
        self._temp_paused = False
        self.poll_armed = False
        self.polling_enabled = False
        self.polling_log_enabled = False
        self.jam_detection_enabled = True
        self.tangling_detection_enabled = False
        self._poll_sample_counter = 0
        self._auto_baseline_requested = False
        self._auto_baseline_pending_frames = 0
        self._awaiting_first_auto_baseline = False
        self._skip_jam_detection = False
        self._baseline_update_running = False
        self._baseline_phase = None
        self._baseline_warmup = []
        self._baseline_samples = []
        # 检测启动后首帧 baseline 参考；后续 baseline 更新与之比较，连续 2 次超阈也可判堵。
        self._detection_baseline_active = False
        self._detection_start_baseline = None
        self._baseline_jam_streak = 0
        self._baseline_jam_trigger_count = 2
        
        printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        self.reactor = printer.get_reactor()
        self._spi = None
        self._polling_timer = self.reactor.register_timer(self._polling_event)
        self._baseline_timer = self.reactor.register_timer(
            self._baseline_update_event
        )
        printer.register_event_handler('klippy:disconnect', self._handle_disconnect)
        
        self._register_commands()

    @staticmethod
    def _parse_spi_device(path):
        match = re.match(r'.*/spidev(\d+)\.(\d+)$', path)
        if not match:
            logging.warning(
                "tension_sensor: cannot parse spi_device %s, use bus=0 dev=0",
                path,
            )
            return 0, 0
        return int(match.group(1)), int(match.group(2))

    def _register_commands(self):
        self.gcode.register_command(
            'TENSION_SENSOR_POLL', self.cmd_FORCE_SENSOR_POLL,
            desc=(
                "TENSION_SENSOR_POLL ENABLE=<0|1> [TANGLING_DETECT=<0|1>]: "
                "arm/disarm only (no sampling); tangling detect default off"
            ),
        )
        self.gcode.register_command(
            'TENSION_SENSOR_START', self.cmd_TENSION_SENSOR_START,
            desc="TENSION_SENSOR_START [LOGGING=<0|1>] [DETECT=<0|1>]: start polling/detection",
        )
        self.gcode.register_command(
            'TENSION_SENSOR_PAUSE',
            self.cmd_TENSION_SENSOR_PAUSE,
            desc='TENSION_SENSOR_PAUSE: temporarily stop polling (requires enable=true)',
        )
        self.gcode.register_command(
            'TENSION_SENSOR_RESUME',
            self.cmd_TENSION_SENSOR_RESUME,
            desc='TENSION_SENSOR_RESUME: resume after TENSION_SENSOR_PAUSE (requires enable=true)',
        )
        self.gcode.register_command('TENSION_SENSOR_STATUS', self.cmd_FORCE_SENSOR_STATUS)
        self.gcode.register_command('SHOW_CALIBRATION', self.cmd_SHOW_CALIBRATION)
        self.gcode.register_command('GET_FORCE_VALUE', self.cmd_GET_FORCE_VALUE)
        self.gcode.register_command('GET_RAW_FORCE_DATA', self.cmd_GET_RAW_FORCE_DATA)
        self.gcode.register_command(
            'TENSION_SET_THRESHOLD',
            self.cmd_SET_THRESHOLD,
            desc="TENSION_SET_THRESHOLD MIN_DEV=<n> [TANGLING_DEV=<n>] [TREND_TOL=<n>]",
        )
        self.gcode.register_command(
            'TENSION_SET_CONSECUTIVE',
            self.cmd_SET_CONSECUTIVE,
            desc="TENSION_SET_CONSECUTIVE FRAMES=<n> (consecutive trend frames, same as trigger_sample_count)",
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

    def _open_spi(self):
        if self._spi is not None:
            return True
        if spidev is None:
            return True
        try:
            spi = spidev.SpiDev()
            spi.open(self._spi_bus, self._spi_dev)
            spi.mode = self.spi_mode
            spi.max_speed_hz = self.spi_speed_hz
            self._spi = spi
            logging.info(
                "tension_sensor: SPI opened spidev%d.%d mode=%d speed=%dHz",
                self._spi_bus, self._spi_dev, self.spi_mode, self.spi_speed_hz,
            )
            return True
        except Exception as e:
            logging.error("tension_sensor: SPI open failed: %s", e)
            self._spi = None
            return False

    def _close_spi(self):
        if self._spi is None:
            return
        try:
            self._spi.close()
        except Exception as e:
            logging.error("tension_sensor: SPI close failed: %s", e)
        self._spi = None
        logging.info("tension_sensor: SPI closed")

    def _close_spi_if_idle(self):
        if not self.polling_enabled and not self._baseline_update_running:
            self._close_spi()

    def _handle_disconnect(self):
        self._close_spi()

    def _read_spi_bytes(self):
        """Read 3 raw bytes. spidev: one xfer per sample (CS cycle); else open/read/close."""
        if self._spi is not None:
            # Full-duplex clocking; each xfer2 is an independent SPI transaction.
            rx = self._spi.xfer2([0xFF, 0xFF, 0xFF])
            if len(rx) != 3:
                return None
            return bytes(rx)
        try:
            with open(self.spi_device, 'rb') as f:
                data = f.read(3)
            return data if len(data) == 3 else None
        except Exception as e:
            logging.error("tension_sensor: SPI read failed: %s", e)
            return None

    def read_spi_once(self):
        need_open = self._spi is None and spidev is not None
        idle_close = (
            not self.polling_enabled and not self._baseline_update_running
        )
        if need_open and not self._open_spi():
            return None
        try:
            data = self._read_spi_bytes()
            if data is None:
                return None
            return self.parse_spi_data(data)
        except Exception as e:
            logging.error("tension_sensor: SPI read failed: %s", e)
            self._close_spi()
            return None
        finally:
            if idle_close:
                self._close_spi()

    def read_raw_spi_bytes(self):
        need_open = self._spi is None and spidev is not None
        idle_close = (
            not self.polling_enabled and not self._baseline_update_running
        )
        if need_open and not self._open_spi():
            return None
        try:
            return self._read_spi_bytes()
        except Exception as e:
            logging.error("tension_sensor: SPI read failed: %s", e)
            self._close_spi()
            return None
        finally:
            if idle_close:
                self._close_spi()

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
        """主簇中位值：异常占比过高则失败，直接取主簇中位值为基准。"""
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
        baseline = center
        details = {
            "cluster_size": len(primary),
            "cluster_center": center,
            "old_baseline": old_baseline,
            "valid_count": len(valid),
            "abnormal_count": len(abnormal),
            "cluster_values": primary,
        }
        return baseline, None, details

    def _trend_detection_tail(self, direction):
        """取最近 N 帧及相对基准的偏离序列。direction: 'jam' | 'tangling'。"""
        baseline = self.force_baseline
        if baseline is None:
            return None
        n = self.trigger_sample_count
        if len(self.detection_buffer) < n:
            return None
        tail = list(self.detection_buffer)[-n:]
        if direction == 'jam':
            limit = self.jam_deviation_limit
            deviations = [baseline - v for v in tail]
        else:
            limit = self.tangling_deviation_limit
            deviations = [v - baseline for v in tail]
        return tail, deviations, n, baseline, limit

    def _reset_detection_baseline_tracking(self):
        self._detection_baseline_active = False
        self._detection_start_baseline = None
        self._baseline_jam_streak = 0

    def _reset_baseline_jam_state(self):
        """重置 baseline 漂移判堵计数与首帧参考，保留检测会话开关。"""
        self._detection_start_baseline = None
        self._baseline_jam_streak = 0

    def _evaluate_baseline_jam(self, new_baseline):
        """检测启动后：首帧 baseline 作参考，后续更新若相对参考负向偏离连续超阈则判堵。"""
        if not self._detection_baseline_active or not self.jam_detection_enabled:
            return False, None
        if self.jamming_triggered:
            return False, None

        ref = self._detection_start_baseline
        if ref is None:
            self._detection_start_baseline = new_baseline
            self._baseline_jam_streak = 0
            return False, (
                f"detection start baseline recorded: {new_baseline}"
            )

        deviation = ref - new_baseline
        limit = self.jam_deviation_limit
        if deviation > limit:
            self._baseline_jam_streak += 1
            reason = (
                f"baseline_jam streak={self._baseline_jam_streak}/"
                f"{self._baseline_jam_trigger_count} "
                f"ref={ref} new={new_baseline} dev={deviation:.0f} "
                f"limit={limit}"
            )
            if self._baseline_jam_streak >= self._baseline_jam_trigger_count:
                return True, reason
            return False, reason

        self._baseline_jam_streak = 0
        return False, (
            f"baseline within limit ref={ref} new={new_baseline} "
            f"dev={deviation:.0f} limit={limit}"
        )

    def _trigger_baseline_jam(self, reason):
        self.jamming_triggered = True
        self.current_state = "JAMMED"
        logging.error(f"JAMMING DETECTED (baseline drift) and LOCKED! {reason}")
        self._record_jamming_snapshot(reason)

    def _evaluate_sustained_trend(self, deviations, limit, label):
        """连续 N 帧均超阈，且偏离持续增大（相邻允许 trend_tolerance 回落）。"""
        n = len(deviations)
        tol = self.trend_tolerance
        over_count = sum(1 for d in deviations if d > limit)
        if over_count < n:
            return False, (
                f"{label}_trend not all over limit: {over_count}/{n} "
                f"dev_limit={limit:.0f} tol={tol}"
            )
        for i in range(n - 1):
            drop = deviations[i] - deviations[i + 1]
            if drop > tol:
                return False, (
                    f"{label}_trend break at {i}->{i + 1}: "
                    f"dev {deviations[i]:.0f}->{deviations[i + 1]:.0f} "
                    f"(drop={drop:.0f}>{tol})"
                )
        return True, (
            f"{label}_trend sustained {n} frames over limit={limit:.0f} "
            f"tol={tol} dev={','.join(f'{d:.0f}' for d in deviations)}"
        )

    def detect_jamming(self):
        stats = self._trend_detection_tail('jam')
        if stats is None:
            if self.force_baseline is None:
                return False, "Collecting(no baseline)"
            return False, "Collecting"
        tail, deviations, n, baseline, limit = stats
        is_jamming, trend_reason = self._evaluate_sustained_trend(
            deviations, limit, 'jam'
        )
        reason = (
            f"{trend_reason} baseline={baseline:.0f} "
            f"tail_min={min(tail):.0f} tail_max={max(tail):.0f}"
        )
        return is_jamming, reason

    def detect_tangling(self):
        stats = self._trend_detection_tail('tangling')
        if stats is None:
            if self.force_baseline is None:
                return False, "Collecting(no baseline)"
            return False, "Collecting"
        tail, deviations, n, baseline, limit = stats
        is_tangling, trend_reason = self._evaluate_sustained_trend(
            deviations, limit, 'tangling'
        )
        reason = (
            f"{trend_reason} baseline={baseline:.0f} "
            f"tail_min={min(tail):.0f} tail_max={max(tail):.0f}"
        )
        return is_tangling, reason

    def _record_jamming_snapshot(self, reason):
        """堵料锁定时将当前缓冲写入 klipper.log。"""
        try:
            stats = self._trend_detection_tail('jam')
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            logging.error(
                "tension_sensor jamming snapshot begin time=%s reason=%s",
                ts,
                reason,
            )
            logging.error(
                "tension_sensor jamming meta interval_s=%.3f trend_N=%d "
                "jam_deviation_limit=%s trend_tolerance=%s baseline=%s "
                "detection_start_baseline=%s baseline_jam_streak=%s",
                self.sample_interval,
                self.trigger_sample_count,
                self.jam_deviation_limit,
                self.trend_tolerance,
                self.force_baseline,
                self._detection_start_baseline,
                self._baseline_jam_streak,
            )
            if stats is not None:
                tail, deviations, n, baseline, limit = stats
                logging.error(
                    "tension_sensor jamming trend deviations=%s limit=%s baseline=%s",
                    deviations,
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

    def _record_tangling_snapshot(self, reason):
        """缠料锁定时将当前缓冲写入 klipper.log。"""
        try:
            stats = self._trend_detection_tail('tangling')
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            logging.error(
                "tension_sensor tangling snapshot begin time=%s reason=%s",
                ts,
                reason,
            )
            logging.error(
                "tension_sensor tangling meta interval_s=%.3f trend_N=%d "
                "tangling_deviation_limit=%s trend_tolerance=%s baseline=%s",
                self.sample_interval,
                self.trigger_sample_count,
                self.tangling_deviation_limit,
                self.trend_tolerance,
                self.force_baseline,
            )
            if stats is not None:
                tail, deviations, n, baseline, limit = stats
                logging.error(
                    "tension_sensor tangling trend deviations=%s limit=%s baseline=%s",
                    deviations,
                    limit,
                    baseline,
                )
            buf = list(self.detection_buffer)
            logging.error(
                "tension_sensor tangling detection_buffer len=%d (idx 0=oldest)",
                len(buf),
            )
            for i, v in enumerate(buf):
                logging.error("tension_sensor tangling detection_buffer[%d]=%s", i, v)
            logging.error("tension_sensor tangling snapshot end")
        except Exception as e:
            logging.error("tension_sensor failed to record tangling snapshot: %s", e)

    def update_detection_state(self, current_value):
        self.value_history.append(current_value)
        
        if self.jamming_triggered:
            return False, "JAMMED", "Permanently jammed"
        if self.tangling_triggered:
            return False, "TANGLING", "Permanently tangled"

        # 异常尖峰不参与 jam 判定窗口，避免污染投票窗口。
        if abs(current_value) > self.max_normal_value:
            return False, "NORMAL", (
                f"abnormal sample ignored: val={current_value} "
                f"limit={self.max_normal_value}"
            )

        self.detection_buffer.append(current_value)
        
        if self._skip_jam_detection:
            return False, "NORMAL", "baseline capture (jam check skipped)"
        if self._awaiting_first_auto_baseline:
            return False, "NORMAL", "awaiting first auto baseline (jam check skipped)"
        
        if self.force_baseline is None:
            return False, "COLLECTING", "no baseline (run TENSION_UPDATE_BASELINE)"
        
        if len(self.detection_buffer) < self.trigger_sample_count:
            return False, "COLLECTING", (
                f"{len(self.detection_buffer)}/{self.trigger_sample_count} "
                "for sustained trend"
            )
        
        is_jamming, reason = self.detect_jamming()
        
        if is_jamming:
            self.jamming_triggered = True
            self.current_state = "JAMMED"
            logging.error(f"JAMMING DETECTED and LOCKED! {reason}")
            self._record_jamming_snapshot(reason)
            return True, "JAMMED", reason

        if self.tangling_detection_enabled:
            is_tangling, tangling_reason = self.detect_tangling()
            if is_tangling:
                self.tangling_triggered = True
                self.current_state = "TANGLING"
                logging.error(f"TANGLING DETECTED and LOCKED! {tangling_reason}")
                self._record_tangling_snapshot(tangling_reason)
                return True, "TANGLING", tangling_reason
        
        return False, "NORMAL", reason

    def _send_jamming_warning(self, details):
        try:
            self.gcode.run_script_from_command(f"M118 FIBER JAMMING DETECTED and LOCKED: {details}")
            self.gcode.run_script_from_command("FIBER_FILAMENT_JAMMED")
            logging.error(f"JAMMING WARNING SENT: {details}")
        except Exception as e:
            logging.error(f"Failed to send warning: {e}")

    def _send_tangling_warning(self, details):
        try:
            self.gcode.run_script_from_command(
                f"M118 FIBER TANGLING DETECTED and LOCKED: {details}"
            )
            self.gcode.run_script_from_command("FIBER_FILAMENT_TANGLINGED")
            logging.error(f"TANGLING WARNING SENT: {details}")
        except Exception as e:
            logging.error(f"Failed to send tangling warning: {e}")

    def _notify_info(self, msg):
        """Send async-safe user-facing info."""
        logging.info("tension_sensor: %s", msg)

    def cmd_RESET_ALARM(self, gcmd):
        was_jammed = self.jamming_triggered
        was_tangled = self.tangling_triggered
        self.jamming_triggered = False
        self.tangling_triggered = False
        self.current_state = "NORMAL"
        self.detection_buffer.clear()
        self.value_history.clear()
        self._reset_baseline_jam_state()
        prev = "NORMAL"
        if was_jammed:
            prev = "JAMMED"
        elif was_tangled:
            prev = "TANGLING"
        gcmd.respond_info(f"Alarm reset. Previous state: {prev}")
        gcmd.respond_info(
            "Detection buffer cleared; baseline jam streak/reference reset; "
            "jam detection uses existing force_baseline and "
            f"{self.trigger_sample_count} new sample(s) for sustained trend."
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

        tangling_dev = gcmd.get_int('TANGLING_DEV', None)
        if tangling_dev is not None and tangling_dev >= 1000:
            self.tangling_deviation_limit = tangling_dev
            changed.append(f"TANGLING_DEV={tangling_dev}")

        trend_tol = gcmd.get_int('TREND_TOL', None)
        if trend_tol is not None and trend_tol >= 0:
            self.trend_tolerance = trend_tol
            changed.append(f"TREND_TOL={trend_tol}")
        
        gcmd.respond_info(f"Updated: {', '.join(changed) if changed else 'None'}")

    def cmd_SET_CONSECUTIVE(self, gcmd):
        frames = gcmd.get_int('FRAMES', None, minval=1, maxval=30)
        if frames is None:
            gcmd.respond_info(
                f"Usage: TENSION_SET_CONSECUTIVE FRAMES=<1..30> "
                f"(consecutive trend frames, current={self.trigger_sample_count})"
            )
            return
        self.trigger_sample_count = frames
        gcmd.respond_info(
            f"Updated trigger_sample_count (consecutive trend frames)="
            f"{self.trigger_sample_count}"
        )

    def cmd_SHOW_CONFIG(self, gcmd):
        gcmd.respond_info("=== TENSION SENSOR CONFIG ===")
        gcmd.respond_info(f"enable: {self.enabled}")
        gcmd.respond_info(f"poll_armed: {self.poll_armed}")
        gcmd.respond_info(f"polling: {self.polling_enabled}")
        gcmd.respond_info(f"temp_paused: {self._temp_paused}")
        gcmd.respond_info(f"sample_interval: {self.sample_interval:.3f}s")
        gcmd.respond_info(f"baseline_sample_count: {self.baseline_sample_count}")
        gcmd.respond_info(f"baseline_cluster_eps: {self.baseline_cluster_eps}")
        gcmd.respond_info(
            f"baseline_cluster_min_points: {self.baseline_cluster_min_points}"
        )
        gcmd.respond_info(
            f"trigger_sample_count (consecutive trend frames N): "
            f"{self.trigger_sample_count}"
        )
        gcmd.respond_info(f"jam_detection_enabled: {self.jam_detection_enabled}")
        gcmd.respond_info(
            f"tangling_detection_enabled: {self.tangling_detection_enabled}"
        )
        gcmd.respond_info(f"jam_deviation_limit: {self.jam_deviation_limit}")
        gcmd.respond_info(f"tangling_deviation_limit: {self.tangling_deviation_limit}")
        gcmd.respond_info(f"trend_tolerance: {self.trend_tolerance}")
        gcmd.respond_info(f"max_normal_value: {self.max_normal_value}")
        gcmd.respond_info(f"force_baseline: {self.force_baseline}")
        gcmd.respond_info(
            f"detection_baseline_active: {self._detection_baseline_active}"
        )
        gcmd.respond_info(
            f"detection_start_baseline: {self._detection_start_baseline}"
        )
        gcmd.respond_info(
            f"baseline_jam_streak: {self._baseline_jam_streak}/"
            f"{self._baseline_jam_trigger_count}"
        )

    def _baseline_update_allowed(self):
        if not self.enabled:
            return False, "tension_sensor enable=false"
        if not self.polling_enabled:
            return False, "polling not active"
        if self._temp_paused:
            return False, "polling temporarily paused"
        return True, None

    def _finish_baseline_update(self):
        self._skip_jam_detection = False
        self._baseline_update_running = False
        self._baseline_phase = None
        self.reactor.update_timer(self._baseline_timer, self.reactor.NEVER)
        self._close_spi_if_idle()

    def _baseline_update_event(self, eventtime):
        if not self._baseline_update_running:
            return self.reactor.NEVER
        allowed, block_reason = self._baseline_update_allowed()
        if not allowed:
            self._notify_info(
                f"TENSION_UPDATE_BASELINE aborted: {block_reason}"
            )
            self._finish_baseline_update()
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
            if self._awaiting_first_auto_baseline:
                self._awaiting_first_auto_baseline = False
                logging.info(
                    "tension_sensor: first auto baseline complete, "
                    "jam detection enabled (baseline=%s)",
                    baseline,
                )
            is_baseline_jam, baseline_reason = self._evaluate_baseline_jam(
                baseline
            )
            if is_baseline_jam:
                self._trigger_baseline_jam(baseline_reason)
                self._finish_baseline_update()
                self.reactor.register_callback(
                    lambda e: self._send_jamming_warning(baseline_reason)
                )
                return self.reactor.NEVER
            if baseline_reason is not None:
                logging.info(
                    "tension_sensor: baseline jam check: %s", baseline_reason
                )
            if details is None:
                self._notify_info(f"Baseline updated to {baseline}")
            else:
                self._notify_info(
                    f"Baseline updated to {baseline} (center={details['cluster_center']}, "
                    f"cluster_size={details['cluster_size']}, old={details['old_baseline']})"
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
        allowed, reason = self._baseline_update_allowed()
        if not allowed:
            return False, reason
        if self._baseline_update_running:
            return False, "previous update still running"
        if spidev is not None and not self._open_spi():
            return False, "SPI open failed"
        self._baseline_update_running = True
        self._skip_jam_detection = True
        self._baseline_phase = "warmup"
        self._baseline_warmup = []
        self._baseline_samples = []
        self.reactor.update_timer(self._baseline_timer, self.reactor.NOW)
        return True, None

    def cmd_TENSION_UPDATE_BASELINE(self, gcmd):
        ok, reason = self._start_baseline_update()
        if not ok:
            gcmd.respond_info(f"TENSION_UPDATE_BASELINE ignored: {reason}")
            return
        gcmd.respond_info(
            "TENSION_UPDATE_BASELINE started in background timer; "
            "result will be reported via M118"
        )

    def cmd_TENSION_DEBUG(self, gcmd):
        gcmd.respond_info("=== TENSION SENSOR DEBUG ===")
        gcmd.respond_info(f"State: {self.current_state}")
        gcmd.respond_info(f"Jamming Triggered: {self.jamming_triggered}")
        gcmd.respond_info(f"Tangling Triggered: {self.tangling_triggered}")
        gcmd.respond_info(f"Jam deviation limit: {self.jam_deviation_limit}")
        gcmd.respond_info(f"Tangling deviation limit: {self.tangling_deviation_limit}")
        gcmd.respond_info(f"Trend tolerance: {self.trend_tolerance}")
        gcmd.respond_info(f"force_baseline: {self.force_baseline}")
        gcmd.respond_info(
            f"detection_baseline_active: {self._detection_baseline_active} "
            f"| detection_start_baseline: {self._detection_start_baseline} "
            f"| baseline_jam_streak: {self._baseline_jam_streak}/"
            f"{self._baseline_jam_trigger_count}"
        )
        
        stats = self._trend_detection_tail('jam')
        if stats is not None:
            tail, deviations, n, baseline, limit = stats
            triggered, trend_reason = self._evaluate_sustained_trend(
                deviations, limit, 'jam'
            )
            gcmd.respond_info(
                f"jam_trend: triggered={triggered} {trend_reason} "
                f"baseline={baseline:.0f}"
            )
            gcmd.respond_info(
                f"Recent tail neg_dev max: {max(deviations):.0f}"
            )
        else:
            gcmd.respond_info("jam_trend: collecting")

        tstats = self._trend_detection_tail('tangling')
        if tstats is not None:
            tail, deviations, n, baseline, limit = tstats
            triggered, trend_reason = self._evaluate_sustained_trend(
                deviations, limit, 'tangling'
            )
            gcmd.respond_info(
                f"tangling_trend: triggered={triggered} {trend_reason} "
                f"baseline={baseline:.0f}"
            )
            gcmd.respond_info(
                f"Recent tail pos_dev max: {max(deviations):.0f}"
            )
        else:
            gcmd.respond_info("tangling_trend: collecting")
        
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
        self.tangling_triggered = False
        self.current_state = "NORMAL"
        self._baseline_jam_streak = 0
        gcmd.respond_info(
            "Detection buffers reset. Baseline unchanged. "
            f"Need {self.trigger_sample_count} sample(s) for sustained trend."
        )

    def cmd_FORCE_SENSOR_STATUS(self, gcmd):
        val = self.read_spi_once()
        if val is None:
            gcmd.respond_info("Read failed")
            return
        
        volt = self.adc_to_voltage(val)
        gcmd.respond_info(f"Current: {val} ({volt:.6f}V)")
        gcmd.respond_info(
            f"Config enable: {self.enabled} | Armed: {self.poll_armed} | "
            f"Polling: {self.polling_enabled} | Temp paused: {self._temp_paused}"
        )
        gcmd.respond_info(
            f"State: {self.current_state} | Jamming Locked: {self.jamming_triggered} "
            f"| Tangling Locked: {self.tangling_triggered}"
        )
        gcmd.respond_info(f"Jam detection enabled: {self.jam_detection_enabled}")
        gcmd.respond_info(
            f"Tangling detection enabled: {self.tangling_detection_enabled}"
        )
        gcmd.respond_info(f"Jam deviation limit: {self.jam_deviation_limit}")
        gcmd.respond_info(f"Tangling deviation limit: {self.tangling_deviation_limit}")
        gcmd.respond_info(f"force_baseline: {self.force_baseline}")
        gcmd.respond_info(
            f"detection_start_baseline: {self._detection_start_baseline} "
            f"| baseline_jam_streak: {self._baseline_jam_streak}/"
            f"{self._baseline_jam_trigger_count}"
        )
        
        stats = self._trend_detection_tail('jam')
        if stats is not None:
            tail, deviations, n, baseline, limit = stats
            triggered, trend_reason = self._evaluate_sustained_trend(
                deviations, limit, 'jam'
            )
            gcmd.respond_info(
                f"baseline={baseline:.0f} | jam_trend triggered={triggered} "
                f"| neg_dev_max={max(deviations):.0f} (limit={limit:.0f}) "
                f"| tol={self.trend_tolerance}"
            )
        tstats = self._trend_detection_tail('tangling')
        if tstats is not None:
            tail, deviations, n, baseline, limit = tstats
            triggered, trend_reason = self._evaluate_sustained_trend(
                deviations, limit, 'tangling'
            )
            gcmd.respond_info(
                f"baseline={baseline:.0f} | tangling_trend triggered={triggered} "
                f"| pos_dev_max={max(deviations):.0f} (limit={limit:.0f}) "
                f"| tol={self.trend_tolerance}"
            )

    def cmd_TENSION_SENSOR_PAUSE(self, gcmd):
        if not self.enabled:
            gcmd.respond_info("ignored: tension_sensor enable=false")
            return
        if not self.polling_enabled:
            gcmd.respond_info("ignored: polling not active")
            return
        if self._temp_paused:
            gcmd.respond_info("already temporarily paused")
            return
        self._temp_paused = True
        self.reactor.update_timer(self._polling_timer, self.reactor.NEVER)
        gcmd.respond_info("Polling temporarily paused")
        logging.info("tension_sensor: polling temporarily paused")

    def cmd_TENSION_SENSOR_RESUME(self, gcmd):
        if not self.enabled:
            gcmd.respond_info("ignored: tension_sensor enable=false")
            return
        if not self._temp_paused:
            gcmd.respond_info("ignored: not temporarily paused")
            return
        self._temp_paused = False
        if self.polling_enabled:
            self.reactor.update_timer(self._polling_timer, self.reactor.NOW)
        gcmd.respond_info("Polling resumed")
        logging.info("tension_sensor: polling resumed after temporary pause")

    def _start_detection(self, gcmd):
        """Start polling; logic formerly tied to TENSION_SENSOR_POLL ENABLE=1."""
        logging_enable = gcmd.get_int('LOGGING', 0)
        detect_enable = gcmd.get_int('DETECT', 1)
        self.jam_detection_enabled = (detect_enable == 1)
        self.polling_log_enabled = (logging_enable == 1)
        if not self.start_polling():
            gcmd.respond_info("ERROR: SPI open failed, polling not started")
            return
        self._detection_baseline_active = True
        self._detection_start_baseline = None
        self._baseline_jam_streak = 0
        self._awaiting_first_auto_baseline = self.jam_detection_enabled
        self._auto_baseline_requested = True
        self._auto_baseline_pending_frames = self.auto_baseline_discard_frames
        mode = "DETECTION" if self.jam_detection_enabled else "LOG_ONLY"
        gcmd.respond_info(
            f"Polling ON ({self.sample_interval*1000:.0f}ms) "
            f"klipper.log={'ON' if self.polling_log_enabled else 'OFF'} "
            f"mode={mode}"
        )
        if self.jam_detection_enabled:
            discard = self.auto_baseline_discard_frames
            capture = self.baseline_sample_count * 2
            gcmd.respond_info(
                "Jam check disabled until first auto baseline completes "
                "(after %d discard frames + %d capture samples)."
                % (discard, capture)
            )
        gcmd.respond_info(
            f"Auto baseline update scheduled after "
            f"{self.auto_baseline_discard_frames} frames "
            "(startup dirty-data discard)."
        )
        gcmd.respond_info(
            f"Baseline drift jam: first baseline update recorded as reference; "
            f"subsequent updates vs reference, "
            f"{self._baseline_jam_trigger_count} consecutive over "
            f"jam_deviation_limit={self.jam_deviation_limit} also triggers jam."
        )

    def cmd_TENSION_SENSOR_START(self, gcmd):
        if not self.enabled:
            gcmd.respond_info("ignored: tension_sensor enable=false")
            return
        if not self.poll_armed:
            gcmd.respond_info(
                "ignored: not armed (run TENSION_SENSOR_POLL ENABLE=1 first)"
            )
            return
        self._start_detection(gcmd)

    def cmd_FORCE_SENSOR_POLL(self, gcmd):
        if not self.enabled:
            gcmd.respond_info("ignored: tension_sensor enable=false")
            return
        enable = gcmd.get_int('ENABLE', None)

        if enable is None:
            gcmd.respond_info(
                "Usage: TENSION_SENSOR_POLL ENABLE=<0|1> "
                "[TANGLING_DETECT=<0|1>] "
                "(arm only; use TENSION_SENSOR_START to begin sampling; "
                "tangling detect default off)"
            )
            gcmd.respond_info(
                f"Armed: {self.poll_armed} | "
                f"Tangling detection: {self.tangling_detection_enabled}"
            )
            return

        if enable:
            tangling_detect = gcmd.get_int('TANGLING_DETECT', 0)
            self.tangling_detection_enabled = (tangling_detect == 1)
            self.poll_armed = True
            gcmd.respond_info(
                "Tension sensor armed. Run TENSION_SENSOR_START to begin "
                "polling/detection."
            )
            gcmd.respond_info(
                f"Tangling detection: "
                f"{'ON' if self.tangling_detection_enabled else 'OFF'}"
            )
        else:
            self.poll_armed = False
            self.tangling_detection_enabled = False
            self.stop_polling()
            gcmd.respond_info("Tension sensor disarmed; polling stopped")

    def start_polling(self):
        if self._baseline_update_running:
            self._finish_baseline_update()
        if spidev is not None and not self._open_spi():
            logging.error("tension_sensor: cannot start polling, SPI open failed")
            return False
        self.detection_buffer.clear()
        self.value_history.clear()
        self.jamming_triggered = False
        self.tangling_triggered = False
        self.current_state = "NORMAL"
        self._reset_detection_baseline_tracking()
        self._temp_paused = False
        self._auto_baseline_requested = False
        self._auto_baseline_pending_frames = 0
        self._awaiting_first_auto_baseline = False
        self.polling_enabled = True
        self._poll_sample_counter = 0
        self.reactor.update_timer(self._polling_timer, self.reactor.NOW)
        logging.info(
            "Polling started: %.0fms; baseline via TENSION_UPDATE_BASELINE "
            "or auto baseline after TENSION_SENSOR_START",
            self.sample_interval * 1000,
        )
        return True

    def stop_polling(self):
        if not self.polling_enabled:
            return
        
        self.polling_enabled = False
        self._temp_paused = False
        self._auto_baseline_requested = False
        self._auto_baseline_pending_frames = 0
        self._awaiting_first_auto_baseline = False
        self._reset_detection_baseline_tracking()
        self.reactor.update_timer(self._polling_timer, self.reactor.NEVER)
        self._close_spi_if_idle()
        logging.info("Polling stopped")

    def _polling_event(self, eventtime):
        if not self.polling_enabled or self._temp_paused:
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
                elif changed and state == "TANGLING":
                    self.reactor.register_callback(
                        lambda e: self._send_tangling_warning(details)
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
                    ok, reason = self._start_baseline_update()
                    if ok:
                        self._notify_info(
                            f"Auto baseline update started after "
                            f"{self.auto_baseline_discard_frames} startup frames"
                        )
                    else:
                        logging.info(
                            "Auto baseline update skipped: %s", reason
                        )
                        self._auto_baseline_requested = True
                        self._auto_baseline_pending_frames = (
                            self.auto_baseline_discard_frames
                        )
        except Exception as e:
            logging.error(f"Polling error: {e}")
        return eventtime + self.sample_interval

def load_config(config):
    return MySerial(config)
