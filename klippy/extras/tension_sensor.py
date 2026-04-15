import logging
import threading
import time
import collections
import math

class MySerial:
    def __init__(self, config):
        self.spi_device = "/dev/spidev0.0"
        
        # Voltage conversion parameters
        self.VREF = 3.3
        self.GAIN = 128
        self.ADC_MAX = 2**23 - 1  # 8388607 (24-bit signed)
        
        self.sample_interval = 0.2  # seconds (200ms)
        
        # 满 30 帧后：每帧用最旧 15 条（滤波）算参考 mean_f 与允许带；
        # 用最新 3 点中位数与带比较判堵；需连续 3 帧均判堵才锁定（无再累计 5 次）。
        self.detection_buffer_size = 30
        self.threshold_sample_count = 15
        self.trigger_sample_count = 3
        
        self.detection_buffer = collections.deque(maxlen=self.detection_buffer_size)
        
        # 判堵：力值为有符号 ADC（可正可负）。以窗口中心 mean_f 为参考，
        # 落在 (mean_f - limit, mean_f + limit) 之外视为堵料（与 |Δ|>limit 等价）。
        self.jam_deviation_limit = config.getint(
            'jam_deviation_limit', 45000, minval=1000, maxval=10000000
        )
        # 幅值饱和：|力值| 超过此值视为异常（正负对称）
        self.max_normal_value = 500000
        
        self.jam_consecutive_frames = config.getint(
            'jam_consecutive_frames', 3, minval=1, maxval=10
        )
        self.consecutive_jam_frames = 0
        
        self.current_state = "NORMAL"
        self.jamming_triggered = False
        
        self.value_history = collections.deque(maxlen=500)
        
        self.calibration_log = []
        self.polling_log = []

        self.polling_enabled = False
        self.polling_log_enabled = False
        
        printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        self.reactor = printer.get_reactor()
        
        self._register_commands()

    def _register_commands(self):
        self.gcode.register_command('TENSION_SENSOR_POLL', self.cmd_FORCE_SENSOR_POLL,
                                    desc="TENSION_SENSOR_POLL ENABLE=<0|1> [LOGGING=<0|1>]")
        self.gcode.register_command('TENSION_SENSOR_STATUS', self.cmd_FORCE_SENSOR_STATUS)
        self.gcode.register_command('SHOW_CALIBRATION', self.cmd_SHOW_CALIBRATION)
        self.gcode.register_command('GET_FORCE_VALUE', self.cmd_GET_FORCE_VALUE)
        self.gcode.register_command('GET_RAW_FORCE_DATA', self.cmd_GET_RAW_FORCE_DATA)
        self.gcode.register_command('CLEAR_POLLING_LOG', self.cmd_CLEAR_POLLING_LOG)
        self.gcode.register_command('SHOW_POLLING_LOG', self.cmd_SHOW_POLLING_LOG)
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
        self.gcode.register_command('TENSION_RESET_LEARNING', self.cmd_RESET_DETECTION_BUFFERS)
        self.gcode.register_command('TENSION_DEBUG', self.cmd_TENSION_DEBUG)
        self.gcode.register_command('TENSION_RESET_ALARM', self.cmd_RESET_ALARM,)
        self.gcode.register_command('TENSION_SHOW_BASELINE', self.cmd_SHOW_BASELINE_DEPRECATED)
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

    def calculate_stats(self, values):
        if len(values) < 10:
            return None, None
        
        mean = sum(values) / len(values)
        
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        std = math.sqrt(variance) if variance > 0 else 1.0
        
        return mean, std

    def _median_filter_window3(self, values):
        """Edge-padded length-3 median filter for fluctuating series."""
        if len(values) < 2:
            return list(values)
        padded = [values[0]] + list(values) + [values[-1]]
        out = []
        for i in range(len(values)):
            w = padded[i : i + 3]
            sw = sorted(w)
            out.append(sw[len(sw) // 2])
        return out

    def _window15_filtered_mean_std(self):
        """最旧 15 点中值滤波后求 mean/std。"""
        buf = list(self.detection_buffer)
        if len(buf) < self.detection_buffer_size:
            return None, None
        oldest_15 = buf[: self.threshold_sample_count]
        filtered = self._median_filter_window3(oldest_15)
        mean_f, std_f = self.calculate_stats(filtered)
        if std_f is not None and std_f <= 0:
            std_f = 1.0
        return mean_f, std_f

    def get_trigger_median(self):
        """Median of the newest 3 samples (indices -3..-1)."""
        if len(self.detection_buffer) < self.trigger_sample_count:
            return None
        newest_3 = list(self.detection_buffer)[-self.trigger_sample_count :]
        newest_3.sort()
        return newest_3[len(newest_3) // 2]

    def detect_jamming(self, current_value):
        if len(self.detection_buffer) < self.detection_buffer_size:
            return False, "Collecting"
        
        mean_f, _std_f = self._window15_filtered_mean_std()
        if mean_f is None:
            return False, "Collecting"
        
        smoothed = self.get_trigger_median()
        if smoothed is None:
            return False, "Collecting"
        
        limit = self.jam_deviation_limit
        # 对称允许带 [mean_f±limit]；|Δ|>limit 等价于在闭区间外（mean_f 为负时区间整体平移）
        in_band = (mean_f - limit) <= smoothed <= (mean_f + limit)
        is_jamming = not in_band
        
        if abs(smoothed) > self.max_normal_value:
            is_jamming = True
        
        delta = smoothed - mean_f
        deviation = abs(delta)
        reason = (
            f"dev={deviation:.0f}/{limit:.0f} delta={delta:+.0f}, val={smoothed:.0f}, mean_f={mean_f:.0f}"
        )
        
        return is_jamming, reason

    def _record_jamming_snapshot(self, reason):
        """堵料锁定时将当前窗口数据逐行写入 klipper.log，避免单条过长。"""
        try:
            mf, sf = self._window15_filtered_mean_std()
            trig = self.get_trigger_median()
            oldest_15_raw = list(self.detection_buffer)[: self.threshold_sample_count]
            filtered_15 = (
                self._median_filter_window3(oldest_15_raw)
                if len(oldest_15_raw) == self.threshold_sample_count
                else []
            )
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            logging.error(
                "tension_sensor jamming snapshot begin time=%s reason=%s",
                ts,
                reason,
            )
            logging.error(
                "tension_sensor jamming meta interval_s=%.3f consecutive_frames=%d jam_limit=%s",
                self.sample_interval,
                self.jam_consecutive_frames,
                self.jam_deviation_limit,
            )
            logging.error(
                "tension_sensor jamming window15 mean_f=%s std_f=%s trig_median3=%s",
                mf,
                sf,
                trig,
            )
            buf = list(self.detection_buffer)
            logging.error(
                "tension_sensor jamming detection_buffer len=%d (idx 0=oldest)",
                len(buf),
            )
            for i, v in enumerate(buf):
                logging.error("tension_sensor jamming detection_buffer[%d]=%s", i, v)
            logging.error(
                "tension_sensor jamming window15_raw len=%d (idx 0=oldest of 15)",
                len(oldest_15_raw),
            )
            for i, v in enumerate(oldest_15_raw):
                logging.error("tension_sensor jamming window15_raw[%d]=%s", i, v)
            logging.error(
                "tension_sensor jamming window15_median_filtered len=%d",
                len(filtered_15),
            )
            for i, v in enumerate(filtered_15):
                logging.error(
                    "tension_sensor jamming window15_median_filtered[%d]=%s", i, v
                )
            logging.error("tension_sensor jamming snapshot end")
        except Exception as e:
            logging.error("tension_sensor failed to record jamming snapshot: %s", e)

    def update_detection_state(self, current_value):
        self.value_history.append(current_value)
        
        if self.jamming_triggered:
            return False, "JAMMED", "Permanently jammed"
        
        self.detection_buffer.append(current_value)
        
        if len(self.detection_buffer) < self.detection_buffer_size:
            return False, "COLLECTING", f"{len(self.detection_buffer)}/{self.detection_buffer_size}"
        
        is_jamming, reason = self.detect_jamming(current_value)
        
        if is_jamming:
            self.consecutive_jam_frames += 1
            
            if self.consecutive_jam_frames >= self.jam_consecutive_frames:
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

    def cmd_RESET_ALARM(self, gcmd):
        was_jammed = self.jamming_triggered
        self.jamming_triggered = False
        self.consecutive_jam_frames = 0
        self.current_state = "NORMAL"
        # 清空窗口，避免堵料期间的旧样仍留在 30 帧里导致复位后立即再判堵/误报
        self.detection_buffer.clear()
        self.value_history.clear()
        gcmd.respond_info(f"Alarm reset. Previous state: {'JAMMED' if was_jammed else 'NORMAL'}")
        gcmd.respond_info(
            f"Detection buffer cleared; need {self.detection_buffer_size} new samples "
            f"({self.detection_buffer_size * self.sample_interval:.1f}s) before jam detection."
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
        # 修改阈值后清除当前累计，避免沿用旧条件下的计数。
        self.consecutive_jam_frames = 0
        gcmd.respond_info(
            f"Updated jam_consecutive_frames={self.jam_consecutive_frames}"
        )

    def cmd_SHOW_CONFIG(self, gcmd):
        gcmd.respond_info("=== TENSION SENSOR CONFIG ===")
        gcmd.respond_info(f"sample_interval: {self.sample_interval:.3f}s")
        gcmd.respond_info(f"detection_buffer_size: {self.detection_buffer_size}")
        gcmd.respond_info(f"threshold_sample_count: {self.threshold_sample_count}")
        gcmd.respond_info(f"trigger_sample_count: {self.trigger_sample_count}")
        gcmd.respond_info(f"jam_deviation_limit: {self.jam_deviation_limit}")
        gcmd.respond_info(f"jam_consecutive_frames: {self.jam_consecutive_frames}")
        gcmd.respond_info(f"max_normal_value: {self.max_normal_value}")

    def cmd_TENSION_DEBUG(self, gcmd):
        gcmd.respond_info("=== TENSION SENSOR DEBUG ===")
        gcmd.respond_info(f"State: {self.current_state}")
        gcmd.respond_info(f"Jamming Triggered: {self.jamming_triggered}")
        gcmd.respond_info(f"Jam deviation limit: {self.jam_deviation_limit}")
        
        if len(self.detection_buffer) >= self.detection_buffer_size:
            mf, sf = self._window15_filtered_mean_std()
            if mf is not None:
                gcmd.respond_info(
                    f"Window15 (filtered): mean={mf:.0f} std={sf:.0f}"
                )
                trig = self.get_trigger_median()
                if trig is not None:
                    dev = abs(trig - mf)
                    gcmd.respond_info(
                        f"Trig median3={trig:.0f} dev={dev:.0f} (limit {self.jam_deviation_limit})"
                    )
        
        gcmd.respond_info(
            f"Consecutive jam frames: {self.consecutive_jam_frames}/{self.jam_consecutive_frames}"
        )
        
        if len(self.detection_buffer) > 0:
            recent = list(self.detection_buffer)[-min(10, len(self.detection_buffer)) :]
            gcmd.respond_info(f"Recent values: min={min(recent)}, max={max(recent)}, avg={sum(recent)/len(recent):.0f}")

    def cmd_SHOW_BASELINE_DEPRECATED(self, gcmd):
        gcmd.respond_info("Saved baseline: not used (learning removed).")

    def cmd_REFERENCE_DEPRECATED(self, gcmd):
        gcmd.respond_info("Reference/learning commands: not used (learning removed).")

    def cmd_RESET_DETECTION_BUFFERS(self, gcmd):
        self.detection_buffer.clear()
        self.consecutive_jam_frames = 0
        self.jamming_triggered = False
        self.current_state = "NORMAL"
        gcmd.respond_info(
            f"Detection buffers reset. Need {self.detection_buffer_size} samples "
            f"({self.detection_buffer_size * self.sample_interval:.1f}s) before jam detection."
        )

    def cmd_CLEAR_POLLING_LOG(self, gcmd):
        try:
            self.detection_buffer.clear()
            self.value_history.clear()
            
            self.calibration_log.clear()
            self.polling_log.clear()
            
            self.current_state = "NORMAL"
            self.jamming_triggered = False
            self.consecutive_jam_frames = 0
            
            gcmd.respond_info("=" * 50)
            gcmd.respond_info("日志与检测缓存已清除")
            gcmd.respond_info(
                f"需重新采集 {self.detection_buffer_size} 个样本后方可判堵"
            )
            gcmd.respond_info("=" * 50)
            
        except Exception as e:
            gcmd.respond_info(f"错误: 清除失败 - {e}")

    def cmd_SHOW_POLLING_LOG(self, gcmd):
        count = gcmd.get_int('COUNT', 10)
        if not self.polling_log:
            gcmd.respond_info("Log empty")
            return
        
        start = max(0, len(self.polling_log) - count)
        for i, e in enumerate(self.polling_log[start:], start+1):
            ts = e.get('timestamp', 'N/A')
            val = e.get('value', 'N/A')
            volt = e.get('voltage', 0)
            gcmd.respond_info(f"{i:4d}. {ts} | {val:8d} | {volt:.6f}V")
        
        gcmd.respond_info(f"Total: {len(self.polling_log)}")

    def cmd_FORCE_SENSOR_STATUS(self, gcmd):
        val = self.read_spi_once()
        if val is None:
            gcmd.respond_info("Read failed")
            return
        
        volt = self.adc_to_voltage(val)
        gcmd.respond_info(f"Current: {val} ({volt:.6f}V)")
        gcmd.respond_info(f"State: {self.current_state} | Jamming Locked: {self.jamming_triggered}")
        gcmd.respond_info(f"Jam deviation limit: {self.jam_deviation_limit}")
        
        if len(self.detection_buffer) >= self.detection_buffer_size:
            mf, _ = self._window15_filtered_mean_std()
            trig = self.get_trigger_median()
            if mf is not None and trig is not None:
                dev = abs(trig - mf)
                gcmd.respond_info(
                    f"Window mean_f={mf:.0f} | trig_median3={trig:.0f} | dev={dev:.0f}"
                )

    def cmd_FORCE_SENSOR_POLL(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        logging_enable = gcmd.get_int('LOGGING', 0)
        
        if enable is None:
            gcmd.respond_info("Usage: TENSION_SENSOR_POLL ENABLE=<0|1> [LOGGING=<0|1>]")
            return
        
        if enable:
            self.polling_log_enabled = (logging_enable == 1)
            if self.polling_log_enabled:
                self.polling_log.clear()
            
            self.start_polling()
            gcmd.respond_info(f"Polling ON ({self.sample_interval*1000:.0f}ms) "
                            f"logging={'ON' if self.polling_log_enabled else 'OFF'}")
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
        self.polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
        self.polling_thread.start()
        
        logging.info(
            f"Polling started: {self.sample_interval*1000:.0f}ms, "
            f"need {self.detection_buffer_size} samples before jam detection"
        )

    def stop_polling(self):
        if not self.polling_enabled:
            return
        
        self.polling_enabled = False
        if hasattr(self, 'polling_thread'):
            self.polling_thread.join(timeout=2)
        
        logging.info("Polling stopped")

    def _polling_loop(self):
        logging.info("Polling loop started")
        
        try:
            while self.polling_enabled:
                val = self.read_spi_once()
                
                if val is None:
                    time.sleep(self.sample_interval)
                    continue
                
                if self.polling_log_enabled:
                    self.polling_log.append({
                        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                        'value': val,
                        'voltage': self.adc_to_voltage(val)
                    })
                
                changed, state, details = self.update_detection_state(val)
                
                if changed and state == "JAMMED":
                    self.reactor.register_callback(lambda e: self._send_jamming_warning(details))
                
                if len(self.value_history) % 60 == 0:
                    logging.info(f"Status: {state} | {details}")
                
                time.sleep(self.sample_interval)
                
        except Exception as e:
            logging.error(f"Polling error: {e}")
        finally:
            logging.info("Polling ended")

def load_config(config):
    return MySerial(config)
