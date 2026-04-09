import logging
import threading
import time
import os
import json
from pathlib import Path
import collections
import math

class MySerial:
    def __init__(self, config):
        self.spi_device = "/dev/spidev0.0"
        self.calibration_file = "/home/anisoprint/force_sensor_calibration.json"
        
        # Voltage conversion parameters
        self.VREF = 3.3
        self.GAIN = 128
        self.ADC_MAX = 2**23 - 1  # 8388607 (24-bit signed)
        
        self.sample_interval = 1  # seconds
        
        # Learning phase parameters
        self.learning_window = 10
        
        self.baseline_buffer = collections.deque(maxlen=self.learning_window)
        self.detection_buffer = collections.deque(maxlen=40)
        
        self.anomaly_threshold_multiplier = 1.5
        self.min_anomaly_deviation = 45000
        self.max_normal_value = 500000
        
        self.anomaly_confirm_count = 5
        self.anomaly_counter = 0
        
        self.baseline_mean = None
        self.baseline_std = None
        self.current_state = "NORMAL"
        self.is_learning = True
        self.learning_progress = 0
        self.jamming_triggered = False
        
        self.learning_history = []
        self.initial_baseline = None
        self.consecutive_deviant_learnings = 0
        self.learning_deviation_threshold = 0.2
        self.learning_jamming_threshold = 3
        
        self.value_history = collections.deque(maxlen=500)
        
        self.saved_baseline = None
        self.baseline_max_age = 120
        
        self.calibration_log = []
        self.polling_log = []

        self.polling_enabled = False
        self.polling_log_enabled = False
        
        self._load_history()
        
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
                                    desc="TENSION_SET_THRESHOLD MULT=<n> MIN_DEV=<n> LEARNING_DEV=<n>")
        self.gcode.register_command('TENSION_RESET_LEARNING', self.cmd_RESET_LEARNING)
        self.gcode.register_command('TENSION_SHOW_BASELINE', self.cmd_SHOW_BASELINE)
        self.gcode.register_command('TENSION_DEBUG', self.cmd_TENSION_DEBUG)
        self.gcode.register_command('TENSION_RESET_ALARM', self.cmd_RESET_ALARM,)
        self.gcode.register_command('TENSION_SET_REFERENCE', self.cmd_SET_REFERENCE,)
        self.gcode.register_command('TENSION_SHOW_REFERENCE', self.cmd_SHOW_REFERENCE,)
        self.gcode.register_command('TENSION_SHOW_LEARNING_HISTORY', self.cmd_SHOW_LEARNING_HISTORY,)

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

    def _load_history(self):
        try:
            if os.path.exists(self.calibration_file):
                with open(self.calibration_file, 'r') as f:
                    data = json.load(f)
                    self.calibration_log = data.get('calibration_log', [])
                    self.polling_log = data.get('polling_log', [])
                    self.saved_baseline = data.get('saved_baseline', None)
                    self.learning_history = data.get('learning_history', [])
                    self.initial_baseline = data.get('initial_baseline', None)
        except Exception as e:
            logging.error(f"Failed to load history: {e}")

    def _save_history(self):
        try:
            os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)
            with open(self.calibration_file, 'w') as f:
                json.dump({
                    'calibration_log': self.calibration_log[-50:],
                    'polling_log': self.polling_log[-200:],
                    'saved_baseline': self.saved_baseline,
                    'learning_history': self.learning_history[-20:],
                    'initial_baseline': self.initial_baseline
                }, f, indent=2)
        except Exception as e:
            logging.error(f"Failed to save history: {e}")

    def calculate_stats(self, values):
        if len(values) < 10:
            return None, None
        
        mean = sum(values) / len(values)
        
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        std = math.sqrt(variance) if variance > 0 else 1.0
        
        return mean, std

    def save_current_baseline(self):
        if self.baseline_mean is None:
            return False
        
        self.saved_baseline = {
            'mean': self.baseline_mean,
            'std': self.baseline_std,
            'threshold': self.calculate_dynamic_threshold(),
            'timestamp': time.time()
        }
        logging.info(f"Saved baseline: mean={self.baseline_mean:.0f}")
        return True

    def load_baseline(self):
        if self.saved_baseline is None:
            return False
        
        if time.time() - self.saved_baseline['timestamp'] > self.baseline_max_age:
            logging.info("Saved baseline expired, will re-learn")
            return False
        
        self.baseline_mean = self.saved_baseline['mean']
        self.baseline_std = self.saved_baseline['std']
        logging.info(f"Loaded baseline: mean={self.baseline_mean:.0f}")
        return True

    def update_baseline(self):
        if len(self.baseline_buffer) < self.learning_window:
            return False
        
        mean, std = self.calculate_stats(list(self.baseline_buffer))
        if mean is None:
            return False
            
        self.baseline_mean = mean
        self.baseline_std = std if std is not None and std > 0 else 1.0
        
        learning_result = {
            'mean': self.baseline_mean,
            'std': self.baseline_std,
            'timestamp': time.time(),
            'threshold': self.calculate_dynamic_threshold()
        }
        self.learning_history.append(learning_result)
        
        self.check_learning_consistency()
        
        return True

    def check_learning_consistency(self):
        if self.initial_baseline is None:
            return
        
        deviation = abs(self.baseline_mean - self.initial_baseline['mean']) / abs(self.initial_baseline['mean'])
        
        if deviation <= self.learning_deviation_threshold:
            self.consecutive_deviant_learnings = 0
            logging.info(f"Learning consistent with reference baseline (dev={deviation:.2%})")
            return
        
        self.consecutive_deviant_learnings += 1
        
        logging.warning(f"Learning deviation from reference baseline: {deviation:.2%} "
                       f"(consecutive: {self.consecutive_deviant_learnings}/{self.learning_jamming_threshold})")
        
        if self.consecutive_deviant_learnings >= self.learning_jamming_threshold:
            logging.error(f"CONSISTENT DEVIATION DETECTED: {self.consecutive_deviant_learnings} consecutive "
                         f"learnings deviated from reference baseline")
            
            self.current_state = "DRIFT_WARNING"
            
            self._send_drift_warning(f"Consistent deviation from reference: {deviation:.2%}")

    def _send_drift_warning(self, details):
        try:
            self.gcode.run_script_from_command(f"M118 FIBER BASELINE DRIFT WARNING: {details}")
            logging.warning(f"DRIFT WARNING SENT: {details}")
        except Exception as e:
            logging.error(f"Failed to send drift warning: {e}")

    def calculate_dynamic_threshold(self):
        if self.baseline_mean is None:
            return self.min_anomaly_deviation
        
        std_based = self.baseline_std * self.anomaly_threshold_multiplier
        
        threshold = max(std_based, self.min_anomaly_deviation)
        
        return threshold

    def get_smoothed_value(self):
        if len(self.detection_buffer) < 10:
            return None
        
        recent = list(self.detection_buffer)[-10:]
        recent.sort()
        return recent[len(recent)//2]

    def detect_jamming(self, current_value):
        if self.baseline_mean is None:
            return False, "No baseline"
        
        smoothed = self.get_smoothed_value()
        if smoothed is None:
            return False, "Collecting"
        
        deviation = abs(smoothed - self.baseline_mean)
        threshold = self.calculate_dynamic_threshold()
        
        is_jamming = deviation > threshold
        
        if smoothed > self.max_normal_value or smoothed < -self.max_normal_value:
            is_jamming = True
        
        reason = f"dev={deviation:.0f}/{threshold:.0f}, val={smoothed:.0f}"
        
        return is_jamming, reason

    def update_detection_state(self, current_value):
        self.value_history.append(current_value)
        
        if self.jamming_triggered:
            return False, "JAMMED", "Permanently jammed"
        
        if self.is_learning:
            self.baseline_buffer.append(current_value)
            self.learning_progress = len(self.baseline_buffer)
            
            if len(self.baseline_buffer) >= self.learning_window:
                if self.update_baseline():
                    self.is_learning = False
                    threshold = self.calculate_dynamic_threshold()
                    self.save_current_baseline()
                    logging.info(f"LEARNED: mean={self.baseline_mean:.0f} std={self.baseline_std:.0f} "
                               f"threshold={threshold:.0f}")
                    return False, "LEARNED", f"Threshold={threshold:.0f}"
            
            return False, "LEARNING", f"{len(self.baseline_buffer)}/{self.learning_window}"
        
        self.detection_buffer.append(current_value)
        
        if len(self.detection_buffer) < 10:
            return False, "COLLECTING", f"{len(self.detection_buffer)}/10"
        
        # 检测堵料
        is_jamming, reason = self.detect_jamming(current_value)
        
        if is_jamming:
            self.anomaly_counter += 1
            
            if self.anomaly_counter >= self.anomaly_confirm_count:
                self.jamming_triggered = True
                self.current_state = "JAMMED"
                logging.error(f"JAMMING DETECTED and LOCKED! {reason}")
                return True, "JAMMED", f"Jamming confirmed after {self.anomaly_counter} samples"
            
            return False, "CHECKING", f"jamming {self.anomaly_counter}/{self.anomaly_confirm_count}"
        else:
            self.anomaly_counter = 0
            
            if self.current_state == "DRIFT_WARNING":
                self.current_state = "NORMAL"
            
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
        self.anomaly_counter = 0
        self.current_state = "NORMAL"
        gcmd.respond_info(f"Alarm reset. Previous state: {'JAMMED' if was_jammed else 'NORMAL'}")

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
        gcmd.respond_info(f"Value: {val} | Voltage: {volt:.6f}V")

    def cmd_SHOW_CALIBRATION(self, gcmd):
        gcmd.respond_info(f"Calibration entries: {len(self.calibration_log)}")

    def cmd_SET_THRESHOLD(self, gcmd):
        changed = []
        
        mult = gcmd.get_float('MULT', None)
        if mult is not None and mult >= 1.0:
            self.anomaly_threshold_multiplier = mult
            changed.append(f"MULT={mult}")
        
        min_dev = gcmd.get_int('MIN_DEV', None)
        if min_dev is not None and min_dev >= 1000:
            self.min_anomaly_deviation = min_dev
            changed.append(f"MIN_DEV={min_dev}")
        
        learning_dev = gcmd.get_float('LEARNING_DEV', None)
        if learning_dev is not None and 0.05 <= learning_dev <= 0.5:
            self.learning_deviation_threshold = learning_dev
            changed.append(f"LEARNING_DEV={learning_dev}")
        
        gcmd.respond_info(f"Updated: {', '.join(changed) if changed else 'None'}")

    def cmd_TENSION_DEBUG(self, gcmd):
        gcmd.respond_info("=== TENSION SENSOR DEBUG ===")
        gcmd.respond_info(f"State: {self.current_state}")
        gcmd.respond_info(f"Jamming Triggered: {self.jamming_triggered}")
        gcmd.respond_info(f"Learning: {self.is_learning} ({self.learning_progress}/{self.learning_window})")
        
        if self.baseline_mean:
            threshold = self.calculate_dynamic_threshold()
            gcmd.respond_info(f"Current Baseline: {self.baseline_mean:.0f} ± {self.baseline_std:.0f}")
            gcmd.respond_info(f"Threshold: {threshold:.0f} (mult={self.anomaly_threshold_multiplier}, min={self.min_anomaly_deviation})")
        
        gcmd.respond_info(f"Anomaly counter: {self.anomaly_counter}/{self.anomaly_confirm_count}")
        
        if len(self.detection_buffer) > 0:
            recent = list(self.detection_buffer)[-10:]
            gcmd.respond_info(f"Recent values: min={min(recent)}, max={max(recent)}, avg={sum(recent)/len(recent):.0f}")
        
        if self.initial_baseline:
            age = time.time() - self.initial_baseline['timestamp']
            deviation = abs(self.baseline_mean - self.initial_baseline['mean']) / abs(self.initial_baseline['mean']) if self.baseline_mean else 0
            gcmd.respond_info(f"Reference Baseline: {self.initial_baseline['mean']:.0f} (age: {age:.0f}s)")
            gcmd.respond_info(f"Deviation from reference: {deviation:.2%}")
        else:
            gcmd.respond_info("Reference Baseline: Not set")
        
        gcmd.respond_info(f"Learning history entries: {len(self.learning_history)}")
        gcmd.respond_info(f"Consecutive deviant learnings: {self.consecutive_deviant_learnings}/{self.learning_jamming_threshold}")

    def cmd_SHOW_BASELINE(self, gcmd):
        if not self.saved_baseline:
            gcmd.respond_info("No saved baseline")
            return
        
        age = time.time() - self.saved_baseline['timestamp']
        age_str = f"{age:.0f}s ago" if age < 3600 else f"{age/3600:.1f}h ago"
        valid = "VALID" if age < self.baseline_max_age else "EXPIRED"
        
        gcmd.respond_info("=== Saved Baseline ===")
        gcmd.respond_info(f"Mean: {self.saved_baseline['mean']:.0f}")
        gcmd.respond_info(f"Std: {self.saved_baseline['std']:.0f}")
        gcmd.respond_info(f"Threshold: {self.saved_baseline.get('threshold', 'N/A'):.0f}")
        gcmd.respond_info(f"Age: {age_str} [{valid}]")

    def cmd_RESET_LEARNING(self, gcmd):
        self.is_learning = True
        self.learning_progress = 0
        self.baseline_buffer.clear()
        self.detection_buffer.clear()
        self.baseline_mean = None
        self.baseline_std = None
        self.anomaly_counter = 0
        self.jamming_triggered = False
        self.current_state = "NORMAL"
        self.saved_baseline = None
        gcmd.respond_info(f"Learning reset. Need {self.learning_window*self.sample_interval:.1f}s to establish baseline.")

    def cmd_CLEAR_POLLING_LOG(self, gcmd):
        try:
            # 清除所有缓冲区
            self.baseline_buffer.clear()
            self.detection_buffer.clear()
            self.value_history.clear()
            
            # 清除所有日志
            self.calibration_log.clear()
            self.polling_log.clear()
            self.learning_history.clear()
            
            # 清除所有保存的数据
            self.saved_baseline = None
            self.initial_baseline = None
            
            # 重置状态
            self.baseline_mean = None
            self.baseline_std = None
            self.current_state = "NORMAL"
            self.is_learning = True
            self.learning_progress = 0
            self.jamming_triggered = False
            self.anomaly_counter = 0
            self.consecutive_deviant_learnings = 0
            
            # 删除并重新创建空的JSON文件
            try:
                if os.path.exists(self.calibration_file):
                    os.remove(self.calibration_file)
                    gcmd.respond_info(f"已删除文件: {self.calibration_file}")
                
                # 创建新的空文件
                os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)
                with open(self.calibration_file, 'w') as f:
                    json.dump({
                        'calibration_log': [],
                        'polling_log': [],
                        'saved_baseline': None,
                        'learning_history': [],
                        'initial_baseline': None
                    }, f, indent=2)
                
                gcmd.respond_info("已创建新的空配置文件")
                
            except Exception as e:
                gcmd.respond_info(f"警告: 无法更新文件 - {e}")
            
            gcmd.respond_info("=" * 50)
            gcmd.respond_info("所有日志和历史数据已清除")
            gcmd.respond_info("系统已重置为初始学习状态")
            gcmd.respond_info(f"需要采集 {self.learning_window} 个样本建立基线")
            gcmd.respond_info("=" * 50)
            
        except Exception as e:
            gcmd.respond_info(f"错误: 清除日志失败 - {e}")

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
        gcmd.respond_info(f"Learning: {self.is_learning}")
        
        if self.baseline_mean:
            threshold = self.calculate_dynamic_threshold()
            dev = abs(val - self.baseline_mean)
            rel_pct = (dev / abs(self.baseline_mean) * 100) if self.baseline_mean != 0 else 0
            gcmd.respond_info(f"Current Baseline: {self.baseline_mean:.0f} | Threshold: {threshold:.0f} | "
                            f"Dev: {dev:.0f} ({rel_pct:.2f}%)")
        
        if self.saved_baseline:
            age = time.time() - self.saved_baseline['timestamp']
            gcmd.respond_info(f"Saved baseline age: {age:.0f}s")
        
        if self.initial_baseline:
            age = time.time() - self.initial_baseline['timestamp']
            gcmd.respond_info(f"Reference baseline: {self.initial_baseline['mean']:.0f} (age: {age:.0f}s)")
            
            if self.baseline_mean:
                ref_dev = abs(self.baseline_mean - self.initial_baseline['mean']) / abs(self.initial_baseline['mean'])
                gcmd.respond_info(f"Deviation from reference: {ref_dev:.2%}")
        
        gcmd.respond_info(f"Learning history: {len(self.learning_history)} entries")
        gcmd.respond_info(f"Consecutive deviant: {self.consecutive_deviant_learnings}/{self.learning_jamming_threshold}")

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
            if self.baseline_mean:
                self.save_current_baseline()
            
            self.stop_polling()
            gcmd.respond_info("Polling OFF")

    def cmd_SET_REFERENCE(self, gcmd):
        clear = gcmd.get_int('CLEAR', 0)
        
        if clear:
            self.initial_baseline = None
            self.consecutive_deviant_learnings = 0
            gcmd.respond_info("Reference baseline cleared")
            self._save_history()
            return
        
        if self.baseline_mean is None:
            gcmd.respond_info("No current baseline to set as reference. Please complete learning first.")
            return
        
        self.initial_baseline = {
            'mean': self.baseline_mean,
            'std': self.baseline_std,
            'timestamp': time.time()
        }
        self.consecutive_deviant_learnings = 0
        
        gcmd.respond_info(f"Reference baseline set to: mean={self.baseline_mean:.0f}, std={self.baseline_std:.0f}")
        gcmd.respond_info(f"All future learnings will be compared to this reference (threshold: {self.learning_deviation_threshold:.1%})")
        
        self._save_history()

    def cmd_SHOW_REFERENCE(self, gcmd):
        if self.initial_baseline is None:
            gcmd.respond_info("No reference baseline set")
            return
        
        age = time.time() - self.initial_baseline['timestamp']
        age_str = f"{age:.0f}s ago" if age < 3600 else f"{age/3600:.1f}h ago"
        
        gcmd.respond_info("=== Reference Baseline ===")
        gcmd.respond_info(f"Mean: {self.initial_baseline['mean']:.0f}")
        gcmd.respond_info(f"Std: {self.initial_baseline['std']:.0f}")
        gcmd.respond_info(f"Set: {age_str}")
        gcmd.respond_info(f"Deviation threshold: {self.learning_deviation_threshold:.1%}")
        
        if self.baseline_mean:
            deviation = abs(self.baseline_mean - self.initial_baseline['mean']) / abs(self.initial_baseline['mean'])
            status = "WITHIN LIMITS" if deviation <= self.learning_deviation_threshold else "EXCEEDED"
            gcmd.respond_info(f"Current deviation: {deviation:.2%} [{status}]")

    def cmd_SHOW_LEARNING_HISTORY(self, gcmd):
        if not self.learning_history:
            gcmd.respond_info("No learning history")
            return
        
        gcmd.respond_info("=== Learning History ===")
        for i, entry in enumerate(self.learning_history[-10:], 1):
            ts = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(entry['timestamp']))
            
            dev_str = ""
            if self.initial_baseline:
                dev = abs(entry['mean'] - self.initial_baseline['mean']) / abs(self.initial_baseline['mean'])
                dev_str = f" | dev={dev:.2%}"
            
            gcmd.respond_info(f"{i:2d}. {ts} | mean={entry['mean']:8.0f} | std={entry['std']:6.0f}{dev_str}")
        
        if self.initial_baseline:
            gcmd.respond_info(f"\nReference baseline: mean={self.initial_baseline['mean']:.0f}")
            gcmd.respond_info(f"Deviation threshold: {self.learning_deviation_threshold:.1%}")

    def start_polling(self):
        if self.polling_enabled:
            return
        
        if self.load_baseline():
            self.is_learning = False
            self.baseline_buffer.clear()
            self.detection_buffer.clear()
            logging.info(f"Using saved baseline, skipping learning")
        else:
            self.is_learning = True
            self.baseline_buffer.clear()
            self.detection_buffer.clear()
            self.baseline_mean = None
            self.baseline_std = None
            logging.info(f"Starting learning mode, need {self.learning_window*self.sample_interval:.1f}s")
        
        self.value_history.clear()
        self.anomaly_counter = 0
        self.jamming_triggered = False
        self.current_state = "NORMAL"
        
        self.polling_enabled = True
        self.polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
        self.polling_thread.start()
        
        mode = "learning" if self.is_learning else "detecting"
        logging.info(f"Polling started: {self.sample_interval*1000:.0f}ms, mode={mode}")

    def stop_polling(self):
        if not self.polling_enabled:
            return
        
        self.polling_enabled = False
        if hasattr(self, 'polling_thread'):
            self.polling_thread.join(timeout=2)
        
        if self.polling_log_enabled and self.polling_log:
            self._save_history()
        
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
                
                if self.polling_log_enabled and len(self.polling_log) % 120 == 0:
                    self._save_history()
                
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