import logging
import threading
import time
import os
import json
from pathlib import Path
import collections

class MySerial:
    def __init__(self, config):
        self.spi_device = "/dev/spidev0.0"
        self.calibration_file = "/home/anisoprint/force_sensor_calibration.json"
        
        # Voltage conversion parameters
        self.VREF = 3.3  # Reference voltage 3.3V
        self.GAIN = 128  # Gain 128
        self.ADC_MAX = 2**23 - 1  # 24-bit ADC maximum value (8388607)
        
        self.window_size = 10
        self.std_threshold = 500
        self.anomaly_confirm_count = 5
        self.recovery_confirm_count = 40
        self.value_history = collections.deque(maxlen = self.window_size)
        self.anomaly_counter = 0
        self.recovery_counter = 0
        self.current_state = "NORMAL"

        self.calibration_log = []
        self.polling_log = []
        
        self.polling_enabled = False
        self.polling_log_enabled = False

        # Load calibration data
        self.zero_calibration_value = 0  # Zero point calibration value (no material)
        self.weight_calibration_value = 0  # Weight calibration value (with material)
        self._load_calibration()
        
        printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        self.reactor = printer.get_reactor()
        
        self.gcode.register_command(
            'TENSION_SENSOR_POLL',
            self.cmd_FORCE_SENSOR_POLL,
            desc="Enable or disable force sensor polling: FORCE_SENSOR_POLL ENABLE=<0|1> [LOGGING=<0|1>]"
        )

        self.gcode.register_command(
            'TENSION_SENSOR_STATUS',
            self.cmd_FORCE_SENSOR_STATUS,
            desc="Get current block status: FORCE_SENSOR_STATUS"
        )

        self.gcode.register_command(
            'TENSION_ZERO_CALIBRATION',
            self.cmd_ZERO_CALIBRATION,
            desc="Force sensor zero calibration (no material): ZERO_CALIBRATION"
        )

        self.gcode.register_command(
            'TENSION_WEIGHT_CALIBRATION',
            self.cmd_WEIGHT_CALIBRATION,
            desc="Force sensor weight calibration (with material): WEIGHT_CALIBRATION"
        )

        self.gcode.register_command(
            'SHOW_CALIBRATION',
            self.cmd_SHOW_CALIBRATION,
            desc="Show current calibration values: SHOW_CALIBRATION"
        )
        
        # Add new command to get current value
        self.gcode.register_command(
            'GET_FORCE_VALUE',
            self.cmd_GET_FORCE_VALUE,
            desc="Get current force sensor value: GET_FORCE_VALUE"
        )
        
        # Add new command to get raw unprocessed data
        self.gcode.register_command(
            'GET_RAW_FORCE_DATA',
            self.cmd_GET_RAW_FORCE_DATA,
            desc="Get raw unprocessed force sensor data: GET_RAW_FORCE_DATA"
        )
        
        # Add new command to clear polling log
        self.gcode.register_command(
            'CLEAR_POLLING_LOG',
            self.cmd_CLEAR_POLLING_LOG,
            desc="Clear polling log: CLEAR_POLLING_LOG"
        )
        
        # Add new command to show polling log
        self.gcode.register_command(
            'SHOW_POLLING_LOG',
            self.cmd_SHOW_POLLING_LOG,
            desc="Show polling log: SHOW_POLLING_LOG [COUNT=<number>]"
        )

    def adc_to_voltage(self, adc_value):
        voltage = (adc_value / self.ADC_MAX) * (self.VREF / self.GAIN)
        return voltage

    def parse_spi_data(self, data_bytes):
        """Parse SPI data"""
        if len(data_bytes) != 3:
            raise ValueError("Data length error")
        value = (data_bytes[0] << 16) | (data_bytes[1] << 8) | data_bytes[2]
        if value & 0x800000:
            value = value - (1 << 24)
        return value

    def read_spi_once(self):
        """Read SPI data once"""
        try:
            with open(self.spi_device, 'rb') as f:
                data = f.read(3)
                if len(data) == 3:
                    value = self.parse_spi_data(data)
                    return value
                else:
                    logging.error(f"SPI data length error: {len(data)}")
                    return None
        except Exception as e:
            logging.error(f"SPI read failed: {e}")
            return None

    def read_raw_spi_bytes(self):
        """Read raw SPI bytes without parsing"""
        try:
            with open(self.spi_device, 'rb') as f:
                data = f.read(3)
                return data
        except Exception as e:
            logging.error(f"SPI read failed: {e}")
            return None

    def _load_calibration(self):
        """Load calibration data from file"""
        try:
            if os.path.exists(self.calibration_file):
                with open(self.calibration_file, 'r') as f:
                    calibration_data = json.load(f)
                    self.zero_calibration_value = calibration_data.get('zero_calibration', 0)
                    self.weight_calibration_value = calibration_data.get('weight_calibration', 0)
                    self.calibration_log = calibration_data.get('calibration_log', [])
                    self.polling_log = calibration_data.get('polling_log', [])
                logging.info(f"Calibration data loaded successfully")
                logging.info(f"Calibration log entries: {len(self.calibration_log)}")
                logging.info(f"Polling log entries: {len(self.polling_log)}")
            else:
                logging.info("Calibration file not found, using default values")
        except Exception as e:
            logging.error(f"Failed to load calibration data: {e}")

    def _save_calibration(self):
        """Save calibration data to file"""
        try:
            os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)

            calibration_data = {
                'zero_calibration': self.zero_calibration_value,
                'weight_calibration': self.weight_calibration_value,
                'calibration_log': self.calibration_log,
                'polling_log': self.polling_log
            }
            with open(self.calibration_file, 'w') as f:
                json.dump(calibration_data, f, indent=4)
            logging.info(f"Calibration data saved successfully")
            logging.info(f"Calibration log entries: {len(self.calibration_log)}")
            logging.info(f"Polling log entries: {len(self.polling_log)}")
        except Exception as e:
            logging.error(f"Failed to save calibration data: {e}")

    def calculate_std(self, values):
        """Calculate standard deviation of a list of values"""
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return variance ** 0.5
    
    def update_detection_state(self, current_value):
        """Update detection state based on new value"""
        self.value_history.append(current_value)
        if len(self.value_history) < self.window_size:
            return False, "INIT", f"Collecting data: {len(self.value_history)}/{self.window_size}"
        
        current_std = self.calculate_std(self.value_history)
        window_range = max(self.value_history) - min(self.value_history)

        is_anomaly = (current_std > self.std_threshold) and (window_range > 500)

        state_changed = False
        details = f"STD={current_std:.2f}, Threshold={self.std_threshold}, Range={window_range}"

        if is_anomaly:
            self.anomaly_counter += 1
            self.recovery_counter = 0
            if self.current_state == "NORMAL" and self.anomaly_counter >= self.anomaly_confirm_count:
                self.current_state = "ANOMALY"
                state_changed = True
                details += f", ANOMALY CONFIRMED (count={self.anomaly_counter})"
            else:
                details += f", anomaly count={self.anomaly_counter}"
        else:
            self.recovery_counter += 1
            self.anomaly_counter = 0
            if self.current_state == "ANOMALY" and self.recovery_counter >= self.recovery_confirm_count:
                self.current_state = "NORMAL"
                state_changed = True
                details += f", RECOVERY CONFIRMED (count={self.recovery_counter})"
            else:
                details += f", recovery count={self.recovery_counter}"
            
        return state_changed, self.current_state, details

    def cmd_GET_RAW_FORCE_DATA(self, gcmd):
        """Get raw unprocessed force sensor data - single reading"""
        raw_bytes = self.read_raw_spi_bytes()
        if raw_bytes is None:
            gcmd.respond_info("ERROR: Unable to read from SPI device")
            return
            
        hex_bytes = ' '.join([f'{b:02X}' for b in raw_bytes])
        dec_bytes = ' '.join([f'{b:3d}' for b in raw_bytes])
        
        try:
            parsed_value = self.parse_spi_data(raw_bytes)
            voltage_value = self.adc_to_voltage(parsed_value)
            value_info = f"{parsed_value} (Voltage: {voltage_value:.6f}V / {voltage_value*1000:.3f}mV)"
        except Exception as e:
            value_info = f"PARSE ERROR: {e}"
        
        gcmd.respond_info(f"Raw HEX: {hex_bytes}")
        gcmd.respond_info(f"Raw DEC: {dec_bytes}")
        gcmd.respond_info(f"Parsed value: {value_info}")
        gcmd.respond_info(f"Conversion parameters: VREF={self.VREF}V, GAIN={self.GAIN}, ADC_MAX={self.ADC_MAX}")

    def cmd_GET_FORCE_VALUE(self, gcmd):
        """Get current force sensor value"""
        current_value = self.read_spi_once()
        if current_value is None:
            gcmd.respond_info("Unable to read force sensor value")
            return
        
        voltage_value = self.adc_to_voltage(current_value)
        
        gcmd.respond_info(f"Current force sensor value: {current_value}")
        gcmd.respond_info(f"Voltage: {voltage_value:.6f}V ({voltage_value*1000:.3f}mV)")
        
        if self.zero_calibration_value > 0:
            net_weight = current_value - self.zero_calibration_value
            net_voltage = self.adc_to_voltage(net_weight)
            gcmd.respond_info(f"Net weight (current - zero): {net_weight}")
            gcmd.respond_info(f"Net voltage: {net_voltage:.6f}V ({net_voltage*1000:.3f}mV)")

    def cmd_SHOW_CALIBRATION(self, gcmd):
        """Show current calibration values"""
        gcmd.respond_info(f"Zero calibration value: {self.zero_calibration_value}")
        if self.zero_calibration_value > 0:
            zero_voltage = self.adc_to_voltage(self.zero_calibration_value)
            gcmd.respond_info(f"Zero calibration voltage: {zero_voltage:.6f}V ({zero_voltage*1000:.3f}mV)")
        
        gcmd.respond_info(f"Weight calibration value: {self.weight_calibration_value}")
        if self.weight_calibration_value > 0:
            weight_voltage = self.adc_to_voltage(self.weight_calibration_value)
            gcmd.respond_info(f"Weight calibration voltage: {weight_voltage:.6f}V ({weight_voltage*1000:.3f}mV)")
            
            if self.zero_calibration_value > 0:
                net_weight_value = self.weight_calibration_value - self.zero_calibration_value
                net_weight_voltage = self.adc_to_voltage(net_weight_value)
                gcmd.respond_info(f"Net weight value: {net_weight_value}")
                gcmd.respond_info(f"Net weight voltage: {net_weight_voltage:.6f}V ({net_weight_voltage*1000:.3f}mV)")
            
            threshold = self.weight_calibration_value * 0.2
            threshold_voltage = self.adc_to_voltage(threshold)
            gcmd.respond_info(f"Alert threshold: {threshold} (Voltage: {threshold_voltage:.6f}V)")
        
        gcmd.respond_info(f"Calibration log entries: {len(self.calibration_log)}")
        gcmd.respond_info(f"Polling log entries: {len(self.polling_log)}")

    def cmd_SET_THRESHOLD(self, gcmd):
        """Set detection threshold parameters"""
        value = gcmd.get_int('VALUE', None)
        if value is None or value <= 0:
            gcmd.respond_info("Invalid VALUE parameter")
            return
        
        self.std_threshold = value
        gcmd.respond_info(f"Detection standard deviation threshold set to: {self.std_threshold}")
        gcmd.respond_info(f"Window size: {self.window_size}, Anomaly confirm count: {self.anomaly_confirm_count}, Recovery confirm count: {self.recovery_confirm_count}")

    def cmd_DEBUG_STATS(self, gcmd):
        """Get current detection statistics"""
        gcmd.respond_info("=== Tension Sensor Detection Stats ===")
        gcmd.respond_info(f"Current state: {self.current_state}")
        gcmd.respond_info(f"Window: {len(self.value_history)}/{self.window_size}")
        gcmd.respond_info(f"Threshold: {self.std_threshold}  (baseline 5*15200)")
        gcmd.respond_info(f"Anomaly: {self.anomaly_counter}/{self.anomaly_confirm_count}")
        gcmd.respond_info(f"Recovery: {self.recovery_counter}/{self.recovery_confirm_count}")
        if len(self.value_history) == self.window_size:
            std = self.calculate_std(self.value_history)
            r = max(self.value_history) - min(self.value_history)
            gcmd.respond_info(f"Real-time std={std:.0f}  range={r:.0f}")
        
        if len(self.value_history) > 0:
            current_std = self.calculate_std(self.value_history)
            mean_val = sum(self.value_history) / len(self.value_history)
            min_val = min(self.value_history)
            max_val = max(self.value_history)
            gcmd.respond_info(f"Current window stats: mean={mean_val:.2f}, std={current_std:.2f}, range=[{min_val}, {max_val}]")
    
            recent = list(self.value_history)[-5:]
            gcmd.respond_info(f"Recent 5 values: {recent}")

    def cmd_CLEAR_POLLING_LOG(self, gcmd):
        """Clear polling log"""
        self.polling_log.clear()
        self._save_calibration()
        gcmd.respond_info("Polling log cleared successfully")

    def cmd_SHOW_POLLING_LOG(self, gcmd):
        """Show polling log"""
        count = gcmd.get_int('COUNT', 20)
        
        if not self.polling_log:
            gcmd.respond_info("Polling log is empty")
            return
        
        gcmd.respond_info(f"=== Polling Log (showing last {min(count, len(self.polling_log))} entries) ===")
        
        start_idx = max(0, len(self.polling_log) - count)
        for i, entry in enumerate(self.polling_log[start_idx:], start_idx + 1):
            timestamp = entry.get('timestamp', 'N/A')
            value = entry.get('value', 'N/A')
            voltage = entry.get('voltage', 'N/A')
            
            gcmd.respond_info(f"{i:4d}. Time: {timestamp}, Value: {value}, Voltage: {voltage:.6f}V ({voltage*1000:.3f}mV)")
        
        gcmd.respond_info(f"Total entries: {len(self.polling_log)}")

    def _take_readings(self, count, interval, gcmd):
        """Take multiple readings with specified interval"""
        readings = []
        for i in range(count):
            current_reading = self.read_spi_once()
            if current_reading is None:
                gcmd.respond_info(f"Failed to read SPI data at reading {i+1}, calibration aborted")
                return None
            
            readings.append(current_reading)
            gcmd.respond_info(f"Reading {i+1}: {current_reading}")
            
            if i < count - 1:
                time.sleep(interval)
        return readings
    
    def _process_readings(self, readings):
        """Process readings by removing highest and lowest, then averaging"""
        if len(readings) >= 3:
            sorted_readings = sorted(readings)
            trimmed_readings = sorted_readings[1:-1]
            return sum(trimmed_readings) / len(trimmed_readings)
        else:
            return sum(readings) / len(readings)

    def cmd_WEIGHT_CALIBRATION(self, gcmd):
        """Weight calibration - record current value as weight with material"""
        readings = []
        for i in range(10):
            current_reading = self.read_spi_once()
            if current_reading is None:
                gcmd.respond_info(f"Failed to read SPI data at reading {i+1}, calibration aborted")
                return
            
            readings.append(current_reading)
            gcmd.respond_info(f"Reading {i+1}: {current_reading}")
            
            if i < 9:
                time.sleep(2)

        if len(readings) >= 3:
            sorted_readings = sorted(readings)
            trimmed_readings = sorted_readings[1:-1]
            current_value = sum(trimmed_readings) / len(trimmed_readings)
        else:
            current_value = sum(readings) / len(readings)
        
        self.weight_calibration_value = current_value
        
        calibration_entry = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'type': 'weight_calibration',
            'value': current_value,
            'voltage': self.adc_to_voltage(current_value)
        }
        self.calibration_log.append(calibration_entry)
        
        self._save_calibration()
        
        voltage_value = self.adc_to_voltage(current_value)
        gcmd.respond_info(f"Weight calibration completed: {current_value:.2f}")
        gcmd.respond_info(f"Voltage: {voltage_value:.6f}V ({voltage_value*1000:.3f}mV)")
        
        if self.zero_calibration_value > 0:
            net_weight = current_value - self.zero_calibration_value
            net_voltage = self.adc_to_voltage(net_weight)
            gcmd.respond_info(f"Net weight: {net_weight}, Net voltage: {net_voltage:.6f}V")

    def cmd_ZERO_CALIBRATION(self, gcmd):
        """Zero calibration - record current value as zero point"""
        readings = []
        for i in range(10):
            current_reading = self.read_spi_once()
            if current_reading is None:
                gcmd.respond_info(f"Failed to read SPI data at reading {i+1}, calibration aborted")
                return
            
            readings.append(current_reading)
            gcmd.respond_info(f"Reading {i+1}: {current_reading}")

            if i < 9:
                time.sleep(2)

        if len(readings) >= 3:
            sorted_readings = sorted(readings)
            trimmed_readings = sorted_readings[1:-1]
            current_value = sum(trimmed_readings) / len(trimmed_readings)
        else:
            current_value = sum(readings) / len(readings)
        
        self.zero_calibration_value = current_value
        
        calibration_entry = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'type': 'zero_calibration',
            'value': current_value,
            'voltage': self.adc_to_voltage(current_value)
        }
        self.calibration_log.append(calibration_entry)
        
        self._save_calibration()
        
        voltage_value = self.adc_to_voltage(current_value)
        gcmd.respond_info(f"Zero calibration completed: {current_value:.2f}")
        gcmd.respond_info(f"Voltage: {voltage_value:.6f}V ({voltage_value*1000:.3f}mV)")

    def cmd_FORCE_SENSOR_STATUS(self, gcmd):
        """Get sensor status"""
        current_value = self.read_spi_once()
        if current_value is None:
            gcmd.respond_info("Unable to read sensor data")
            return
        
        voltage_value = self.adc_to_voltage(current_value)
            
        gcmd.respond_info(f"Current value: {current_value}")
        gcmd.respond_info(f"Current voltage: {voltage_value:.6f}V ({voltage_value*1000:.3f}mV)")
        gcmd.respond_info(f"Zero calibration: {self.zero_calibration_value}")
        gcmd.respond_info(f"Weight calibration: {self.weight_calibration_value}")
        
        gcmd.respond_info(f"Detection state: {self.current_state}")
        gcmd.respond_info(f"Polling enabled: {self.polling_enabled}")
        gcmd.respond_info(f"Polling log enabled: {self.polling_log_enabled}")
        gcmd.respond_info(f"Anomaly counter: {self.anomaly_counter}/{self.anomaly_confirm_count}")
        gcmd.respond_info(f"Recovery counter: {self.recovery_counter}/{self.recovery_confirm_count}")
        
        if len(self.value_history) == self.window_size:
            current_std = self.calculate_std(self.value_history)
            r = max(self.value_history) - min(self.value_history)
            gcmd.respond_info(f"Current window stats: range={r:.0f}, std={current_std:.2f}, threshold: {self.std_threshold}")
        
        gcmd.respond_info(f"Polling log entries: {len(self.polling_log)}")

    def cmd_FORCE_SENSOR_POLL(self, gcmd):
        """Enable or disable polling detection"""
        enable = gcmd.get_int('ENABLE', None)
        logging_enable = gcmd.get_int('LOGGING', 0)
        
        if enable is None:
            gcmd.respond_info("Missing ENABLE parameter (0 or 1)")
            return
            
        if enable:
            self.polling_log_enabled = (logging_enable == 1)
            
            if self.polling_log_enabled:
                self.polling_log.clear()
            
            self.start_polling()
            
            if self.polling_log_enabled:
                gcmd.respond_info("Force sensor polling enabled WITH logging")
                logging.info("Polling started with logging enabled")
            else:
                gcmd.respond_info("Force sensor polling enabled WITHOUT logging")
                logging.info("Polling started without logging")
        else:
            self.stop_polling()
            gcmd.respond_info("Force sensor polling disabled")

    def _send_anomaly_warning(self, details):
        try:
            message = f"M118 TENSION ANOMALY DETECTED! {details}"
            self.gcode.run_script_from_command(message)
            self.gcode.run_script_from_command("FIBER_FILAMENT_TRIGGER")
            logging.info(f"Anomaly warning sent: {details}")
        except Exception as e:
            logging.error(f"Failed to send anomaly warning: {e}")

    def _send_recovery_notice(self, details):
        try:
            message = f"M118 TENSION RECOVERED! {details}"
            self.gcode.run_script_from_command(message)
            logging.info(f"Recovery notice sent: {details}")
        except Exception as e:
            logging.error(f"Failed to send recovery notice: {e}")

    def start_polling(self):
        """Start polling detection"""
        if self.polling_enabled:
            logging.info("Polling already enabled")
            return
            
        self.polling_enabled = True
        self.value_history.clear()
        self.anomaly_counter = 0
        self.recovery_counter = 0
        self.current_state = "NORMAL"
        
        # Check calibration status
        if self.weight_calibration_value <= 0:
            logging.warning("Weight calibration value not set, polling may be inaccurate")
        
        if hasattr(self, 'polling_thread') and self.polling_thread.is_alive():
            self.polling_enabled = False
            self.polling_thread.join(timeout=2)
        
        self.polling_enabled = True
        self.polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
        self.polling_thread.start()
        
        if self.polling_log_enabled:
            logging.info(f"Force sensor polling started WITH logging")
        else:
            logging.info(f"Force sensor polling started WITHOUT logging")
        
        logging.info(f"Window size: {self.window_size}, Threshold: {self.std_threshold}")

    def stop_polling(self):
        """Stop polling detection"""
        if not self.polling_enabled:
            logging.info("Polling already disabled")
            return
            
        logging.info("Stopping polling...")
        self.polling_enabled = False
        
        if hasattr(self, 'polling_thread') and self.polling_thread.is_alive():
            logging.info("Waiting for polling thread to finish...")
            self.polling_thread.join(timeout=2)
            if self.polling_thread.is_alive():
                logging.warning("Polling thread did not stop properly")
            else:
                logging.info("Polling thread stopped successfully")
        
        if self.polling_log_enabled and self.polling_log:
            self._save_calibration()
            logging.info(f"Force sensor polling stopped and data saved (log entries: {len(self.polling_log)})")
        else:
            logging.info(f"Force sensor polling stopped (no logging or empty log)")
        
        self.polling_log_enabled = False

    def _polling_loop(self):
        """Polling detection main loop with logging"""
        if self.polling_log_enabled:
            logging.info("Polling loop started - logging enabled")
        else:
            logging.info("Polling loop started - logging disabled")
        
        try:
            while self.polling_enabled:
                current_value = self.read_spi_once()
                
                if current_value is None:
                    logging.error("SPI read failed")
                    time.sleep(2)
                    continue
                
                if current_value < 1000 or current_value > 8380000:
                    logging.warning(f"Invalid data detected: {current_value}")
                    time.sleep(2)
                    continue
                
                if self.polling_log_enabled:
                    polling_entry = {
                        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                        'value': current_value,
                        'voltage': self.adc_to_voltage(current_value)
                    }
                    self.polling_log.append(polling_entry)
                    
                    if len(self.polling_log) % 10 == 0:
                        logging.info(f"Polling - Logged value {len(self.polling_log)}: {current_value} (Voltage: {self.adc_to_voltage(current_value):.6f}V)")
                
                state_changed, new_state, details = self.update_detection_state(current_value)
                
                if self.polling_log_enabled and len(self.polling_log) % 30 == 0 and len(self.polling_log) > 0:
                    self._save_calibration()
                    logging.info(f"Auto-saved polling log to file (total entries: {len(self.polling_log)})")
                
                if state_changed:
                    logging.info(f"State changed to: {new_state}, {details}")
                    if new_state == "ANOMALY":
                        self.reactor.register_callback(
                            lambda event: self._send_anomaly_warning(details))
                    elif new_state == "NORMAL":
                        self.reactor.register_callback(
                            lambda event: self._send_recovery_notice(details))
                
                time.sleep(2)
        except Exception as e:
            logging.error(f"Error in polling loop: {e}")
        finally:
            logging.info("Polling loop ended")

def load_config(config):
    return MySerial(config)