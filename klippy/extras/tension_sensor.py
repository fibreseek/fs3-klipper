import logging
import threading
import time
import os
import json
from pathlib import Path

class MySerial:
    def __init__(self, config):
        self.spi_device = "/dev/spidev0.0"
        self.calibration_file = "/home/anisoprint/force_sensor_calibration.json"
        
        # Voltage conversion parameters
        self.VREF = 3.3  # Reference voltage 3.3V
        self.GAIN = 128  # Gain 128
        self.ADC_MAX = 2**23 - 1  # 24-bit ADC maximum value (8388607)
        
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
            desc="Enable or disable force sensor polling: FORCE_SENSOR_POLL ENABLE=<0|1>"
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
                logging.info(f"Calibration data loaded successfully: zero={self.zero_calibration_value}, weight={self.weight_calibration_value}")
            else:
                logging.info("Calibration file not found, using default values")
        except Exception as e:
            logging.error(f"Failed to load calibration data: {e}")

    def _save_calibration(self):
        """Save calibration data to file"""
        try:
            calibration_data = {
                'zero_calibration': self.zero_calibration_value,
                'weight_calibration': self.weight_calibration_value
            }
            with open(self.calibration_file, 'w') as f:
                json.dump(calibration_data, f)
            logging.info(f"Calibration data saved successfully: zero={self.zero_calibration_value}, weight={self.weight_calibration_value}")
        except Exception as e:
            logging.error(f"Failed to save calibration data: {e}")

    def cmd_GET_RAW_FORCE_DATA(self, gcmd):
        """Get raw unprocessed force sensor data - single reading"""
        # Read raw bytes
        raw_bytes = self.read_raw_spi_bytes()
        if raw_bytes is None:
            gcmd.respond_info("ERROR: Unable to read from SPI device")
            return
            
        # Convert to different formats
        hex_bytes = ' '.join([f'{b:02X}' for b in raw_bytes])
        dec_bytes = ' '.join([f'{b:3d}' for b in raw_bytes])
        
        # Parse the value
        try:
            parsed_value = self.parse_spi_data(raw_bytes)
            voltage_value = self.adc_to_voltage(parsed_value)
            value_info = f"{parsed_value} (Voltage: {voltage_value:.6f}V / {voltage_value*1000:.3f}mV)"
        except Exception as e:
            value_info = f"PARSE ERROR: {e}"
        
        # Output the result
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
        
        # Convert to voltage value
        voltage_value = self.adc_to_voltage(current_value)
        
        gcmd.respond_info(f"Current force sensor value: {current_value}")
        gcmd.respond_info(f"Voltage: {voltage_value:.6f}V ({voltage_value*1000:.3f}mV)")
        
        # Also show net weight if zero calibration is set
        if self.zero_calibration_value > 0:
            net_weight = current_value - self.zero_calibration_value
            # Calculate voltage corresponding to net weight
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
            
            # Calculate net weight voltage (weight calibration value - zero calibration value)
            if self.zero_calibration_value > 0:
                net_weight_value = self.weight_calibration_value - self.zero_calibration_value
                net_weight_voltage = self.adc_to_voltage(net_weight_value)
                gcmd.respond_info(f"Net weight value: {net_weight_value}")
                gcmd.respond_info(f"Net weight voltage: {net_weight_voltage:.6f}V ({net_weight_voltage*1000:.3f}mV)")
            
            threshold = self.weight_calibration_value * 0.2
            threshold_voltage = self.adc_to_voltage(threshold)
            gcmd.respond_info(f"Alert threshold: {threshold} (Voltage: {threshold_voltage:.6f}V)")

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
                time.sleep(1)

        if len(readings) >= 3:
            sorted_readings = sorted(readings)
            trimmed_readings = sorted_readings[1:-1]
            current_value = sum(trimmed_readings) / len(trimmed_readings)
        else:
            current_value = sum(readings) / len(readings)
        
        self.weight_calibration_value = current_value
        self._save_calibration()
        
        voltage_value = self.adc_to_voltage(current_value)
        gcmd.respond_info(f"Weight calibration completed: {current_value:.2f}")
        gcmd.respond_info(f"Voltage: {voltage_value:.6f}V ({voltage_value*1000:.3f}mV)")
        
        if self.zero_calibration_value > 0:
            net_weight = current_value - self.zero_calibration_value
            net_voltage = self.adc_to_voltage(net_weight)

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
                time.sleep(1)

        if len(readings) >= 3:
            sorted_readings = sorted(readings)
            trimmed_readings = sorted_readings[1:-1]
            current_value = sum(trimmed_readings) / len(trimmed_readings)
        else:
            current_value = sum(readings) / len(readings)
        
        self.zero_calibration_value = current_value
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
        
        # Display material blockage detection related information
        if hasattr(self, 'weight_before_drop') and self.weight_before_drop is not None:
            gcmd.respond_info(f"Weight before drop: {self.weight_before_drop}")
        if hasattr(self, 'block_count'):
            gcmd.respond_info(f"Block count: {self.block_count}")
        if hasattr(self, 'block_status'):
            gcmd.respond_info(f"Block status: {'BLOCKED' if self.block_status else 'NORMAL'}")

    def cmd_FORCE_SENSOR_POLL(self, gcmd):
        """Enable or disable polling detection"""
        enable = gcmd.get_int('ENABLE', None)
        if enable is None:
            gcmd.respond_info("Missing ENABLE parameter (0 or 1)")
            return
            
        if enable:
            self.start_polling()
            gcmd.respond_info("Force sensor polling enabled")
        else:
            self.stop_polling()
            gcmd.respond_info("Force sensor polling disabled")

    def _send_block_warning(self, weight_before_drop, current_weight):
        """Send block warning"""
        try:
            # Convert to voltage value for warning message
            voltage_before = self.adc_to_voltage(weight_before_drop)
            voltage_current = self.adc_to_voltage(current_weight)
            
            message = f"M118 DU TOU LA! Weight dropped from {weight_before_drop} ({voltage_before*1000:.2f}mV) to {current_weight} ({voltage_current*1000:.2f}mV)"
            self.gcode.run_script_from_command(message)
            self.gcode.run_script_from_command("FIBER_FILAMENT_TRIGGER")
            logging.info(f"Block warning sent: {message}")
        except Exception as e:
            logging.error(f"Failed to send block warning: {e}")

    def _send_weight_drop_warning(self, weight_before_drop, current_weight):
        """Send weight drop warning"""
        try:
            # Convert to voltage value for warning message
            voltage_before = self.adc_to_voltage(weight_before_drop)
            voltage_current = self.adc_to_voltage(current_weight)
            
            message = f"M118 Weight drop detected! From {weight_before_drop} ({voltage_before*1000:.2f}mV) to {current_weight} ({voltage_current*1000:.2f}mV)"
            self.gcode.run_script_from_command(message)
            logging.info(f"Weight drop warning sent: {message}")
        except Exception as e:
            logging.error(f"Failed to send weight drop warning: {e}")

    def start_polling(self):
        """Start polling detection"""
        if hasattr(self, 'polling_enabled') and self.polling_enabled:
            return
            
        self.polling_enabled = True
        self.last_weight = None
        self.weight_before_drop = None
        self.block_count = 0
        self.block_status = False
        self.block_warning_sent = False
        self.block_recover_count = 0
        
        # Check calibration status
        if self.weight_calibration_value <= 0:
            logging.warning("Weight calibration value not set, polling may be inaccurate")
            
        self.polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
        self.polling_thread.start()
        logging.info("Force sensor polling started with block detection")

    def stop_polling(self):
        """Stop polling detection"""
        self.polling_enabled = False
        logging.info("Force sensor polling stopped")

    def _polling_loop(self):
        """Polling detection main loop with block detection based on weight drop"""
        while getattr(self, 'polling_enabled', False):
            current_value = self.read_spi_once()
            
            if current_value is not None:
                if current_value < 1000 or current_value > 8380000:
                    logging.warning(f"Invalid data detected: {current_value}, valid range: 1000 to 8380000")
                    time.sleep(1)
                    continue
                
                if not hasattr(self, 'weight_before_drop'):
                    self.weight_before_drop = None
                    
                # Save previous weight value for comparison
                previous_weight = self.last_weight
                self.last_weight = current_value
                
                if previous_weight is not None:
                    diff = previous_weight - current_value
                    
                    # Detect weight drop
                    if diff >= 3900:
                        if self.weight_before_drop is None:
                            self.weight_before_drop = previous_weight
                            self.reactor.register_callback(
                                lambda event: self._send_weight_drop_warning(
                                    self.weight_before_drop, current_value))
                        
                    # If weight before drop has been recorded, check total drop amount
                    if self.weight_before_drop is not None:
                        drop_diff = self.weight_before_drop - current_value
                        if drop_diff >= 3900:
                            self.block_count += 1
                            if self.block_count >= 5 and not self.block_status:
                                self.block_status = True
                                self.block_warning_sent = False
                        else:
                            self.weight_before_drop = None
                            self.block_count = 0
                            self.block_status = False
                            self.block_warning_sent = False
                
                # Blockage detected and warning not sent
                if self.block_status and not self.block_warning_sent:
                    self.reactor.register_callback(
                        lambda event: self._send_block_warning(
                            self.weight_before_drop, current_value))
                    self.block_warning_sent = True 

                # Blockage recovery detection
                if self.block_status and previous_weight is not None:
                    diff_recover = abs(previous_weight - current_value)
                    if diff_recover < 500:
                        self.block_recover_count += 1
                        if self.block_recover_count >= 5:
                            self.block_status = False
                            self.block_count = 0
                            self.weight_before_drop = None
                            self.block_recover_count = 0
                            self.block_warning_sent = False
                            logging.info("Block status recovered")
                    else:
                        self.block_recover_count = 0
                        
            elif current_value is None:
                logging.error("SPI read failed")
                
            time.sleep(1)

def load_config(config):
    return MySerial(config)