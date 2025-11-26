import logging
import serial
import threading
import time
from serial import SerialException

class MySerial:
    def __init__(self, config):
        self.serial_port = config.get("serial")
        self.baud = config.getint("baud", default=9600)
        self.serial_conn = None
        
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=0.1
            )
            logging.info(f"Serial port initialized: {self.serial_port} @ {self.baud} baud")
        except SerialException as e:
            logging.error(f"Serial connection failed: {e}")
            return
        
        printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        self.reactor = printer.get_reactor()
        
        self.gcode.register_command(
            'SEND_TENSION_COMMAND',
            self.cmd_SEND_TENSION_COMMAND,
            desc="Send tension sensor command: A3 00 A2 A4 A5"
        )

        self.gcode.register_command(
            'FORCE_SENSOR_POLL',
            self.cmd_FORCE_SENSOR_POLL,
            desc="Enable or disable force sensor polling: FORCE_SENSOR_POLL ENABLE=<0|1>"
        )

        self.gcode.register_command(
            'FORCE_SENSOR_STATUS',
            self.cmd_FORCE_SENSOR_STATUS,
            desc="Get current block status: FORCE_SENSOR_STATUS"
        )

        self.gcode.register_command(
            'ZERO_CALIBRATION',
            self.cmd_ZERO_CALIBRATION,
            desc="Force sensor zero calibration: ZERO_CALIBRATION"
        )

        self.gcode.register_command(
            'WEIGHT_CALIBRATION',
            self.cmd_WEIGHT_CALIBRATION,
            desc="Force sensor weight calibration: WEIGHT_CALIBRATION"
        )

    def cmd_WEIGHT_CALIBRATION(self, gcmd):
        """Send weight calibration command (1000g), no response required"""
        if not self.serial_conn or not self.serial_conn.is_open:
            gcmd.respond_info("Serial port is not available")
            return

        send_data = bytes([0xAD, 0x00, 0x03, 0xE8, 0x46])
        try:
            self.serial_conn.write(send_data)
            gcmd.respond_info("Weight calibration command sent: AD 00 03 E8 46 (1000g)")
        except SerialException as e:
            logging.error(f"Weight calibration send failed: {e}")
            gcmd.respond_info(f"Error sending weight calibration command: {e}")

    def cmd_ZERO_CALIBRATION(self, gcmd):
        """Send zero calibration command and check response"""
        if not self.serial_conn or not self.serial_conn.is_open:
            gcmd.respond_info("Serial port is not available")
            return

        # Send zero calibration command
        send_data = bytes([0xAA, 0x00, 0xA9, 0xAB, 0xA8])
        try:
            self.serial_conn.write(send_data)
            gcmd.respond_info("Zero calibration command sent: AA 00 A9 AB A8")
            try:
                response = self.serial_conn.read(10)
                if response:
                    gcmd.respond_info(f"Received: {response.hex(' ').upper()}")
                    valid, weight =self.validate_and_parse(response)
                    if valid:
                        gcmd.respond_info(f"Check passed, weight: {weight}")
                    else:
                        gcmd.respond_info(f"Check failed: {weight}")
                else:
                    gcmd.respond_info("No response received.")
            except SerialException as e:
                logging.error(f"Error reading response: {e}")
                gcmd.respond_info(f"Error reading response: {e}")
        except SerialException as e:
            logging.error(f"Command send failed: {e}")
            gcmd.respond_info(f"Error sending command: {e}")

    def cmd_FORCE_SENSOR_STATUS(self, gcmd):
        status = getattr(self, 'block_status', False)
        gcmd.respond_info('yes' if status else 'no')

    def cmd_FORCE_SENSOR_POLL(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is None:
            gcmd.respond_info("Missing ENABLE parameter (0 or 1)")
            return
        if enable:
            self.start_polling()
            gcmd.respond_info("Force sensor polling enabled.")
        else:
            self.stop_polling()
            gcmd.respond_info("Force sensor polling disabled.")


    def validate_and_parse(self, data):
        if len(data) != 10:
            return False, None, "Length error"
        if data[0] != 0xAA or data[9] != 0xFF:
            return False, None, "Header or tail error"
        checksum = sum(data[1:7])
        checkval = data[7] * 256 + data[8]
        if checksum != checkval:
            return False, None, f"Checksum error: {checksum} != {checkval}"
        weight = data[4] * 65536 + data[5] * 256 + data[6]
        # If the highest bit of byte 4 is set, it is negative
        if data[4] != 0:
            weight = -weight
        return True, weight

    def cmd_SEND_TENSION_COMMAND(self, gcmd):
        if not self.serial_conn or not self.serial_conn.is_open:
            gcmd.respond_info("Serial port is not available")
            return
        
        send_data = bytes([0xA3, 0x00, 0xA2, 0xA4, 0xA5])
        try:
            self.serial_conn.write(send_data)
            gcmd.respond_info("Command sent: A3 00 A2 A4 A5")
            try:
                response = self.serial_conn.read(10)
                if response:
                    gcmd.respond_info(f"Received: {response.hex(' ').upper()}")
                    valid, weight = self.validate_and_parse(response)
                    if valid:
                        gcmd.respond_info(f"Check passed, weight: {weight}")
                    else:
                        gcmd.respond_info(f"Check failed: {weight}")
                else:
                    gcmd.respond_info("No response received.")
            except SerialException as e:
                logging.error(f"Error reading response: {e}")
                gcmd.respond_info(f"Error reading response: {e}")
        except SerialException as e:
            logging.error(f"Command send failed: {e}")
            gcmd.respond_info(f"Error sending command: {e}")

    def start_polling(self):
        if hasattr(self, 'polling_enabled') and self.polling_enabled:
            return
        self.last_weight = None
        self.block_count = 0
        self.block_status = False
        self.polling_enabled = True
        self.block_warning_sent = False  # New: Blockage warning sent flag
        self.polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
        self.polling_thread.start()

    def stop_polling(self):
        self.polling_enabled = False

    def _send_block_warning(self, weight_before_drop, current_weight):
        try:
            message = f"M118 DU TOU LA! Weight dropped from {weight_before_drop} to {current_weight}"
            self.gcode.run_script_from_command(message)
            self.gcode.run_script_from_command("FIBER_FILAMENT_TRIGGER")
            logging.info(f"Block warning sent: {message}")
        except Exception as e:
            logging.error(f"Failed to send block warning: {e}")

    def _send_weight_drop_warning(self, weight_before_drop, current_weight):
        try:
            message = f"M118 Weight drop detected! From {weight_before_drop} to {current_weight}"
            self.gcode.run_script_from_command(message)
            logging.info(f"Weight drop warning sent: {message}")
        except Exception as e:
            logging.error(f"Failed to send weight drop warning: {e}")

    def _polling_loop(self):
        if not hasattr(self, 'block_recover_count'):
            self.block_recover_count = 0

        while getattr(self, 'polling_enabled', False):
            weight = self._send_and_get_weight()
            if weight is not None:
                if not hasattr(self, 'weight_before_drop'):
                    self.weight_before_drop = None
                    
                # Save the previous weight value for comparison
                previous_weight = self.last_weight
                self.last_weight = weight
                
                if previous_weight is not None:
                    diff = previous_weight - weight
                    if diff >= 600:
                        # First time a weight drop is detected, record the weight before the drop
                        if self.weight_before_drop is None:
                            self.weight_before_drop = previous_weight
                            # Send weight drop warning
                            self.reactor.register_callback(
                                lambda event: self._send_weight_drop_warning(
                                    self.weight_before_drop, weight))
                        
                    if self.weight_before_drop is not None:
                        drop_diff = self.weight_before_drop - weight
                        if drop_diff >= 600:
                            self.block_count += 1
                            if self.block_count >= 10 and not self.block_status:
                                self.block_status = True
                                self.block_warning_sent = False  # Reset warning flag
                        else:
                            self.weight_before_drop = None
                            self.block_count = 0
                            self.block_status = False
                            self.block_warning_sent = False
                
                # Blockage detected and warning not yet sent
                if self.block_status and not self.block_warning_sent:
                    # Use reactor to safely send GCODE command in main thread
                    self.reactor.register_callback(
                        lambda event: self._send_block_warning(
                            self.weight_before_drop, weight))
                    self.block_warning_sent = True  # Mark warning as sent

                # Blockage recovery logic
                if self.block_status and previous_weight is not None:
                    diff_recover = abs(previous_weight - weight)
                    if diff_recover < 100:
                        self.block_recover_count += 1
                        if self.block_recover_count >= 5:
                            self.block_status = False
                            self.block_count = 0
                            self.weight_before_drop = None
                            self.block_recover_count = 0
                            self.block_warning_sent = False  
                    else:
                        self.block_recover_count = 0
            time.sleep(1)

    def _send_and_get_weight(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        send_data = bytes([0xA3, 0x00, 0xA2, 0xA4, 0xA5])
        try:
            self.serial_conn.write(send_data)
            response = self.serial_conn.read(10)
            if response:
                valid, weight = self.validate_and_parse(response)
                if valid:
                    return weight
        except SerialException:
            pass
        return None

def load_config(config):
    return MySerial(config)