import logging
import serial
from serial import SerialException

SERIAL_TIMER = 3  # Timer interval in seconds

class my_serial:
    def __init__(self, config):
        self.serial_port = config.get("serial")
        self.baud = config.getint("baud", default=9600)

        self.last_value = None         # Tension sensor value
        self.last_ad_value = None      # Raw AD (ADC) value
        self._in_position_counter = 0  # Count consecutive positive reads

        self.is_sending = False        # Control for periodic command sending

        # Try to connect automatically on initialization
        try:
            self.serial = serial.Serial(self.serial_port, self.baud, timeout=0.1, write_timeout=0.1)
        except SerialException as e:
            logging.error(f"Failed to connect to UART: {e}")
            return

        printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')

        # Register G-code commands
        self.gcode.register_command(
            'TENSION_SENSOR_STATUS',
            self.cmd_TENSION_SENSOR_STATUS,
            desc="Check tension sensor status: in position and current value"
        )

        self.gcode.register_command(
            'READ_AD_VALUE',
            self.cmd_READ_AD_VALUE,
            desc="Read current AD (ADC) value from the module. Sends A1 00 A0 A2 A3."
        )

        self.gcode.register_command(
            'TENSION_SENSOR_START',
            self.cmd_TENSION_SENSOR_START,
            desc="Start sending command periodically. Auto send zero calibration once."
        )

        self.gcode.register_command(
            'TENSION_SENSOR_STOP',
            self.cmd_TENSION_SENSOR_STOP,
            desc="Stop sending command periodically."
        )

        self.gcode.register_command(
            'TENSION_SENSOR_ZERO_CALIBRATION',
            self.cmd_TENSION_SENSOR_ZERO_CALIBRATION,
            desc="Send zero calibration command: AA 00 A9 AB A8"
        )

        self.gcode.register_command(
            'TENSION_SENSOR_TARE_ZERO',
            self.cmd_TENSION_SENSOR_TARE_ZERO,
            desc="Send tare zero command: AB 00 AA AC AD"
        )

        self.reactor = printer.get_reactor()

        # Initialize read timer (always active)
        self.read_timer = self.reactor.register_timer(self._run_Read, self.reactor.NOW)

        # Initially no send timer
        self.send_timer = None

    def cmd_TENSION_SENSOR_START(self, gcmd):
        if self.send_timer is not None:
            gcmd.respond_info("Tension sensor is already running.")
            return

        # Send zero calibration command automatically when starting
        self.send_zero_calibration_command()

        self.is_sending = True
        self.send_timer = self.reactor.register_timer(self._send_Read_Command_Timer, self.reactor.NOW)
        gcmd.respond_info("Started periodic tension sensor data requests.")

    def cmd_TENSION_SENSOR_STOP(self, gcmd):
        self.is_sending = False
        if self.send_timer is not None:
            self.reactor.unregister_timer(self.send_timer)
            self.send_timer = None
        gcmd.respond_info("Stopped periodic tension sensor data requests.")

    def cmd_TENSION_SENSOR_ZERO_CALIBRATION(self, gcmd):
        self.send_zero_calibration_command()
        gcmd.respond_info("Sent zero calibration command manually.")

    def cmd_TENSION_SENSOR_TARE_ZERO(self, gcmd):
        self.send_tare_zero_command()
        gcmd.respond_info("Sent tare zero command manually.")

    def _send_Read_Command_Timer(self, eventtime):
        if self.is_sending:
            self.send_command()
        return eventtime + SERIAL_TIMER

    def send_command(self):
        """
        Send main command: A3 00 A2 A4 A5 (for tension sensor data)
        """
        if not hasattr(self, 'serial') or not self.serial.is_open:
            logging.error("Serial port is not open.")
            return

        send_data = bytes([0xA3, 0x00, 0xA2, 0xA4, 0xA5])
        try:
            self.serial.write(send_data)
            logging.info("Sent command: A3 00 A2 A4 A5")
        except SerialException as e:
            logging.error(f"Failed to send command: {e}")

    def send_zero_calibration_command(self):
        """
        Send zero calibration command: AA 00 A9 AB A8
        """
        if not hasattr(self, 'serial') or not self.serial.is_open:
            logging.error("Serial port is not open.")
            return

        send_data = bytes([0xAA, 0x00, 0xA9, 0xAB, 0xA8])
        try:
            self.serial.write(send_data)
            logging.info("Sent zero calibration command: AA 00 A9 AB A8")
        except SerialException as e:
            logging.error(f"Failed to send zero calibration command: {e}")

    def send_tare_zero_command(self):
        """
        Send tare zero command: AB 00 AA AC AD
        """
        if not hasattr(self, 'serial') or not self.serial.is_open:
            logging.error("Serial port is not open.")
            return

        send_data = bytes([0xAB, 0x00, 0xAA, 0xAC, 0xAD])
        try:
            self.serial.write(send_data)
            logging.info("Sent tare zero command: AB 00 AA AC AD")
        except SerialException as e:
            logging.error(f"Failed to send tare zero command: {e}")

    def _run_Read(self, eventtime):
        """
        Read from UART periodically and process valid packets.
        Supports two types of packets:

        - Tension Sensor Packet: starts with AA and ends with FF
        - ADC Value Packet: starts with BB and ends with FE
        """
        try:
            raw_bytes = self.serial.read_all()
            if len(raw_bytes) == 0:
                return eventtime + SERIAL_TIMER

            logging.debug(f"Raw received data: {[hex(b) for b in raw_bytes]}")

            start_index = -1
            end_index = -1

            for i in range(len(raw_bytes)):
                if raw_bytes[i] == 0xAA or raw_bytes[i] == 0xBB:
                    start_index = i
                elif raw_bytes[i] == 0xFF or raw_bytes[i] == 0xFE:
                    if start_index != -1:
                        end_index = i + 1
                        break

            if start_index != -1 and end_index != -1:
                packet = raw_bytes[start_index:end_index]
                if packet[0] == 0xAA:
                    if self.validate_packet(packet):
                        self.handle_received_data(packet)
                    else:
                        logging.warning("Received TENSION packet failed validation.")
                elif packet[0] == 0xBB:
                    if self.validate_ad_packet(packet):
                        self.handle_received_ad_data(packet)
                    else:
                        logging.warning("Received AD packet failed validation.")
            else:
                logging.warning("No complete valid packet found.")

        except SerialException:
            logging.error("Unable to communicate with UART")
            return self.reactor.NEVER

        return eventtime + SERIAL_TIMER

    def validate_packet(self, packet):
        """
        Validate tension sensor packet:
        - Length == 10
        - Start == 0xAA
        - End == 0xFF
        - Checksum: sum(bytes[1:7]) == (bytes[7] * 256 + bytes[8])
        """
        if len(packet) != 10:
            return False
        if packet[0] != 0xAA or packet[-1] != 0xFF:
            return False

        checksum = sum(packet[1:7]) & 0xFFFF
        expected = packet[7] * 256 + packet[8]

        return checksum == expected

    def validate_ad_packet(self, packet):
        """
        Validate AD value packet:
        - Length == 10
        - Start == 0xBB
        - End == 0xFE
        - Same checksum rule
        """
        if len(packet) != 10:
            return False
        if packet[0] != 0xBB or packet[-1] != 0xFE:
            return False

        checksum = sum(packet[1:7]) & 0xFFFF
        expected = packet[7] * 256 + packet[8]

        return checksum == expected

    def handle_received_data(self, data):
        """
        Process tension sensor data
        """
        hex_data = list(data)
        logging.info(f"Received {len(hex_data)} bytes: {[f'{b:02X}' for b in hex_data]}")

        sign_byte = hex_data[3]
        byte4 = hex_data[4]
        byte5 = hex_data[5]
        byte6 = hex_data[6]

        value = byte4 * 65536 + byte5 * 256 + byte6

        if sign_byte == 0x01:
            value = -((1 << 24) - value)  # 24-bit negative via two's complement
            logging.info(f"Negative value calculated: {value}")
        elif sign_byte == 0x00:
            logging.info(f"Positive value calculated: {value}")
        else:
            logging.warning("Unknown sign byte. Expected 0x00 or 0x01.")

        # §Ø?
        if self.last_value is not None and abs(value - self.last_value) > 20:
            logging.info(f"Tension sensor value changed too much: {self.last_value} -> {value}, marking as NOT in position.")
            self._in_position_counter = 0  # 
        else:
            if value > 0:
                self._in_position_counter += 1
            else:
                self._in_position_counter = 0

        self.last_value = value

    def handle_received_ad_data(self, data):
        """
        Process AD value data
        """
        hex_data = list(data)
        logging.info(f"Received AD packet: {[f'{b:02X}' for b in hex_data]}")

        byte4 = hex_data[4]
        byte5 = hex_data[5]
        byte6 = hex_data[6]
        ad_value = byte4 * 65536 + byte5 * 256 + byte6

        self.last_ad_value = ad_value
        logging.info(f"Current AD Value: {ad_value}")

    def is_tension_sensor_in_position(self):
        """
        Return True if the sensor has been detected in position for 3 consecutive reads.
        """
        return self._in_position_counter >= 3

    def cmd_TENSION_SENSOR_STATUS(self, gcmd):
        """
        G-code command handler for TENSION_SENSOR_STATUS
        """
        status = self.is_tension_sensor_in_position()
        value = self.last_value if self.last_value is not None else "N/A"

        msg = f"Tension Sensor Status:\n"
        msg += f" - In Position: {'Yes' if status else 'No'}\n"
        msg += f" - Value: {value}"

        gcmd.respond_info(msg)

    def cmd_READ_AD_VALUE(self, gcmd):
        """
        G-code handler for READ_AD_VALUE
        Sends one-time AD measurement command and returns last received value.
        """
        logging.info("READ_AD_VALUE command received, sending AD request...")
        self.send_read_ad_command()

        # Wait briefly for response (non-blocking wait)
        self.reactor.pause(self.reactor.monotonic() + 0.5)

        ad_value = self.last_ad_value if self.last_ad_value is not None else "N/A"
        msg = f"Current AD Value: {ad_value}"
        gcmd.respond_info(msg)

    def send_read_ad_command(self):
        """
        Send the command: A1 00 A0 A2 A3 (for reading raw ADC value)
        """
        if not hasattr(self, 'serial') or not self.serial.is_open:
            logging.error("Serial port is not open.")
            return

        send_data = bytes([0xA1, 0x00, 0xA0, 0xA2, 0xA3])
        try:
            self.serial.write(send_data)
            logging.info("Sent AD read command: A1 00 A0 A2 A3")
        except SerialException as e:
            logging.error(f"Failed to send AD read command: {e}")

def load_config(config):
    return my_serial(config)