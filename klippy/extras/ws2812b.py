import logging
import serial
from serial import SerialException
import struct
from reactor import Reactor
from gcode import GCodeCommand

class LEDController:
    def __init__(self, config):
        # Klipper
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # 串口配置
        self.serial_port = config.get("serial")
        self.baud = config.getint("baud", default=115200)
        self.serial = None
        
        # LED控制相关变量
        self.led_data = bytearray(129)
        self.brightness = 30
        self.direction = 1
        self.current_led = 0
        self.all_lit = False
        self.led_count = 37
        self.current_color = (255, 0, 0)  # 默认红色，顺序为RGB
        self.led_order = 'reverse'
        
        # Blinking control variables
        self.blink_counter = 0
        self.blink_toggle = False
        
        # Printer state variables
        self.printer_state = "idle"
        self.pre_pause_state = "idle"
        self.pre_pause_color = (0, 0, 0)
        self.print_mode = 0  # 0:常规模式(绿), 1:增强模式(橙), 2:超强模式(紫), 3:空闲(白), 4:错误(红)
        
        # Fixed color mappings
        self.mode_colors = {
            0: (0, 255, 0),      # Green - 常规模式
            1: (255, 165, 0),    # Orange - 增强模式
            2: (128, 0, 128),    # Purple - 超强模式
            3: (255, 255, 255),  # White - 空闲
            4: (255, 0, 0)       # Red - 错误
        }  # 顺序为RGB
        
        # State restoration variables
        self.last_non_pause_state = "idle"
        self.last_non_pause_color = (255, 0, 0)  # 默认红色
        self.last_non_pause_mode = 0  # 默认常规模式
        
        # 打印进度相关
        self.write_timer = None
        self.vsd = None
        self.last_active = False
        self.print_progress = 0.0
        self.progress_timer = None
        
        # 初始化串口
        try:
            self.serial = serial.Serial(
                self.serial_port,
                self.baud,
                timeout=0,
                write_timeout=0,
                dsrdtr=False
            )
            logging.info(f"UART initialized on {self.serial_port} at {self.baud} baud")
        except SerialException as e:
            logging.error(f"UART initialization failed: {str(e)}")
        
        # Initialize the fixed part of the frame header
        self.init_frame_header()
        # Initialize LED data
        self.update_led_data()
        
        self.gcode.register_command("M2000", self.cmd_M2000,
                                   desc="Set LED mode: M2000 L<mode> [S<print_mode>] [R<red> G<green> B<blue>]")
        
        self.progress_timer = self.reactor.register_timer(
            self._check_print_progress, self.reactor.monotonic() + 1.0)

    def init_frame_header(self):
        """Initialize the fixed part of the LED control frame header"""
        # Frame header bytes
        self.led_data[0:3] = b'\xDD\x55\xEE'         # Frame start identifier
        self.led_data[3:5] = struct.pack('>H', 0)    # Group address
        self.led_data[5:7] = struct.pack('>H', 1)    # Device address
        self.led_data[7] = 0                         # Port number
        self.led_data[8] = 0x99                      # Function code
        self.led_data[9] = 0x01                      # LED type
        self.led_data[10:12] = b'\x00\x00'           # Reserved
        self.led_data[12:14] = struct.pack('>H', 111) # Data length: 37 LEDs * 3 bytes
        self.led_data[14:16] = struct.pack('>H', 1)   # Repeat count
        self.led_data[127:129] = b'\xAA\xBB'          # Frame footer

    def build_frame_data(self, color_data):
        """
        Build the full LED control frame using provided color data.
        :param color_data: list of bytes, RGB-ordered values (logical order: 1-37)
        """
        # Reverse the LED order if needed (37th LED becomes first)
        if self.led_order == 'reverse':
            reversed_data = []
            for i in range(37, 0, -1):
                idx = (i-1)*3
                reversed_data.extend(color_data[idx:idx+3])
            color_data = reversed_data
        # Fill the frame data
        for i in range(len(color_data)):
            self.led_data[16 + i] = color_data[i]
        return self.led_data

    def update_led_data(self, print_progress=0.0):
        """Update LED data based on current state and print progress"""
        color_data = []
        r, g, b = self.current_color
        
        # Handle print progress mode (mode 6)
        if self.printer_state == "printing":
            led_count = int(round(print_progress * 37))
            # Ensure at least 1 LED is lit for minimal progress
            if print_progress > 0 and led_count == 0:
                led_count = 1
            # Progress bar with current color (RGB顺序)
            for i in range(37):
                if i < led_count:
                    color_data.extend([r, g, b])  # RGB顺序
                else:
                    color_data.extend([0, 0, 0])
        else:
            # Handle other modes
            if self.printer_state == "breathing":
                # Breathing light effect
                if self.brightness < 50:
                    step = 2
                else:
                    step = 5
                self.brightness += self.direction * step
                if self.brightness >= 200 or self.brightness <= 30:
                    self.direction *= -1
                scale = self.brightness / 255.0
                scaled_r = int(r * scale)
                scaled_g = int(g * scale)
                scaled_b = int(b * scale)
                # Apply to all LEDs (RGB顺序)
                color_data = [scaled_r, scaled_g, scaled_b] * 37
            elif self.printer_state == "blinking":
                # Blinking effect
                self.blink_counter += 1
                if self.blink_counter >= 10:  # Toggle every 10 updates (approx 1 second)
                    self.blink_toggle = not self.blink_toggle
                    self.blink_counter = 0
                # Apply to all LEDs (RGB顺序)
                if self.blink_toggle:
                    color_data = [r, g, b] * 37
                else:
                    color_data = [0, 0, 0] * 37
            elif self.printer_state == "solid":
                # Solid light (RGB顺序)
                color_data = [r, g, b] * 37
            else:  # idle or error
                color_data = [0, 0, 0] * 37
        
        # Ensure color data has correct length
        while len(color_data) < 37 * 3:
            color_data.extend([0, 0, 0])
        
        self.build_frame_data(color_data)

    def save_current_state(self):
        """Save current state for later restoration"""
        self.last_non_pause_state = self.printer_state
        self.last_non_pause_color = self.current_color
        self.last_non_pause_mode = self.print_mode

    def restore_previous_state(self):
        """Restore previously saved state"""
        self.printer_state = self.last_non_pause_state
        self.current_color = self.last_non_pause_color
        self.print_mode = self.last_non_pause_mode

    def _check_print_progress(self, eventtime):
        """Check print progress and update LEDs if in printing mode"""
        if self.printer_state == "printing":
            try:
                if self.vsd is None:
                    self.vsd = self.printer.lookup_object('virtual_sdcard')
                
                if self.vsd.is_active():
                    self.print_progress = self.vsd.progress()
                else:
                    self.print_progress = 0.0
            except:
                logging.exception("Error getting print status")
                self.print_progress = 0.0
            
            # Update LEDs with current progress
            self.update_led_data(self.print_progress)
            self._send_led_data()
        
        return eventtime + 1.0

    def cmd_M2000(self, gcmd):
        """Handle M2000 command to set LED mode"""
        # Parse parameters
        mode = gcmd.get_int('L', minval=0, maxval=6)
        print_mode = gcmd.get_int('S', -1, minval=0, maxval=4)
        r = gcmd.get_int('R', -1, minval=0, maxval=255)
        g = gcmd.get_int('G', -1, minval=0, maxval=255)
        b = gcmd.get_int('B', -1, minval=0, maxval=255)
        
        # Handle restore command (L2)
        if mode == 2:
            self.restore_previous_state()
            new_state = self.printer_state
            # Cancel existing timer if not needed
            if self.write_timer is not None:
                self.reactor.unregister_timer(self.write_timer)
                self.write_timer = None
            
            # Start timer for animated effects if needed
            if new_state in ["breathing", "blinking"]:
                self.write_timer = self.reactor.register_timer(
                    self._run_write, self.reactor.NOW + 0.1)
            
            # Update LED data and send immediately
            self.update_led_data()
            self._send_led_data()
            
            # Send response
            state_names = {
                "solid": "常亮",
                "breathing": "呼吸",
                "blinking": "闪烁",
                "printing": "打印进度",
                "idle": "空闲"
            }
            mode_name = state_names.get(new_state, "未知")
            gcmd.respond_info(f"已恢复LED状态: {mode_name}, 颜色: {self.current_color}")
            return
        
        # Save current state for future restoration
        if mode != 6:  # Don't save printing state for restoration
            self.save_current_state()
        
        # Determine new state based on mode
        if mode == 0:
            new_state = "solid"
        elif mode == 1:
            new_state = "breathing"
        elif mode == 5:
            new_state = "blinking"
        elif mode == 6:
            new_state = "printing"
        else:
            new_state = "solid"
        
        # Update printer state
        self.printer_state = new_state
        
        # Update print mode if specified
        if print_mode >= 0:
            self.print_mode = print_mode
        
        # Set color based on parameters
        if print_mode >= 0:
            self.current_color = self.mode_colors.get(print_mode, (0, 0, 0))
        elif r >= 0 and g >= 0 and b >= 0:
            self.current_color = (r, g, b)
        else:
            pass
        
        # Cancel existing timer if not needed
        if self.write_timer is not None:
            self.reactor.unregister_timer(self.write_timer)
            self.write_timer = None
        
        # Start timer for animated effects
        if new_state in ["breathing", "blinking"]:
            self.write_timer = self.reactor.register_timer(
                self._run_write, self.reactor.NOW + 0.1)
        
        # Update LED data and send immediately
        self.update_led_data()
        self._send_led_data()
        
        # Log the command details
        logging.info(f"M2000 command: mode={mode}, state={new_state}, color={self.current_color}")
        
        # Send response
        state_names = {
            "solid": "常亮",
            "breathing": "呼吸",
            "blinking": "闪烁",
            "printing": "打印进度",
            "idle": "空闲"
        }
        mode_name = state_names.get(new_state, "未知")
        gcmd.respond_info(f"LED模式已设置: {mode_name}, 颜色: {self.current_color}")

    def _send_led_data(self):
        """Send LED data to serial device"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(bytes(self.led_data))
                logging.debug("LED data sent successfully")
            except SerialException as e:
                logging.error(f"UART写入失败: {str(e)}")
        else:
            logging.warning("UART port not open, cannot send LED data")

    def _run_write(self, eventtime):
        """Timer callback for animated effects"""
        if not self.serial or not self.serial.is_open:
            return self.reactor.NEVER

        try:
            self.update_led_data()
            self._send_led_data()
        except SerialException as e:
            logging.error(f"UART写入失败: {str(e)}")
        
        return eventtime + 0.1

def load_config(config):
    return LEDController(config)