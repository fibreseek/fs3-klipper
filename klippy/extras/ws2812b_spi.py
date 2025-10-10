import logging
import struct
from reactor import Reactor
from gcode import GCodeCommand

import sys
logging.info(f"Python path: {sys.path}")
# 注意：要把spidev模块安装到下面的目录下
sys.path.append('/usr/local/lib/python3.8/dist-packages')
import spidev
logging.info("spidev module loaded")

class LEDDataSender:
    def __init__(self, bus=3, device=0):
        try:
            # 创建SPI对象
            self.spi = spidev.SpiDev()
            self.spi.open(bus, device)  # 打开SPI设备，注意：要有打开这个SPI设备的权限

            # 设置时钟频率（Hz），实际速率可能受硬件限制
            self.spi.max_speed_hz = 8_000_000 # 设置最大速度为8MHz

            # 设置 SPI 模式（0-3），决定时钟极性（CPOL）和相位（CPHA）
            # 模式 0：CPOL=0，CPHA=0（空闲时低电平，第一个时钟沿采样）
            self.spi.mode = 0

            # 设置每个字的位数（默认 8）
            self.spi.bits_per_word = 8
        except Exception as e:
            logging.error(f"SPI initial error: {e}")

    def send(self, led_data):
        """Send LED data to LED band"""
        # 因为ws812b有其自身的驱动原理，所以这里需要将RGB的3个字节的数据扩展成24个字节的数据
        # 每个字节的每一位扩展成8位，1扩展成11111100(0xFC)，0扩展成11000000(0xC0)
        # 并且需要将RGB顺序调整为GRB顺序
        buf = []
        element = 0
        for i in range(0, len(led_data), 3):
            # 扩展Green数据
            element = led_data[i+1]
            for j in range(8):
                if element & 0x80 == 0x80:
                    buf.append(0xFC)
                else:
                    buf.append(0xC0)
                element <<= 1

            # 扩展Red数据
            element = led_data[i]
            for j in range(8):
                if element & 0x80 == 0x80:
                    buf.append(0xFC)
                else:
                    buf.append(0xC0)
                element <<= 1

            # 扩展Blue数据
            element = led_data[i+2]
            for j in range(8):
                if element & 0x80 == 0x80:
                    buf.append(0xFC)
                else:
                    buf.append(0xC0)
                element <<= 1
        try:
            self.spi.writebytes(bytes(buf))
        except IOError as e:
            logging.error(f"SPI write failed: {str(e)}")

    def close(self):
        try:
            if 'spi' in locals():
                self.spi.close()
        except Exception as e:
            logging.error(f"SPI close error: {e}")

class LEDController:
    LED_TOTAL_COUNT = 37  # 灯条上包含的LED总颗数
    def __init__(self, config):
        # Klipper
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # LED控制相关变量
        self.led_data = [0x00, 0x00, 0x00] * LEDController.LED_TOTAL_COUNT
        self.brightness = 30
        self.direction = 1
        self.current_led = 0
        self.all_lit = False
        self.led_count = 37
        self.current_color = (255, 0, 0)  # 默认红色，顺序为RGB
        self.led_order = 'reverse'

        # LED数据发送器
        self.data_sender = LEDDataSender()
        
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
            1: (255, 50, 0),    # Orange - 增强模式
            2: (108, 28, 193),    # Purple - 超强模式
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
        
        # Initialize LED data
        self.update_led_data()
        
        self.gcode.register_command("M2000", self.cmd_M2000,
                                   desc="Set LED mode: M2000 L<mode> [S<print_mode>] [R<red> G<green> B<blue>]")
        
        self.progress_timer = self.reactor.register_timer(
            self._check_print_progress, self.reactor.monotonic() + 1.0)

    def build_frame_data(self, color_data):
        """
        Build the full LED control frame using provided color data.
        :param color_data: list of bytes, RGB-ordered values (logical order: 1-37)
        """
        # Reverse the LED order if needed (last LED becomes first)
        if self.led_order == 'reverse':
            reversed_data = []
            for i in range(LEDController.LED_TOTAL_COUNT, 0, -1):
                idx = (i-1)*3
                reversed_data.extend(color_data[idx:idx+3])
            color_data = reversed_data
        # Fill the frame data
        for i in range(len(color_data)):
            self.led_data[i] = color_data[i]

    def update_led_data(self, print_progress=0.0):
        """Update LED data based on current state and print progress"""
        color_data = []
        r, g, b = self.current_color
        
        # Handle print progress mode (mode 6)
        if self.printer_state == "printing":
            led_count = int(round(print_progress * LEDController.LED_TOTAL_COUNT))
            # Ensure at least 1 LED is lit for minimal progress
            if print_progress > 0 and led_count == 0:
                led_count = 1
            # Progress bar with current color (RGB顺序)
            for i in range(LEDController.LED_TOTAL_COUNT):
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
                color_data = [scaled_r, scaled_g, scaled_b] * LEDController.LED_TOTAL_COUNT
            elif self.printer_state == "blinking":
                # Blinking effect
                self.blink_counter += 1
                if self.blink_counter >= 10:  # Toggle every 10 updates (approx 1 second)
                    self.blink_toggle = not self.blink_toggle
                    self.blink_counter = 0
                # Apply to all LEDs (RGB顺序)
                if self.blink_toggle:
                    color_data = [r, g, b] * LEDController.LED_TOTAL_COUNT
                else:
                    color_data = [0, 0, 0] * LEDController.LED_TOTAL_COUNT
            elif self.printer_state == "solid":
                # Solid light (RGB顺序)
                color_data = [r, g, b] * LEDController.LED_TOTAL_COUNT
            else:  # idle or error
                color_data = [0, 0, 0] * LEDController.LED_TOTAL_COUNT
        
        # Ensure color data has correct length
        while len(color_data) < 3 * LEDController.LED_TOTAL_COUNT:
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
        if self.data_sender:
            self.data_sender.send(self.led_data)
        else:
            logging.error("LED data sender not initialized")

    def _run_write(self, eventtime):
        try:
            self.update_led_data()
            self._send_led_data()
        except Exception as e:
            logging.error(f"LED data send failed: {str(e)}")
        
        return eventtime + 0.1

def load_config(config):
    return LEDController(config)