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
            self.spi.max_speed_hz = 3_300_000

            # 设置 SPI 模式（0-3），决定时钟极性（CPOL）和相位（CPHA）
            # 模式 0：CPOL=0，CPHA=0（空闲时低电平，第一个时钟沿采样）
            self.spi.mode = 0

            # 设置每个字的位数（默认 8）
            self.spi.bits_per_word = 8
        except Exception as e:
            logging.error(f"SPI initial error: {e}")

    def send(self, led_data):
        """Send LED data to LED band"""
        # 因为ws812b有其自身的驱动原理，所以这里需要将RGB的3个字节的数据扩展成9个字节的数据
        # 每个字节的每一位扩展成3位，1扩展成110(0x06)，0扩展成100(0x04)
        # 并且需要将RGB顺序调整为GRB顺序

        def _byte_to_3bytes(byte):
            # 位映射规则：1 -> 3位二进制串'110'，0 -> 3位二进制串'100'（核心，直接拼接字符串避免数值运算）
            BIT_MAP = {'1': '110', '0': '100'}
            """工具函数:单个原始字节(0-255)→ 按规则扩展为3个字节(返回列表)
            步骤: 8位高位优先→逐位扩3位→24位二进制串→切分3个8位→转3个整数字节
            """
            if not 0 <= byte <= 255:
                raise ValueError(f"无效字节值：{byte}, 必须为0-255")
            # 转为8位高位优先二进制字符串，补前导0
            bin8 = format(byte, '08b')
            # 逐位扩展为24位二进制串（8位×3位/位）
            bin24 = ''.join([BIT_MAP[bit] for bit in bin8])
            # 24位按8位切分，转为3个整数字节（高位组在前）
            return [
                int(bin24[0:8], 2),   # 第1个8位
                int(bin24[8:16], 2),  # 第2个8位
                int(bin24[16:24], 2)  # 第3个8位
            ]      
        reset_code = [0x00] * 36  # 复位码，至少需要80us的低电平，这里发送36个字节(80us)
        buf = []
        buf.extend(reset_code)
        for i in range(0, len(led_data), 3):
            red_byte   = led_data[i]      # 原R字节
            green_byte = led_data[i+1]    # 原G字节
            blue_byte  = led_data[i+2]    # 原B字节

            # 每个颜色字节3个扩展字节，依次追加到缓冲区（3×3=9字节/像素）
            # 将RGB顺序调整为GRB顺序
            buf.extend(_byte_to_3bytes(green_byte))
            buf.extend(_byte_to_3bytes(red_byte))
            buf.extend(_byte_to_3bytes(blue_byte))
        buf.extend(reset_code)
        try:
            # 只发送，不接收数据
            # logging.info(f"Sending SPI data: {buf}")
            self.spi.xfer2(buf)
        except IOError as e:
            logging.error(f"SPI write failed: {str(e)}")

    def close(self):
        try:
            if 'spi' in locals():
                self.spi.close()
        except Exception as e:
            logging.error(f"SPI close error: {e}")

class LEDController:
    def __init__(self, config):
        # Klipper
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # LED控制相关变量
        self.led_total_count = 37 # 灯带上包含的LED总颗数, 之后可以从配置文件读取，来支持灯珠数量不同的灯带 默认为37颗。
        self.led_data = [0x00, 0x00, 0x00] * self.led_total_count
        self.brightness = 30
        self.direction = 1  # 呼吸灯方向，1表示变亮，-1表示变暗
        self.current_color = (255, 0, 0)  # 默认红色，顺序为RGB
        self.led_order = 'reverse'

        # LED数据发送器
        self.data_sender = LEDDataSender()
        
        # Blinking control variables
        self.blink_counter = 0
        self.blink_toggle = False
        # Progress bar variables
        self.progress_index = 0  # 当前亮起的LED数量
        self.progress_direction = 1  # 1表示增加，-1表示减少
        self.progress_speed = 1  # 每次更新增加的LED数量
        
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
        
        # 固定数量模式相关
        self.fixed_count = 10  # 默认点亮10个LED
        self.fixed_start = 0   # 默认从第一个LED开始
        
        # 周期性发送LED数据的定时器
        self.send_timer = None

        # Initialize LED data
        self.update_led_data()
        
        self.gcode.register_command("M2000", self.cmd_M2000,
                                   desc="Set LED mode: M2000 L<mode> [S<print_mode>] [R<red> G<green> B<blue>] [C<count>] [F<start>]")
        
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
            for i in range(self.led_total_count, 0, -1):
                idx = (i-1)*3
                reversed_data.extend(color_data[idx:idx+3])
            color_data = reversed_data
        # Fill the frame data
        for i in range(len(color_data)):
            self.led_data[i] = color_data[i]

    def _send_led_data(self):
        if self.data_sender:
            self.data_sender.send(self.led_data)
        else:
            logging.error("LED data sender not initialized")

    # 周期性发送LED数据的回调函数（500ms间隔）
    def _periodic_send_led(self, eventtime):
        """周期性发送LED数据(500ms间隔)"""
        try:
            self._send_led_data()
        except Exception as e:
            logging.error(f"Periodic LED send failed: {str(e)}")
        # 返回下一次执行时间（500毫秒后）
        return eventtime + 0.5

    def update_led_data(self, print_progress=0.0):
        """Update LED data based on current state and print progress"""
        color_data = []
        r, g, b = self.current_color
        
        if self.printer_state == "progress_cycle":
            if self.progress_index == 0:
                color_data = [0, 0, 0] * self.led_total_count
            elif self.progress_index == self.led_total_count:
                color_data = [r, g, b] * self.led_total_count
            elif self.progress_index > 0 and self.progress_index < self.led_total_count:
                color_data.extend([r, g, b] * self.progress_index)
                color_data.extend([0, 0, 0] * (self.led_total_count - self.progress_index))
            else:
                logging.error(f"Invalid progress index in progress_cycle mode. progress_index: {self.progress_index}")
                return

        elif self.printer_state == "printing":
            led_count = int(round(print_progress * self.led_total_count))
            # Ensure at least 1 LED is lit for minimal progress
            if print_progress > 0 and led_count == 0:
                led_count = 1

            if led_count == 0:
                color_data = [0, 0, 0] * self.led_total_count
            elif led_count == self.led_total_count:
                color_data = [r, g, b] * self.led_total_count
            elif led_count > 0 and led_count < self.led_total_count:
                color_data.extend([r, g, b] * led_count)
                color_data.extend([0, 0, 0] * (self.led_total_count - led_count))
            else:
                logging.error(f"Invalid led count in printing mode. led_count: {led_count}")

        elif self.printer_state == "fixed_count":
            # 固定数量模式：只点亮指定数量的LED
            for i in range(self.led_total_count):
                if self.fixed_start <= i < self.fixed_start + self.fixed_count:
                    color_data.extend([r, g, b]) 
                else:
                    color_data.extend([0, 0, 0])

        elif self.printer_state == "breathing":
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
            color_data = [scaled_r, scaled_g, scaled_b] * self.led_total_count

        elif self.printer_state == "blinking":
            # Blinking effect
            self.blink_counter += 1
            if self.blink_counter >= 10:  # Toggle every 10 updates (approx 1 second)
                self.blink_toggle = not self.blink_toggle
                self.blink_counter = 0
            # Apply to all LEDs (RGB顺序)
            if self.blink_toggle:
                color_data = [r, g, b] * self.led_total_count
            else:
                color_data = [0, 0, 0] * self.led_total_count

        elif self.printer_state == "solid":
            color_data = [r, g, b] * self.led_total_count

        else:  # idle or error
            color_data = [0, 0, 0] * self.led_total_count
        
        # Ensure color data has correct length
        while len(color_data) < 3 * self.led_total_count:
            color_data.extend([0, 0, 0])
        
        self.build_frame_data(color_data)
        self._send_led_data()

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
        
        return eventtime + 1.0

    def cmd_M2000(self, gcmd):
        """Handle M2000 command to set LED mode"""
        # Parse parameters
        mode = gcmd.get_int('L', minval=0, maxval=7)
        print_mode = gcmd.get_int('S', -1, minval=0, maxval=4)
        r = gcmd.get_int('R', -1, minval=0, maxval=255)
        g = gcmd.get_int('G', -1, minval=0, maxval=255)
        b = gcmd.get_int('B', -1, minval=0, maxval=255)
        count = gcmd.get_int('C', -1, minval=1, maxval=self.led_total_count)
        start = gcmd.get_int('F', 0, minval=0, maxval=self.led_total_count-1)
        
        # Handle restore command (L2)
        if mode == 2:
            self.restore_previous_state()
            new_state = self.printer_state
            # Cancel existing timer if not needed
            if self.write_timer is not None:
                self.reactor.unregister_timer(self.write_timer)
                self.write_timer = None
            
            # Start timer for animated effects if needed
            if new_state in ["breathing", "blinking", "progress_cycle"]:
                self.write_timer = self.reactor.register_timer(
                    self._run_write, self.reactor.NOW + 0.1)
            
            # Update LED data and send immediately
            self.update_led_data()
            
            # Send response
            state_names = {
                "solid": "常亮",
                "breathing": "呼吸",
                "blinking": "闪烁",
                "progress_cycle": "进度循环",
                "fixed_count": "固定数量",
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
        elif mode == 3:  # 新增：进度循环模式
            new_state = "progress_cycle"
            self.progress_index = 0  # 重置进度
            self.progress_direction = 1  # 正向增加
            self.progress_speed = 1  # 每次增加1盏灯
        elif mode == 4:  # 新增：固定数量模式
            new_state = "fixed_count"
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
        
        # Set fixed count parameters if specified
        if count >= 0:
            self.fixed_count = count
        if start >= 0:
            self.fixed_start = start
        
        if count >= 0 and self.fixed_start + self.fixed_count > self.led_total_count:
            self.fixed_start = self.led_total_count - self.fixed_count
            if self.fixed_start < 0:
                self.fixed_start = 0
        
        # 取消所有原有定时器（避免冲突）
        if self.write_timer is not None:
            self.reactor.unregister_timer(self.write_timer)
            self.write_timer = None
            logging.info("Cancelled existing write timer")
        if self.send_timer is not None:
            self.reactor.unregister_timer(self.send_timer)
            self.send_timer = None
            logging.info("Cancelled existing send timer")
        
        # 根据新状态注册对应定时器
        if new_state in ["printing", "solid"]:
            # 注册500ms周期性发送定时器
            self.send_timer = self.reactor.register_timer(
                self._periodic_send_led, self.reactor.NOW + 0.5)
            logging.info("Registered periodic send timer for printing/solid mode")
        elif new_state in ["breathing", "blinking", "progress_cycle"]:
            # 注册原有动画效果定时器
            self.write_timer = self.reactor.register_timer(
                self._run_write, self.reactor.NOW + 0.1)
            logging.info("Registered write timer for animated effect mode")
        
        # Update LED data and send immediately
        self.update_led_data()
        
        # Log the command details
        logging.info(f"M2000 command: mode={mode}, state={new_state}, color={self.current_color}")
        
        # Send response
        state_names = {
            "solid": "常亮",
            "breathing": "呼吸",
            "blinking": "闪烁",
            "progress_cycle": "进度循环",
            "fixed_count": "固定数量",
            "printing": "打印进度",
            "idle": "空闲"
        }
        mode_name = state_names.get(new_state, "未知")
        
        if new_state == "fixed_count":
            gcmd.respond_info(f"LED模式已设置: {mode_name}, 颜色: {self.current_color}, 数量: {self.fixed_count}, 起始位置: {self.fixed_start}")
        else:
            gcmd.respond_info(f"LED模式已设置: {mode_name}, 颜色: {self.current_color}")

    def _run_write(self, eventtime):
        try:
            # 如果是进度循环模式，更新进度索引
            if self.printer_state == "progress_cycle":
                # 更新进度
                self.progress_index += self.progress_speed
                
                # 检查是否到达边界
                if self.progress_index > self.led_total_count:
                    self.progress_index = 0  # 重置从头开始
            
            # 更新并发送LED数据
            self.update_led_data()
        except Exception as e:
            logging.error(f"LED data send failed: {str(e)}")
        
        if self.printer_state == "progress_cycle":
            return eventtime + 0.1
        else:
            return eventtime + 0.1

def load_config(config):
    return LEDController(config)