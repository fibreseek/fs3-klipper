import logging
import serial
from serial import SerialException
import struct

# LED color configuration (R, G, B order), now support up to 37 LEDs
LED_COLORS = [
    (255, 0, 0),       # Red
    (255, 16, 0),      # Orange
    (255, 255, 255),   # White
    (255, 128, 0),     # Yellow
    (0, 255, 0),       # Green
    (0, 0, 255),       # Blue
    (0, 64, 128),      # Cyan
    (16, 16, 16),      # Gray
    (108, 28, 193)     # Purple
]

class LEDController:
    def __init__(self):
        # Initialize a 129-byte buffer for the LED data frame (16 header + 111 LED data + 2 footer)
        self.led_data = bytearray(129)
        self.brightness = 30
        self.direction = 1
        self.current_led = 0
        self.all_lit = False
        self.active_mode = 0
        self.led_count = 37   # Default to controlling the first 37 LEDs
        self.current_color = (255, 0, 0)
        self.led_order = 'reverse'  # 'normal' or 'reverse' (37th LED as first)
        
        # Blinking control variables
        self.blink_counter = 0  # Counter for blinking state (0 = on, 1 = off)
        self.blink_toggle = False  # State for blinking effect
        
        self.total_layers = 0     # Total number of layers (M530 L)
        self.current_layer = 0    # Current completed layer number (M532 L)
        self.progress_percent = 0.0  # Track progress percentage
        
        # NEW: Progress bar color mode variable
        self.progress_color_mode = 0  # Default to green (0 = green, 1 = red, etc.)
        
        # Printer state variables
        self.printer_state = "idle"  # "idle", "paused", "error"
        self.pre_pause_state = "idle"
        self.pre_pause_mode = 0
        self.pre_pause_color = (0, 0, 0)
        self.pre_pause_count = 37
        self.state_colors = {
            "idle": (0, 255, 0),     # Green
            "paused": (0, 0, 255),   # Blue
            "error": (255, 0, 0),    # Red
            "turn_off": (0, 0, 0)    # Off
        }

        # Initialize the fixed part of the frame header
        self.init_frame_header()

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
        self.led_data[12:14] = struct.pack('>H', 111) # Data length: 37 LEDs  3 bytes
        self.led_data[14:16] = struct.pack('>H', 1)   # Repeat count
        self.led_data[127:129] = b'\xAA\xBB'          # Frame footer

    def build_frame_data(self, color_data):
        """
        Build the full LED control frame using provided color data.
        :param color_data: list of bytes, GRB-ordered RGB values (logical order: 1-37)
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

    def calculate_led_count(self):
        """Calculate the number of LEDs to light up based on printing progress"""
        if self.total_layers <= 0:
            return 0
            
        # Calculate progress percentage (0-100)
        progress_percent = (self.current_layer / self.total_layers) * 100
        
        # Calculate number of LEDs to light up (0-37)
        return min(37, int(round(progress_percent * 37 / 100)))


class MySerial:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.serial_port = config.get("serial")
        self.baud = config.getint("baud", default=115200)
        self.led = LEDController()
        self.write_timer = None
        self.is_static_mode = True  # Initial static mode
        self.vsd = None
        self.last_active = False

        try:
            self.serial = serial.Serial(
                self.serial_port,
                self.baud,
                timeout=0,
                write_timeout=0,
                dsrdtr=False
            )
        except SerialException as e:
            logging.error(f"UART initialization failed: {str(e)}")

        # Register G-code commands
        self._register_commands()

        try:
            self.vsd = self.printer.lookup_object('virtual_sdcard')
        except:
            logging.exception("Failed to find virtual_sdcard")
        
        self.progress_timer = self.reactor.register_timer(
            self._check_print_progress, self.reactor.monotonic() + 3.0)

    def _register_commands(self):
        self.gcode.register_command("SET_LED_MODE", self.cmd_set_led_mode,
                                    desc="Set LED mode parameters: MODE R G B COUNT")
        # Register M530 and M532 commands
        # self.gcode.register_command("M530", self.cmd_M530,
        #                            desc="Set total printing layers: M530 L<total>")
        # self.gcode.register_command("M532", self.cmd_M532,
        #                            desc="Set current printing layer: M532 L<current>")
        # Register M2000 command for printer state
        self.gcode.register_command("M2000", self.cmd_M2000,
                                   desc="Set printer state: M2000 L<state> R<red> G<green> B<blue>")
        # NEW: Register M2001 command for progress color mode
        self.gcode.register_command("M2001", self.cmd_M2001,
                                   desc="Set progress bar color mode: M2001 S<mode> (0=green, 1=red, 2=Purple.)")
    
    def _check_print_progress(self, eventtime):
        if self.vsd is None:
            try:
                self.vsd = self.printer.lookup_object('virtual_sdcard')
            except:
                return eventtime + 1.0
            
        try:
            is_active = self.vsd.is_active()
            progress = self.vsd.progress()
            #self.gcode.run_script_from_command("M118 progress=%f"% (progress))
        except:
            logging.exception("Error getting print status")
            return eventtime + 1.0
        
        if self.last_active and not is_active:
            led_count = 37
            color_modes = [
                (0, 255, 0),
                (255, 50, 0),
                (108, 28, 193)
            ]
            mode = min(self.led.progress_color_mode, len(color_modes) - 1)
            progress_color = color_modes[mode]
            self.led.active_mode = 0
            self.led.current_color = progress_color
            self.led.led_count = led_count
            self.update_led_data()
            self._send_led_data()
        if is_active and self.led.printer_state not in ["paused", "error"]:
            led_count = int(round(progress * 37))
            if progress >= 0.999:
                led_count = 37
            elif progress < 0.027:
                led_count = 1
                
            color_modes = [
                (0, 255, 0),
                (255, 50, 0),
                (108, 28, 193)
            ]
            mode = min(self.led.progress_color_mode, len(color_modes) - 1)
            progress_color = color_modes[mode]
            self.led.active_mode = 0
            self.led.current_color = progress_color
            self.led.led_count = led_count
            self.update_led_data()
            self._send_led_data()
        
        self.last_active = is_active
        
        return eventtime + 1.0

    def cmd_set_led_mode(self, gcmd):
        # Parse G-code command arguments
        mode = gcmd.get_int('MODE', 0, minval=0, maxval=37)
        r = gcmd.get_int('R', 255, minval=0, maxval=255)
        g = gcmd.get_int('G', 255, minval=0, maxval=255)
        b = gcmd.get_int('B', 255, minval=0, maxval=255)
        count = gcmd.get_int('COUNT', 37, minval=0, maxval=37)

        # Reset printer state to idle when setting new LED mode
        if self.led.printer_state != "idle":
            logging.info("SET_LED_MODE reset printer state to idle")
            self.led.printer_state = "idle"
        
        # Update internal state
        self.led.active_mode = mode
        self.led.led_count = count
        self.led.current_color = (r, g, b)
        
        # Reset blink state when entering blink mode (mode 5)
        if mode == 5:
            self.led.blink_counter = 0
            self.led.blink_toggle = False

        # Always cancel existing timer
        if self.write_timer is not None:
            self.reactor.unregister_timer(self.write_timer)
            self.write_timer = None
            
        # Determine if static mode (no animation)
        is_static = mode not in (1, 2, 3, 5)
        self.is_static_mode = is_static
        
        if not is_static:
            # Start timer for dynamic updates
            self.write_timer = self.reactor.register_timer(
                self._run_write, self.reactor.NOW + 0.1)

        # Update LED data and send to device
        self.update_led_data()
        self._send_led_data()
        gcmd.respond_info(f"LED values updated: Mode{mode} RGB({r},{g},{b}) Count{count}")

    # M530 command handler - Set total layers
    def cmd_M530(self, gcmd):
        total_layers = gcmd.get_int('L', minval=1)
        self.led.total_layers = max(1, total_layers)  # Ensure at least 1 layer
        gcmd.respond_info(f"Total layers set to: {total_layers}")

    # M532 command handler - Set current layer and display progress bar
    def cmd_M532(self, gcmd):
        current_layer = gcmd.get_int('L', minval=0)
        self.led.progress_percent = gcmd.get_float('P', 0.0, minval=0.0, maxval=100.0)
        
        # NEW: Select color based on progress_color_mode
        color_modes = [
            (0, 255, 0),   # Green - mode 0
            (255, 50, 0), # Orange - mode 1
            (108, 28, 193)  # Purple - mode 2
        ]
        
        # Ensure mode is within valid range
        mode = min(self.led.progress_color_mode, len(color_modes) - 1)
        mode = max(mode, 0)
        progress_color = color_modes[mode]
        
        # Special case: when current_layer equals total_layers, light all LEDs
        if current_layer == self.led.total_layers:
            self.led.current_layer = self.led.total_layers
            self.led.led_count = 37  # Light all LEDs
            self.led.active_mode = 0
            self.led.current_color = progress_color  # Green (completion color)
        else:
            # Normal case: calculate completed layers (current_layer - 1)
            completed_layers = max(0, current_layer - 1)
            self.led.current_layer = max(0, min(completed_layers, self.led.total_layers))
            
            # Special handling for first layer
            if current_layer == 1 and (self.led.progress_percent / 100 * 37) <= 1:
                self.led.led_count = 1
            else:
                self.led.led_count = self.led.calculate_led_count()
            
            self.led.active_mode = 0
            self.led.current_color = progress_color  # Use selected progress color
        
        # Update and send LED data
        self.update_led_data()
        self._send_led_data()
        
        # Response information
        progress_percent = (self.led.current_layer / self.led.total_layers) * 100 if self.led.total_layers > 0 else 0
        gcmd.respond_info(f"Started layer: {current_layer}/{self.led.total_layers} | "
                         f"Completed: {self.led.current_layer} layers | "
                         f"LEDs lit: {self.led.led_count} (color mode: {self.led.progress_color_mode})")

    # NEW: M2001 command handler - Set progress bar color mode
    def cmd_M2001(self, gcmd):
        mode = gcmd.get_int('S', minval=0, maxval=8)
        self.led.progress_color_mode = mode
        gcmd.respond_info(f"Progress bar color mode set to: {mode}")

    # M2000 command handler - Set printer state
    def cmd_M2000(self, gcmd):
        state_code = gcmd.get_int('L', minval=0, maxval=4)
        r = gcmd.get_int('R', None, minval=0, maxval=255)
        g = gcmd.get_int('G', None, minval=0, maxval=255)
        b = gcmd.get_int('B', None, minval=0, maxval=255)
        
        # Map state code to state string
        state_map = {
            0: "idle",
            1: "paused",
            2: "error",
            3: "turn_off",
            4: "resume"
        }
        new_state = state_map.get(state_code, "idle")

        if new_state == "paused" and self.led.printer_state != "paused":
            self.led.pre_pause_state = self.led.printer_state
            self.led.pre_pause_mode = self.led.active_mode
            self.led.pre_pause_color = self.led.current_color
            self.led.pre_pause_count = self.led.led_count

        self.led.printer_state = new_state
        
        # Update state colors if provided
        if r is not None and g is not None and b is not None:
            self.led.state_colors[new_state] = (r, g, b)
        
        # Handle each state by calling SET_LED_MODE
        if new_state == "idle":
            color = self.led.state_colors["idle"]
            self.gcode.run_script_from_command(
                f"SET_LED_MODE MODE=1 R={color[0]} G={color[1]} B={color[2]} COUNT=37"
            )
        elif new_state == "paused":
            color = self.led.state_colors["paused"]
            self.gcode.run_script_from_command(
                f"SET_LED_MODE MODE=5 R={color[0]} G={color[1]} B={color[2]} COUNT=37"
            )
        elif new_state == "error":
            color = self.led.state_colors["error"]
            self.gcode.run_script_from_command(
                f"SET_LED_MODE MODE=0 R={color[0]} G={color[1]} B={color[2]} COUNT=37"
            )
        elif new_state == "turn_off":
            color = self.led.state_colors["turn_off"]
            self.gcode.run_script_from_command(
                f"SET_LED_MODE MODE=0 R={color[0]} G={color[1]} B={color[2]} COUNT=37"
            )
        elif new_state == "resume":
            self.led.printer_state = self.led.pre_pause_state
            r, g, b = self.led.pre_pause_color
            self.gcode.run_script_from_command(
                f"SET_LED_MODE MODE={self.led.pre_pause_mode} "
                f"R={r} G={g} B={b} COUNT={self.led.pre_pause_count}"
            )
        
        gcmd.respond_info(f"Printer state set to: {new_state} with color {self.led.current_color}")

    def update_led_data(self):
        # Extract current settings
        r, g, b = self.led.current_color
        mode = self.led.active_mode
        led_count = self.led.led_count
        state = self.led.printer_state

        color_data = []

        # Handle paused and error states
        if state in ["paused", "error"]:
            color_data = [r, g, b] * 37
            self.led.build_frame_data(color_data)
            return
            
        # Select appropriate LED effect based on mode
        if mode == 0:  # Static mode
            # Light only the specified number of LEDs (from left to right)
            for i in range(37):
                if i < led_count:
                    color_data.extend([r, g, b])  # GRB order - lit LED
                else:
                    color_data.extend([0, 0, 0])  # Off LED
        elif mode == 1:  # Breathing light
            if self.led.brightness < 50:
                step = 2
            else:
                step = 5
                
            self.led.brightness += self.led.direction * step
            if self.led.brightness >= 200 or self.led.brightness <= 30:
                self.led.direction *= -1
            scale = self.led.brightness / 255.0
            scaled_color = [int(r * scale), int(g * scale), int(b * scale)]  # GRB order
            color_data = scaled_color * led_count
            # Keep remaining LEDs off
            color_data += [0, 0, 0] * (37 - led_count)
        elif mode == 2:  # Chasing light (from left to right)
            color_data = self.mode_chase(r, g, b)
        elif mode == 3:  # Progress + chase (from left to right)
            color_data = self.mode_progress_chase(r, g, b)
        elif mode == 4:  # Progress pause (from left to right)
            color_data = self.mode_progress_pause(r, g, b)
        elif mode == 5:  # Blink next LED (from left to right)
            color_data = self.mode_blink_next(r, g, b)

        # Ensure color data has correct length (37 LEDs  3 bytes)
        while len(color_data) < 37 * 3:
            color_data.extend([0, 0, 0])

        self.led.build_frame_data(color_data)
    
    def _send_led_data(self):
        """Send LED data to serial device"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(bytes(self.led.led_data))
            except SerialException as e:
                logging.error(f"UART write failed: {str(e)}")

    def mode_chase(self, r, g, b):
        current_led = self.led.current_led
        led_count = self.led.led_count
        all_lit = self.led.all_lit
    
        color_data = [0] * 111  # 37 LEDs  3 bytes
    
        if not all_lit:
            for i in range(current_led + 1):
                pos = i
                color_data[pos * 3 + 0] = r  # G
                color_data[pos * 3 + 1] = g  # R
                color_data[pos * 3 + 2] = b  # B
            
            current_led += 1
            if current_led >= 37:
                self.led.all_lit = True
                current_led = 0
        else:
            for i in range(37):
                color_data[i * 3 + 0] = 0  # G
                color_data[i * 3 + 1] = 0  # R
                color_data[i * 3 + 2] = 0  # B
            self.led.all_lit = False
            current_led = 0
    
        self.led.current_led = current_led
        return color_data

    def mode_progress_chase(self, r, g, b):
        """Progress chase mode (from left to right)"""
        color_data = []
        current_led = self.led.current_led
        led_count = self.led.led_count

        # Progress bar from left to right
        for i in range(37):
            if i < led_count:
                color_data.extend([r, g, b])  # GRB order - lit LED
            else:
                color_data.extend([0, 0, 0])  # Off LED

        # Chase effect at the end of progress
        if led_count < 37:
            chase_pos = (37 - current_led - 1) % 37
            if chase_pos >= led_count and chase_pos < led_count + 3:
                idx = chase_pos * 3
                color_data[idx:idx+3] = [r, g, b]

        self.led.current_led = (current_led + 1) % 37
        return color_data

    def mode_progress_pause(self, r, g, b):
        """Progress bar mode (from left to right)"""
        led_count = self.led.led_count
        color_data = []

        # Light up LEDs from left to right
        for i in range(37):
            if i < led_count:
                color_data.extend([r, g, b])  # GRB order - lit LED
            else:
                color_data.extend([0, 0, 0])  # Off LED

        return color_data

    def mode_blink_next(self, r, g, b):
        """Blink all lit LEDs using a counter for consistent timing"""
        led_count = self.led.led_count
        color_data = []
        
        # Toggle state every other update (100ms interval = 200ms period)
        self.led.blink_counter += 1
        if self.led.blink_counter >= 10:
            self.led.blink_toggle = not self.led.blink_toggle
            self.led.blink_counter = 0
        
        # Apply current blink state to all lit LEDs
        for i in range(37):
            if i < led_count:
                # On during first half of period, off during second half
                if self.led.blink_toggle:
                    color_data.extend([r, g, b])
                else:
                    color_data.extend([0, 0, 0])
            else:
                # Keep unlit LEDs off
                color_data.extend([0, 0, 0])
                
        return color_data

    def _run_write(self, eventtime):
        """Timer callback to periodically send LED control data for animated effects"""
        if not self.serial or not self.serial.is_open or self.is_static_mode:
            return self.reactor.NEVER

        try:
            self.update_led_data()
            self._send_led_data()
        except SerialException as e:
            logging.error(f"UART write failed: {str(e)}")
            return eventtime + 0.1

        return eventtime + 0.1

def load_config(config):
    return MySerial(config)