# Code to implement asynchronous save gcode progress from a background thread
#
# Copyright (C) 2023-2023  Cao Chunliang <chunliang.cao@raise3d.com>
#

try:
    import queue
except ImportError:
    import Queue as queue
import threading
import logging, time, json, os

class PowerlossProgress():
    def __init__(self, prefix, max_file_num = 1):
        self.prefix = prefix
        self.max_file_num = max_file_num
        self.file_idx = 0
        self.bg_queue = queue.Queue()
        self.save_prog_thread = threading.Thread(target=self._save_prog_thread)
        self.save_prog_thread.start()
        self.save_prog_thread.setName("PowerlossProgressThread")
        self.powerloss_init_file = self.prefix + ".init"
        self.current_print_file = None
        # For async scan operations
        self.scan_results = {}
        self.scan_callbacks = {}
        self.scan_counter = 0
        self.scan_lock = threading.Lock()

    def exit(self):
        self.bg_queue.put_nowait(None)
        self.save_prog_thread.join()

    def _save_file(self, file_idx, record_args, flush=True):
        save_file = self.prefix + ".%d" %(file_idx)
        # Ensure directory exists before writing file
        dir_path = os.path.dirname(save_file)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)
        with open(save_file, "w") as json_file:
            json.dump(record_args, json_file)
            if flush:
                json_file.flush()
    
    def _save_init_file(self, record_args, flush=True):
        save_file = self.prefix + ".init"
        # Ensure directory exists before writing file
        dir_path = os.path.dirname(save_file)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)
        with open(save_file, "w") as json_file:
            json.dump(record_args, json_file)
            if flush:
                json_file.flush()

    def _save_prog_thread(self):
        while 1:
            record = self.bg_queue.get(True)
            if record is None:
                break

            record_args = record.get('args', {})
            command = record.get('command', 'record')
            if command == 'start':
                self.file_idx = 0
                self._save_init_file(record_args, False)
            elif command == 'record':
                save_file = self.prefix + ".%d" %(self.file_idx)
                self._save_file(self.file_idx, record_args)
                self.file_idx += 1
                if self.file_idx >= self.max_file_num:
                    self.file_idx = 0
            elif command == 'scan':
                # Async scan operation
                scan_id = record.get('scan_id')
                stop_byte = record_args.get('stop_byte')
                try:
                    result = self.scan_print_file_info(stop_byte)
                    with self.scan_lock:
                        self.scan_results[scan_id] = {'status': 'completed', 'data': result}
                        callback = self.scan_callbacks.get(scan_id)
                    if callback:
                        try:
                            callback(result)
                        except Exception:
                            logging.exception("Scan callback error for scan_id %s" % scan_id)
                except Exception as e:
                    logging.exception("Async scan failed for scan_id %s" % scan_id)
                    with self.scan_lock:
                        self.scan_results[scan_id] = {'status': 'failed', 'error': str(e)}

    def scan_print_file_info_async(self, stop_byte=None, callback=None):
        """Start an asynchronous scan of the print file in the background thread.
        
        Args:
            stop_byte: Optional byte position to stop scanning at
            callback: Optional callback function(result_dict) to be called when scan completes
        
        Returns:
            scan_id: A unique identifier to check scan status later
        """
        with self.scan_lock:
            self.scan_counter += 1
            scan_id = "scan_%d" % self.scan_counter
            self.scan_results[scan_id] = {'status': 'pending'}
            if callback:
                self.scan_callbacks[scan_id] = callback
        
        record = {
            'command': 'scan',
            'scan_id': scan_id,
            'args': {'stop_byte': stop_byte}
        }
        try:
            self.bg_queue.put_nowait(record)
            logging.debug("Async scan started: scan_id=%s, stop_byte=%s" % (scan_id, stop_byte))
        except Exception:
            logging.exception("Failed to queue async scan")
            with self.scan_lock:
                self.scan_results[scan_id] = {'status': 'failed', 'error': 'Queue error'}
        
        return scan_id
    
    def get_scan_result(self, scan_id):
        """Get the result of an async scan operation.
        
        Args:
            scan_id: The scan identifier returned by scan_print_file_info_async
        
        Returns:
            A dict with keys:
            - 'status': 'pending', 'completed', or 'failed'
            - 'data': scan result dict (if status is 'completed')
            - 'error': error message (if status is 'failed')
        """
        with self.scan_lock:
            return self.scan_results.get(scan_id, {'status': 'unknown'})
    
    def start_record(self, file_path=None):
        record_args = {'MODIFYTIME':time.time(), 'FILE_PATH':file_path}
        record = {'command':'start', 'args':record_args}
        try:
            logging.debug("powerloss_progress start record: seek_position:[%u], modify_time:[%f], queue_len:[%u]" %(0, record['args']['MODIFYTIME'], self.bg_queue.qsize()))
            self.bg_queue.put_nowait(record)
        except Exception:
            logging.exception("start record progress put queue fail.")

    def record(self, position, line, current_z):
        record = {'command':'record', 'args':{'MODIFYTIME':time.time(), 'SEEKPOSITION':position, 'RECORD_LINE':line, 'CURRENT_Z': current_z}}
        try:
            logging.debug("powerloss_progress record: record_line:[%s] seek_position:[%u], modify_time:[%f], queue_len:[%u]" %(str(line).strip(), position, record['args']['MODIFYTIME'], self.bg_queue.qsize()))
            self.bg_queue.put_nowait(record)
        except Exception:
            logging.exception("record progress put queue fail.")

    def check_powerloss_file(self):
        init_file_found, powerloss_file_found = False, False
        if os.path.exists(self.powerloss_init_file):
            try:
                with open(self.powerloss_init_file, "r") as f:
                    data = json.load(f)
                    file_path = data.get('FILE_PATH')
                    if file_path and os.path.exists(file_path):
                        self.current_print_file = file_path
                        init_file_found = True
            except Exception:
                logging.exception("Failed to read powerloss init file.")
            
            try:
                for i in range(self.max_file_num):
                    file_path = self.prefix + ".%d" % i
                    if os.path.exists(file_path):
                        powerloss_file_found = True   
            except Exception:
                logging.exception("Failed to read powerloss files.")
        return init_file_found and powerloss_file_found
    
    def scan_print_file_info(self, stop_byte=None):
        """Scan the current print file and extract gcode information.
        
        Reads the file in chunks of 8192 bytes, parses gcode lines,
        and extracts nozzle temperature, bed temperature, chamber temperature,
        acceleration, fan speeds, Z height, etc.
        
        Args:
            stop_byte: Optional byte position to stop scanning at. If specified,
                      scanning will stop when this byte position is reached.
        
        Returns a dictionary with the latest extracted information.
        """
        if not self.current_print_file or not os.path.exists(self.current_print_file):
            logging.warning("No current print file to scan")
            return {}
        
        # Initialize data structures to store extracted information
        # We store only the latest values for each parameter
        info = {
            'left_nozzle_temp': None,       # Nozzle temperatures by extruder index {0: temp_left, 1: temp_right}
            'right_nozzle_temp': None,      # Nozzle temperatures by extruder index {0: temp_left, 1: temp_right
            'bed_temp': None,         # Latest bed temperature
            'chamber_temp': None,     # Latest chamber temperature
            'acceleration': None,     # Latest acceleration value
            'fan_speeds': {},         # Latest fan speeds by fan index
            'z_height': None,         # Latest Z height
            'current_tool': None,     # Current nozzle/tool (T0, T1, etc.)
            'feedrate': None,         # Current print speed (F parameter)
            'x_position': None,       # Current X position
            'y_position': None,       # Current Y position
            'extrusion': None,        # Current extrusion value (E parameter)
            'extrusion_mode': None,   # Extrusion mode: 'absolute' (M82) or 'relative' (M83)
            'movement_mode': None,    # Movement mode: 'absolute' (G90) or 'relative' (G91)
            'pressure_advance': {},   # Pressure advance settings
            'velocity_limit': {},     # Velocity limit settings
            'gcode_lines': 0,         # Total gcode lines processed
            'file_size': 0,           # Total file size in bytes
            'scanned_bytes': 0,       # Total bytes scanned (may be less than file_size if stop_byte is used)
            'total_layer': None,
            'current_layer': None,
        }
        
        try:
            file_size = os.path.getsize(self.current_print_file)
            info['file_size'] = file_size
            
            # If stop_byte is specified, ensure it's within file bounds
            if stop_byte is not None:
                if stop_byte < 0:
                    stop_byte = 0
                elif stop_byte > file_size:
                    stop_byte = file_size
            
            with open(self.current_print_file, 'r') as f:
                buffer = ''
                chunk_size = 8192
                total_bytes_read = 0
                
                while True:
                    # Calculate how many bytes to read in this chunk
                    bytes_to_read = chunk_size
                    if stop_byte is not None:
                        remaining_bytes = stop_byte - total_bytes_read
                        if remaining_bytes <= 0:
                            break
                        bytes_to_read = min(chunk_size, remaining_bytes)
                    
                    chunk = f.read(bytes_to_read)
                    if not chunk:
                        break
                    
                    total_bytes_read += len(chunk)
                    buffer += chunk
                    
                    # Process complete lines in buffer
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        # Skip empty lines and comments
                        if not line or line.startswith(';'):
                            continue
                        
                        info['gcode_lines'] += 1
                        self._parse_gcode_line(line, info)
                    
                    # Check if we've reached the stop_byte
                    if stop_byte is not None and total_bytes_read >= stop_byte:
                        # Process any remaining buffer content before stopping
                        if buffer.strip():
                            line = buffer.strip()
                            if not line.startswith(';'):
                                info['gcode_lines'] += 1
                                self._parse_gcode_line(line, info)
                        break
                    
                    # If we're at the end of file, process remaining buffer
                    if len(chunk) < bytes_to_read:
                        if buffer.strip():
                            line = buffer.strip()
                            if not line.startswith(';'):
                                info['gcode_lines'] += 1
                                self._parse_gcode_line(line, info)
                        break
            
            info['scanned_bytes'] = total_bytes_read
            logging.debug("Scanned print file: %s lines, %d bytes scanned (total file: %d bytes)" % 
                         (info['gcode_lines'], info['scanned_bytes'], info['file_size']))
            return info
            
        except Exception as e:
            logging.exception("Failed to scan print file: %s" % str(e))
            return {}
    
    def _parse_gcode_line(self, line, info):
        """Parse a single gcode line and update info dictionary."""
        # Remove any inline comments
        if ';' in line:
            line = line.split(';')[0].strip()
        
        # Split command and parameters
        parts = line.split()
        if not parts:
            return
        
        command = parts[0].upper()
        
        # Parse parameters
        params = {}
        for part in parts[1:]:
            if '=' in part:
                key, value = part.split('=', 1)
                params[key.upper()] = value
            elif len(part) > 1 and part[0].isalpha():
                # Handle parameters like S260, T1, P1, Z0.200, F30000, X165.010
                key = part[0].upper()
                value = part[1:]
                params[key] = value
        
        # Process specific gcode commands
        if command in ['M104', 'M109']:
            # Nozzle temperature: M104 S260 T1 (T0=左喷嘴, T1=右喷嘴)
            if 'S' in params:
                try:
                    temp = float(params['S'])
                    extruder = 0  # Default extruder (T0 = 左喷嘴)
                    if 'T' in params:
                        try:
                            extruder = int(params['T'])
                        except ValueError:
                            pass
                    # Store temperature for specific extruder: 0=左喷嘴(left), 1=右喷嘴(right)
                    if extruder == 0:
                        info['left_nozzle_temp'] = temp
                    elif extruder == 1:
                        info['right_nozzle_temp'] = temp
                except ValueError:
                    pass
        
        elif command in ['M140', 'M190']:
            # Bed temperature: M140 S75
            if 'S' in params:
                try:
                    temp = float(params['S'])
                    info['bed_temp'] = temp
                except ValueError:
                    pass
        
        elif command in ['M141', 'M191']:
            # Chamber temperature: M141 S60
            if 'S' in params:
                try:
                    temp = float(params['S'])
                    info['chamber_temp'] = temp
                except ValueError:
                    pass
        
        elif command == 'M106':
            # Fan speed: M106 P1 S255
            if 'S' in params:
                try:
                    speed = float(params['S'])
                    fan_index = 0  # Default fan
                    if 'P' in params:
                        try:
                            fan_index = int(params['P'])
                        except ValueError:
                            pass
                    info['fan_speeds'][fan_index] = speed
                except ValueError:
                    pass
        
        elif command == 'M204':
            # Acceleration: M204 S3000
            if 'S' in params:
                try:
                    accel = float(params['S'])
                    info['acceleration'] = accel
                except ValueError:
                    pass
        
        # Parse T command for current nozzle/tool
        elif command.startswith('T'):
            # If command is 'T1', extract tool number from command itself
            # Only T0 and T1 are valid tool heads
            tool = None
            
            # First try to get tool number from command (e.g., 'T1')
            if len(command) > 1:
                try:
                    tool = int(command[1:])
                except ValueError:
                    pass
            
            # Only set current_tool if it's 0 or 1
            if tool is not None and tool in (0, 1):
                info['current_tool'] = tool
        
        # Parse M82/M83 for extrusion mode
        elif command == 'M82':
            # Absolute extrusion mode
            info['extrusion_mode'] = 'absolute'
        elif command == 'M83':
            # Relative extrusion mode
            info['extrusion_mode'] = 'relative'
        
        # Parse G90/G91 for movement mode
        elif command == 'G90':
            # Absolute movement mode
            info['movement_mode'] = 'absolute'
        elif command == 'G91':
            # Relative movement mode
            info['movement_mode'] = 'relative'
        
        # Parse G0/G1/G00/G01 commands
        elif command in ['G0', 'G1', 'G00', 'G01']:
            # Z height: G0 Z0.200 or G1 Z0.200
            if 'Z' in params:
                try:
                    z_height = float(params['Z'])
                    info['z_height'] = z_height
                except ValueError:
                    pass
            
            # X and Y position
            if 'X' in params:
                try:
                    x_pos = float(params['X'])
                    info['x_position'] = x_pos
                except ValueError:
                    pass
            
            if 'Y' in params:
                try:
                    y_pos = float(params['Y'])
                    info['y_position'] = y_pos
                except ValueError:
                    pass
            
            # Feedrate (F parameter) - print speed
            if 'F' in params:
                try:
                    feedrate = float(params['F'])
                    info['feedrate'] = feedrate
                except ValueError:
                    pass
            
            # E (extrusion) parameter
            if 'E' in params:
                try:
                    extrusion = float(params['E'])
                    info['extrusion'] = extrusion
                except ValueError:
                    pass
        
        # Parse SET_PRESSURE_ADVANCE command
        elif command == 'SET_PRESSURE_ADVANCE':
            # SET_PRESSURE_ADVANCE EXTRUDER=extruder1 SMOOTH_TIME=0.02
            # SET_PRESSURE_ADVANCE EXTRUDER=extruder1 ADVANCE=0.05
            for key, value in params.items():
                if key == 'EXTRUDER':
                    info['pressure_advance']['extruder'] = value
                elif key == 'SMOOTH_TIME':
                    try:
                        smooth_time = float(value)
                        info['pressure_advance']['smooth_time'] = smooth_time
                    except ValueError:
                        pass
                elif key == 'ADVANCE':
                    try:
                        advance = float(value)
                        info['pressure_advance']['advance'] = advance
                    except ValueError:
                        pass
        
        # Parse SET_VELOCITY_LIMIT command
        elif command == 'SET_VELOCITY_LIMIT':
            # SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=0.8
            # SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=5
            for key, value in params.items():
                if key == 'MINIMUM_CRUISE_RATIO':
                    try:
                        ratio = float(value)
                        info['velocity_limit']['minimum_cruise_ratio'] = ratio
                    except ValueError:
                        pass
                elif key == 'SQUARE_CORNER_VELOCITY':
                    try:
                        velocity = float(value)
                        info['velocity_limit']['square_corner_velocity'] = velocity
                    except ValueError:
                        pass

        # Parse SET_PRINT_STATS_INFO command
        elif command == 'SET_PRINT_STATS_INFO':
            if 'TOTAL_LAYER' in params:
                info['total_layer'] = int(params['TOTAL_LAYER'])
            
            if 'CURRENT_LAYER' in params:
                info['current_layer'] = int(params['CURRENT_LAYER'])

    def get_powerloss_point(self):
        """Scan powerloss files from self.prefix+".0" to self.prefix+".max_file_num-1"
        and return the SEEKPOSITION from the file with the latest MODIFYTIME.
        Returns None if no valid file is found."""
        latest_time = 0
        latest_seek_position = None
        latest_z_position = None
        
        for i in range(self.max_file_num):
            file_path = self.prefix + ".%d" % i
            if not os.path.exists(file_path):
                continue
                
            try:
                with open(file_path, "r") as f:
                    data = json.load(f)
                    modify_time = data.get('MODIFYTIME', 0)
                    seek_position = data.get('SEEKPOSITION')
                    latest_z_position = data.get('CURRENT_Z')
                    
                    if modify_time > latest_time and seek_position is not None:
                        latest_time = modify_time
                        latest_seek_position = seek_position
            except Exception:
                logging.exception("Failed to read powerloss file: %s" % file_path)
                continue
        
        return latest_seek_position, latest_z_position

    def clear_record(self):
        """Clear all powerloss record files."""
        for i in range(self.max_file_num):
            file_path = self.prefix + ".%d" % i
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
            except Exception:
                logging.exception("Failed to remove powerloss file: %s" % file_path)
        
        # Also remove init file
        try:
            if os.path.exists(self.powerloss_init_file):
                os.remove(self.powerloss_init_file)
        except Exception:
            logging.exception("Failed to remove powerloss init file: %s" % self.powerloss_init_file)
        self.current_print_file = None

MainPowerlossProgress = None

def setup_powerloss_progress(prefix, max_file_num = 1):
    global MainPowerlossProgress
    if MainPowerlossProgress is not None:
        if MainPowerlossProgress.prefix != prefix or MainPowerlossProgress.max_file_num != max_file_num:
            MainPowerlossProgress.exit()
            MainPowerlossProgress = None
        else:
            logging.debug("powerloss progress thread:[%s] already exist, will use it." %(MainPowerlossProgress.save_prog_thread.getName()))

    if MainPowerlossProgress is None:
        MainPowerlossProgress = PowerlossProgress(prefix, max_file_num)
        logging.debug("create powerloss progress thread: MainPowerlossProgress:[%s]." %(MainPowerlossProgress.save_prog_thread.getName()))
    
    return MainPowerlossProgress
