# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, sys, logging, io, re
from . import powerloss_progress
import threading
import queue
import configparser

VALID_GCODE_EXTS = ['gcode', 'GCODE', 'g', 'gco']
POWERLOSS_FILE_NAME = "/home/anisoprint/printer_data/powerloss/klipper-print.powerloss"

DEFAULT_ERROR_GCODE = """
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
"""

class VirtualSD:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:shutdown",
                                            self.handle_shutdown)
        self.printer.register_event_handler("axisz:move",self.get_zmove_value)
        # sdcard state
        sd = config.get('path')
        self.sdcard_dirname = os.path.normpath(os.path.expanduser(sd))
        self.current_file = None
        self.file_position = self.file_size = 0
        # Print Stat Tracking
        self.print_stats = self.printer.load_object(config, 'print_stats')
        # Work timer
        self.reactor = self.printer.get_reactor()
        self.must_pause_work = self.cmd_from_sd = False
        self.next_file_position = 0
        self.work_timer = None
        # wait Pause 
        self.waiting_pause_request = False
        self.extruder_switch_detected = False
        # Error handling
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.on_error_gcode = gcode_macro.load_template(
            config, 'on_error_gcode', DEFAULT_ERROR_GCODE)
        self.variables = config.getsection('save_variables')
        # Precompile regex patterns for performance
        self.gcode_pattern = re.compile(r'^\s*G[01]\b')
        self.p_pattern = re.compile(r'P([\d\.]+)')
        self.u_pattern = re.compile(r'\bU[\d\.\-]+\b')
        self.v_pattern = re.compile(r'\bV[\d\.\-]+\b')
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        for cmd in ['M20', 'M21', 'M23', 'M24', 'M25', 'M26', 'M27']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))
        for cmd in ['M28', 'M29', 'M30']:
            self.gcode.register_command(cmd, self.cmd_error)
        self.gcode.register_command(
            "SDCARD_RESET_FILE", self.cmd_SDCARD_RESET_FILE,
            desc=self.cmd_SDCARD_RESET_FILE_help)
        self.gcode.register_command(
            "SDCARD_PRINT_FILE", self.cmd_SDCARD_PRINT_FILE,
            desc=self.cmd_SDCARD_PRINT_FILE_help)

        self.gcode.register_command(
            "WAITTING_PAUSE", self.cmd_WAITTING_PAUSE,
            desc=self.cmd_WAITTING_PAUSE_help)
        
        self.gcode.register_command(
            "CHECK_POWERLOSS_FILE", self.cmd_CHECK_POWERLOSS_FILE,
            desc="Check if powerloss file exists")
        
        self.gcode.register_command(
            "PREPARE_RECOVER_PRINT_FILE", self.cmd_PREPARE_RECOVER_PRINT_FILE,
            desc=self.cmd_PREPARE_RECOVER_PRINT_FILE_help)

        self.gcode.register_command(
            "RECOVER_PRINT_FILE", self.cmd_RECOVER_PRINT_FILE,
            desc=self.cmd_RECOVER_PRINT_FILE_help)

        self.print_progress = 0
        self.printed_pos_z = 0
        self.powerloss_record_time = self.reactor.monotonic()
        self.is_start_move = False
        self.powerloss_progress = powerloss_progress.setup_powerloss_progress(POWERLOSS_FILE_NAME, 5)
        self.is_recover_print = False
        self.powerloss_file_exist = self.powerloss_progress.check_powerloss_file()
        self.powerloss_info = None
        self.powerloss_position = None
        
        # Double buffer mechanism for prefetching
        self.prefetch_queue = queue.Queue(maxsize=2)  # Queue for prefetched data
        self.prefetch_thread = None
        self.prefetch_stop_event = threading.Event()
        self.prefetch_partial_input = ""  # Shared partial input between buffers
        self.file_lock = threading.Lock()  # Lock for protecting file operations

    def cmd_WAITTING_PAUSE(self, gcmd):
        F = gcmd.get_int('F', 0)
        self.waiting_pause_request = F
        gcmd.respond_raw("WAITTING_PAUSE %d" % (self.waiting_pause_request))

    cmd_WAITTING_PAUSE_help = "WAITTING_PAUSE COMMAND  eg: WAITTING_PAUSE F=1 force puase"


    def handle_shutdown(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            try:
                readpos = max(self.file_position - 1024, 0)
                readcount = self.file_position - readpos
                self.current_file.seek(readpos)
                data = self.current_file.read(readcount + 128)
            except:
                logging.exception("virtual_sdcard shutdown read")
                return
            logging.info("Virtual sdcard (%d): %s\nUpcoming (%d): %s",
                         readpos, repr(data[:readcount]),
                         self.file_position, repr(data[readcount:]))
    
    def get_zmove_value(self, z_move_val):
        if self.work_timer is not None and self.is_start_move:
            # self.gcode.respond_raw("get z move value %.3f"%(z_move_val))
            if self.printed_pos_z != z_move_val:
                self.printed_pos_z = z_move_val
                self.powerloss_progress.record(self.file_position, "", self.printed_pos_z)
    def stats(self, eventtime):
        if self.work_timer is None:
            return False, ""
        return True, "sd_pos=%d" % (self.file_position,)
    def get_file_list(self, check_subdirs=False):
        if check_subdirs:
            flist = []
            for root, dirs, files in os.walk(
                    self.sdcard_dirname, followlinks=True):
                for name in files:
                    ext = name[name.rfind('.')+1:]
                    if ext not in VALID_GCODE_EXTS:
                        continue
                    full_path = os.path.join(root, name)
                    r_path = full_path[len(self.sdcard_dirname) + 1:]
                    size = os.path.getsize(full_path)
                    flist.append((r_path, size))
            return sorted(flist, key=lambda f: f[0].lower())
        else:
            dname = self.sdcard_dirname
            try:
                filenames = os.listdir(self.sdcard_dirname)
                return [(fname, os.path.getsize(os.path.join(dname, fname)))
                        for fname in sorted(filenames, key=str.lower)
                        if not fname.startswith('.')
                        and os.path.isfile((os.path.join(dname, fname)))]
            except:
                logging.exception("virtual_sdcard get_file_list")
                raise self.gcode.error("Unable to get file list")
    def get_status(self, eventtime):
        return {
            'file_path': self.file_path(),
            'progress': self.progress(),
            'is_active': self.is_active(),
            'file_position': self.file_position,
            'file_size': self.file_size,
        }
    def file_path(self):
        if self.current_file:
            return self.current_file.name
        return None
    def progress(self):
        if self.file_size:
            process = float(self.file_position) / self.file_size
            return process > 1.0 and 1.0 or process
        else:
            return 0.
    def is_active(self):
        return self.work_timer is not None
    def do_pause(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            while self.work_timer is not None and not self.cmd_from_sd:
                self.reactor.pause(self.reactor.monotonic() + .001)
                self.powerloss_record_time = self.reactor.monotonic()
    def do_resume(self):
        if self.work_timer is not None:
            raise self.gcode.error("RESUME called when already printing")
        self.must_pause_work = False
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW)
        self.powerloss_record_time = self.reactor.monotonic()
    def do_cancel(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
            self.print_stats.note_cancel()
        self.file_position = self.file_size = 0
        self.powerloss_record_time = self.reactor.monotonic()
        self.powerloss_file_exist = False
        self.powerloss_progress.clear_record()
    # G-Code commands
    def cmd_error(self, gcmd):
        raise gcmd.error("SD write not supported")
    def _reset_file(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
        self.file_position = self.file_size = 0
        self.extruder_switch_detected = False
        self.powerloss_file_exist = False
        self.powerloss_progress.clear_record()
        self.print_stats.reset()
        self.printer.send_event("virtual_sdcard:reset_file")
    cmd_SDCARD_RESET_FILE_help = "Clears a loaded SD File. Stops the print "\
        "if necessary"
    def cmd_SDCARD_RESET_FILE(self, gcmd):
        if self.cmd_from_sd:
            raise gcmd.error(
                "SDCARD_RESET_FILE cannot be run from the sdcard")
        self._reset_file()
    cmd_SDCARD_PRINT_FILE_help = "Loads a SD file and starts the print.  May "\
        "include files in subdirectories."
    def cmd_SDCARD_PRINT_FILE(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("Printer is already printing")
        self._reset_file()
        filename = gcmd.get("FILENAME")
        if filename[0] == '/':
            filename = filename[1:]
        self.print_progress = 0
        self._load_file(gcmd, filename, check_subdirs=True)
        file_path = self.current_file.name if self.current_file else None
        self.powerloss_file_exist = False
        self.powerloss_progress.clear_record()
        self.powerloss_progress.start_record(file_path)
        self.do_resume()
        self.printer.send_event("virtual_sdcard:print_start")
    def cmd_M20(self, gcmd):
        # List SD card
        files = self.get_file_list()
        gcmd.respond_raw("Begin file list")
        for fname, fsize in files:
            gcmd.respond_raw("%s %d" % (fname, fsize))
        gcmd.respond_raw("End file list")
    def cmd_M21(self, gcmd):
        # Initialize SD card
        gcmd.respond_raw("SD card ok")
    def cmd_M23(self, gcmd):
        # Select SD file
        if self.work_timer is not None:
            raise gcmd.error("Printer is currently printing")
        self._reset_file()
        filename = gcmd.get_raw_command_parameters().strip()
        if filename.startswith('/'):
            filename = filename[1:]
        self._load_file(gcmd, filename)
    def _load_file(self, gcmd, filename, check_subdirs=False):
        files = self.get_file_list(check_subdirs)
        flist = [f[0] for f in files]
        files_by_lower = { fname.lower(): fname for fname, fsize in files }
        fname = filename
        try:
            if fname not in flist:
                fname = files_by_lower[fname.lower()]
            fname = os.path.join(self.sdcard_dirname, fname)
            f = io.open(fname, 'r', newline='')
            f.seek(0, os.SEEK_END)
            fsize = f.tell()

            # 从文件末尾开始向前扫描，查找 "; SESSION:" 标记
            scanned_bytes = 0
            found_session = False
            buffer_size = 8192  # 每次读取 8KB
            file_pos = fsize
            buffer = ""
            
            while file_pos > 0:
                # 读取一块数据，从末尾向前
                read_size = min(buffer_size, file_pos)
                file_pos -= read_size
                f.seek(file_pos)
                chunk = f.read(read_size)
                buffer = chunk + buffer
                
                # 按行处理缓冲区
                lines = buffer.split('\n')
                buffer = lines[0]  # 保留未完成的行
                
                # 从末尾向前处理完整的行
                for i in range(len(lines) - 1, 0, -1):
                    line = lines[i]
                    scanned_bytes += len(line.encode()) + 1  # +1 for newline
                    
                    if "; SESSION:" in line:
                        found_session = True
                        break
                    
                    # 如果没有找到 SESSION，扫描到不出现";"时停止
                    if ';' not in line and ';' not in line:
                        break
                
                if found_session:
                    break
            
            # 如果找到了 SESSION，减去扫描的字节数
            if found_session:
                fsize = fsize - scanned_bytes
            f.seek(0)
        except:
            logging.exception("virtual_sdcard file open")
            raise gcmd.error("Unable to open file")
        gcmd.respond_raw("File opened:%s Size:%d" % (filename, fsize))
        gcmd.respond_raw("File selected")
        self.current_file = f
        self.file_position = 0
        self.file_size = fsize
        self.print_stats.set_current_file(filename)
    def cmd_M24(self, gcmd):
        # Start/resume SD print
        self.do_resume()
    def cmd_M25(self, gcmd):
        # Pause SD print
        self.do_pause()
    def cmd_M26(self, gcmd):
        # Set SD position
        if self.work_timer is not None:
            raise gcmd.error("Printer is currently printing")
        pos = gcmd.get_int('S', minval=0)
        self.file_position = pos
    def cmd_M27(self, gcmd):
        # Report SD print status
        if self.current_file is None:
            gcmd.respond_raw("Not SD printing.")
            return
        gcmd.respond_raw("SD printing byte %d/%d"
                         % (self.file_position, self.file_size))
    def get_file_position(self):
        return self.next_file_position
    def set_file_position(self, pos):
        self.next_file_position = pos
    def is_cmd_from_sd(self):
        return self.cmd_from_sd
    
    """
    Handle M39 command for SD card recovery print functionality.
    
    This command resumes printing from a specific position in a G-code file.
    It expects the format: M39 <filename>*<filepos>
    
    Example:
    ---------
    M39 /path/to/file.gcode*12345
    """
    def cmd_M39(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("Printer is currently printing")
        try:
            orig = gcmd.get_commandline()
            command_line = orig[orig.find("M39") + 4:].strip()
            if '*' in command_line:
                cmd_msg = command_line.split("*")
                filename = str(cmd_msg[0]).strip()
                filepos  = int(cmd_msg[1])
                gcmd.respond_raw("file_name = %s filepos = %d"%(filename,filepos))
            else:
                gcmd.respond_error("cmd should has pos parameter")
                return
        except:
            raise gcmd.error("Unable to extract filename")
            # check if the file is exist
        if not os.path.exists(filename):
            gcmd.respond_raw("Error: File %s not exsit!" % filename)
            return
        files = filename.rsplit("/", 1)
        if len(files) != 2 or not files[1].endswith('gcode'):
            gcmd.respond_raw("warning : Ivalide file type!" % filename)
            return
        self.print_progress = 0
        self.sdcard_dirname = os.path.normpath(files[0])
        self._load_file(gcmd, files[1], check_subdirs=True)
        if '*' in command_line:
            self.file_position = filepos
        try:
            self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard recover seek")
        self.is_start_move = False
        self.do_resume()
        self.is_recover_print = True
        gcmd.respond_raw("get recover print %s %d" % (filename, self.file_position))

    def cmd_CHECK_POWERLOSS_FILE(self, gcmd):
        # Check if powerloss file exists
        if self.powerloss_file_exist:
            gcmd.respond_raw("[POWERLOSS] Powerloss file exists. [FILE: %s]" % self.powerloss_progress.current_print_file)
        else:
            gcmd.respond_raw("[POWERLOSS] No powerloss file found.")

    cmd_PREPARE_RECOVER_PRINT_FILE_help = "Prepare for recovering print from powerloss. "\
        "This command checks for the existence of a powerloss file and "\
        "retrieves the last known print position. It does not start the recovery "\
        "process but provides information about the powerloss state."
    
    def cmd_PREPARE_RECOVER_PRINT_FILE(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("PREPARE RECOVER PRINT Error: Printer is currently printing.")
        
        # Check if powerloss file exists
        if not self.powerloss_file_exist:
            gcmd.respond_raw("No powerloss file found. Cannot recover print.")
            return
        
        # Get powerloss point (byte position)
        powerloss_position, powerloss_z = self.powerloss_progress.get_powerloss_point()
        if powerloss_position is None:
            gcmd.respond_raw("No valid powerloss position found.")
            return
        
        self.powerloss_position = powerloss_position
        gcmd.respond_raw("Powerloss position: %d bytes, Z height: %.2f" % (powerloss_position, powerloss_z))
        
        # Check if current print file is available
        if not self.powerloss_progress.current_print_file:
            gcmd.respond_raw("No current print file found in powerloss record.")
            return
        
        file_path = self.powerloss_progress.current_print_file
        if not os.path.exists(file_path):
            gcmd.respond_raw("Print file not found: %s" % file_path)
            return
        
        # Start asynchronous file scanning in background thread
        gcmd.respond_raw("Starting async file scan (non-blocking)...")
        scan_id = self.powerloss_progress.scan_print_file_info_async(stop_byte=powerloss_position)
        
        # Poll for scan completion with timeout
        max_wait_time = 60.0  # Maximum 60 seconds wait
        poll_interval = 0.5   # Check every 0.5 seconds
        elapsed = 0.0
        
        while elapsed < max_wait_time:
            scan_result = self.powerloss_progress.get_scan_result(scan_id)
            status = scan_result.get('status')
            
            if status == 'completed':
                self.powerloss_info = scan_result.get('data', {})
                gcmd.respond_raw("File scan completed: %d gcode lines processed" % 
                               self.powerloss_info.get('gcode_lines', 0))
                break
            elif status == 'failed':
                error = scan_result.get('error', 'Unknown error')
                gcmd.respond_raw("File scan failed: %s" % error)
                return
            
            # Still pending, wait a bit
            self.reactor.pause(self.reactor.monotonic() + poll_interval)
            elapsed += poll_interval
            
            # Provide progress feedback every 5 seconds
            if int(elapsed) % 5 == 0 and elapsed > 0:
                gcmd.respond_raw("Scanning in progress... (%.0fs elapsed)" % elapsed)
        else:
            # Timeout
            gcmd.respond_raw("File scan timeout after %.0fs. Aborting recovery." % max_wait_time)
            return
        
        # Apply recovered parameters
        self._apply_prepare_recover_parameters(gcmd, self.powerloss_info, powerloss_z)
        
        gcmd.respond_raw("Prepare recover print started: %s at position %d, Z height: %.2f" % (file_path, self.file_position, powerloss_z))
    
    def _apply_prepare_recover_parameters(self, gcmd, file_info, powerloss_z):
        """Apply recovered parameters as gcode commands."""
        gcode_script = []
        new_powerloss_z = powerloss_z
        
        # Apply extrusion mode (M82/M83)
        extrusion_mode = file_info.get('extrusion_mode')
        if extrusion_mode == 'absolute':
            gcode_script.append("M82")
        elif extrusion_mode == 'relative':
            gcode_script.append("M83")
        
        # Apply movement mode (G90/G91)
        movement_mode = file_info.get('movement_mode')
        if movement_mode == 'absolute':
            gcode_script.append("G90")
        elif movement_mode == 'relative':
            gcode_script.append("G91")

        current_tool = file_info.get('current_tool', 0)
        
        # Apply nozzle temperature
        pre_nozzle_temp = 180  # Default safe temp
        left_nozzle_temp = file_info.get('left_nozzle_temp')
        right_nozzle_temp = file_info.get('right_nozzle_temp')

        if current_tool == 0 and left_nozzle_temp is not None:
            pre_nozzle_temp = left_nozzle_temp - 50
        elif current_tool == 1 and right_nozzle_temp is not None:
            pre_nozzle_temp = right_nozzle_temp - 50
        
        if pre_nozzle_temp < 180: pre_nozzle_temp = 180
        gcode_script.append("M104 S%.1f T%d" % (pre_nozzle_temp, current_tool))
        
        # Apply bed temperature
        bed_temp = file_info.get('bed_temp')
        if bed_temp is not None:
            gcode_script.append("M140 S%.1f" % bed_temp)
    
        # Apply fan speeds
        fan_speeds = file_info.get('fan_speeds', {})
        for fan_index, speed in fan_speeds.items():
            gcode_script.append("M106 P%d S%.1f" % (fan_index, speed))

        # Apply chamber temperature
        chamber_temp = file_info.get('chamber_temp')
        if chamber_temp is not None:
            gcode_script.append("M141 S%.1f" % chamber_temp)
        
        # Apply acceleration
        acceleration = file_info.get('acceleration')
        if acceleration is not None:
            gcode_script.append("M204 S%.1f" % acceleration)

        # Apply pressure advance settings
        pressure_advance = file_info.get('pressure_advance', {})
        if pressure_advance:
            pa_cmd = "SET_PRESSURE_ADVANCE"
            if 'extruder' in pressure_advance:
                pa_cmd += " EXTRUDER=%s" % pressure_advance['extruder']
            if 'smooth_time' in pressure_advance:
                pa_cmd += " SMOOTH_TIME=%.3f" % pressure_advance['smooth_time']
            if 'advance' in pressure_advance:
                pa_cmd += " ADVANCE=%.3f" % pressure_advance['advance']
            gcode_script.append(pa_cmd)
        
        # Apply velocity limit settings
        velocity_limit = file_info.get('velocity_limit', {})
        if velocity_limit:
            vl_cmd = "SET_VELOCITY_LIMIT"
            if 'minimum_cruise_ratio' in velocity_limit:
                vl_cmd += " MINIMUM_CRUISE_RATIO=%.2f" % velocity_limit['minimum_cruise_ratio']
            if 'square_corner_velocity' in velocity_limit:
                vl_cmd += " SQUARE_CORNER_VELOCITY=%.1f" % velocity_limit['square_corner_velocity']
            gcode_script.append(vl_cmd)

        # 等待喷嘴加热到位
        gcode_script.append("M118 [POWERLOSS] Preparing to heat nozzle T%d to %.1f°C" % (current_tool, pre_nozzle_temp))
        gcode_script.append("M109 S%.1f T%d" % (pre_nozzle_temp, current_tool))

        # 如果是左喷嘴，切换纤维
        if current_tool == 0:
            gcode_script.append("M2800")

        # 加载调平数据
        if self.config.has_section('bed_mesh model_leveling'):
            gcode_script.append("BED_MESH_PROFILE LOAD=model_leveling")
            gcmd.respond_raw("Loading bed mesh profile: model_leveling")
        elif self.config.has_section('bed_mesh default'):
            gcode_script.append("BED_MESH_PROFILE LOAD=default")
            gcmd.respond_raw("Loading bed mesh profile: default")
        
        # 获取z offset值
        cfg_path = os.path.expanduser(self.variables.get('filename'))
        config = configparser.ConfigParser()
        config.read(cfg_path)
        zoffset = config.getfloat('Variables', 'zoffset')
        
        if zoffset is not None and current_tool == 1:
            new_powerloss_z = powerloss_z + zoffset
            gcmd.respond_raw("Right nozzle z offset applied: %.3f, adjusted Z: %.3f" % (zoffset, new_powerloss_z))
        
        # current z 强制设定
        gcode_script.append("SET_KINEMATIC_POSITION Z=%.3f" % new_powerloss_z)
        # 相对向下移动4mm
        gcode_script.append("G1 Z%.3f F300" % (new_powerloss_z + 4.0))

        # xy归零
        gcode_script.append("M118 [POWERLOSS] Prepare to home X and Y axes")
        gcode_script.append("G28 X Y")

        # 移动到擦料站位置,结束
        gcode_script.append("MOVE_TO_WASTE_STATION")
        gcode_script.append("M400")
        gcode_script.append("M118 [POWERLOSS] Prepare to recover print finished, current_tool: %d" % current_tool)
        
        # Execute all gcode commands
        if gcode_script:
            gcmd.respond_raw("Prepare applying recovery parameters...")
            for cmd in gcode_script:
                try:
                    cmd = cmd + '\n'
                    self.gcode.run_script_from_command(cmd)
                except Exception as e:
                    logging.warning("Failed to execute recovery command %s: %s" % (cmd, str(e)))
                    gcmd.respond_raw("Warning: Failed to execute %s" % cmd)
        else:
            gcmd.respond_raw("No recovery parameters to apply.")
    
    cmd_RECOVER_PRINT_FILE_help = "Automatically recover print from powerloss. "\
        "This command scans powerloss files to find the last print position, "\
        "extracts gcode parameters (temperatures, speeds, positions), and "\
        "resumes printing from that position. "\
        "File scanning is done asynchronously to avoid blocking."
    
    def cmd_RECOVER_PRINT_FILE(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("RECOVER PRINT Error: Printer is currently printing.")
        
        if self.powerloss_info is None:
            gcmd.respond_raw("Error: No prepared powerloss info found. Please run PREPARE_RECOVER_PRINT_FILE first.")
            return

        file_path = self.powerloss_progress.current_print_file
        if not os.path.exists(file_path):
            gcmd.respond_raw("Print file not found: %s" % file_path)
            return
        
        # Extract filename from full path
        filename = os.path.basename(file_path)
        dirname = os.path.dirname(file_path)
        
        # Set sdcard directory to file's directory
        self.sdcard_dirname = os.path.normpath(dirname)
        
        # Load the file
        self._reset_file()
        self._load_file(gcmd, filename, check_subdirs=True)
        
        # Set file position to powerloss position
        self.file_position = self.powerloss_position
        self.print_progress = 0
        
        # Seek to position
        try:
            self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard recover seek")
            gcmd.respond_raw("Failed to seek to position %d" % self.powerloss_position)
            return
        
        # Apply recovered parameters
        self._apply_recovery_parameters(gcmd, self.powerloss_info)
        
        # Start recording again
        self.powerloss_progress.start_record(file_path)
        
        # Resume printing
        self.is_start_move = False
        self.do_resume()
        self.is_recover_print = True
        
        gcmd.respond_raw("Recover print started: %s at position %d" % (filename, self.file_position))
    
    def _apply_recovery_parameters(self, gcmd, file_info):
        """Apply recovered parameters as gcode commands."""
        gcode_script = []

        # Apply bed temperature
        bed_temp = file_info.get('bed_temp')
        if bed_temp is not None:
            gcode_script.append("M140 S%.1f" % bed_temp)

        current_tool = file_info.get('current_tool', 0)
        
        # Apply nozzle temperature
        pre_nozzle_temp = 180  # Default safe temp
        left_nozzle_temp = file_info.get('left_nozzle_temp')
        right_nozzle_temp = file_info.get('right_nozzle_temp')

        # Handle temperature adjustments based on current tool
        # Ensure temperatures are not None before performing operations
        if current_tool == 0:
            if right_nozzle_temp is not None:
                right_nozzle_temp = right_nozzle_temp - 80
                if right_nozzle_temp < 180: 
                    right_nozzle_temp = 180
        elif current_tool == 1:
            if left_nozzle_temp is not None:
                left_nozzle_temp = left_nozzle_temp - 80
                if left_nozzle_temp < 180: 
                    left_nozzle_temp = 180
        
        # Set nozzle temperatures
        if left_nozzle_temp is not None:
            gcode_script.append("M104 S%.1f T0" % (left_nozzle_temp))
        if right_nozzle_temp is not None:
            gcode_script.append("M104 S%.1f T1" % (right_nozzle_temp))
            
        # Wait for temperatures
        gcode_script.append("M118 [POWERLOSS] Heating nozzles to target temperatures.")
        if left_nozzle_temp is not None:
            gcode_script.append("M109 S%.1f T0" % (left_nozzle_temp))
        if right_nozzle_temp is not None:
            gcode_script.append("M109 S%.1f T1" % (right_nozzle_temp))
        if bed_temp is not None:
            gcode_script.append("M190 S%.1f" % bed_temp)
        
        # Apply chamber temperature
        chamber_temp = file_info.get('chamber_temp')
        if chamber_temp is not None:
            gcode_script.append("M141 S%.1f" % chamber_temp)
        
        # Tool-specific cleaning procedures
        gcode_script.append("M118 [POWERLOSS] Cleaning nozzle T%d before resuming print." % current_tool)
        if current_tool == 0:
            gcode_script.append("CLEAN_LEFT_NOZZLE")
        elif current_tool == 1:
            gcode_script.append("T1 NO_OFFSET=1")
            gcode_script.append("MOVE_RIGHT_PRELOAD_POSITION")
            gcode_script.append("PRELOADING_RETRACT_FILAMENT")
            gcode_script.append("RELEASE_WASTE_FILAMENT")
            gcode_script.append("MOVE_RIGHT_BRUSH_POSITION")
            gcode_script.append("BRUSH_NOZZLE2")
            gcode_script.append("MOVE_OUT_WASTE_STATION")
        
        gcode_script.append("M118 [POWERLOSS] Recovering print with nozzle T%d" % current_tool)
        gcode_script.append("T%d" % current_tool)
        
        # Apply position (X, Y, Z, E) if available
        x_position = file_info.get('x_position')
        y_position = file_info.get('y_position')
        z_height = file_info.get('z_height')
        
        # First move to XY position if available
        if x_position is not None or y_position is not None:
            move_cmd = "G1"
            if x_position is not None:
                move_cmd += " X%.3f" % x_position
            if y_position is not None:
                move_cmd += " Y%.3f" % y_position
            move_cmd += " F12000"
            gcode_script.append(move_cmd)

        # Then move to Z height if available
        if z_height is not None:
            gcode_script.append("G1 Z%.3f F600" % z_height)
        
        if current_tool == 1:
            gcode_script.append("G1 E6 F600")  # 恢复挤出6mm以补偿预拉伸

        # Apply feedrate (print speed)
        feedrate = file_info.get('feedrate')
        if feedrate is not None:
            gcode_script.append("G1 F%.1f" % feedrate)
        
        gcode_script.append("M400")
        gcode_script.append("M118 [POWERLOSS] Recover print finished.")
        
        # Execute all gcode commands
        if gcode_script:
            gcmd.respond_raw("Applying recovery parameters...")
            for cmd in gcode_script:
                try:
                    cmd = cmd + '\n'
                    self.gcode.run_script_from_command(cmd)
                except Exception as e:
                    logging.warning("Failed to execute recovery command %s: %s" % (cmd, str(e)))
                    gcmd.respond_raw("Warning: Failed to execute %s" % cmd)
        else:
            gcmd.respond_raw("No recovery parameters to apply.")
    
    def _prefetch_worker(self, initial_partial):
        """
        Background worker thread for prefetching and processing file data.
        Uses shared self.current_file with lock protection.
        Reads 8192 bytes, splits into lines, processes with handle_extruder_mode,
        and puts the result into prefetch_queue for the main thread.
        """
        partial_input = initial_partial
        pending_data = None  # Buffer to hold data if queue is full
        
        while not self.prefetch_stop_event.is_set():
            try:
                # If we have pending data from previous iteration, try to put it first
                if pending_data is not None:
                    try:
                        self.prefetch_queue.put(pending_data, timeout=0.5)
                        pending_data = None  # Successfully put, clear pending
                    except queue.Full:
                        if self.prefetch_stop_event.is_set():
                            break
                        continue  # Keep trying to put pending data
                
                # Read next chunk with lock protection
                with self.file_lock:
                    if self.current_file is None:
                        self.prefetch_queue.put((None, partial_input, True))
                        break
                    try:
                        data = self.current_file.read(8192)
                    except Exception as e:
                        logging.exception("virtual_sdcard prefetch worker read error")
                        self.prefetch_queue.put((None, partial_input, True))
                        break
                
                if not data:
                    # End of file - signal with None
                    self.prefetch_queue.put((None, partial_input, True))
                    break
                
                # Process the data (outside lock - CPU work)
                lines = data.split('\n')
                lines[0] = partial_input + lines[0]
                partial_input = lines.pop()  # Save incomplete line for next iteration
                
                # Process lines with handle_extruder_mode
                processed_lines = self.handle_extruder_mode(lines)
                processed_lines.reverse()  # Reverse for pop() usage
                
                # Put processed data into queue (blocks if queue is full)
                try:
                    self.prefetch_queue.put(
                        (processed_lines, partial_input, False), 
                        timeout=0.5
                    )
                except queue.Full:
                    # Save data for next iteration, don't discard!
                    pending_data = (processed_lines, partial_input, False)
                    if self.prefetch_stop_event.is_set():
                        break
                    continue
                    
            except Exception as e:
                logging.exception("virtual_sdcard prefetch worker error")
                self.prefetch_queue.put((None, partial_input, True))
                break
    
    def _start_prefetch_thread(self, initial_partial):
        """Start the prefetch worker thread using shared file handle with lock."""
        self.prefetch_stop_event.clear()
        # Clear any old data in queue
        while not self.prefetch_queue.empty():
            try:
                self.prefetch_queue.get_nowait()
            except queue.Empty:
                break
        
        self.prefetch_thread = threading.Thread(
            target=self._prefetch_worker,
            args=(initial_partial,),
            daemon=True
        )
        self.prefetch_thread.start()
    
    def _stop_prefetch_thread(self):
        """Stop the prefetch worker thread."""
        self.prefetch_stop_event.set()
        if self.prefetch_thread is not None:
            self.prefetch_thread.join(timeout=2.0)
            self.prefetch_thread = None
        # Clear queue
        while not self.prefetch_queue.empty():
            try:
                self.prefetch_queue.get_nowait()
            except queue.Empty:
                break

    def handle_extruder_mode(self, lines):
        """Process lines and insert extruder mode commands.
        Inserted commands are tuples (line, is_injected) where is_injected=True
        means the line should not count towards file_position.
        """
        processed_lines = []
        gcode_pattern = self.gcode_pattern
        u_pattern = self.u_pattern
        v_pattern = self.v_pattern
        p_pattern = self.p_pattern
        
        for line in lines:
            # Skip comments early
            stripped = line.strip()
            if stripped.startswith(';'):
                processed_lines.append((line, False))  # Original line
                continue
            
            # Only process G0/G1 commands with U/V parameters
            if gcode_pattern.match(line):
                # Check for U and V parameters in one pass
                has_u = u_pattern.search(line)
                has_v = v_pattern.search(line)
                
                if has_u and has_v:
                    # Both U and V present
                    p_match = p_pattern.search(line)
                    p_value = float(p_match.group(1)) if p_match else 0
                    processed_lines.append((f"SET_EXTRUDER_MODE S=1 P={p_value}", True))  # Injected
                elif has_u:
                    # Only U present
                    processed_lines.append(("SET_EXTRUDER_MODE S=0", True))  # Injected
                elif has_v:
                    # Only V present
                    processed_lines.append(("SET_EXTRUDER_MODE S=2", True))  # Injected
            
            processed_lines.append((line, False))  # Original line
        
        return processed_lines

    # Background work timer with double-buffer prefetching
    def work_handler(self, eventtime):
        logging.info("Starting SD card print (position %d)", self.file_position)
        self.reactor.unregister_timer(self.work_timer)
        try:
            with self.file_lock:
                self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard seek")
            self.work_timer = None
            return self.reactor.NEVER
        self.print_stats.note_start()
        gcode_mutex = self.gcode.get_mutex()
        
        # Double buffer mechanism:
        # - current_buffer: lines being processed by main thread
        # - prefetch_queue: next buffer being prepared by worker thread (with lock protection)
        current_buffer = []  # Current lines being processed
        partial_input = ""   # Partial line from previous read
        prefetch_started = False
        end_of_file = False
        error_message = None
        data_flag = False
        
        while not self.must_pause_work:
            if not current_buffer:
                if end_of_file:
                    # Already processed all data, exit
                    with self.file_lock:
                        if self.current_file is not None:
                            self.current_file.close()
                            self.current_file = None
                    self.powerloss_file_exist = False
                    self.powerloss_progress.clear_record()
                    logging.info("Finished SD card print")
                    self.gcode.run_script("PRINT_COMPLETED")
                    self.gcode.respond_raw("Done printing file")
                    break
                
                if not prefetch_started:
                    # First read - do synchronously by main thread to fill first buffer
                    try:
                        with self.file_lock:
                            data = self.current_file.read(8192)
                    except:
                        logging.exception("virtual_sdcard read")
                        break
                    if not data:
                        # Empty file
                        end_of_file = True
                        continue
                    
                    lines = data.split('\n')
                    lines[0] = partial_input + lines[0]
                    partial_input = lines.pop()
                    current_buffer = self.handle_extruder_mode(lines)
                    current_buffer.reverse()
                    
                    # Start prefetch thread for next buffer (uses shared file handle with lock)
                    self._start_prefetch_thread(partial_input)
                    prefetch_started = True
                    self.reactor.pause(self.reactor.NOW)
                    continue
                else:
                    # Get next buffer from prefetch queue
                    try:
                        prefetch_data = self.prefetch_queue.get(timeout=0.1)
                        prefetched_lines, new_partial, is_eof = prefetch_data
                        
                        if is_eof or prefetched_lines is None:
                            end_of_file = True
                            partial_input = new_partial
                            self._stop_prefetch_thread()
                            continue
                        
                        current_buffer = prefetched_lines
                        partial_input = new_partial
                        self.reactor.pause(self.reactor.NOW)
                        continue
                        
                    except queue.Empty:
                        # Prefetch not ready yet, wait a bit
                        self.reactor.pause(self.reactor.monotonic() + 0.001)
                        continue
            
            # Pause if any other request is pending in the gcode class
            if gcode_mutex.test():
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                continue
            
            # Dispatch command
            self.cmd_from_sd = True
            item = current_buffer.pop()
            
            # Handle both tuple (line, is_injected) and plain string formats
            if isinstance(item, tuple):
                line, is_injected = item
            else:
                line, is_injected = item, False

            if not self.extruder_switch_detected and ("switch extruder type to" in line or "; switch extruder type to:" in line):
                self.extruder_switch_detected = True
                self.gcode.run_script("M118 [PAUSE] pause enabling")

            if "Cutting completed" in line or "; Cutting completed." in line:
                if self.waiting_pause_request == 1:
                    self.gcode.run_script("M400")
                    self.gcode.run_script("PAUSE F=1")
                    self.gcode.run_script("WAITTING_PAUSE F=0")
                    self.gcode.run_script("M118 [PAUSE] Delay pause completed!")

            # Only update file_position for original lines, not injected commands
            if not is_injected:
                if sys.version_info.major >= 3:
                    next_file_position = self.file_position + len(line.encode()) + 1
                else:
                    next_file_position = self.file_position + len(line) + 1
                self.next_file_position = next_file_position
            
            try:
                # logging.info("SD card line (%d): %s", self.file_position, repr(line))
                if not line.startswith(';') and line.strip() != '':
                    self.gcode.run_script(line)
                if not self.is_start_move and (line.startswith('G1') or line.startswith('G0')) :
                    self.gcode.respond_raw("Print move start: %s"%(line))
                    self.is_start_move = True
            except self.gcode.error as e:
                error_message = str(e)
                try:
                    self.gcode.run_script(self.on_error_gcode.render())
                except:
                    logging.exception("virtual_sdcard on_error")
                break
            except:
                logging.exception("virtual_sdcard dispatch")
                break
            self.cmd_from_sd = False
            
            # Only update file_position for original lines
            if not is_injected:
                self.file_position = self.next_file_position

            # 500ms record powerloss file
            now = self.reactor.monotonic()
            record_timeout = now - self.powerloss_record_time
            # self.gcode.respond_raw("check record timeout .... %.3f is_start_move = %d"%(record_timeout,self.is_start_move))
            if self.is_start_move and record_timeout >= 0.5 :
                self.powerloss_progress.record(self.file_position, line, self.printed_pos_z)
                self.powerloss_record_time = now

            # Do we need to skip around? (only check for non-injected lines)
            if not is_injected and self.next_file_position != next_file_position:
                # Position changed externally, need to restart prefetch
                self._stop_prefetch_thread()
                try:
                    with self.file_lock:
                        self.current_file.seek(self.file_position)
                except:
                    logging.exception("virtual_sdcard seek")
                    self.work_timer = None
                    return self.reactor.NEVER
                current_buffer = []
                partial_input = ""
                prefetch_started = False
                end_of_file = False
        
        # Cleanup prefetch thread on exit
        self._stop_prefetch_thread()
        
        logging.info("Exiting SD card print (position %d)", self.file_position)
        self.work_timer = None
        self.cmd_from_sd = False
        if error_message is not None:
            self.print_stats.note_error(error_message)
        elif self.current_file is not None:
            self.print_stats.note_pause()
        else:
            self.print_stats.note_complete()
        return self.reactor.NEVER

def load_config(config):
    return VirtualSD(config)
