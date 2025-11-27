import logging

class BlindsHallMonitor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # 获取霍尔传感器对象名称
        self.sensor1_name = "filament_switch_sensor blinds_hall_sensor1"
        self.sensor2_name = "filament_switch_sensor blinds_hall_sensor2"
        
        # 配置参数
        self.state_change_timeout = config.getfloat('state_change_timeout', 60.0, above=0.0)
        self.recovery_timeout = config.getfloat('recovery_timeout', 60.0, above=0.0)
        self.check_interval = config.getfloat('check_interval', 0.5, above=0.0)
        
        # 状态管理
        self.monitoring_active = False
        self.recovery_mode_active = False
        self.expected_state_change_time = None
        self.expected_final_state = None
        self.initial_state = None
        self.monitoring_timer = None
        
        # 当前状态缓存
        self.last_sensor1_state = None
        self.last_sensor2_state = None
        
        # 注册G代码命令
        self.gcode.register_command(
            'START_BLINDS_MONITOR', 
            self.cmd_START_BLINDS_MONITOR,
            desc=self.cmd_START_BLINDS_MONITOR_help
        )
        self.gcode.register_command(
            'STOP_BLINDS_MONITOR', 
            self.cmd_STOP_BLINDS_MONITOR,
            desc=self.cmd_STOP_BLINDS_MONITOR_help
        )

        self.gcode.register_command(
            'M141', 
            self.cmd_M141,
            desc=self.cmd_M141_help
        )
        self.gcode.register_command(
            'M191', 
            self.cmd_M191,
            desc=self.cmd_M191_help
        )
        
        # 注册事件处理
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
    
    def _handle_ready(self):
        try:
            sensor1 = self.printer.lookup_object(self.sensor1_name)
            sensor2 = self.printer.lookup_object(self.sensor2_name)
            logging.info("BlindsHallMonitor: Sensors initialized successfully")
        except Exception as e:
            logging.error("BlindsHallMonitor: Failed to find sensors: %s" % str(e))
    
    def _get_sensor_states(self):
        try:
            sensor1 = self.printer.lookup_object(self.sensor1_name)
            sensor2 = self.printer.lookup_object(self.sensor2_name)
            
            # 获取传感器状态
            eventtime = self.reactor.monotonic()
            status1 = sensor1.get_status(eventtime)
            status2 = sensor2.get_status(eventtime)
            
            return (
                bool(status1.get('filament_detected', False)),
                bool(status2.get('filament_detected', False))
            )
        except Exception as e:
            logging.error("BlindsHallMonitor: Failed to get sensor states: %s" % str(e))
            return None, None
    
    def _get_blinds_state(self, sensor1_state, sensor2_state):
        if sensor1_state is None or sensor2_state is None:
            return "UNKNOWN"
        
        if sensor1_state and not sensor2_state:
            return "OPEN"  # 开门状态
        elif not sensor1_state and sensor2_state:
            return "CLOSED"  # 关门状态
        elif sensor1_state and sensor2_state:
            return "BOTH_HIGH"  # 异常状态
        else:
            return "BOTH_LOW"  # 异常状态
    
    def _start_recovery_mode(self, initial_state):
        self.gcode.respond_info("BlindsHallMonitor: Starting recovery mode for initial state: %s" % initial_state)
        
        # 关闭传感器电机
        self.gcode.run_script_from_command("SET_PIN PIN=SENSOR_MOTOR VALUE=0.00")
        self.gcode.respond_info("BlindsHallMonitor: Turned off sensor motor")
        
        # 设置恢复模式参数
        self.recovery_mode_active = True
        self.expected_final_state = "CLOSED"
        self.expected_state_change_time = self.reactor.monotonic() + self.recovery_timeout
        self.initial_state = initial_state
        
        # 获取当前状态
        sensor1_state, sensor2_state = self._get_sensor_states()
        self.last_sensor1_state = sensor1_state
        self.last_sensor2_state = sensor2_state
        
        logging.info("BlindsHallMonitor: Recovery mode started, waiting %.1f seconds for CLOSED state" % 
                    self.recovery_timeout)
        
        # 启动监控循环
        self._start_monitoring_loop()
    
    def _check_recovery_state(self):
        if not self.recovery_mode_active:
            return
        
        current_time = self.reactor.monotonic()
        sensor1_state, sensor2_state = self._get_sensor_states()
        
        if sensor1_state is None or sensor2_state is None:
            return
        
        # 获取当前百叶窗状态
        current_blinds_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        # 更新状态缓存
        state_changed = (sensor1_state != self.last_sensor1_state or 
                        sensor2_state != self.last_sensor2_state)
        
        if state_changed:
            logging.info("BlindsHallMonitor: State changed to %s during recovery" % current_blinds_state)
            
            # 更新最后状态
            self.last_sensor1_state = sensor1_state
            self.last_sensor2_state = sensor2_state
            
            # 检查是否达到预期状态
            if current_blinds_state == self.expected_final_state:
                logging.info("BlindsHallMonitor: Recovery successful, reached CLOSED state")
                self._stop_monitoring()
                return
        
        # 检查恢复超时
        if current_time > self.expected_state_change_time:
            error_msg = "Blinds Hall state exception: Failed to reach CLOSED state after recovery timeout"
            logging.error(error_msg)
            self.gcode.respond_info("ERROR: " + error_msg)
            self.printer.invoke_shutdown(error_msg)
            self._stop_monitoring()
    
    def _validate_initial_state(self, current_state):
        if current_state in ["BOTH_HIGH", "BOTH_LOW", "UNKNOWN"]:
            self._start_recovery_mode(current_state)
            return False
        return True
    
    def _validate_state_change(self, initial_state, target_state):
        if initial_state == target_state:
            error_msg = "Blinds Hall state exception: Initial state is same as target state - %s" % initial_state
            logging.error(error_msg)
            self.gcode.respond_info("ERROR: " + error_msg)
            self.printer.invoke_shutdown(error_msg)
            return False
        return True
    
    def _check_state_change(self):
        if not self.monitoring_active:
            return
        
        if self.recovery_mode_active:
            self._check_recovery_state()
            return
        
        # 正常状态变化检查逻辑
        current_time = self.reactor.monotonic()
        sensor1_state, sensor2_state = self._get_sensor_states()
        
        if sensor1_state is None or sensor2_state is None:
            return
        
        # 获取当前百叶窗状态
        current_blinds_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        # 检查是否处于异常状态
        if current_blinds_state in ["BOTH_HIGH", "BOTH_LOW"]:
            error_msg = "Blinds Hall state exception: Invalid state detected during monitoring - %s" % current_blinds_state
            logging.error(error_msg)
            self.gcode.respond_info("ERROR: " + error_msg)
            self.printer.invoke_shutdown(error_msg)
            self._stop_monitoring()
            return
        
        # 更新状态缓存
        state_changed = (sensor1_state != self.last_sensor1_state or 
                        sensor2_state != self.last_sensor2_state)
        
        if state_changed:
            logging.info("BlindsHallMonitor: State changed to %s" % current_blinds_state)
            
            # 更新最后状态
            self.last_sensor1_state = sensor1_state
            self.last_sensor2_state = sensor2_state
            
            # 检查是否达到预期状态
            if (self.expected_final_state is not None and 
                current_blinds_state == self.expected_final_state):
                logging.info("BlindsHallMonitor: Expected state reached successfully")
                self._stop_monitoring()
                return
        
        # 检查超时
        if (self.expected_state_change_time is not None and 
            current_time > self.expected_state_change_time):
            error_msg = "Blinds Hall state exception: State change timeout"
            logging.error(error_msg)
            self.gcode.respond_info("ERROR: " + error_msg)
            self.printer.invoke_shutdown(error_msg)
            self._stop_monitoring()
    
    def _start_monitoring_loop(self):
        if self.monitoring_timer is None:
            self.monitoring_timer = self.reactor.register_timer(
                self._monitoring_loop, self.reactor.NOW)
            logging.info("BlindsHallMonitor: Monitoring loop started")
    
    def _stop_monitoring_loop(self):
        if self.monitoring_timer is not None:
            self.reactor.unregister_timer(self.monitoring_timer)
            self.monitoring_timer = None
            logging.info("BlindsHallMonitor: Monitoring loop stopped")
    
    def _start_state_change_monitoring(self, target_state):
        sensor1_state, sensor2_state = self._get_sensor_states()
        current_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        if not self._validate_initial_state(current_state):
            return
        
        if not self._validate_state_change(current_state, target_state):
            return
        
        self.initial_state = current_state
        self.expected_final_state = target_state
        self.expected_state_change_time = self.reactor.monotonic() + self.state_change_timeout
        self.last_sensor1_state = sensor1_state
        self.last_sensor2_state = sensor2_state
        self.monitoring_active = True
        self.recovery_mode_active = False
        
        logging.info("BlindsHallMonitor: Monitoring state change from %s to %s, timeout: %.1fs" % 
                    (current_state, target_state, self.state_change_timeout))
        
        # 启动监控循环
        self._start_monitoring_loop()
    
    def _stop_monitoring(self):
        self.monitoring_active = False
        self.recovery_mode_active = False
        self.expected_state_change_time = None
        self.expected_final_state = None
        self.initial_state = None
        self._stop_monitoring_loop()
        logging.info("BlindsHallMonitor: Monitoring stopped")
    
    def _monitoring_loop(self, eventtime):
        if self.monitoring_active or self.recovery_mode_active:
            self._check_state_change()
            return eventtime + self.check_interval
        else:
            self._stop_monitoring_loop()
            return self.reactor.NEVER
    
    def start_open_to_close_monitoring(self):
        self._start_state_change_monitoring("CLOSED")
    
    def start_close_to_open_monitoring(self):
        self._start_state_change_monitoring("OPEN")
    
    def _parse_gcode_parameters(self, gcmd):
        params = {}
        command_line = gcmd.get_commandline()
        
        parts = command_line.split()
        for part in parts[1:]:
            if '=' in part:
                key, value = part.split('=', 1)
                params[key.upper()] = value
            else:
                if part.upper().startswith('S'):
                    params['S'] = part[1:]
        
        return params
    
    def _wait_for_temperature(self, target_temp, timeout=300):
        try:
            # 获取加热器对象
            heater_name = "inside_temperature"
            heater = self.printer.lookup_object('heater').heaters.get(heater_name)
            if not heater:
                try:
                    heater = self.printer.lookup_object('heater extruder')
                except:
                    raise Exception("Heater 'extruder' not found")
            
            # 设置等待超时
            start_time = self.reactor.monotonic()
            while True:
                current_time = self.reactor.monotonic()
                elapsed_time = current_time - start_time
                
                if elapsed_time > timeout:
                    raise Exception("Heater timeout: Failed to reach target temperature in %d seconds" % timeout)
                
                # 获取当前温度
                if hasattr(heater, 'get_temp'):
                    temp, target = heater.get_temp(current_time)
                else:
                    status = heater.get_status(current_time)
                    temp = status.get('temperature', 0)
                    target = status.get('target', 0)
                
                if abs(temp - target_temp) <= 2:
                    break
                
                # 等待一小段时间再检查
                self.reactor.pause(current_time + 1.0)
                
        except Exception as e:
            logging.error("Temperature wait error: %s" % str(e))
            raise
    
    cmd_M141_help = "Close blinds, fan full speed, set chamber temperature (no wait) or S0 to open blinds and cool down"
    def cmd_M141(self, gcmd):
        try:
            params = self._parse_gcode_parameters(gcmd)
            
            # 检查是否为冷却模式 (S0)
            if 'S' in params and params['S'] == '0':
                self._execute_cooling_procedure(gcmd, wait_for_temp=False)
                return
            
            # 正常加热模式
            # 获取温度参数
            temperature = float(params.get('S', 70.0))
            
            # 验证参数范围
            if temperature < 0 or temperature > 300:
                gcmd.respond_info("ERROR: Temperature out of range (0-300)")
                return
            
            # 停止现有监控
            self._stop_monitoring()
            
            # 第一步：关闭百叶窗和风扇全速
            # 1. 风扇全速
            fan_command = "SET_FAN_SPEED FAN=fan6 SPEED=1.0"
            self.gcode.run_script_from_command(fan_command)
            
            # 2. 百叶窗关闭
            blinds_command = "SET_PIN PIN=SENSOR_MOTOR VALUE=1.00"
            self.gcode.run_script_from_command(blinds_command)
            
            # 启动监控，确保百叶窗正确关闭
            self.start_open_to_close_monitoring()
            
            # 第二步：设置加热器温度（不等待）
            heater_command = "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=%.1f" % temperature
            self.gcode.run_script_from_command(heater_command)
            
            gcmd.respond_info("M141 executed: Blinds closed, Fan full speed, Temperature=%.1f°C (no wait)" % temperature)
            
        except Exception as e:
            logging.error("M141 command error: %s" % str(e))
            gcmd.respond_info("ERROR: M141 command failed - %s" % str(e))
    
    cmd_M191_help = "Close blinds, fan full speed, set chamber temperature and wait or S0 to open blinds and cool down (wait)"
    def cmd_M191(self, gcmd):
        try:
            params = self._parse_gcode_parameters(gcmd)
            
            # 检查是否为冷却模式 (S0)
            if 'S' in params and params['S'] == '0':
                self._execute_cooling_procedure(gcmd, wait_for_temp=True)
                return
            
            # 正常加热模式
            # 获取温度参数
            temperature = float(params.get('S', 70.0)) 
            
            # 验证参数范围
            if temperature < 0 or temperature > 300:
                gcmd.respond_info("ERROR: Temperature out of range (0-300)")
                return
            
            # 停止现有监控
            self._stop_monitoring()
            
            # 第一步：关闭百叶窗和风扇全速
            # 1. 风扇全速
            fan_command = "SET_FAN_SPEED FAN=fan6 SPEED=1.0"
            self.gcode.run_script_from_command(fan_command)
            
            # 2. 百叶窗关闭
            blinds_command = "SET_PIN PIN=SENSOR_MOTOR VALUE=1.00"
            self.gcode.run_script_from_command(blinds_command)
            
            # 启动监控，确保百叶窗正确关闭
            self.start_open_to_close_monitoring()
            
            # 第二步：设置加热器温度并等待
            heater_command = "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=%.1f" % temperature
            self.gcode.run_script_from_command(heater_command)
            
            # 等待加热完成
            gcmd.respond_info("M191: Waiting for temperature to reach %.1f°C..." % temperature)
            self._wait_for_temperature(temperature)
            gcmd.respond_info("M191: Temperature reached %.1f°C" % temperature)
            
            gcmd.respond_info("M191 executed: Blinds closed, Fan full speed, Temperature=%.1f°C (waited)" % temperature)
            
        except Exception as e:
            logging.error("M191 command error: %s" % str(e))
            gcmd.respond_info("ERROR: M191 command failed - %s" % str(e))
    
    def _execute_cooling_procedure(self, gcmd, wait_for_temp=False):
        try:
            # 停止现有监控
            self._stop_monitoring()
            
            # 第一步：关闭风扇
            fan_command = "SET_FAN_SPEED FAN=fan6 SPEED=0.0"
            self.gcode.run_script_from_command(fan_command)
            
            # 第二步：打开百叶窗
            blinds_command = "SET_PIN PIN=SENSOR_MOTOR VALUE=0.00"
            self.gcode.run_script_from_command(blinds_command)
            
            # 启动监控，确保百叶窗正确打开
            self.start_close_to_open_monitoring()
            
            # 第三步：设置加热器温度为0
            heater_command = "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=0.0"
            self.gcode.run_script_from_command(heater_command)
            
            if wait_for_temp:
                # 等待温度降到安全温度
                gcmd.respond_info("M191 S0: Waiting for temperature to cool down to 30°C...")
                self._wait_for_temperature(30.0, timeout=600)
                gcmd.respond_info("M191 S0: Temperature cooled down to 30°C")
                gcmd.respond_info("M191 S0 executed: Blinds opened, Fan off, Temperature cooled down")
            else:
                gcmd.respond_info("M141 S0 executed: Blinds opened, Fan off, Temperature set to 0°C (no wait)")
                
        except Exception as e:
            logging.error("Cooling procedure error: %s" % str(e))
            gcmd.respond_info("ERROR: Cooling procedure failed - %s" % str(e))
    
    cmd_START_BLINDS_MONITOR_help = "Start blinds state change monitoring"
    def cmd_START_BLINDS_MONITOR(self, gcmd):
        # 获取参数，确定监控方向
        direction = gcmd.get("DIRECTION", "OPEN2CLOSE")
        
        if direction.upper() in ["OPEN2CLOSE", "OPEN_TO_CLOSE"]:
            self.start_open_to_close_monitoring()
            if self.recovery_mode_active:
                gcmd.respond_info("Recovery mode started: Waiting for CLOSED state, timeout: %.1fs" % self.recovery_timeout)
            else:
                gcmd.respond_info("Started monitoring: OPEN -> CLOSED, timeout: %.1fs" % self.state_change_timeout)
        elif direction.upper() in ["CLOSE2OPEN", "CLOSE_TO_OPEN"]:
            self.start_close_to_open_monitoring()
            if self.recovery_mode_active:
                gcmd.respond_info("Recovery mode started: Waiting for CLOSED state, timeout: %.1fs" % self.recovery_timeout)
            else:
                gcmd.respond_info("Started monitoring: CLOSED -> OPEN, timeout: %.1fs" % self.state_change_timeout)
        else:
            gcmd.respond_info("ERROR: Invalid DIRECTION. Use OPEN2CLOSE or CLOSE2OPEN")
    
    cmd_STOP_BLINDS_MONITOR_help = "Stop blinds state change monitoring"
    def cmd_STOP_BLINDS_MONITOR(self, gcmd):
        self._stop_monitoring()
        gcmd.respond_info("Blinds monitoring stopped")
    
    def get_status(self, eventtime):
        sensor1_state, sensor2_state = self._get_sensor_states()
        current_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        status = {
            'monitoring_active': self.monitoring_active,
            'recovery_mode_active': self.recovery_mode_active,
            'current_state': current_state,
            'sensor1_state': sensor1_state,
            'sensor2_state': sensor2_state
        }
        
        if self.monitoring_active or self.recovery_mode_active:
            status.update({
                'initial_state': self.initial_state,
                'expected_final_state': self.expected_final_state,
                'time_remaining': max(0, self.expected_state_change_time - eventtime) if self.expected_state_change_time else 0
            })
        
        return status

def load_config(config):
    return BlindsHallMonitor(config)

def load_config_prefix(config):
    return BlindsHallMonitor(config)