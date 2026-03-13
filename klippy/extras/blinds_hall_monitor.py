import logging
import traceback

class BlindsHallMonitor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        
        # 获取霍尔传感器对象名称
        self.sensor1_name = "filament_switch_sensor blinds_hall_sensor1"
        self.sensor2_name = "filament_switch_sensor blinds_hall_sensor2"
        self.door_sensor_name = "filament_switch_sensor door_hall_sensor"
        
        # 配置参数
        self.state_change_timeout = config.getfloat('state_change_timeout', 60.0, above=0.0)
        self.recovery_timeout = config.getfloat('recovery_timeout', 60.0, above=0.0)
        self.check_interval = config.getfloat('check_interval', 0.5, above=0.0)
        self.cooling_fan_delay = config.getfloat('cooling_fan_delay', 60.0, above=0.0)
        self.temperature_tolerance = config.getfloat('temperature_tolerance', 15.0, above=0.0)
        self.door_check_interval = config.getfloat('door_check_interval', 5.0, above=0.0)
        
        # 状态管理
        self.monitoring_active = False
        self.recovery_mode_active = False
        self.expected_state_change_time = None
        self.expected_final_state = None
        self.initial_state = None
        self.monitoring_timer = None
        
        # 门检测相关
        self.door_monitoring_active = False
        self.last_door_check_time = None
        self.door_alert_sent = False
        
        # 加热相关 - 只要温度>0就为True
        self.heating_active = False
        self.current_target_temp = 0.0  # 当前目标温度
        
        # 当前状态缓存
        self.last_sensor1_state = None
        self.last_sensor2_state = None
        self.last_door_sensor_state = None
        
        # 冷却风扇定时器
        self.cooling_fan_timer = None
        self.cooling_fan_start_time = None
        
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
            'STOP_DOOR_MONITOR', 
            self.cmd_STOP_DOOR_MONITOR,
            desc=self.cmd_STOP_DOOR_MONITOR_help
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
            door_sensor = self.printer.lookup_object(self.door_sensor_name)
            logging.info("BlindsHallMonitor: Sensors initialized successfully")
        except Exception as e:
            logging.error("BlindsHallMonitor: Failed to find sensors: %s" % str(e))
    
    def _get_sensor_states(self):
        try:
            sensor1 = self.printer.lookup_object(self.sensor1_name)
            sensor2 = self.printer.lookup_object(self.sensor2_name)
            door_sensor = self.printer.lookup_object(self.door_sensor_name)
            
            eventtime = self.reactor.monotonic()
            status1 = sensor1.get_status(eventtime)
            status2 = sensor2.get_status(eventtime)
            status_door = door_sensor.get_status(eventtime)
            
            return (
                bool(status1.get('filament_detected', False)),
                bool(status2.get('filament_detected', False)),
                bool(status_door.get('filament_detected', False))
            )
        except Exception as e:
            logging.error("BlindsHallMonitor: Failed to get sensor states: %s" % str(e))
            return None, None, None
    
    def _get_blinds_state(self, sensor1_state, sensor2_state):
        if sensor1_state is None or sensor2_state is None:
            return "UNKNOWN"
        
        if sensor1_state and not sensor2_state:
            return "OPEN"
        elif not sensor1_state and sensor2_state:
            return "CLOSED"
        elif sensor1_state and sensor2_state:
            return "BOTH_HIGH"
        else:
            return "BOTH_LOW"
    
    def _check_door_sensor_during_heating(self):
        """
        加热过程中的门传感器检查 - 每5秒检查一次
        """
        if not self.door_monitoring_active or not self.heating_active:
            return
        
        current_time = self.reactor.monotonic()
        
        # 检查是否需要执行门检测
        if (self.last_door_check_time is not None and 
            current_time - self.last_door_check_time < self.door_check_interval):
            return
        
        # 更新上次检测时间
        self.last_door_check_time = current_time
        
        # 获取门传感器状态
        sensor1_state, sensor2_state, door_state = self._get_sensor_states()
        
        if door_state is None:
            logging.error("BlindsHallMonitor: Failed to get door sensor state during heating")
            return
        
        # 记录门状态变化
        if self.last_door_sensor_state != door_state:
            logging.info(f"BlindsHallMonitor: Door state changed from {self.last_door_sensor_state} to {door_state}")
        
        self.last_door_sensor_state = door_state
        
        if not door_state:  # 低电平 - 门打开
            if not self.door_alert_sent:
                warning_msg = f"Door opened during heating! Sensor is LOW - Sending alert"
                logging.warning("BlindsHallMonitor: " + warning_msg)
                self.gcode.respond_info("WARNING: " + warning_msg)
                
                try:
                    self.gcode.run_script_from_command("PRINTED_DOOR_DETECTION")
                    logging.info("BlindsHallMonitor: Sent PRINTED_DOOR_DETECTION macro (door opened during heating)")
                    self.door_alert_sent = True
                except Exception as e:
                    logging.error(f"BlindsHallMonitor: Failed to send PRINTED_DOOR_DETECTION macro: {str(e)}")
            else:
                logging.debug("BlindsHallMonitor: Door still open during heating (alert already sent)")
        else:  # 高电平 - 门关闭
            if self.door_alert_sent:
                info_msg = "Door closed during heating - Resetting alert flag"
                logging.info("BlindsHallMonitor: " + info_msg)
                self.gcode.respond_info("INFO: " + info_msg)
                self.door_alert_sent = False
            else:
                logging.debug("BlindsHallMonitor: Door remains closed during heating")
    
    def _start_door_monitoring(self):
        """启动门检测（立即执行第一次检查）"""
        self.door_monitoring_active = True
        self.heating_active = True
        self.last_door_check_time = self.reactor.monotonic()
        self.door_alert_sent = False
        
        # 立即执行第一次检查
        self._check_door_sensor_during_heating()
        
        logging.info("BlindsHallMonitor: Door monitoring started (check interval: %.1fs)" % self.door_check_interval)
        # 确保监控循环在运行
        self._start_monitoring_loop()
    
    def _stop_door_monitoring(self):
        """停止门检测"""
        if self.door_monitoring_active or self.heating_active:
            logging.info("BlindsHallMonitor: Door monitoring stopped - Call stack:\n" + 
                        ''.join(traceback.format_stack()))
        self.door_monitoring_active = False
        self.heating_active = False
        self.last_door_check_time = None
        self.door_alert_sent = False
    
    def _check_heater_status(self):
        """检查加热器状态，如果目标温度为0则停止加热标志"""
        try:
            heater_name = "chamber"
            full_heater_name = 'heater_generic ' + heater_name
            heater = self.printer.lookup_object(full_heater_name)
            
            eventtime = self.reactor.monotonic()
            status = heater.get_status(eventtime)
            target = status.get('target', 0)

            logging.info(f"BlindsHallMonitor: Heater check - target={target}, heating_active={self.heating_active}")
            
            # 如果目标温度为0，且加热标志为True，则停止加热标志
            if target <= 0 and self.heating_active:
                logging.info("BlindsHallMonitor: Heater target is 0, stopping heating flag")
                self.heating_active = False
                
            # 更新当前目标温度
            self.current_target_temp = target
            
        except Exception as e:
            logging.error(f"BlindsHallMonitor: Failed to check heater status: {str(e)}")
    
    def _start_recovery_mode(self, initial_state):
        self.gcode.respond_info("BlindsHallMonitor: Starting recovery mode for initial state: %s" % initial_state)
        
        self.gcode.run_script_from_command("SET_PIN PIN=SENSOR_MOTOR VALUE=0.00")
        self.gcode.respond_info("BlindsHallMonitor: Turned off sensor motor")
        
        self.recovery_mode_active = True
        self.expected_final_state = "CLOSED"
        self.expected_state_change_time = self.reactor.monotonic() + self.recovery_timeout
        self.initial_state = initial_state
        
        sensor1_state, sensor2_state, door_state = self._get_sensor_states()
        self.last_sensor1_state = sensor1_state
        self.last_sensor2_state = sensor2_state
        self.last_door_sensor_state = door_state
        
        logging.info("BlindsHallMonitor: Recovery mode started, waiting %.1f seconds for CLOSED state" % 
                    self.recovery_timeout)
        
        self._start_monitoring_loop()
    
    def _check_recovery_state(self):
        if not self.recovery_mode_active:
            return
        
        current_time = self.reactor.monotonic()
        sensor1_state, sensor2_state, door_state = self._get_sensor_states()
        
        if sensor1_state is None or sensor2_state is None:
            return
        
        current_blinds_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        state_changed = (sensor1_state != self.last_sensor1_state or 
                        sensor2_state != self.last_sensor2_state)
        
        if state_changed:
            logging.info("BlindsHallMonitor: State changed to %s during recovery" % current_blinds_state)
            
            self.last_sensor1_state = sensor1_state
            self.last_sensor2_state = sensor2_state
            self.last_door_sensor_state = door_state
            
            if current_blinds_state == self.expected_final_state:
                logging.info("BlindsHallMonitor: Recovery successful, reached CLOSED state")
                self._stop_monitoring()
                return
        
        if current_time > self.expected_state_change_time:
            error_msg = "Louver closing failure. Please inspect the drive unit."
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
            if initial_state in ["BOTH_HIGH", "BOTH_LOW", "UNKNOWN"]:
                error_msg = "Louver state exception. Please inspect the drive unit."
                logging.error(error_msg)
                self.gcode.respond_info("ERROR: " + error_msg)
                self.printer.invoke_shutdown(error_msg)
                return False
            else:
                return True
        return True
    
    def _check_state_change(self):
        if not self.monitoring_active:
            return
        
        if self.recovery_mode_active:
            self._check_recovery_state()
            return
        
        current_time = self.reactor.monotonic()
        sensor1_state, sensor2_state, door_state = self._get_sensor_states()
        
        if sensor1_state is None or sensor2_state is None:
            return
        
        current_blinds_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        state_changed = (sensor1_state != self.last_sensor1_state or 
                        sensor2_state != self.last_sensor2_state)
        
        if state_changed:
            logging.info("BlindsHallMonitor: State changed to %s" % current_blinds_state)
            
            self.last_sensor1_state = sensor1_state
            self.last_sensor2_state = sensor2_state
            self.last_door_sensor_state = door_state
            
            if (self.expected_final_state is not None and 
                current_blinds_state == self.expected_final_state):
                logging.info("BlindsHallMonitor: Expected state reached successfully")
                # 只停止百叶窗监控，不停止门检测
                self.monitoring_active = False
                self.recovery_mode_active = False
                self.expected_state_change_time = None
                self.expected_final_state = None
                self.initial_state = None
                return
        
        if (self.expected_state_change_time is not None and 
            current_time > self.expected_state_change_time):
            if self.expected_final_state == "CLOSED":
                error_msg = "Louver closing failure. Please inspect the drive unit."
            elif self.expected_final_state == "OPEN":
                error_msg = "Louver opening failure. Please inspect the drive unit."
            else:
                error_msg = "Blinds state change timeout."
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
        sensor1_state, sensor2_state, door_state = self._get_sensor_states()
        current_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        if not self._validate_initial_state(current_state):
            return
        
        if not self._validate_state_change(current_state, target_state):
            return
        
        if current_state == target_state:
            logging.info(f"BlindsHallMonitor: Already shows {target_state}, but will execute action to verify")
            
            self.initial_state = current_state
            self.expected_final_state = target_state
            self.expected_state_change_time = self.reactor.monotonic() + self.state_change_timeout
            self.last_sensor1_state = sensor1_state
            self.last_sensor2_state = sensor2_state
            self.last_door_sensor_state = door_state
            self.monitoring_active = True
            self.recovery_mode_active = False
            
            self._start_monitoring_loop()
            return
        
        self.initial_state = current_state
        self.expected_final_state = target_state
        self.expected_state_change_time = self.reactor.monotonic() + self.state_change_timeout
        self.last_sensor1_state = sensor1_state
        self.last_sensor2_state = sensor2_state
        self.last_door_sensor_state = door_state
        self.monitoring_active = True
        self.recovery_mode_active = False
        
        logging.info("BlindsHallMonitor: Monitoring state change from %s to %s, timeout: %.1fs" % 
                    (current_state, target_state, self.state_change_timeout))
        
        self._start_monitoring_loop()
    
    def _stop_monitoring(self):
        """完全停止所有监控（包括门检测）- 用于紧急情况"""
        logging.info("BlindsHallMonitor: _stop_monitoring called - Call stack:\n" + 
                    ''.join(traceback.format_stack()))
        self.monitoring_active = False
        self.recovery_mode_active = False
        self.expected_state_change_time = None
        self.expected_final_state = None
        self.initial_state = None
        self._stop_monitoring_loop()
        self._stop_door_monitoring()
    
    def _monitoring_loop(self, eventtime):
        # 定期检查加热器状态
        self._check_heater_status()
        
        logging.debug(f"BlindsHallMonitor: Loop - monitoring_active={self.monitoring_active}, "
                     f"recovery_mode_active={self.recovery_mode_active}, "
                     f"heating_active={self.heating_active}, "
                     f"door_monitoring_active={self.door_monitoring_active}, "
                     f"target_temp={self.current_target_temp}")
        
        if self.monitoring_active or self.recovery_mode_active or self.heating_active:
            # 检查百叶窗状态变化（如果激活）
            if self.monitoring_active or self.recovery_mode_active:
                self._check_state_change()
            
            # 如果门检测激活且正在加热，检查门状态
            if self.door_monitoring_active and self.heating_active:
                self._check_door_sensor_during_heating()
            
            return eventtime + self.check_interval
        else:
            if self.monitoring_timer is not None:
                logging.info("BlindsHallMonitor: No active monitoring, stopping loop")
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
    
    def _start_cooling_fan_delay(self, gcmd):
        """启动风扇延迟关闭定时器"""
        if self.cooling_fan_timer is not None:
            self.reactor.unregister_timer(self.cooling_fan_timer)
            self.cooling_fan_timer = None
        
        fan_command = "SET_FAN_SPEED FAN=fan6 SPEED=1.0"
        self.gcode.run_script_from_command(fan_command)
        
        gcmd.respond_info(f"M141/M191 S0: Fan will run for {self.cooling_fan_delay} seconds before stopping")
        
        self.cooling_fan_start_time = self.reactor.monotonic()
        
        def cooling_fan_timer_callback(eventtime):
            current_time = self.reactor.monotonic()
            elapsed_time = current_time - self.cooling_fan_start_time
            
            if elapsed_time >= self.cooling_fan_delay:
                try:
                    fan_stop_command = "SET_FAN_SPEED FAN=fan6 SPEED=0"
                    self.gcode.run_script_from_command(fan_stop_command)
                    gcmd.respond_info(f"M141/M191 S0: Fan stopped after {self.cooling_fan_delay} seconds")
                    logging.info(f"Cooling fan stopped after {self.cooling_fan_delay} seconds delay")
                except Exception as e:
                    logging.error(f"Error turning off cooling fan: {str(e)}")
                
                self.cooling_fan_timer = None
                self.cooling_fan_start_time = None
                return self.reactor.NEVER
            
            remaining_time = max(0, self.cooling_fan_delay - elapsed_time)
            if remaining_time <= 10 and int(remaining_time) % 5 == 0:
                gcmd.respond_info(f"M141/M191 S0: Fan stopping in {int(remaining_time)} seconds")
            
            return eventtime + 1.0
        
        self.cooling_fan_timer = self.reactor.register_timer(
            cooling_fan_timer_callback, self.reactor.NOW)
        
        logging.info(f"Cooling fan timer started, will run for {self.cooling_fan_delay} seconds")
    
    def _stop_cooling_fan_delay(self):
        """停止风扇延迟关闭定时器"""
        if self.cooling_fan_timer is not None:
            self.reactor.unregister_timer(self.cooling_fan_timer)
            self.cooling_fan_timer = None
            self.cooling_fan_start_time = None
            logging.info("Cooling fan timer stopped")
    
    cmd_M141_help = "Close blinds, fan full speed, set chamber temperature (no wait) or S0 to open blinds and cool down"
    def cmd_M141(self, gcmd):
        try:
            params = self._parse_gcode_parameters(gcmd)
            
            if 'S' in params and params['S'] == '0':
                self._execute_cooling_procedure(gcmd, wait_for_temp=False)
                return
            
            temperature = float(params.get('S', 70.0))
            
            if temperature < 0 or temperature > 300:
                gcmd.respond_info("ERROR: Temperature out of range (0-300)")
                return
            
            # 停止现有监控
            self._stop_monitoring()
            self._stop_cooling_fan_delay()
            
            # 关闭百叶窗和风扇
            fan_command = "SET_FAN_SPEED FAN=fan6 SPEED=1.0"
            self.gcode.run_script_from_command(fan_command)

            # 设置排风风扇速度为0（关闭）
            self.gcode.run_script_from_command("EXHAUST_FAN S=0")
            
            blinds_command = "SET_PIN PIN=SENSOR_MOTOR VALUE=1.00"
            self.gcode.run_script_from_command(blinds_command)
            
            # 启动百叶窗监控
            self.start_open_to_close_monitoring()
            
            # 设置加热器温度
            heater_command = "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=%.1f" % temperature
            self.gcode.run_script_from_command(heater_command)
            
            # 只要温度>0，就启动门检测
            if temperature > 0:
                self._start_door_monitoring()
                gcmd.respond_info("M141 executed: Blinds closed, Fan full speed, Temperature=%.1f°C (no wait) - Door monitoring active while temperature > 0 (every %.1fs)" % 
                                (temperature, self.door_check_interval))
            else:
                gcmd.respond_info("M141 executed: Blinds closed, Fan full speed, Temperature=%.1f°C (no wait)" % temperature)
            
        except Exception as e:
            logging.error("M141 command error: %s" % str(e))
            gcmd.respond_info("ERROR: M141 command failed - %s" % str(e))
    
    cmd_M191_help = "Close blinds, fan full speed, set chamber temperature and wait or S0 to open blinds and cool down (wait)"
    def cmd_M191(self, gcmd):
        try:
            params = self._parse_gcode_parameters(gcmd)
            
            if 'S' in params and params['S'] == '0':
                self._execute_cooling_procedure(gcmd, wait_for_temp=True)
                return
            
            temperature = float(params.get('S', 70.0)) 
            
            if temperature < 0 or temperature > 300:
                gcmd.respond_info("ERROR: Temperature out of range (0-300)")
                return
            
            # 停止现有监控
            self._stop_monitoring()
            self._stop_cooling_fan_delay()
            
            # 关闭百叶窗和风扇
            fan_command = "SET_FAN_SPEED FAN=fan6 SPEED=1.0"
            self.gcode.run_script_from_command(fan_command)

            # 关闭排风风扇  
            self.gcode.run_script_from_command("EXHAUST_FAN S=0")
            
            blinds_command = "SET_PIN PIN=SENSOR_MOTOR VALUE=1.00"
            self.gcode.run_script_from_command(blinds_command)
            
            # 启动百叶窗监控
            self.start_open_to_close_monitoring()
            
            # 设置加热器温度
            heater_command = "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=%.1f" % temperature
            self.gcode.run_script_from_command(heater_command)
            
            # 只要温度>0，就启动门检测
            if temperature > 0:
                self._start_door_monitoring()
                gcmd.respond_info("M191: Waiting for temperature to reach %.1f°C (tolerance: ±%.1f°C) - Door monitoring active while temperature > 0 (every %.1fs)..." % 
                                (temperature, self.temperature_tolerance, self.door_check_interval))
                
                # 等待加热完成
                self._wait_for_temperature(temperature)
                
                gcmd.respond_info("M191: Temperature reached %.1f°C" % temperature)
                gcmd.respond_info("M191 executed: Blinds closed, Fan full speed, Temperature=%.1f°C (waited)" % temperature)
            else:
                gcmd.respond_info("M191 executed: Blinds closed, Fan full speed, Temperature=%.1f°C (waited)" % temperature)
            
        except Exception as e:
            logging.error("M191 command error: %s" % str(e))
            gcmd.respond_info("ERROR: M191 command failed - %s" % str(e))
            self._stop_door_monitoring()
    
    def _wait_for_temperature(self, target_temp, timeout=300):
        try:
            heater_name = "chamber"
            full_heater_name = 'heater_generic ' + heater_name

            try:
                heater = self.printer.lookup_object(full_heater_name)
            except:
                raise Exception("Heater 'chamber' not found")
            
            start_time = self.reactor.monotonic()
            while True:
                current_time = self.reactor.monotonic()
                elapsed_time = current_time - start_time
                
                if elapsed_time > timeout:
                    raise Exception("Heater timeout: Failed to reach target temperature in %d seconds" % timeout)
                
                status = heater.get_status(current_time)
                temp = status.get('temperature', 0)
                target = status.get('target', 0)
                
                if abs(temp - target_temp) <= self.temperature_tolerance:
                    break
                
                self.reactor.pause(current_time + 1.0)
                
        except Exception as e:
            logging.error("Temperature wait error: %s" % str(e))
            raise
        finally:
            # 温度达到后，不自动停止门检测，让 _check_heater_status 根据目标温度决定
            pass
    
    def _execute_cooling_procedure(self, gcmd, wait_for_temp=False):
        try:
            # 冷却模式 - 不检查门传感器
            self._stop_monitoring()
            
            fan_command = "SET_FAN_SPEED FAN=fan6 SPEED=1.0"
            self.gcode.run_script_from_command(fan_command)
            
            # 第一步：设置排风风扇速度为0（关闭）
            self.gcode.run_script_from_command("EXHAUST_FAN S=0")
            
            blinds_command = "SET_PIN PIN=SENSOR_MOTOR VALUE=0.00"
            self.gcode.run_script_from_command(blinds_command)
            
            self.start_close_to_open_monitoring()
            
            heater_command = "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=0.0"
            self.gcode.run_script_from_command(heater_command)
            
            if wait_for_temp:
                gcmd.respond_info("M191 S0: Waiting for temperature to cool down to 30°C (tolerance: ±%.1f°C)..." % 
                                self.temperature_tolerance)
                self._wait_for_temperature(30.0, timeout=600)
                gcmd.respond_info("M191 S0: Temperature cooled down to 30°C")
                
                self._start_cooling_fan_delay(gcmd)
                
                gcmd.respond_info("M191 S0 executed: Blinds opened, Fan running for cooling")
            else:
                self._start_cooling_fan_delay(gcmd)
                gcmd.respond_info("M141 S0 executed: Blinds opened, Fan running for cooling")
                
        except Exception as e:
            logging.error("Cooling procedure error: %s" % str(e))
            gcmd.respond_info("ERROR: Cooling procedure failed - %s" % str(e))
    
    cmd_START_BLINDS_MONITOR_help = "Start blinds state change monitoring"
    def cmd_START_BLINDS_MONITOR(self, gcmd):
        direction = gcmd.get("DIRECTION", "OPEN2CLOSE")
        
        if direction.upper() in ["OPEN2CLOSE", "OPEN_TO_CLOSE"]:
            self.start_open_to_close_monitoring()
            if self.recovery_mode_active:
                gcmd.respond_info("Recovery mode started: Waiting for CLOSED state, timeout: %.1fs" % self.recovery_timeout)
            elif self.monitoring_active:
                gcmd.respond_info("Started monitoring: OPEN -> CLOSED, timeout: %.1fs" % self.state_change_timeout)
            else:
                gcmd.respond_info("Already in CLOSED state, no monitoring needed")
        elif direction.upper() in ["CLOSE2OPEN", "CLOSE_TO_OPEN"]:
            self.start_close_to_open_monitoring()
            if self.recovery_mode_active:
                gcmd.respond_info("Recovery mode started: Waiting for CLOSED state, timeout: %.1fs" % self.recovery_timeout)
            elif self.monitoring_active:
                gcmd.respond_info("Started monitoring: CLOSED -> OPEN, timeout: %.1fs" % self.state_change_timeout)
            else:
                gcmd.respond_info("Already in OPEN state, no monitoring needed")
        else:
            gcmd.respond_info("ERROR: Invalid DIRECTION. Use OPEN2CLOSE or CLOSE2OPEN")
    
    cmd_STOP_BLINDS_MONITOR_help = "Stop blinds state change monitoring"
    def cmd_STOP_BLINDS_MONITOR(self, gcmd):
        self._stop_monitoring()
        gcmd.respond_info("Blinds monitoring stopped")
    
    cmd_STOP_DOOR_MONITOR_help = "Stop door monitoring manually"
    def cmd_STOP_DOOR_MONITOR(self, gcmd):
        self._stop_door_monitoring()
        gcmd.respond_info("Door monitoring stopped manually")
    
    def get_status(self, eventtime):
        sensor1_state, sensor2_state, door_state = self._get_sensor_states()
        current_state = self._get_blinds_state(sensor1_state, sensor2_state)
        
        status = {
            'monitoring_active': self.monitoring_active,
            'recovery_mode_active': self.recovery_mode_active,
            'current_state': current_state,
            'sensor1_state': sensor1_state,
            'sensor2_state': sensor2_state,
            'door_sensor_state': door_state,
            'door_monitoring_active': self.door_monitoring_active,
            'heating_active': self.heating_active,
            'current_target_temp': self.current_target_temp,
            'door_alert_sent': self.door_alert_sent,
            'cooling_fan_delay': self.cooling_fan_delay,
            'temperature_tolerance': self.temperature_tolerance,
            'door_check_interval': self.door_check_interval
        }
        
        if self.monitoring_active or self.recovery_mode_active:
            status.update({
                'initial_state': self.initial_state,
                'expected_final_state': self.expected_final_state,
                'time_remaining': max(0, self.expected_state_change_time - eventtime) if self.expected_state_change_time else 0
            })
        
        if self.cooling_fan_timer is not None and self.cooling_fan_start_time is not None:
            elapsed_time = eventtime - self.cooling_fan_start_time
            remaining_time = max(0, self.cooling_fan_delay - elapsed_time)
            status.update({
                'cooling_fan_active': True,
                'cooling_fan_remaining': remaining_time,
                'cooling_fan_elapsed': elapsed_time
            })
        else:
            status.update({
                'cooling_fan_active': False,
                'cooling_fan_remaining': 0,
                'cooling_fan_elapsed': 0
            })
        
        return status

def load_config(config):
    return BlindsHallMonitor(config)

def load_config_prefix(config):
    return BlindsHallMonitor(config)