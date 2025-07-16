from . import output_pin
import logging

class aniso_bed_mesh:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.pin1_num = stm32_pin_to_number(config.get('pin1', 'PF10'))
        self.pin2_num = stm32_pin_to_number(config.get('pin2', 'PF9'))
        self.pin3_num = stm32_pin_to_number(config.get('pin3', 'PF6'))
        self.pin4_num = stm32_pin_to_number(config.get('pin4', 'PF5'))
        self.adc_trigger = config.getfloat('adc_trigger', 180)
        self.mcu = self.printer.lookup_object('mcu')
        self.oid = self.mcu.create_oid()
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self.build_config)
        self.gcode = config.get_printer().lookup_object('gcode')
        self.gcode.register_command('MY_ADC_QUERY', self.cmd_my_custom_adc)
        self.mcu.register_response(self.handle_aniso_bed_mesh_msg,"aniso_bed_mesh", self.oid)
        self.gcode.register_command('PROBE_ALIGN', self.cmd_probe_align)
        self.sendop = 0
        self.revop = 0
        self.step0 = 0
        self.step1 = 0
        self.printer.add_object("aniso_bed_mesh ", self)
        self.zmicrosteps = config.getsection('stepper_z').getfloat('microsteps',8)

    # val 0 success
    # val 1 error
    def handle_aniso_bed_mesh_msg(self, params):
        op = params['op']
        val = params['val']

        if op == 1:
            if val == 0:
                self.gcode.respond_info("aniso bed mesh init success!")
                self.revop = op
            else:
                self.gcode.respond_info("aniso bed mesh init failed!")    
        elif op == 2:
            if val == 0:
                self.gcode.respond_info("aniso bed mesh start vibrate!") 
                self.revop = op
            else:
                self.gcode.respond_info("aniso bed mesh start vibrate failed!")     
        elif op == 3:
            self.gcode.respond_info("aniso bed mesh align finish!")
            self.revop = op
        elif op == 4:
            self.gcode.respond_info("aniso bed mesh val:%d" % val)    
        elif op == 5:
            self.gcode.respond_info("aniso bed mesh record z zero step success! step:%d" % val)
            self.step0 = val
            self.revop = 5
        elif op == 6:
            self.gcode.respond_info("aniso bed mesh re record z zero step success! step:%d" % val)
            self.step1 = val
            #self.gcode.run_script_from_command("G1 Z8")
            self.revop = 6  
        elif op == 8:  
            self.align3 = (val >> 20) & 0x3FF
            self.align2 = (val >> 10) & 0x3FF
            self.align1 = val & 0x3FF
            self.refresh = 1
            self.gcode.respond_info("aniso bed mesh step align data: %d %d %d" % (self.align1, self.align2, self.align3))

        
    def cmd_probe_align(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        status = self.printer.lookup_object('toolhead').get_status(curtime)['homed_axes']
        self.sendop = 0
        self.revop = 0
        wait_count = 0;
        self.adc_trigger_val = gcmd.get_float('T', 0)
        if self.adc_trigger_val == 0:
            self.adc_trigger_val = self.adc_trigger

        self.gcode.respond_info("adc_trigger_val: %f" % self.adc_trigger_val)    
        self.sendop = 1
        self.gcode.run_script_from_command("DISABLE_BED_MESH")
        self.gcode.run_script_from_command("G90")
        self.gcode.run_script_from_command("FORCE_MOVE STEPPER=stepper_z DISTANCE=5 VELOCITY=10")
        if status != 'xyz' and status != 'xy':
            self.gcode.run_script_from_command("G28.1 y")
            self.gcode.run_script_from_command("G1 y5")
            self.gcode.run_script_from_command("M400")
            self.gcode.run_script_from_command("G4 P2000")
            self.gcode.run_script_from_command("G28.1 x")
            self.gcode.run_script_from_command("G1 x5")
            
        self.gcode.run_script_from_command("T0")
        self.gcode.run_script_from_command("M400")
        
        pin_num = (self.pin4_num << 24) | (self.pin3_num << 16) | (self.pin2_num << 8) | self.pin1_num
        self.update_uart_cmd.send([self.oid, self.sendop, pin_num])
        #self.gcode.respond_info("aniso bed mesh init 0x%x",(self.pin4_num << 24) | (self.pin3_num << 16) | (self.pin2_num << 8) | self.pin1_num]) 
        self.gcode.respond_info("aniso bed mesh val:%d" % pin_num)  
        while wait_count < 10:
            self.gcode.run_script_from_command("G4 P300")
            wait_count = wait_count + 1
            if self.revop == 1:
                self.sendop = 2
                break;
        wait_count = 0
        
        if self.sendop == 2:
            self.update_uart_cmd.send([self.oid, self.sendop, round(self.adc_trigger_val)])
            while wait_count < 10:
                self.gcode.run_script_from_command("G4 P300")
                wait_count = wait_count + 1
                if self.revop == 2:
                    self.sendop = 3
                    break;
            wait_count = 0

            if self.sendop == 3:
                self.gcode.run_script_from_command("G1 X65 Y50 F3000")
                self.gcode.run_script_from_command("RESTORE_BED_MESH")
                for i in range(10):
                    self.gcode.run_script_from_command("FORCE_MOVE STEPPER=stepper_z DISTANCE=1 VELOCITY=10")
                    self.gcode.run_script_from_command("FORCE_MOVE STEPPER=stepper_z DISTANCE=-1 VELOCITY=10")
                self.gcode.run_script_from_command("M400")
                self.update_uart_cmd.send([self.oid, 3, round(self.zmicrosteps)])
                self.gcode.respond_info("aniso bed mesh zmicrosteps val:%d" % self.zmicrosteps)  
                while wait_count < 10:
                    self.gcode.run_script_from_command("G4 P300")
                    wait_count = wait_count + 1
                    if self.revop == 3:
                        self.sendop = 4
                        break;
                wait_count = 0
                if self.sendop == 4:    
                    self.gcode.run_script_from_command("G28.1 Z")
                    self.gcode.run_script_from_command("M400")
                    self.update_uart_cmd.send([self.oid, 5, 0])
                    while wait_count < 10:
                        self.gcode.run_script_from_command("G4 P300")
                        wait_count = wait_count + 1
                        if self.revop == 5:
                            self.sendop = 5
                            break;
                    wait_count = 0
                    #self.gcode.run_script_from_command("G1 Z0")
                    if self.sendop == 5:    
                        self.gcode.run_script_from_command("G1 Z5 F600")
                        self.gcode.run_script_from_command("G28.1 Z")
                        self.gcode.run_script_from_command("M400")
                        self.gcode.run_script_from_command("G4 P2000")
                        self.gcode.run_script_from_command("G1 Z0")
                        self.update_uart_cmd.send([self.oid, 6, 0])
                        while wait_count < 10:
                            self.gcode.run_script_from_command("G4 P300")
                            wait_count = wait_count + 1
                            if self.revop == 6:
                                break;
                        wait_count = 0
                        if abs(self.step0 - self.step1) > 400:
                            self.gcode.respond_info("aniso bed mesh step align failed, step0:%d step1:%d , waring!!!" % (self.step0, self.step1))
                            #self.gcode.run_script_from_command("PROBE_ALIGN T=%f" % self.adc_trigger_val) 


    #op 0 none
    #op 1 init          val=(b3 << 24) | (b2 << 16) | (b1 << 8) | b0
    #op 2 start vibrate
    #op 3 stop vibrate
    #op 4 query adc     val:[0-3]

    def build_config(self):
        self.mcu.add_config_cmd("aniso_bed_mesh_cmd oid=%u op=%u val=%u" % (self.oid, 0, 0))
        self.update_uart_cmd = self.mcu.lookup_command("aniso_bed_mesh_cmd oid=%u op=%u val=%u", cq=self.cmd_queue)

    def cmd_my_custom_adc(self, gcmd):
        # Set fan speed
        O = gcmd.get_int('O', 0)
        V = gcmd.get_int('V', 0)
        self.update_uart_cmd.send([self.oid, O, V])
        

def stm32_pin_to_number(pin_name: str) -> int:
    if len(pin_name) < 3 or not pin_name[0] == 'P':
        raise ValueError(f"无效的引脚名: {pin_name}")

    port_letter = pin_name[1].upper()
    if not ('A' <= port_letter <= 'I'):
        raise ValueError(f"不支持的端口: {port_letter}")

    try:
        pin_number = int(pin_name[2:])
    except ValueError:
        raise ValueError(f"无效的引脚号: {pin_name[2:]}")

    if not (0 <= pin_number <= 15):
        raise ValueError("引脚号应为 0~15")

    port_index = ord(port_letter) - ord('A')  # A=0, B=1, ...
    return port_index * 16 + pin_number

def load_config(config):
    return aniso_bed_mesh(config)