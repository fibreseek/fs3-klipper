from . import output_pin
import logging

class aniso_bed_mesh:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.pin_num = stm32_pin_to_number(config.get('pin', 'PF5'))
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

        if op == 2:
            if val == 0:
                self.gcode.respond_info("aniso bed mesh init!") 
                self.revop = op
        elif op == 3:
            self.gcode.respond_info("aniso bed mesh align finish!")
            self.revop = op
        elif op == 4:
            self.gcode.respond_info("aniso bed mesh val:%d" % val)     
        elif op == 8: 
            self.align5 = val & 0x3F
            self.align4 = (val >> 6) & 0x3F
            self.align3 = (val >> 12) & 0x3F
            self.align2 = (val >> 18) & 0x3F
            self.align1 = (val >> 24) & 0x3F

            self.refresh = 1
            self.gcode.respond_info("aniso bed mesh step align data: %d %d %d %d %d" % (self.align1, self.align2, self.align3, self.align4, self.align5))

        
    def cmd_probe_align(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        status = self.printer.lookup_object('toolhead').get_status(curtime)['homed_axes']
        self.sendop = 0
        self.revop = 0
        wait_count = 0;
        self.adc_trigger_val = gcmd.get_float('T', 0)
        if self.adc_trigger_val == 0:
            self.adc_trigger_val = self.adc_trigger

        self.gcode.respond_info("adc_pin: %d" % self.pin_num)    
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
        
        
        self.sendop = 2
        self.gcode.run_script_from_command("G1 X150 Y150 F3000")
        self.update_uart_cmd.send([self.oid, self.sendop, round(self.pin_num)])
        while wait_count < 10:
            self.gcode.run_script_from_command("G4 P300")
            wait_count = wait_count + 1
            if self.revop == 2:
                self.sendop = 3
                break;
        wait_count = 0

        if self.sendop == 3:
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