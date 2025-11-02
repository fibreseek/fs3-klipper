# import logging

class aniso_bed_mesh:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.broadcast = config.getboolean('broadcast', False) #广播使能标志位
        ppins = self.printer.lookup_object('pins')
        adc_pin = config.get('adc_pin', 'PA3')
        adc_ppin = ppins.lookup_pin(adc_pin)
        self.adc_pin = adc_ppin['pin']

        trigger_pin = config.get('trigger_pin', 'PB5')
        trigger_ppin = ppins.lookup_pin(trigger_pin)
        self.trigger_pin = trigger_ppin['pin']
        self.mcu = self.printer.lookup_object('mcu toolhead')
        self.oid = self.mcu.create_oid()
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self.build_config)
        self.mcu.register_response(self.handle_aniso_bed_mesh_msg, "aniso_bed_mesh", self.oid)
        self.gcode = config.get_printer().lookup_object('gcode')
        self.gcode.register_command('MY_ADC_QUERY', self.cmd_my_custom_adc)
        self.gcode.register_command('PROBE_ALIGN', self.cmd_probe_align)
        self.printer.add_object("aniso_bed_mesh", self)

    def build_config(self):
        self.mcu.add_config_cmd("config_aniso_bed_mesh oid=%d adc_pin=%s trigger_pin=%s" % (self.oid, self.adc_pin, self.trigger_pin)) #设置adc引脚
        self.mcu.add_config_cmd("setup_aniso_bed_mesh oid=%u op=%u val=%u" % (self.oid, 3, self.broadcast)) #是否开启广播
        self.update_uart_cmd = self.mcu.lookup_command("setup_aniso_bed_mesh oid=%u op=%u val=%u", cq=self.cmd_queue)

    def handle_aniso_bed_mesh_msg(self, params):
        # op = params['op']
        # val = params['val']
        pass
        
    def cmd_probe_align(self, gcmd):
        commands = [
            "G90",
            "FORCE_MOVE STEPPER=stepper_z DISTANCE=5 VELOCITY=10"
            ]
        
        # 如果没有完成XY归零，则先进行XY归零
        curtime = self.printer.get_reactor().monotonic()
        status = self.printer.lookup_object('toolhead').get_status(curtime)['homed_axes']
        if status != 'xyz' and status != 'xy':
            commands.extend([
                "G28.1 y",
                "G1 y5",
                "M400",
                "G4 P2000",
                "G28.1 x",
                "G1 x5"
            ])

        commands.extend([
            "T0",
            "M400",
            "G1 X150 Y150 F6000",
            "M400",
            "G28.1 Z",
            "M400"
        ])

        script = "\n".join(commands)
        self.gcode.run_script_from_command(script)

    def cmd_my_custom_adc(self, gcmd):
        O = gcmd.get_int('O', 0)
        V = gcmd.get_int('V', 0)
        self.update_uart_cmd.send([self.oid, O, V])

def load_config(config):
    return aniso_bed_mesh(config)