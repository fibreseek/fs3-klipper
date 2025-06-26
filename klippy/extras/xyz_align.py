import pins, logging
import configparser
from . import output_pin
import time

class xyz_align:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.session_x = config.getsection('stepper_x')
        self.session_z = config.getsection('stepper_z')
        #self.mcu = self.printer.lookup_object('mcu')
        self.gcode = self.printer.lookup_object('gcode')
        self.t0_offset_x = config.getfloat('t0_offset_x')
        self.t0_offset_y = config.getfloat('t0_offset_y')
        # self.t0_offset_z = config.getfloat('t0_offset_z') 
        # self.t0_offset_z_press = config.getfloat('t0_offset_z_press')
        self.t1_offset_x = config.getfloat('t1_offset_x')
        self.t1_offset_y = config.getfloat('t1_offset_y')
        # self.t1_offset_z = config.getfloat('t1_offset_z')
        # self.t1_offset_z_press = config.getfloat('t1_offset_z_press')
        self.move_offset = config.getfloat('move_offset')
        self.speed = config.getfloat('speed',3000)
        self.offset_z = 10
        self.z_value = 1
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(config.get('pin'))
        self.mcu = pin_params['chip']
        self.oid = self.mcu.create_oid()
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self.build_config)

        self.printer.add_object('xyz_align', self)
        self.gcode.register_command('XYZ_ALIGN', self.cmd_XYZ_ALIGN, desc=self.cmd_XYZ_ALIGN_help)
        self.gcode.register_command('XYZ_ALGIN_TEST', self.cmd_XYZ_ALIGN_TEST, desc=self.cmd_XYZ_ALIGN_help)
        self.mcu.register_response(self.handle_align_end,"xyz_align_end", self.oid)
        
   
    def build_config(self):
        self.mcu.add_config_cmd("align_start oid=%d val=%d" % (self.oid, 0))
        self.align_start_cmd = self.mcu.lookup_command("align_start oid=%c val=%c", cq=self.cmd_queue)

    def handle_align_end(self, params):
        # Expand the message ack_count from 8-bit
        res = params['res']
        if(res == 1):
            z0 = params['z0']
            z1 = params['z1']
            x0 = params['x0']
            y0 = params['y0']
            x1 = params['x1']
            y1 = params['y1']
            x_rotation_distance = self.session_x.getfloat('rotation_distance')
            x_microsteps =self.session_x.getfloat('microsteps')
            z_rotation_distance = self.session_z.getfloat('rotation_distance')
            z_microsteps = self.session_z.getfloat('microsteps')
            z_offset = ((z1 - z0)*z_rotation_distance)/(z_microsteps*200)
            x_offset = x1*x_rotation_distance/(x_microsteps*200) - (x0*x_rotation_distance/(x_microsteps*200)) + self.t0_offset_x - self.t1_offset_x
            y_offset = y1*x_rotation_distance/(x_microsteps*200) - (y0*x_rotation_distance/(x_microsteps*200)) + self.t0_offset_y - self.t1_offset_y
            
            #self.gcode.respond_info("handle_align_end offset : x=%f y=%f z=%f" % (x_offset,y_offset,z_offset))
            self.gcode.run_script_from_command("M118 XYZ_ALIGN=1 x_offset=%f y_offset=%f z_offset=%f"% (x_offset,y_offset,z_offset))
            
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=xoffset VALUE=%f" % (0-x_offset))
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=yoffset VALUE=%f" % (0-y_offset))
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=zoffset VALUE=%f" % (0-z_offset))

        elif(res == 0):
            z0 = params['z0']
            z1 = params['z1']
            x0 = params['x0']
            y0 = params['y0']
            x1 = params['x1']
            y1 = params['y1']
            self.gcode.run_script_from_command("M118 XYZ_ALIGN=0")
            self.gcode.respond_info("handle_align_end: res = %d x0 = %d x1 = %d y0 = %d y1 = %d z0 = %d z1 = %d" % (res,x0,x1,y0,y1,z0,z1))

        elif(res == 2):
            self.z_value = params['z0']
            self.gcode.respond_info("handle_align_end: z = %d" % (self.z_value))

    def cmd_XYZ_ALIGN_TEST(self, gcmd):
        self.align_start_cmd.send([self.oid, 1])
       #self.gcode.respond_info("xyz_align: %d" % (self.oid))
    def get_z_press_val(self):
        z_offset = 1;
        while(z_offset < 15):
            self.gcode.run_script_from_command("G1 Z%d\n" % (self.offset_z - z_offset))
            self.gcode.run_script_from_command("M400\n")
            self.z_value = 1
            self.align_start_cmd.send([self.oid, 2])
            time.sleep(1.0) 
            #self.gcode.run_script_from_command("G4 P1000\n")
            if (self.z_value == 0):
                self.offset_z_press = self.offset_z - z_offset
                break
            z_offset += 1    
            
    def cmd_XYZ_ALIGN(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        status = self.printer.lookup_object('toolhead').get_status(curtime)['homed_axes']
        if status != 'xyz':
            raise self.printer.command_error("Must home before probe")
        else:
            self.align_start_cmd.send([self.oid, 0])
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE NAME=pre_offset_state\n"
                "G90\n"
                "G1 Z%d\n"
                "G1 F3000\n"
                "T0\n"
                "G1 X100 Y100 F3000\n"
                "G1 X%d Y%d\n"
                % (self.offset_z, self.t0_offset_x, self.t0_offset_y))

            self.get_z_press_val()    
            if (self.z_value == 1):
                self.gcode.respond_info("Z press value not found, aborting alignment")
                return

            self.gcode.run_script_from_command(
                "G1 F%d\n"
                "G1 Z%d\n"
                "G1 Z%d\n"
                % (self.speed,self.offset_z,self.offset_z_press))
            self.gcode.run_script_from_command(
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 Y%d\n"
                "G1 Z%d\n"
                "G1 Y%d\n"
                "G1 Z%d\n"
                "G1 Y%d\n"
                "G1 Z%d\n"   
                % (self.t0_offset_x+self.move_offset,self.offset_z,self.t0_offset_x,self.offset_z_press,self.t0_offset_x-self.move_offset,self.offset_z,self.t0_offset_x,self.offset_z_press,self.t0_offset_y+self.move_offset,self.offset_z,self.t0_offset_y,self.offset_z_press,self.t0_offset_y-self.move_offset,self.offset_z))
                #add change to t1
            self.gcode.run_script_from_command("G1 F3000")    
            self.gcode.run_script_from_command("G1 Y100")
            self.gcode.run_script_from_command("T1")
            self.gcode.run_script_from_command("SET_GCODE_OFFSET X=0 Y=0 Z=0")
            self.gcode.run_script_from_command(
                "G1 Z%d F3000\n"
                "G1 X%d Y%d\n"
                % (self.offset_z, self.t1_offset_x, self.t1_offset_y))
                
            self.get_z_press_val()    
            if (self.z_value == 1):
                self.gcode.respond_info("Z press value not found, aborting alignment")
                return

            self.gcode.run_script_from_command(
                "G1 F%d\n"
                "G1 Z%d\n"
                "G1 Z%d\n"
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 X%d\n"
                "G1 Z%d\n"
                "G1 Y%d\n"
                "G1 Z%d\n"
                "G1 Y%d\n"
                "G1 Z%d\n"
                "G1 Y%d\n"
                "G1 Z%d\n"
                "RESTORE_GCODE_STATE NAME=pre_offset_state\n"
                % (self.speed,self.offset_z,self.offset_z_press,self.t1_offset_x+self.move_offset,self.offset_z,self.t1_offset_x,self.offset_z_press,self.t1_offset_x-self.move_offset,self.offset_z,self.t1_offset_x,self.offset_z_press,self.t1_offset_y+self.move_offset-2,self.offset_z,self.t1_offset_y,self.offset_z_press,self.t1_offset_y-self.move_offset,self.offset_z))
            self.gcode.run_script_from_command("G1 F3000")
            self.gcode.run_script_from_command("G1 Y100")
            self.gcode.run_script_from_command("T0")
            self.gcode.run_script_from_command("M400") 
            self.align_start_cmd.send([self.oid, 1])
    
    
    
    cmd_XYZ_ALIGN_help = "This is a aniso customG-code command"

def load_config(config):
    return xyz_align(config)
    