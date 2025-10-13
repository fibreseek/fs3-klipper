import pins, logging
import configparser
from . import output_pin
import time
import os

class xyz_align:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.session_x = config.getsection('stepper_x')
        self.session_y = config.getsection('stepper_y')
        self.session_z = config.getsection('stepper_z')
        self.z_init_value = 10.0
        self.gcode = self.printer.lookup_object('gcode')
        self.t0_offset_x = config.getfloat('t0_offset_x')
        self.t0_offset_y = config.getfloat('t0_offset_y')
        self.t0_offset_z = self.z_init_value
        self.t1_offset_x = config.getfloat('t1_offset_x')
        self.t1_offset_y = config.getfloat('t1_offset_y')
        self.t1_offset_z = self.z_init_value
        self.move_offset = config.getfloat('move_offset')
        self.speed = config.getfloat('speed',3000)
        self.default_t0_offset_x = self.t0_offset_x
        self.default_t0_offset_y = self.t0_offset_y
        self.default_t1_offset_x = self.t1_offset_x
        self.default_t1_offset_y = self.t1_offset_y
        self.offset_z = 10.0
        self.z_value = 1.0
        self.y_max_travel = self.session_y.getfloat('position_max', 337.0)
        # Use absolute path to avoid path resolution issues
        self.ini_path = os.path.expanduser("~/printer_data/config/xyz_offset.ini")
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(config.get('pin'))
        self.mcu = pin_params['chip']
        self.oid = self.mcu.create_oid()
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self.build_config)

        self.printer.add_object('xyz_align', self)
        self.gcode.register_command('XYZ_ALIGN', self.cmd_XYZ_ALIGN, desc=self.cmd_XYZ_ALIGN_help)
        self.gcode.register_command('XYZ_ALIGN_INFO', self.cmd_XYZ_ALIGN_INFO, desc=self.cmd_XYZ_ALIGN_help)
        self.gcode.register_command('XYZ_ALIGN_T1', self.cmd_XYZ_ALIGN_T1, desc=self.cmd_XYZ_ALIGN_help)
        self.gcode.register_command('XYZ_ALIGN_T0', self.cmd_XYZ_ALIGN_T0, desc=self.cmd_XYZ_ALIGN_help)
        self.gcode.register_command('XYZ_ALIGN_CLEAR', self.cmd_XYZ_ALIGN_TEST, desc=self.cmd_XYZ_ALIGN_help)
        self.mcu.register_response(self.handle_align_end,"xyz_align_end", self.oid)
        self.printer.register_event_handler("klippy:ready", self.xyz_init_offset)

    def xyz_init_offset(self):
        config = configparser.ConfigParser()
        
        try:
            if os.path.exists(self.ini_path):
                config.read(self.ini_path)
                
                if 't0_offset' in config:
                    self.t0_offset_x = config.getfloat('t0_offset', 'x_offset')
                    self.t0_offset_y = config.getfloat('t0_offset', 'y_offset')
                    # If t0_offset_y + move_offset exceeds Y max travel, reset t0_offset_y
                    if self.t0_offset_y + self.move_offset > self.y_max_travel:
                        self.t0_offset_y = self.default_t0_offset_y

                if 't1_offset' in config:
                    self.t1_offset_x = config.getfloat('t1_offset', 'x_offset')
                    self.t1_offset_y = config.getfloat('t1_offset', 'y_offset')
                    # If t1_offset_y + move_offset exceeds Y max travel, reset t1_offset_y
                    if self.t1_offset_y + self.move_offset > self.y_max_travel:
                        self.t1_offset_y = self.default_t1_offset_y
                
                logging.info("Loaded xyz_offset.ini value: T0(%.3f,%.3f) T1(%.3f,%.3f)" % 
                           (self.t0_offset_x, self.t0_offset_y, self.t1_offset_x, self.t1_offset_y))
            else:
                logging.info("xyz_offset.ini file not found, using default values")
                
        except Exception as e:
            logging.error("Failed to load xyz_offset.ini: %s" % str(e))
            self.gcode.respond_info("Warning: Failed to load xyz_offset.ini, using default values")

        logging.info("init xyz_offset.ini file end")


    def build_config(self):
        self.mcu.add_config_cmd("align_start oid=%d val=%d" % (self.oid, 0))
        self.align_start_cmd = self.mcu.lookup_command("align_start oid=%c val=%c", cq=self.cmd_queue)
        #self.xyz_init_offset()

    def rm_xyz_offset_ini(self):
        file_path = self.ini_path

        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"{file_path} xyz_offset.ini deleted successfully.")
        else:
            print("xyz_offset.ini file does not exist.")

    def xyz_update_offset(self, x0, y0, x1, y1):
        x_rotation_distance = self.session_x.getfloat('rotation_distance')
        x_microsteps =self.session_x.getfloat('microsteps')
        z_rotation_distance = self.session_z.getfloat('rotation_distance') 
        z_microsteps = self.session_z.getfloat('microsteps')
        config = configparser.ConfigParser()
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(self.ini_path), exist_ok=True)
        config.read(self.ini_path)

        # Always update the configuration (remove the restrictive condition)
        if 't0_offset' not in config:
            config['t0_offset'] = {}
        if 't1_offset' not in config:
            config['t1_offset'] = {}
        
        temp_t0_offset_x = x0*x_rotation_distance/(x_microsteps*200)
        temp_t0_offset_y = y0*x_rotation_distance/(x_microsteps*200)
        temp_t1_offset_x = x1*x_rotation_distance/(x_microsteps*200)
        temp_t1_offset_y = y1*x_rotation_distance/(x_microsteps*200)

        # self.t0_offset_x = self.t0_offset_x - int(temp_t0_offset_x)
        # self.t0_offset_y = self.t0_offset_y - int(temp_t0_offset_y)
        # self.t1_offset_x = self.t1_offset_x - int(temp_t1_offset_x)
        # self.t1_offset_y = self.t1_offset_y - int(temp_t1_offset_y)
        self.t0_offset_x = self.t0_offset_x - temp_t0_offset_x
        self.t0_offset_y = self.t0_offset_y - temp_t0_offset_y
        self.t1_offset_x = self.t1_offset_x - temp_t1_offset_x
        self.t1_offset_y = self.t1_offset_y - temp_t1_offset_y
        # If t0_offset_y + move_offset exceeds Y max travel, reset t0_offset_y
        if self.t0_offset_y + self.move_offset > self.y_max_travel:
            self.t0_offset_y = self.default_t0_offset_y
        # If t1_offset_y + move_offset exceeds Y max travel, reset t1_offset_y
        if self.t1_offset_y + self.move_offset > self.y_max_travel:
            self.t1_offset_y = self.default_t1_offset_y

        config['t0_offset']['x_offset'] = str(self.t0_offset_x)
        config['t0_offset']['y_offset'] = str(self.t0_offset_y)
        config['t1_offset']['x_offset'] = str(self.t1_offset_x)
        config['t1_offset']['y_offset'] = str(self.t1_offset_y)
        logging.info("Updated xyz_offset.ini: T0(%.3f,%.3f) T1(%.3f,%.3f)" % 
                           (self.t0_offset_x, self.t0_offset_y, self.t1_offset_x, self.t1_offset_y))

        try:
            with open(self.ini_path, 'w') as configfile:
                config.write(configfile)
            self.gcode.respond_info("update xyz_offset.ini file success")
        except Exception as e:
            self.gcode.respond_info("update xyz_offset.ini file failed: %s" % str(e))
            logging.error("Failed to update xyz_offset.ini: %s" % str(e))

        # self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=xoffset VALUE=%f" % (x_offset))
        # self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=yoffset VALUE=%f" % (y_offset))
        # self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=zoffset VALUE=%f" % (z_offset))
        

    def handle_align_end(self, params):
        # Expand the message ack_count from 8-bit
        res = params['res']
        self.gcode.respond_info("handle_align_end: res = %d" % (res))
        
        if(res == 50):
            z0 = params['z0']
            z1 = params['z1']
            x0 = params['x0']/2
            y0 = params['y0']/2
            x1 = params['x1']/2
            y1 = params['y1']/2

            x_rotation_distance = self.session_x.getfloat('rotation_distance')
            x_microsteps =self.session_x.getfloat('microsteps')
            z_rotation_distance = self.session_z.getfloat('rotation_distance') 
            z_microsteps = self.session_z.getfloat('microsteps')

            z_offset = ((z1 - z0)*z_rotation_distance)/(z_microsteps*200)
            x_offset = (x1*x_rotation_distance/(x_microsteps*200)) - (x0*x_rotation_distance/(x_microsteps*200))+ self.t0_offset_x - self.t1_offset_x
            y_offset = (y1*x_rotation_distance/(x_microsteps*200)) - (y0*x_rotation_distance/(x_microsteps*200)) + self.t0_offset_y - self.t1_offset_y
            
            # self.gcode.run_script_from_command("M118 [NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=1 x_offset=%f y_offset=%f z_offset=%f"% (x_offset,y_offset,z_offset))
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=xoffset VALUE=%f" % (0-x_offset))
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=yoffset VALUE=%f" % (0-y_offset))
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=zoffset VALUE=%f" % (0-z_offset))
            temp_xoffset = (x1*x_rotation_distance/(x_microsteps*200)) - (x0*x_rotation_distance/(x_microsteps*200))
            temp_yoffset = (y1*x_rotation_distance/(x_microsteps*200)) - (y0*x_rotation_distance/(x_microsteps*200))
            self.gcode.respond_info("[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=1 x_offset=%f y_offset=%f z_offset=%f"% (x_offset,y_offset,z_offset))
            self.gcode.respond_info("handle_align_end: res = %d x0 = %f x1 = %f y0 = %f y1 = %f z0 = %d z1 = %d temp_xoffset = %f temp_yoffset = %f" % (res,x0,x1,y0,y1,z0,z1,temp_xoffset,temp_yoffset))
            
            
            self.xyz_update_offset(x0, y0, x1, y1)
            self.t0_offset_z = self.z_init_value
            self.t1_offset_z = self.z_init_value
            with open('xyz_align.txt', 'a') as file:
                file.write("%f %f %f \n" % ((0-x_offset), (0-y_offset), (0-z_offset)))
        elif(res == 2):
            self.z_value = params['z0']
            self.gcode.respond_info("handle_align_end: z = %d" % (self.z_value))
        else:
            self.gcode.respond_info("[NOZZLE_OFFSET_CALIBRATION] XYZ_ALIGN=0 count=%d" % (res))


    def cmd_XYZ_ALIGN_TEST(self, gcmd):
        self.align_start_cmd.send([self.oid, 0])
       #self.gcode.respond_info("xyz_align: %d" % (self.oid))
    def get_z_press_val(self, t):
        z_offset = 1
        self.z_value = 1
        while(z_offset <= (15 - t*3)):
            self.gcode.run_script_from_command("G1 Z%f F80\n" % (self.offset_z - z_offset))
            self.gcode.run_script_from_command("M400\n")
            self.align_start_cmd.send([self.oid, 2])
            #time.sleep(0.1) 
            #self.gcode.run_script_from_command("G4 P1000\n")
            self.printer.get_reactor().pause(0.1)

            if (self.z_value == 0):
                self.offset_z_press = self.offset_z - z_offset -0.3
                self.gcode.respond_info("cmd_XYZ_ALIGN: offset_z_press = %f, self.offset_z = %f, z_offset = %f" % (self.offset_z_press, self.offset_z, z_offset))
                break
            z_offset += 0.5    
            
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
                "G1 Z%.3f\n"
                "G1 F3000\n"
                "T0\n"
                # "G1 X100 Y100 F3000\n"
                "G1 X%.3f Y%.3f F3000\n"
                % (self.offset_z, self.t0_offset_x, self.t0_offset_y))
            self.gcode.respond_info("cmd_XYZ_ALIGN: tx0 = %.3f ty0 = %.3f; tx1 = %.3f ty1 = %.3f" % (self.t0_offset_x,self.t0_offset_y,self.t1_offset_x,self.t1_offset_y))    
            for i in range(5):
                if(self.t0_offset_z == self.z_init_value):
                    self.get_z_press_val(0)  
                    if(self.z_value == 0):
                        self.t0_offset_z = self.offset_z_press
                else:
                    self.z_value = 0
                    self.offset_z_press = self.t0_offset_z
                    self.gcode.run_script_from_command("G1 Z%.3f F300\n" % (self.offset_z_press))
                    self.gcode.run_script_from_command("G1 Z%.3f F80\n" % (self.offset_z))
                    self.gcode.respond_info("cmd_XYZ_ALIGN: z_press0 = %.3f z_lift0 = %.3f" % (self.offset_z_press, self.offset_z))

                if (self.z_value == 1):
                    self.gcode.respond_info("[NOZZLE_OFFSET_CALIBRATION] Error: The left nozzle cannot locate the offset calibration sensor.")
                    self.rm_xyz_offset_ini()
                    return

                self.gcode.run_script_from_command("M118 [NOZZLE_OFFSET_CALIBRATION] Positioning the left nozzle location...")
                self.gcode.run_script_from_command(
                    "G1 F%d\n"
                    "G1 Z%.3f\n"
                    "G1 Z%.3f\n"
                    % (self.speed,self.offset_z,self.offset_z_press))
                self.gcode.run_script_from_command(
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    % (self.t0_offset_x+self.move_offset,self.offset_z,self.t0_offset_x,self.offset_z_press,self.t0_offset_x-self.move_offset,self.offset_z,self.t0_offset_x,self.offset_z_press,self.t0_offset_y+self.move_offset,self.offset_z,self.t0_offset_y,self.offset_z_press,self.t0_offset_y-self.move_offset,self.offset_z,self.t0_offset_y))
                    #add change to t1
                # self.gcode.run_script_from_command("G1 F3000")    
                # self.gcode.run_script_from_command("G1 Y100")
            self.gcode.run_script_from_command("T1")
            self.gcode.run_script_from_command("SET_GCODE_OFFSET X=0 Y=0 Z=0")
            self.gcode.run_script_from_command(
                "G1 Z%.3f F600\n"
                "G1 X%.3f Y%.3f F3000\n"
                % (self.offset_z, self.t1_offset_x, self.t1_offset_y))
            self.gcode.respond_info("cmd_XYZ_ALIGN: x1 = %f y1 = %f" % (self.t1_offset_x,self.t1_offset_y))    
            for i in range(5):
                if(self.t1_offset_z == self.z_init_value):
                    self.get_z_press_val(1)  
                    self.t1_offset_z = self.offset_z_press
                else:
                    self.z_value = 0
                    self.offset_z_press = self.t1_offset_z  
                    self.gcode.run_script_from_command("G1 Z%.3f F300\n" % (self.offset_z_press))  
                    self.gcode.run_script_from_command("G1 Z%.3f F80\n" % (self.offset_z))  
                    self.gcode.respond_info("cmd_XYZ_ALIGN: z_press1 = %.3f z_lift1 = %.3f" % (self.offset_z_press, self.offset_z))

                if (self.z_value == 1):
                    self.rm_xyz_offset_ini()
                    self.gcode.respond_info("[NOZZLE_OFFSET_CALIBRATION] Error: The right nozzle cannot locate the offset calibration sensor.")
                    return

                self.gcode.run_script_from_command("M118 [NOZZLE_OFFSET_CALIBRATION] Positioning the right nozzle location...")
                self.gcode.run_script_from_command(
                    "G1 F%d\n"
                    "G1 Z%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 X%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    "G1 Z%.3f\n"
                    "G1 Y%.3f\n"
                    "RESTORE_GCODE_STATE NAME=pre_offset_state\n"
                    % (self.speed,self.offset_z,self.offset_z_press,self.t1_offset_x+self.move_offset,self.offset_z,self.t1_offset_x,self.offset_z_press,self.t1_offset_x-self.move_offset,self.offset_z,self.t1_offset_x,self.offset_z_press,self.t1_offset_y+self.move_offset-2,self.offset_z,self.t1_offset_y,self.offset_z_press,self.t1_offset_y-self.move_offset,self.offset_z,self.t1_offset_y))
                self.gcode.run_script_from_command("G1 Z10 F600")
            self.gcode.run_script_from_command("G1 Y280 F3000")
            self.gcode.run_script_from_command("T0")
            self.gcode.run_script_from_command("M400") 
            self.align_start_cmd.send([self.oid, 1])

    
    cmd_XYZ_ALIGN_help = "This is a aniso customG-code command"

    def cmd_XYZ_ALIGN_T1(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        status = self.printer.lookup_object('toolhead').get_status(curtime)['homed_axes']
        if status != 'xyz':
            raise self.printer.command_error("Must home before probe")
        else:
            self.gcode.run_script_from_command("T1")
            self.gcode.run_script_from_command("SET_GCODE_OFFSET X=0 Y=0 Z=0")
            self.gcode.run_script_from_command(
                "G1 Z%.3f F600\n"
                "G1 X%.3f Y%.3f F3000\n"
                % (self.offset_z, self.t1_offset_x, self.t1_offset_y))
                
            self.get_z_press_val(1)    
            if (self.z_value == 1):
                self.gcode.respond_info("[NOZZLE_OFFSET_CALIBRATION] Error: Z press value not found, aborting alignment")
                return

            self.gcode.run_script_from_command("M118 [NOZZLE_OFFSET_CALIBRATION] Positioning the right nozzle location...")
            self.gcode.run_script_from_command(
                "G1 F%d\n"
                "G1 Z%.3f\n"
                "G1 Z%.3f\n"
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 Y%.3f\n"
                "G1 Z%.3f\n"
                "G1 Y%.3f\n"
                "G1 Z%.3f\n"
                "G1 Y%.3f\n"
                "G1 Z%.3f\n"
                % (self.speed,self.offset_z,self.offset_z_press,self.t1_offset_x+self.move_offset,self.offset_z,self.t1_offset_x,self.offset_z_press,self.t1_offset_x-self.move_offset,self.offset_z,self.t1_offset_x,self.offset_z_press,self.t1_offset_y+self.move_offset-2,self.offset_z,self.t1_offset_y,self.offset_z_press,self.t1_offset_y-self.move_offset,self.offset_z))
            self.gcode.run_script_from_command("G1 Z10 F600")
            self.gcode.run_script_from_command("G1 Y280 F3000")
            # self.gcode.run_script_from_command("T0")
            self.gcode.run_script_from_command("M400") 
            self.gcode.respond_info("xyz_align end")
            self.align_start_cmd.send([self.oid, 3])


    def cmd_XYZ_ALIGN_T0(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        status = self.printer.lookup_object('toolhead').get_status(curtime)['homed_axes']
        if status != 'xyz':
            raise self.printer.command_error("Must home before probe")
        else:
            self.align_start_cmd.send([self.oid, 0])
            self.gcode.run_script_from_command(
                "G90\n"
                "G1 Z%.3f\n"
                "G1 F3000\n"
                #"T0\n"
                # "G1 X100 Y100 F3000\n"
                "G1 X%.3f Y%.3f F3000\n"
                % (self.offset_z, self.t0_offset_x, self.t0_offset_y))

            self.get_z_press_val(0)    
            if (self.z_value == 1):
                self.gcode.respond_info("[NOZZLE_OFFSET_CALIBRATION] Error: Z press value not found, aborting alignment")
                return

            self.gcode.run_script_from_command("M118 [NOZZLE_OFFSET_CALIBRATION] Positioning the left nozzle location...")
            self.gcode.run_script_from_command(
                "G1 F%d\n"
                "G1 Z%.3f\n"
                "G1 Z%.3f\n"
                % (self.speed,self.offset_z,self.offset_z_press))
            self.gcode.run_script_from_command(
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 X%.3f\n"
                "G1 Z%.3f\n"
                "G1 Y%.3f\n"
                "G1 Z%.3f\n"
                "G1 Y%.3f\n"
                "G1 Z%.3f\n"
                "G1 Y%.3f\n"
                "G1 Z%.3f\n"
                "M400\n"
                % (self.t0_offset_x+self.move_offset,self.offset_z,self.t0_offset_x,self.offset_z_press,self.t0_offset_x-self.move_offset,self.offset_z,self.t0_offset_x,self.offset_z_press,self.t0_offset_y+self.move_offset,self.offset_z,self.t0_offset_y,self.offset_z_press,self.t0_offset_y-self.move_offset,self.offset_z))
               
            self.align_start_cmd.send([self.oid, 3])

    def cmd_XYZ_ALIGN_INFO(self, gcmd):
        self.align_start_cmd.send([self.oid, 4])

def load_config(config):
    return xyz_align(config)
    