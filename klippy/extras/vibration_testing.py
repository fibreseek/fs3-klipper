import subprocess
import os
from pathlib import Path
import re
import logging
from datetime import datetime

class ResonanceTester:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config
        
        # Register G-code command
        self.gcode.register_command(
            'FULL_RESONANCE_TEST',
            self.cmd_FULL_RESONANCE_TEST,
            desc=self.cmd_FULL_RESONANCE_TEST_help)
        
        # Setup logger
        self.logger = logging.getLogger(__name__)
        
        # Klipper paths
        self.home_dir = Path.home()
        self.klipper_scripts_dir = self.home_dir / "klipper" / "scripts"
        self.printer_config_dir = self.home_dir / "printer_data" / "config"
        self.calibrate_shaper_script = self.klipper_scripts_dir / "calibrate_shaper.py"
        
        # Verify script exists
        if not self.calibrate_shaper_script.exists():
            raise config.error(f"Calibration script not found at {self.calibrate_shaper_script}")

    cmd_FULL_RESONANCE_TEST_help = """
    Perform full resonance testing for both X and Y axes
    Usage: FULL_RESONANCE_TEST
    """

    def cmd_FULL_RESONANCE_TEST(self, gcmd):
        gcmd.respond_info("Starting full resonance testing for X and Y axes...")
        self.gcode.run_script_from_command("M118 [Calibration] Start Vibration Testing Calibration")
        
        # Execute tests for both axes
        self.run_resonance_tests('xy')
        
        # Process results
        x_freq = self.get_frequency('x')
        y_freq = self.get_frequency('y')
        
        gcmd.respond_info(f"X-axis recommended frequency: {x_freq} Hz")
        gcmd.respond_info(f"Y-axis recommended frequency: {y_freq} Hz")
        
        # Update config file
        self.update_config_file('x', x_freq)
        self.update_config_file('y', y_freq)
        
        gcmd.respond_info("Resonance testing completed. Please RESTART or FIRMWARE_RESTART to apply changes")
        self.gcode.run_script_from_command("M118 [Calibration] Vibration Testing Calibration Complete")

    def update_config_file(self, axis, frequency):
        """Update resonance_compensation.cfg with new frequency value"""
        try:
            # Always target resonance_compensation.cfg
            config_file = self.printer_config_dir / "resonance_compensation.cfg"
            
            # If file doesn't exist, create it
            if not config_file.exists():
                with open(config_file, 'w') as f:
                    f.write(f"[input_shaper]\nshaper_freq_{axis}: {frequency}\n")
                self.gcode.respond_info(f"Created {config_file} with shaper_freq_{axis}: {frequency}")
                return
            
            # Read current content
            with open(config_file, 'r') as f:
                lines = f.readlines()
            
            # Find and update the shaper_freq line
            updated = False
            pattern = re.compile(rf'^\s*shaper_freq_{axis}\s*:')
            
            with open(config_file, 'w') as f:
                for line in lines:
                    if pattern.match(line):
                        f.write(f"shaper_freq_{axis}: {frequency}\n")
                        updated = True
                    else:
                        f.write(line)
            
            if updated:
                self.gcode.respond_info(f"Updated {config_file} with shaper_freq_{axis}: {frequency}")
            else:
                # Add to existing [input_shaper] section or create new section
                self.add_shaper_freq_to_config(config_file, axis, frequency)
                
        except Exception as e:
            error_msg = f"Failed to update config file: {str(e)}"
            self.logger.error(error_msg)
            self.gcode.respond_error(error_msg)

    def add_shaper_freq_to_config(self, config_file, axis, frequency):
        """Add shaper config to [input_shaper] section in resonance_compensation.cfg"""
        try:
            with open(config_file, 'r') as f:
                content = f.read()
            
            # Find [input_shaper] section
            section_pattern = re.compile(r'^\[input_shaper\]', re.MULTILINE)
            match = section_pattern.search(content)
            
            if match:
                # Insert after section header
                pos = match.end()
                new_content = content[:pos] + f"\nshaper_freq_{axis}: {frequency}" + content[pos:]
                
                with open(config_file, 'w') as f:
                    f.write(new_content)
                
                self.gcode.respond_info(f"Added shaper_freq_{axis} to [input_shaper] section")
            else:
                # Add new section at end
                with open(config_file, 'a') as f:
                    f.write(f"\n\n[input_shaper]\nshaper_freq_{axis}: {frequency}\n")
                
                self.gcode.respond_info("Created [input_shaper] section with new parameters")
                
        except Exception as e:
            error_msg = f"Failed to add shaper_freq_{axis} to config: {str(e)}"
            self.logger.error(error_msg)
            self.gcode.respond_error(error_msg)

    def run_resonance_tests(self, axes):
        """Execute resonance test commands"""
        for axis in axes:
            self.gcode.respond_info(f"Testing {axis.upper()}-axis resonances...")
            self.gcode.run_script_from_command(f"TEST_RESONANCES AXIS={axis.upper()}")

    def get_frequency(self, axis):
        """Get recommended frequency for specified axis"""
        axis = axis.lower()
        try:
            csv_file = self.find_latest_csv(axis)
            if not csv_file:
                raise Exception(f"No resonance data found for {axis}-axis")
            
            self.gcode.respond_info(f"Analyzing {axis.upper()}-axis data...")
            output = self.run_calibration_script(csv_file, axis)
            freq = self.extract_final_frequency(output)
            
            if not freq:
                raise Exception("Failed to parse recommended frequency")
            
            return freq
            
        except Exception as e:
            error_msg = f"{axis.upper()}-axis processing failed: {str(e)}"
            self.logger.error(error_msg)
            self.gcode.respond_error(error_msg)
            return 0.0  # Return default value to prevent crash

    def find_latest_csv(self, axis):
        """Find latest resonance data file"""
        csv_files = sorted(Path('/tmp').glob(f"resonances_{axis}_*.csv"), 
                         key=os.path.getmtime)
        return csv_files[-1] if csv_files else None

    def run_calibration_script(self, csv_file, axis):
        """Run calibration script"""
        cmd = [
            str(self.calibrate_shaper_script),
            str(csv_file),
            "-o",
            f"/tmp/shaper_calibrate_{axis}.png"
        ]
        
        result = subprocess.run(
            cmd,
            check=True,
            capture_output=True,
            text=True
        )
        return result.stdout

    def extract_final_frequency(self, output):
        """Extract recommended frequency"""
        lines = output.strip().split('\n')
        last_line = lines[-1] if lines else ""
        
        match = re.search(r"@ ([\d.]+) Hz$", last_line)
        return float(match.group(1)) if match else None

def load_config(config):
    return ResonanceTester(config)