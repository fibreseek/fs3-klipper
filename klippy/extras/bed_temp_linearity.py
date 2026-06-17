#
# Measure probe trigger height at a single bed point across temperatures
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import csv
import datetime
import os

from . import probe


class BedTempLinearity:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.travel_speed = config.getfloat('travel_speed', 100., above=0.)
        self.output_path = None
        self.sample_count = 0
        self.gcode.register_command(
            'BED_TEMP_LINEARITY_INIT', self.cmd_BED_TEMP_LINEARITY_INIT,
            desc=self.cmd_BED_TEMP_LINEARITY_INIT_help)
        self.gcode.register_command(
            'BED_TEMP_LINEARITY_SAMPLE', self.cmd_BED_TEMP_LINEARITY_SAMPLE,
            desc=self.cmd_BED_TEMP_LINEARITY_SAMPLE_help)
        self.gcode.register_command(
            'BED_TEMP_LINEARITY_FINISH', self.cmd_BED_TEMP_LINEARITY_FINISH,
            desc=self.cmd_BED_TEMP_LINEARITY_FINISH_help)

    def _require_probe(self):
        pprobe = self.printer.lookup_object('probe', None)
        if pprobe is None:
            raise self.printer.command_error("bed_temp_linearity: probe not configured")
        return pprobe

    def _get_output_path(self, requested_path):
        if requested_path:
            path = os.path.abspath(os.path.expanduser(requested_path))
        else:
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(
                '/tmp', 'bed_temp_linearity_center_%s.csv' % (stamp,))
        directory = os.path.dirname(path) or '.'
        os.makedirs(directory, exist_ok=True)
        return path

    def _get_mesh_config(self):
        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if bed_mesh is None:
            raise self.printer.command_error(
                "bed_temp_linearity: bed_mesh not configured")
        calibration = bed_mesh.bmc.dump_calibration()
        return calibration['config']

    def _get_center_target(self, gcmd):
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if (x is None) != (y is None):
            raise gcmd.error("BED_TEMP_LINEARITY_SAMPLE requires both X and Y")
        if x is not None:
            return x, y
        cfg = self._get_mesh_config()
        radius = cfg['radius']
        origin = cfg['origin']
        if radius is not None:
            if origin is None:
                return 0., 0.
            return float(origin[0]), float(origin[1])
        mesh_min = cfg['mesh_min']
        mesh_max = cfg['mesh_max']
        return ((float(mesh_min[0]) + float(mesh_max[0])) * 0.5,
                (float(mesh_min[1]) + float(mesh_max[1])) * 0.5)

    def _get_probe_points(self, gcmd):
        points_x = gcmd.get_int('POINTS_X', 5, minval=1)
        points_y = gcmd.get_int('POINTS_Y', 5, minval=1)
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if (x is None) != (y is None):
            raise gcmd.error("BED_TEMP_LINEARITY_SAMPLE requires both X and Y")
        if (points_x > 1 or points_y > 1) and x is not None:
            raise gcmd.error(
                "BED_TEMP_LINEARITY_SAMPLE multi-point mode does not support X/Y override")
        if points_x == 1 and points_y == 1:
            return [self._get_center_target(gcmd)]
        cfg = self._get_mesh_config()
        radius = cfg['radius']
        origin = cfg['origin']
        if radius is not None:
            if origin is None:
                origin = (0., 0.)
            min_x = float(origin[0] - radius)
            max_x = float(origin[0] + radius)
            min_y = float(origin[1] - radius)
            max_y = float(origin[1] + radius)
        else:
            mesh_min = cfg['mesh_min']
            mesh_max = cfg['mesh_max']
            min_x = float(mesh_min[0])
            max_x = float(mesh_max[0])
            min_y = float(mesh_min[1])
            max_y = float(mesh_max[1])
        xs = [min_x] if points_x == 1 else [
            min_x + (max_x - min_x) * idx / float(points_x - 1)
            for idx in range(points_x)]
        ys = [min_y] if points_y == 1 else [
            min_y + (max_y - min_y) * idx / float(points_y - 1)
            for idx in range(points_y)]
        return [(bed_x, bed_y) for bed_y in ys for bed_x in xs]

    def _raise_for_xy_move(self, gcmd, pprobe):
        toolhead = self.printer.lookup_object('toolhead')
        current_pos = toolhead.get_position()
        probe_params = pprobe.get_probe_params(gcmd)
        clearance_z = probe_params['sample_retract_dist']
        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if bed_mesh is not None:
            clearance_z = max(clearance_z, bed_mesh.horizontal_move_z)
        if current_pos[2] < clearance_z:
            toolhead.manual_move([None, None, clearance_z],
                                 probe_params['lift_speed'])
            toolhead.wait_moves()

    def _get_bed_temperature(self):
        heater_bed = self.printer.lookup_object('heater_bed', None)
        if heater_bed is None:
            raise self.printer.command_error(
                "bed_temp_linearity: heater_bed not configured")
        eventtime = self.printer.get_reactor().monotonic()
        return heater_bed.get_status(eventtime)['temperature']

    def _append_row(self, row):
        with open(self.output_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    cmd_BED_TEMP_LINEARITY_INIT_help = "Initialize center-point bed temperature linearity capture"
    def cmd_BED_TEMP_LINEARITY_INIT(self, gcmd):
        self._require_probe()
        self.output_path = self._get_output_path(gcmd.get('OUTPUT', None))
        self.sample_count = 0
        created_at = datetime.datetime.now().isoformat(timespec='seconds')
        with open(self.output_path, 'w', newline='', encoding='utf-8') as f:
            f.write("# bed_temp_linearity capture\n")
            f.write("# created_at,%s\n" % (created_at,))
            f.write("temp_index,point_index,target_temp,actual_temp,probe_z,x,y,timestamp\n")
        gcmd.respond_info(
            "BED_TEMP_LINEARITY_INIT: writing samples to %s" % (self.output_path,))

    cmd_BED_TEMP_LINEARITY_SAMPLE_help = "Probe one point and append a CSV sample"
    def cmd_BED_TEMP_LINEARITY_SAMPLE(self, gcmd):
        if not self.output_path:
            raise gcmd.error("BED_TEMP_LINEARITY_SAMPLE requires BED_TEMP_LINEARITY_INIT first")
        temp_index = gcmd.get_int('INDEX', minval=1)
        target_temp = gcmd.get_float('TARGET')
        pprobe = self._require_probe()
        probe_x_offset, probe_y_offset, _ = pprobe.get_offsets()
        toolhead = self.printer.lookup_object('toolhead')
        point_results = []
        for point_index, (bed_x, bed_y) in enumerate(self._get_probe_points(gcmd), start=1):
            self._raise_for_xy_move(gcmd, pprobe)
            tool_x = bed_x - probe_x_offset
            tool_y = bed_y - probe_y_offset
            toolhead.manual_move([tool_x, tool_y], self.travel_speed)
            toolhead.wait_moves()
            trig_pt = probe.run_single_probe(pprobe, gcmd)
            actual_temp = self._get_bed_temperature()
            timestamp = datetime.datetime.now().isoformat(timespec='seconds')
            self._append_row([
                temp_index,
                point_index,
                "%.2f" % (target_temp,),
                "%.4f" % (actual_temp,),
                "%.6f" % (trig_pt[2],),
                "%.4f" % (bed_x,),
                "%.4f" % (bed_y,),
                timestamp,
            ])
            point_results.append((point_index, actual_temp, trig_pt[2], bed_x, bed_y))
            self.sample_count += 1
        avg_temp = sum(result[1] for result in point_results) / len(point_results)
        avg_z = sum(result[2] for result in point_results) / len(point_results)
        min_z = min(result[2] for result in point_results)
        max_z = max(result[2] for result in point_results)
        gcmd.respond_info(
            "BED_TEMP_LINEARITY_SAMPLE: temp_index=%d target=%.2fC points=%d "
            "actual_avg=%.4fC probe_z_avg=%.6f min=%.6f max=%.6f" % (
                temp_index, target_temp, len(point_results),
                avg_temp, avg_z, min_z, max_z))

    cmd_BED_TEMP_LINEARITY_FINISH_help = "Finalize bed temperature linearity capture"
    def cmd_BED_TEMP_LINEARITY_FINISH(self, gcmd):
        if not self.output_path:
            raise gcmd.error("BED_TEMP_LINEARITY_FINISH requires BED_TEMP_LINEARITY_INIT first")
        gcmd.respond_info(
            "BED_TEMP_LINEARITY_FINISH: %d samples written to %s" % (
                self.sample_count, self.output_path))
        self.output_path = None
        self.sample_count = 0


def load_config(config):
    return BedTempLinearity(config)
