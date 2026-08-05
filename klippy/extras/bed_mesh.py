# Mesh Bed Leveling
#
# Copyright (C) 2018-2019 Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, json, collections, os, csv, time
from . import probe, aniso_temp_compensation
import numpy as np
from scipy.spatial.distance import pdist, squareform

PROFILE_VERSION = 1
PROFILE_OPTIONS = {
    'min_x': float, 'max_x': float, 'min_y': float, 'max_y': float,
    'x_count': int, 'y_count': int, 'mesh_x_pps': int, 'mesh_y_pps': int,
    'algo': str, 'tension': float
}
# profile 命名契约：
#   default        —— 双温完整校准保存的全床基准（calibration.cfg 写入）
#   runtime_active —— 当前生效网床的跨重启快照（替代原 npz），续打恢复只认它
BASELINE_PROFILE = "default"
RUNTIME_ACTIVE_PROFILE = "runtime_active"

class BedMeshError(Exception):
    pass

# PEP 485 isclose()
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# return true if a coordinate is within the region
# specified by min_c and max_c
def within(coord, min_c, max_c, tol=0.0):
    return (max_c[0] + tol) >= coord[0] >= (min_c[0] - tol) and \
        (max_c[1] + tol) >= coord[1] >= (min_c[1] - tol)

# Constrain value between min and max
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

# Linear interpolation between two values
def lerp(t, v0, v1):
    return (1. - t) * v0 + t * v1

def _is_temperature_mesh_profile(profile_name):
    if not profile_name:
        return False
    return '@' in profile_name and profile_name.endswith('C')

def calc_fast_bed_mesh_point_delta(
        z_mesh, temp_cpst, mesh_source, x, y, temp_trig, z_trig,
        probe_z_offset):
    z_trig_normalized = z_trig - probe_z_offset
    z_default = z_mesh.calc_z(x, y)
    should_apply_temp_comp = not (
        mesh_source == "active"
        and _is_temperature_mesh_profile(z_mesh.get_profile_name())
    )
    z_comp = temp_cpst.predict(x, y, temp_trig) if should_apply_temp_comp else 0.0
    z_predict = z_default + z_comp
    z_delta = z_trig_normalized - z_predict
    return {
        'z_trig_normalized': z_trig_normalized,
        'z_default': z_default,
        'z_comp': z_comp,
        'z_predict': z_predict,
        'z_delta': z_delta,
    }

def calc_fast_bed_mesh_strategy(
        z_deltas, max_abs_threshold, range_threshold, shift_apply_min=0.01):
    z_shift = float(np.median(z_deltas))
    residuals = [z - z_shift for z in z_deltas]
    residual_max = max(residuals)
    residual_min = min(residuals)
    residual_range = residual_max - residual_min
    residual_max_abs = max(abs(residual_max), abs(residual_min))
    raw_max = max(z_deltas)
    raw_min = min(z_deltas)
    raw_range = raw_max - raw_min
    raw_max_abs = max(abs(raw_max), abs(raw_min))

    if raw_max_abs <= max_abs_threshold and raw_range <= range_threshold:
        action = "load_profile_mesh"
    elif residual_max_abs <= max_abs_threshold and residual_range <= range_threshold:
        if abs(z_shift) >= shift_apply_min:
            action = "apply_mesh_z_shift"
        else:
            action = "load_profile_mesh"
    else:
        action = "recalibrate_mesh"

    return {
        'action': action,
        'z_shift': z_shift,
        'residual_max': residual_max,
        'residual_min': residual_min,
        'residual_range': residual_range,
        'residual_max_abs': residual_max_abs,
        'raw_max': raw_max,
        'raw_min': raw_min,
        'raw_range': raw_range,
        'raw_max_abs': raw_max_abs,
    }

def should_auto_save_calibrated_profile(temp_sampling_session, profile_name):
    """是否在 probe_finalize 时立即写入正式 profile。

    温补采样会话内返回 False：首轮基准只 stash，等 P=2 模型落盘成功后再 commit，
    避免中途取消留下「新 default + 旧 temp_comp.npz」。
    """
    if profile_name is None:
        return False
    if temp_sampling_session is not None:
        return False
    return True

# retrieve comma separated pair from config
def parse_config_pair(config, option, default, minval=None, maxval=None):
    pair = config.getintlist(option, (default, default))
    if len(pair) != 2:
        if len(pair) != 1:
            raise config.error("bed_mesh: malformed '%s' value: %s"
                               % (option, config.get(option)))
        pair = (pair[0], pair[0])
    if minval is not None:
        if pair[0] < minval or pair[1] < minval:
            raise config.error(
                "Option '%s' in section bed_mesh must have a minimum of %s"
                % (option, str(minval)))
    if maxval is not None:
        if pair[0] > maxval or pair[1] > maxval:
            raise config.error(
                "Option '%s' in section bed_mesh must have a maximum of %s"
                % (option, str(maxval)))
    return pair

# retrieve comma separated pair from a g-code command
def parse_gcmd_pair(gcmd, name, minval=None, maxval=None):
    try:
        pair = [int(v.strip()) for v in gcmd.get(name).split(',')]
    except:
        raise gcmd.error("Unable to parse parameter '%s'" % (name,))
    if len(pair) != 2:
        if len(pair) != 1:
            raise gcmd.error("Unable to parse parameter '%s'" % (name,))
        pair = (pair[0], pair[0])
    if minval is not None:
        if pair[0] < minval or pair[1] < minval:
            raise gcmd.error("Parameter '%s' must have a minimum of %d"
                             % (name, minval))
    if maxval is not None:
        if pair[0] > maxval or pair[1] > maxval:
            raise gcmd.error("Parameter '%s' must have a maximum of %d"
                             % (name, maxval))
    return pair

# retrieve comma separated coordinate from a g-code command
def parse_gcmd_coord(gcmd, name):
    try:
        v1, v2 = [float(v.strip()) for v in gcmd.get(name).split(',')]
    except:
        raise gcmd.error("Unable to parse parameter '%s'" % (name,))
    return v1, v2

def calc_open_tsp(pts):
    """
    轻量级tsp求解器
    - 点数不超过10组
    - 输入输出的首组点均固定为起点（当前点）
    - 各组点仅访问一次的最短开放路径
    
    参数:
        pts: List[List[float]]，如 [[x0,y0], [x1,y1], ...]
        
    返回:
        List[List[float]]: 最短路径点序列
    """
    n = len(pts)
    if n < 2:
        return pts
    
    # 转换为numpy数组
    pts = np.array(pts)

    # 1. 计算初始上界（原始顺序路径长度）
    upper_bound = 0.0
    for i in range(n - 1):
        dx = pts[i+1][0] - pts[i][0]
        dy = pts[i+1][1] - pts[i][1]
        upper_bound += np.sqrt(dx**2 + dy**2)

    # 2. 预计算距离矩阵
    dist = squareform(pdist(pts, metric='euclidean'))

    # 3. 初始化 DP 和 parent
    # dp[mask, i]=从某起点到点i、已访问点集为mask的最小路径长度
    # parent[mask, i]=状态(mask, i)的前一个点索引（用于回溯路径）
    dp = np.full((1 << n, n), np.inf, dtype=np.float32)
    parent = np.full((1 << n, n), -1, dtype=np.int16)
    HEAD_IDX = 0 # 起点索引，指定使用pts首点作为起点
    mask_now = 1 << HEAD_IDX
    dp[mask_now, HEAD_IDX] = 0.0 # 起点无前驱
    full_mask = (1 << n) - 1 # 所有点都已访问的掩码

    # 4. 动态规划
    # 遍历所有可能的已访问点集（位掩码）
    for mask_now in range(1, 1 << n):
        for i in range(n):
            # 跳过无效状态：当前点i不在mask 中
            if not (mask_now & (1 << i)) or dp[mask_now, i] >= np.inf:
                continue
            cost_now = dp[mask_now, i]
            
            # 剪枝 1: 当前成本已不优于已知上界，无需扩展
            if cost_now >= upper_bound:
                continue

            # 尝试从当前点 i 走到每一个未访问的点 j
            for j in range(n):
                if mask_now & (1 << j):
                    continue  # j 已访问，跳过
                
                mask_new = mask_now | (1 << j)
                cost_new = cost_now + dist[i][j]
                
                # 剪枝 2: 新路径已不优于上界，跳过
                if cost_new >= upper_bound:
                    continue

                # 更新DP表
                if cost_new < dp[mask_new, j]:
                    dp[mask_new, j] = cost_new
                    parent[mask_new, j] = i
                    
                    # 动态更新上界：当完成全访问时
                    if mask_new == full_mask and cost_new < upper_bound:
                        upper_bound = cost_new
    
    del dist, mask_now # 及时释放不再需要的变量

    # 5. 找最优终点（全访问状态下成本最小的终点）
    best_end = -1
    best_cost = np.inf
    for i in range(n):
        if dp[full_mask, i] < best_cost:
            best_cost = dp[full_mask, i]
            best_end = i

    # 安全回退：理论上不应发生
    if best_end == -1:
        return pts.tolist()

    # 6. 回溯路径
    path_idx = []
    curr = best_end
    curr_mask = full_mask
    while curr != -1:
        path_idx.append(curr)
        prev = parent[curr_mask, curr] # 找到前一个状态
        curr_mask ^= (1 << curr) # 从掩码中移除当前点
        curr = prev

    del dp, parent # 及时释放不再需要的变量
    path_idx.reverse() # 反转路径，因为是回溯得到的
    
    # 按索引顺序返回原始点坐标
    return [pts[i].tolist() for i in path_idx]

class BedMesh:
    FADE_DISABLE = 0x7FFFFFFF
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.last_position = [0., 0., 0., 0.]
        self.bmc = BedMeshCalibrate(config, self)
        self.z_mesh = None
        self.toolhead = None
        self.horizontal_move_z = config.getfloat('horizontal_move_z', 5.)
        self.fade_start = config.getfloat('fade_start', 1.)
        self.fade_end = config.getfloat('fade_end', 0.)
        self.fade_dist = self.fade_end - self.fade_start
        if self.fade_dist <= 0.:
            self.fade_start = self.fade_end = self.FADE_DISABLE
        self.log_fade_complete = False
        self.base_fade_target = config.getfloat('fade_target', None)
        self.fade_target = 0.
        self.tool_offset = 0.
        self.gcode = self.printer.lookup_object('gcode')
        self.splitter = MoveSplitter(config, self.gcode)
        # setup persistent storage
        self.pmgr = ProfileManager(config, self)
        # G28 清网床前的进程内暂存（不落盘）
        self._held_mesh = None
        self.save_profile = self.pmgr.save_profile
        # register gcodes
        self.gcode.register_command(
            'BED_MESH_OUTPUT', self.cmd_BED_MESH_OUTPUT,
            desc=self.cmd_BED_MESH_OUTPUT_help)
        self.gcode.register_command(
            'BED_MESH_MAP', self.cmd_BED_MESH_MAP,
            desc=self.cmd_BED_MESH_MAP_help)
        self.gcode.register_command(
            'BED_MESH_CLEAR', self.cmd_BED_MESH_CLEAR,
            desc=self.cmd_BED_MESH_CLEAR_help)
        self.gcode.register_command(
            'BED_MESH_OFFSET', self.cmd_BED_MESH_OFFSET,
            desc=self.cmd_BED_MESH_OFFSET_help)
        self.gcode.register_command(
            'BED_MESH_CHECK_RANGE', self.cmd_BED_MESH_CHECK_RANGE,
            desc=self.cmd_BED_MESH_CHECK_RANGE_help)
        self.gcode.register_command(
            'BED_MESH_VERIFY', self.cmd_BED_MESH_VERIFY,
            desc=self.cmd_BED_MESH_VERIFY_help)
        # G28 宏用：内存暂存 / 装回当前网床
        self.gcode.register_command(
            'BED_MESH_HOLD', self.cmd_BED_MESH_HOLD,
            desc=self.cmd_BED_MESH_HOLD_help)
        self.gcode.register_command(
            'BED_MESH_RESTORE_HOLD', self.cmd_BED_MESH_RESTORE_HOLD,
            desc=self.cmd_BED_MESH_RESTORE_HOLD_help)
        # Register dump webhooks
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint(
            "bed_mesh/dump_mesh", self._handle_dump_request
        )
        # Register transform
        gcode_move = self.printer.load_object(config, 'gcode_move')
        gcode_move.set_move_transform(self)
        # initialize status dict
        self.update_status()
    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.bmc.print_generated_points(logging.info, truncate=True)
    def set_mesh(self, mesh):
        if mesh is not None and self.fade_end != self.FADE_DISABLE:
            self.log_fade_complete = True
            if self.base_fade_target is None:
                self.fade_target = mesh.get_z_average()
            else:
                self.fade_target = self.base_fade_target
                min_z, max_z = mesh.get_z_range()
                if (not min_z <= self.fade_target <= max_z and
                        self.fade_target != 0.):
                    # fade target is non-zero, out of mesh range
                    err_target = self.fade_target
                    self.z_mesh = None
                    self.fade_target = 0.
                    raise self.gcode.error(
                        "bed_mesh: ERROR, fade_target lies outside of mesh z "
                        "range\nmin: %.4f, max: %.4f, fade_target: %.4f"
                        % (min_z, max_z, err_target))
            min_z, max_z = mesh.get_z_range()
            if self.fade_dist <= max(abs(min_z), abs(max_z)):
                self.z_mesh = None
                self.fade_target = 0.
                raise self.gcode.error(
                    "bed_mesh:  Mesh extends outside of the fade range, "
                    "please see the fade_start and fade_end options in"
                    "example-extras.cfg. fade distance: %.2f mesh min: %.4f"
                    "mesh max: %.4f" % (self.fade_dist, min_z, max_z))
        else:
            self.fade_target = 0.
        self.tool_offset = 0.
        self.z_mesh = mesh
        self.splitter.initialize(mesh, self.fade_target)
        # cache the current position before a transform takes place
        gcode_move = self.printer.lookup_object('gcode_move')
        gcode_move.reset_last_position()
        self.update_status()
    def _persist_config_no_restart(self):
        """将 pending 配置写入 printer.cfg，不重启（SAVE_CONFIG R=0）。"""
        try:
            # SAVE_CONFIG 主线程拼装配置会阻塞 reactor；落盘前先等运动结束。
            # 额外 dwell 很小幅预留 MCU 调度缓冲，避免与最后的步序重叠。
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.wait_moves()
            toolhead.dwell(0.05)
            self.gcode.run_script_from_command("SAVE_CONFIG R=0\n")
            return True
        except Exception:
            logging.exception(
                "bed_mesh: SAVE_CONFIG R=0 failed while persisting runtime_active")
            return False
    def save_runtime_mesh_snapshot(self, mesh=None, path=None):
        """把当前（或指定）网床写入会话 profile runtime_active，并 SAVE_CONFIG R=0 落盘。

        path 参数保留以兼容旧调用，已忽略（不再写 npz）。
        """
        z_mesh = mesh if mesh is not None else self.get_mesh()
        if z_mesh is None:
            logging.info("bed_mesh: runtime_active save skipped, no active mesh")
            return False
        probed_matrix = z_mesh.get_probed_matrix()
        if not probed_matrix or not probed_matrix[0]:
            logging.info(
                "bed_mesh: runtime_active save skipped, probed matrix empty")
            return False
        try:
            self.pmgr.save_profile_from_mesh(
                RUNTIME_ACTIVE_PROFILE, z_mesh, announce=False)
            if not self._persist_config_no_restart():
                return False
            logging.info(
                "bed_mesh: runtime_active saved to printer.cfg (SAVE_CONFIG R=0)")
            return True
        except Exception:
            logging.exception("bed_mesh: failed to save runtime_active profile")
            return False
    def load_runtime_mesh_snapshot(self, path=None):
        """从会话/配置中的 runtime_active 加载网床（不二次落盘）。

        path 参数保留以兼容旧调用，已忽略。
        """
        profile = self.pmgr.get_profiles().get(RUNTIME_ACTIVE_PROFILE)
        if profile is None:
            logging.info(
                "bed_mesh: runtime_active profile missing, cannot load snapshot")
            return False
        try:
            self.pmgr.load_profile(
                RUNTIME_ACTIVE_PROFILE, save_snapshot=False)
            logging.info(
                "bed_mesh: runtime_active profile loaded as recovery snapshot")
            return True
        except Exception:
            logging.exception(
                "bed_mesh: failed to load runtime_active profile")
            return False
    cmd_BED_MESH_HOLD_help = "Hold current bed mesh in memory for later restore"
    def cmd_BED_MESH_HOLD(self, gcmd):
        # 职责：G28 清网床前把当前网格深拷贝到内存，不写盘
        z_mesh = self.get_mesh()
        if z_mesh is None:
            self._held_mesh = None
            gcmd.respond_info("bed_mesh: HOLD skipped, no active mesh")
            return
        probed_matrix = z_mesh.get_probed_matrix()
        if not probed_matrix or not probed_matrix[0]:
            self._held_mesh = None
            gcmd.respond_info("bed_mesh: HOLD skipped, probed matrix empty")
            return
        self._held_mesh = {
            "points": [list(row) for row in probed_matrix],
            "mesh_params": collections.OrderedDict(z_mesh.get_mesh_params()),
            "profile_name": z_mesh.get_profile_name(),
        }
        gcmd.respond_info(
            "bed_mesh: held profile [%s] in memory"
            % (self._held_mesh["profile_name"],))
    cmd_BED_MESH_RESTORE_HOLD_help = (
        "Restore bed mesh previously held by BED_MESH_HOLD")
    def cmd_BED_MESH_RESTORE_HOLD(self, gcmd):
        # 职责：G28 归零后装回 HOLD 的网格；失败只提示，不抛错以免打断 G28
        if self._held_mesh is None:
            gcmd.respond_info("bed_mesh: RESTORE_HOLD skipped, nothing held")
            return
        try:
            mesh_params = collections.OrderedDict(self._held_mesh["mesh_params"])
            z_mesh = ZMesh(mesh_params, self._held_mesh["profile_name"])
            z_mesh.build_mesh(self._held_mesh["points"])
            self.set_mesh(z_mesh)
            gcmd.respond_info(
                "bed_mesh: restored held profile [%s]"
                % (self._held_mesh["profile_name"],))
        except Exception:
            logging.exception("bed_mesh: RESTORE_HOLD failed")
            gcmd.respond_info("bed_mesh: RESTORE_HOLD failed, mesh not restored")
    def get_z_factor(self, z_pos):
        z_pos += self.tool_offset
        if z_pos >= self.fade_end:
            return 0.
        elif z_pos >= self.fade_start:
            return (self.fade_end - z_pos) / self.fade_dist
        else:
            return 1.
    def get_position(self):
        # Return last, non-transformed position
        if self.z_mesh is None:
            # No mesh calibrated, so send toolhead position
            self.last_position[:] = self.toolhead.get_position()
            self.last_position[2] -= self.fade_target
        else:
            # return current position minus the current z-adjustment
            cur_pos = self.toolhead.get_position()
            x, y, z = cur_pos[:3]
            max_adj = self.z_mesh.calc_z(x, y)
            factor = 1.
            z_adj = max_adj - self.fade_target
            fade_z_pos = z + self.tool_offset
            if min(fade_z_pos, (fade_z_pos - max_adj)) >= self.fade_end:
                # Fade out is complete, no factor
                factor = 0.
            elif max(fade_z_pos, (fade_z_pos - max_adj)) >= self.fade_start:
                # Likely in the process of fading out adjustment.
                # Because we don't yet know the gcode z position, use
                # algebra to calculate the factor from the toolhead pos
                factor = ((self.fade_end + self.fade_target - fade_z_pos) /
                          (self.fade_dist - z_adj))
                factor = constrain(factor, 0., 1.)
            final_z_adj = factor * z_adj + self.fade_target
            self.last_position[:] = [x, y, z - final_z_adj] + cur_pos[3:]
        return list(self.last_position)
    def move(self, newpos, speed, material_axes=None):
        factor = self.get_z_factor(newpos[2])
        if (self.z_mesh is not None and factor
            and newpos[:3] == self.last_position[:3]):
            # A pure extrusion/material move must retain the mesh-adjusted
            # physical XYZ position.  Reusing the raw G-Code Z here creates a
            # tiny spatial move that can incorrectly drive material velocity
            # up to the extruder limit.
            curpos = self.toolhead.get_position()
            self.toolhead.move(list(curpos[:3]) + newpos[3:], speed,
                               material_axes)
        elif self.z_mesh is None or not factor:
            # No mesh calibrated, or mesh leveling phased out.
            x, y, z = newpos[:3]
            if self.log_fade_complete:
                self.log_fade_complete = False
                logging.info(
                    "bed_mesh fade complete: Current Z: %.4f fade_target: %.4f "
                    % (z, self.fade_target))
            self.toolhead.move([x, y, z + self.fade_target] + newpos[3:],
                               speed, material_axes)
        else:
            # 材料切分由 MoveSplitter 按沿程比例完成，避免外部再算 ratio
            self.splitter.build_move(self.last_position, newpos, factor,
                                     material_axes)
            while not self.splitter.traverse_complete:
                split_move, split_material = self.splitter.split()
                if split_move is not None:
                    # splitter 已按沿程比例切好该段材料，直接下发
                    self.toolhead.move(split_move, speed, split_material)
                else:
                    raise self.gcode.error(
                        "Mesh Leveling: Error splitting move ")
        self.last_position[:] = newpos
    def get_status(self, eventtime=None):
        return self.status
    def update_status(self):
        self.status = {
            "profile_name": "",
            "mesh_min": (0., 0.),
            "mesh_max": (0., 0.),
            "probed_matrix": [[]],
            "mesh_matrix": [[]],
            "profiles": self.pmgr.get_profiles()
        }
        if self.z_mesh is not None:
            params = self.z_mesh.get_mesh_params()
            mesh_min = (params['min_x'], params['min_y'])
            mesh_max = (params['max_x'], params['max_y'])
            probed_matrix = self.z_mesh.get_probed_matrix()
            mesh_matrix = self.z_mesh.get_mesh_matrix()
            self.status['profile_name'] = self.z_mesh.get_profile_name()
            self.status['mesh_min'] = mesh_min
            self.status['mesh_max'] = mesh_max
            self.status['probed_matrix'] = probed_matrix
            self.status['mesh_matrix'] = mesh_matrix
    def get_mesh(self):
        return self.z_mesh
    cmd_BED_MESH_OUTPUT_help = "Retrieve interpolated grid of probed z-points"
    def cmd_BED_MESH_OUTPUT(self, gcmd):
        if gcmd.get_int('PGP', 0):
            # Print Generated Points instead of mesh
            self.bmc.print_generated_points(gcmd.respond_info)
        elif self.z_mesh is None:
            gcmd.respond_info("Bed has not been probed")
        else:
            self.z_mesh.print_probed_matrix(gcmd.respond_info)
            self.z_mesh.print_mesh(gcmd.respond_raw, self.horizontal_move_z)
    cmd_BED_MESH_MAP_help = "Serialize mesh and output to terminal"
    def cmd_BED_MESH_MAP(self, gcmd):
        if self.z_mesh is not None:
            params = self.z_mesh.get_mesh_params()
            outdict = {
                'mesh_min': (params['min_x'], params['min_y']),
                'mesh_max': (params['max_x'], params['max_y']),
                'z_positions': self.z_mesh.get_probed_matrix()}
            gcmd.respond_raw("mesh_map_output " + json.dumps(outdict))
        else:
            gcmd.respond_info("Bed has not been probed")
    cmd_BED_MESH_CLEAR_help = "Clear the Mesh so no z-adjustment is made"
    def cmd_BED_MESH_CLEAR(self, gcmd):
        self.set_mesh(None)
    cmd_BED_MESH_CHECK_RANGE_help = (
        "Check probed mesh z range; error if range, |min|, or |max| "
        "exceeds MAX (mm)")
    def cmd_BED_MESH_CHECK_RANGE(self, gcmd):
        if self.z_mesh is None:
            raise gcmd.error("bed_mesh: no mesh loaded for range check")
        max_range = gcmd.get_float('MAX', 1.5, above=0.)
        matrix = self.z_mesh.get_probed_matrix()
        if not matrix or not matrix[0]:
            raise gcmd.error("bed_mesh: probed matrix is empty")
        z_min = min([min(row) for row in matrix])
        z_max = max([max(row) for row in matrix])
        mesh_range = z_max - z_min
        gcmd.respond_info(
            "bed_mesh range check: min=%.4f max=%.4f range=%.4f (limit=%.4f)"
            % (z_min, z_max, mesh_range, max_range))
        if mesh_range > max_range:
            raise gcmd.error(
                "bed_mesh: platform not flat, range %.2f mm > %.2f mm."
                % (mesh_range, max_range))
        if abs(z_min) > max_range or abs(z_max) > max_range:
            raise gcmd.error(
                "bed_mesh: mesh z out of range, |min|=%.2f |max|=%.2f "
                "> %.2f mm."
                % (abs(z_min), abs(z_max), max_range))
    cmd_BED_MESH_VERIFY_help = (
        "Probe at X,Y and compare with mesh z; error if delta exceeds "
        "LIMIT (mm)")
    def cmd_BED_MESH_VERIFY(self, gcmd):
        if self.z_mesh is None:
            raise gcmd.error("bed_mesh: no mesh loaded for verify")
        pprobe = self.printer.lookup_object('probe', None)
        if pprobe is None:
            raise gcmd.error("bed_mesh: probe not configured")
        x = gcmd.get_float('X', 150.)
        y = gcmd.get_float('Y', 150.)
        limit = gcmd.get_float('LIMIT', 0.05, above=0.)
        samples = gcmd.get_int('SAMPLES', 4, minval=1)
        speed = gcmd.get_float('SPEED', 100., above=0.)
        mesh_params = self.z_mesh.get_mesh_params()
        min_c = (mesh_params['min_x'], mesh_params['min_y'])
        max_c = (mesh_params['max_x'], mesh_params['max_y'])
        if not within((x, y), min_c, max_c, tol=.001):
            raise gcmd.error(
                "bed_mesh: verify point (%.2f, %.2f) outside mesh bounds "
                "(%.2f, %.2f) to (%.2f, %.2f)"
                % (x, y, min_c[0], min_c[1], max_c[0], max_c[1]))
        mesh_z = self.z_mesh.calc_z(x, y)
        z_offset = pprobe.get_offsets()[2]
        pparams = pprobe.get_probe_params(gcmd)
        lift_z = max(self.horizontal_move_z,
                     pparams['sample_retract_dist'] + 1.)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.manual_move([None, None, lift_z], speed)
        toolhead.wait_moves()
        toolhead.manual_move([x, y, None], speed)
        toolhead.wait_moves()
        probe_gcmd = self.gcode.create_gcode_command(
            "PROBE",
            "PROBE SAMPLES=%d SAMPLES_TOLERANCE_RETRIES=0"
            % (samples,),
            {'SAMPLES': str(samples), 'SAMPLES_TOLERANCE_RETRIES': '0'})
        pos = probe.run_single_probe(pprobe, probe_gcmd)
        probe_mesh_z = pos[2] - z_offset
        delta = probe_mesh_z - mesh_z
        gcmd.respond_info(
            "bed_mesh verify at (%.2f, %.2f): probe_z=%.4f z_offset=%.4f "
            "probe_mesh_z=%.4f mesh_z=%.4f delta=%.4f (limit=%.4f)"
            % (x, y, pos[2], z_offset, probe_mesh_z, mesh_z, delta, limit))
        if abs(delta) > limit:
            raise gcmd.error(
                "bed_mesh: verify failed, delta %.4f mm > %.4f mm, mesh unreliable"
                % (abs(delta), limit))
    cmd_BED_MESH_OFFSET_help = "Add X/Y offsets to the mesh lookup"
    def cmd_BED_MESH_OFFSET(self, gcmd):
        if self.z_mesh is not None:
            offsets = [None, None]
            for i, axis in enumerate(['X', 'Y']):
                offsets[i] = gcmd.get_float(axis, None)
            self.z_mesh.set_mesh_offsets(offsets)
            tool_offset = gcmd.get_float("ZFADE", None)
            if tool_offset is not None:
                self.tool_offset = tool_offset
            gcode_move = self.printer.lookup_object('gcode_move')
            gcode_move.reset_last_position()
        else:
            gcmd.respond_info("No mesh loaded to offset")
    def _handle_dump_request(self, web_request):
        eventtime = self.printer.get_reactor().monotonic()
        prb = self.printer.lookup_object("probe", None)
        th_sts = self.printer.lookup_object("toolhead").get_status(eventtime)
        result = {"current_mesh": {}, "profiles": self.pmgr.get_profiles()}
        if self.z_mesh is not None:
            result["current_mesh"] = {
                "name": self.z_mesh.get_profile_name(),
                "probed_matrix": self.z_mesh.get_probed_matrix(),
                "mesh_matrix": self.z_mesh.get_mesh_matrix(),
                "mesh_params": self.z_mesh.get_mesh_params()
            }
        mesh_args = web_request.get_dict("mesh_args", {})
        gcmd = None
        if mesh_args:
            gcmd = self.gcode.create_gcode_command("", "", mesh_args)
            with self.gcode.get_mutex():
                result["calibration"] = self.bmc.dump_calibration(gcmd)
        else:
            result["calibration"] = self.bmc.dump_calibration()
        offsets = [0, 0, 0] if prb is None else prb.get_offsets()
        result["probe_offsets"] = offsets
        result["axis_minimum"] = th_sts["axis_minimum"]
        result["axis_maximum"] = th_sts["axis_maximum"]
        web_request.send(result)


class ZrefMode:
    DISABLED = 0  # Zero reference disabled
    IN_MESH = 1   # Zero reference position within mesh
    PROBE = 2     # Zero refrennce position outside of mesh, probe needed


class BedMeshCalibrate:
    ALGOS = ['lagrange', 'bicubic']
    TEMP_CSV_FIELDNAMES = (
        "run_id",
        "stage_index",
        "stage_count",
        "target_temp",
        "measured_temp",
        "row_index",
        "col_index",
        "x",
        "y",
        "z",
    )
    def __init__(self, config, bedmesh):
        self.config = config
        self.printer = config.get_printer()
        self.orig_config = {'radius': None, 'origin': None}
        self.radius = self.origin = None
        self.mesh_min = self.mesh_max = (0., 0.)
        self.adaptive_margin = config.getfloat('adaptive_margin', 0.0)
        self.bedmesh = bedmesh
        self.mesh_config = collections.OrderedDict()
        self._init_mesh_config(config)
        self.probe_mgr = ProbeManager(
            config, self.orig_config, self.probe_finalize
        )
        try:
            self.probe_mgr.generate_points(
                self.mesh_config, self.mesh_min, self.mesh_max,
                self.radius, self.origin
            )
        except BedMeshError as e:
            raise config.error(str(e))
        self.temp_cpst = None # 温度补偿器
        self._temp_sampling_session = None
        self._temp_csv_session = None
        self._temp_csv_dir = "~/printer_data/config/bed_mesh_samples"
        self._profile_name = "default"
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'BED_MESH_CALIBRATE', self.cmd_BED_MESH_CALIBRATE,
            desc=self.cmd_BED_MESH_CALIBRATE_help)
        self.gcode.register_command(
            'BED_MESH_CALIBRATE_FAST', self.cmd_BED_MESH_CALIBRATE_FAST)
        self.gcode.register_command(
            'BED_MESH_TEMP', self.cmd_BED_MESH_TEMP) # 温度补偿接口
        self.gcode.register_command(
            'BED_MESH_TEMP_LOAD', self.cmd_BED_MESH_TEMP_LOAD)
    def _get_temp_comp(self):
        # Temperature model is for the full configured bed mesh. Do not size it
        # from mesh_config/mesh_min/max: those may still hold leftover adaptive
        # sizes from a prior BED_MESH_CALIBRATE with PRINT_MIN/PRINT_MAX.
        if self.temp_cpst is None:
            self.temp_cpst = aniso_temp_compensation.AnisoTempCompensation(
                self.orig_config['mesh_min'], self.orig_config['mesh_max'],
                self.orig_config['x_count'],
                self.orig_config['y_count'],
                root_config=self.config)
        return self.temp_cpst
    def _load_profile_mesh(self, profile_name):
        profile = self.bedmesh.pmgr.profiles.get(profile_name, None)
        if profile is None:
            raise self.gcode.error(
                f"bed_mesh: Unknown profile {profile_name}")
        z_mesh = ZMesh(profile['mesh_params'], profile_name)
        try:
            z_mesh.build_mesh(profile['points'])
        except BedMeshError as e:
            raise self.gcode.error(str(e))
        return z_mesh
    def _get_probe_axis_values(self, mesh_params):
        x_count = mesh_params['x_count']
        y_count = mesh_params['y_count']
        min_x = mesh_params['min_x']
        max_x = mesh_params['max_x']
        min_y = mesh_params['min_y']
        max_y = mesh_params['max_y']
        if x_count == 1:
            xs = [float(min_x)]
        else:
            xs = [
                float(min_x + (max_x - min_x) * idx / float(x_count - 1))
                for idx in range(x_count)
            ]
        if y_count == 1:
            ys = [float(min_y)]
        else:
            ys = [
                float(min_y + (max_y - min_y) * idx / float(y_count - 1))
                for idx in range(y_count)
            ]
        return xs, ys
    def build_temperature_mesh(self, target_temp, profile_name="default"):
        z_mesh = self._load_profile_mesh(profile_name)
        temp_cpst = self._get_temp_comp()
        if not temp_cpst.load_model():
            logging.info(
                "bed_mesh: temp model missing, skip predicted mesh build")
            self.temp_cpst = None
            return None
        mesh_params = z_mesh.get_mesh_params()
        xs, ys = self._get_probe_axis_values(mesh_params)
        base_points = z_mesh.get_probed_matrix()
        predicted_points = []
        for y_idx, row in enumerate(base_points):
            predicted_row = []
            for x_idx, base_z in enumerate(row):
                x = xs[x_idx]
                y = ys[y_idx]
                z_comp = temp_cpst.predict(x, y, target_temp)
                predicted_row.append(round(float(base_z + z_comp), 6))
            predicted_points.append(predicted_row)
        predicted_name = "%s@%.1fC" % (profile_name, target_temp)
        predicted_mesh = ZMesh(mesh_params, predicted_name)
        predicted_mesh.build_mesh(predicted_points)
        self.temp_cpst = None
        return predicted_mesh
    def load_temperature_mesh(self, target_temp, profile_name="default",
                              save_snapshot=True):
        predicted_mesh = self.build_temperature_mesh(target_temp, profile_name)
        if predicted_mesh is None:
            self.bedmesh.pmgr.load_profile(profile_name)
            if save_snapshot:
                self.bedmesh.save_runtime_mesh_snapshot()
            return False
        self.bedmesh.set_mesh(predicted_mesh)
        if save_snapshot:
            self.bedmesh.save_runtime_mesh_snapshot(predicted_mesh)
        return True
    def _parse_temp_list(self, gcmd):
        raw = gcmd.get("TEMPS", None)
        if raw is None:
            return None
        raw = raw.strip().strip('"').strip("'")
        temps = [float(v.strip()) for v in raw.split(',') if v.strip()]
        if not temps:
            raise gcmd.error("Parameter 'TEMPS' must contain at least one value")
        return temps
    def _reset_temp_csv_session(self):
        self._temp_csv_session = None
    def _reset_temp_sampling_session(self):
        self._temp_sampling_session = None
    def _start_temp_sampling_session(self, temps):
        if not temps:
            self._reset_temp_sampling_session()
            return None
        self._temp_sampling_session = {
            "temps": list(temps),
            # baseline_stashed：首轮网床已暂存；baseline_profile_saved 兼容旧日志语义
            "baseline_stashed": False,
            "baseline_profile_saved": False,
            "baseline_probed_matrix": None,
            "baseline_mesh_params": None,
            "baseline_profile_name": "default",
        }
        return self._temp_sampling_session
    def abort_temp_sampling_session(self):
        """取消温补采样：丢弃未提交的 stash，不改动正式 default / temp_comp.npz。

        返回值说明当前清理结果，供 P=3 文案区分：
        - discarded: 丢弃了未提交的基准 stash
        - cleared: 有采样会话但无未提交 stash（例如 P=2 后残留清理）
        - idle: 本来就没有活跃采样会话（幂等空操作）
        """
        session = self._temp_sampling_session
        if session is None:
            result = "idle"
        elif session.get("baseline_stashed"):
            result = "discarded"
            logging.info(
                "bed_mesh: abort temp-sampling session, discard stashed baseline")
        else:
            result = "cleared"
            logging.info(
                "bed_mesh: abort temp-sampling session, no stashed baseline")
        self.temp_cpst = None
        self._reset_temp_sampling_session()
        self._reset_temp_csv_session()
        return result
    def _commit_temp_sampling_baseline(self):
        """模型落盘成功后，把 stash 的首轮基准提升为正式 profile（通常 default）。"""
        session = self._temp_sampling_session
        if session is None or not session.get("baseline_stashed"):
            logging.info("bed_mesh: no stashed baseline to commit")
            return False
        probed_matrix = session.get("baseline_probed_matrix")
        mesh_params = session.get("baseline_mesh_params")
        prof_name = session.get("baseline_profile_name") or "default"
        if not probed_matrix or not mesh_params:
            logging.warning("bed_mesh: stashed baseline incomplete, skip commit")
            return False
        z_mesh = ZMesh(mesh_params, prof_name)
        try:
            z_mesh.build_mesh(probed_matrix)
        except BedMeshError:
            logging.exception(
                "bed_mesh: failed to rebuild stashed baseline for [%s]",
                prof_name)
            return False
        ok = self.bedmesh.pmgr.save_profile_from_mesh(
            prof_name, z_mesh, announce=True)
        if ok:
            logging.info(
                "bed_mesh: committed temp-sampling baseline to profile [%s]",
                prof_name)
        return bool(ok)
    def _stash_temp_sampling_baseline(self, z_mesh, profile_name):
        """首轮探床结果写入 session，不提升正式 default。"""
        session = self._temp_sampling_session
        if session is None or z_mesh is None:
            return False
        if session.get("baseline_stashed"):
            return False
        probed_matrix = z_mesh.get_probed_matrix()
        mesh_params = z_mesh.get_mesh_params()
        if not probed_matrix or not probed_matrix[0]:
            logging.warning("bed_mesh: skip baseline stash, probed matrix empty")
            return False
        session["baseline_probed_matrix"] = [list(row) for row in probed_matrix]
        session["baseline_mesh_params"] = collections.OrderedDict(mesh_params)
        session["baseline_profile_name"] = profile_name or "default"
        session["baseline_stashed"] = True
        session["baseline_profile_saved"] = True
        logging.info(
            "bed_mesh: stashed temp-sampling baseline for deferred promote "
            "to [%s] (not writing official profile yet)",
            session["baseline_profile_name"])
        return True
    def _start_temp_csv_session(self, temps):
        self._reset_temp_csv_session()
        if not temps:
            return None
        sample_dir = os.path.expanduser(self._temp_csv_dir)
        run_id = time.strftime("%Y%m%d_%H%M%S")
        try:
            os.makedirs(sample_dir, exist_ok=True)
            filename = "bed_mesh_full_%s.csv" % (run_id,)
            filepath = os.path.join(sample_dir, filename)
            suffix = 1
            while os.path.exists(filepath):
                filename = "bed_mesh_full_%s_%02d.csv" % (run_id, suffix)
                filepath = os.path.join(sample_dir, filename)
                suffix += 1
            session = {
                "run_id": filename[len("bed_mesh_full_"):-len(".csv")],
                "path": filepath,
                "temps": list(temps),
                "stage_index": 0,
            }
            with open(filepath, "w", newline="") as csv_file:
                writer = csv.DictWriter(
                    csv_file, fieldnames=self.TEMP_CSV_FIELDNAMES)
                writer.writeheader()
            self._temp_csv_session = session
            logging.info("bed_mesh: sample csv session started: %s", filepath)
            return session
        except Exception:
            logging.exception(
                "bed_mesh: failed to start sample csv session in %s",
                sample_dir)
            self._reset_temp_csv_session()
            return None
    def _group_base_points_by_row(self, base_points):
        if not base_points:
            return []
        grouped_rows = []
        current_row = [base_points[0]]
        prev_y = base_points[0][1]
        for point in base_points[1:]:
            if not isclose(point[1], prev_y, abs_tol=.1):
                if len(current_row) > 1 and current_row[0][0] > current_row[-1][0]:
                    current_row.reverse()
                grouped_rows.append(current_row)
                current_row = [point]
            else:
                current_row.append(point)
            prev_y = point[1]
        if len(current_row) > 1 and current_row[0][0] > current_row[-1][0]:
            current_row.reverse()
        grouped_rows.append(current_row)
        return grouped_rows
    def _build_temp_csv_rows(self, base_points, probed_matrix, measured_temp):
        session = self._temp_csv_session
        if session is None or not probed_matrix or not probed_matrix[0]:
            return []
        grouped_points = self._group_base_points_by_row(base_points)
        if len(grouped_points) != len(probed_matrix):
            raise BedMeshError(
                "bed_mesh: sample csv point rows mismatch, base=%d probe=%d"
                % (len(grouped_points), len(probed_matrix)))
        stage_index = session["stage_index"]
        stage_count = len(session["temps"])
        target_temp = ""
        if stage_index < stage_count:
            target_temp = session["temps"][stage_index]
        rows = []
        for row_index, (row_points, row_values) in enumerate(
                zip(grouped_points, probed_matrix)):
            if len(row_points) != len(row_values):
                raise BedMeshError(
                    "bed_mesh: sample csv point count mismatch, row=%d base=%d probe=%d"
                    % (row_index, len(row_points), len(row_values)))
            for col_index, ((x_pos, y_pos), z_pos) in enumerate(
                    zip(row_points, row_values)):
                rows.append({
                    "run_id": session["run_id"],
                    "stage_index": stage_index,
                    "stage_count": stage_count,
                    "target_temp": target_temp,
                    "measured_temp": measured_temp,
                    "row_index": row_index,
                    "col_index": col_index,
                    "x": round(float(x_pos), 6),
                    "y": round(float(y_pos), 6),
                    "z": round(float(z_pos), 6),
                })
        return rows
    def _append_temp_csv_sample(self, probed_matrix, measured_temp):
        session = self._temp_csv_session
        if session is None:
            return False
        try:
            rows = self._build_temp_csv_rows(
                self.probe_mgr.get_base_points(), probed_matrix, measured_temp)
            if not rows:
                logging.info("bed_mesh: sample csv skipped, probed matrix empty")
                return False
            with open(session["path"], "a", newline="") as csv_file:
                writer = csv.DictWriter(
                    csv_file, fieldnames=self.TEMP_CSV_FIELDNAMES)
                writer.writerows(rows)
            logging.info(
                "bed_mesh: sample csv appended, stage=%d rows=%d path=%s",
                session["stage_index"], len(rows), session["path"])
            return True
        except Exception:
            logging.exception(
                "bed_mesh: failed to append sample csv stage=%d path=%s",
                session["stage_index"], session["path"])
            return False
        finally:
            session["stage_index"] += 1
    def print_generated_points(self, print_func, truncate=False):
        x_offset = y_offset = 0.
        probe = self.printer.lookup_object('probe', None)
        if probe is not None:
            x_offset, y_offset = probe.get_offsets()[:2]
        print_func("bed_mesh: generated points\nIndex"
                   " |  Tool Adjusted  |   Probe")
        points = self.probe_mgr.get_base_points()
        for i, (x, y) in enumerate(points):
            if i >= 50 and truncate:
                end = len(points) - 1
                print_func("...points %d through %d truncated" % (i, end))
                break
            adj_pt = "(%.1f, %.1f)" % (x - x_offset, y - y_offset)
            mesh_pt = "(%.1f, %.1f)" % (x, y)
            print_func(
                "  %-4d| %-16s| %s" % (i, adj_pt, mesh_pt))
        zero_ref_pos = self.probe_mgr.get_zero_ref_pos()
        if zero_ref_pos is not None:
            print_func(
                "bed_mesh: zero_reference_position is (%.2f, %.2f)"
                % (zero_ref_pos[0], zero_ref_pos[1])
            )
        substitutes = self.probe_mgr.get_substitutes()
        if substitutes:
            print_func("bed_mesh: faulty region points")
            for i, v in substitutes.items():
                pt = points[i]
                print_func("%d (%.2f, %.2f), substituted points: %s"
                           % (i, pt[0], pt[1], repr(v)))
    def _get_fast_bed_mesh_safe_bounds(self, mesh_min, mesh_max, probe_offsets):
        min_x, min_y = mesh_min
        max_x, max_y = mesh_max
        x_margin = abs(float(probe_offsets[0]))
        y_margin = abs(float(probe_offsets[1]))
        safe_min_x = min_x + x_margin
        safe_max_x = max_x - x_margin
        safe_min_y = min_y + y_margin
        safe_max_y = max_y - y_margin
        if safe_min_x > safe_max_x or safe_min_y > safe_max_y:
            raise BedMeshError(
                "bed_mesh: fast bed mesh safe bounds invalid. "
                "mesh_min=%s mesh_max=%s probe_offsets=%s"
                % (mesh_min, mesh_max, probe_offsets))
        return safe_min_x, safe_min_y, safe_max_x, safe_max_y
    def _clamp_fast_bed_mesh_points(self, points, mesh_min, mesh_max,
                                    probe_offsets):
        safe_min_x, safe_min_y, safe_max_x, safe_max_y = \
            self._get_fast_bed_mesh_safe_bounds(
                mesh_min, mesh_max, probe_offsets)
        clamped_points = []
        seen = set()
        for x, y in points:
            clamped_x = constrain(float(x), safe_min_x, safe_max_x)
            clamped_y = constrain(float(y), safe_min_y, safe_max_y)
            key = (round(clamped_x, 6), round(clamped_y, 6))
            if key in seen:
                continue
            seen.add(key)
            clamped_points.append([clamped_x, clamped_y])
        return clamped_points, (
            safe_min_x, safe_min_y), (safe_max_x, safe_max_y)
    def _init_mesh_config(self, config):
        mesh_cfg = self.mesh_config
        orig_cfg = self.orig_config
        self.radius = config.getfloat('mesh_radius', None, above=0.)
        if self.radius is not None:
            self.origin = config.getfloatlist('mesh_origin', (0., 0.), count=2)
            x_cnt = y_cnt = config.getint('round_probe_count', 5, minval=3)
            # round beds must have an odd number of points along each axis
            if not x_cnt & 1:
                raise config.error(
                    "bed_mesh: probe_count must be odd for round beds")
            # radius may have precision to .1mm
            self.radius = math.floor(self.radius * 10) / 10
            orig_cfg['radius'] = self.radius
            orig_cfg['origin'] = self.origin
            min_x = min_y = -self.radius
            max_x = max_y = self.radius
        else:
            # rectangular
            x_cnt, y_cnt = parse_config_pair(config, 'probe_count', 3, minval=3)
            min_x, min_y = config.getfloatlist('mesh_min', count=2)
            max_x, max_y = config.getfloatlist('mesh_max', count=2)
            if max_x <= min_x or max_y <= min_y:
                raise config.error('bed_mesh: invalid min/max points')
        orig_cfg['x_count'] = mesh_cfg['x_count'] = x_cnt
        orig_cfg['y_count'] = mesh_cfg['y_count'] = y_cnt
        orig_cfg['mesh_min'] = self.mesh_min = (min_x, min_y)
        orig_cfg['mesh_max'] = self.mesh_max = (max_x, max_y)

        pps = parse_config_pair(config, 'mesh_pps', 2, minval=0)
        orig_cfg['mesh_x_pps'] = mesh_cfg['mesh_x_pps'] = pps[0]
        orig_cfg['mesh_y_pps'] = mesh_cfg['mesh_y_pps'] = pps[1]
        orig_cfg['algo'] = mesh_cfg['algo'] = \
            config.get('algorithm', 'lagrange').strip().lower()
        orig_cfg['tension'] = mesh_cfg['tension'] = config.getfloat(
            'bicubic_tension', .2, minval=0., maxval=2.)
        self._verify_algorithm(config.error)
    def _verify_algorithm(self, error):
        params = self.mesh_config
        x_pps = params['mesh_x_pps']
        y_pps = params['mesh_y_pps']
        if params['algo'] not in self.ALGOS:
            raise error(
                "bed_mesh: Unknown algorithm <%s>"
                % (self.mesh_config['algo']))
        # Check the algorithm against the current configuration
        max_probe_cnt = max(params['x_count'], params['y_count'])
        min_probe_cnt = min(params['x_count'], params['y_count'])
        if max(x_pps, y_pps) == 0:
            # Interpolation disabled
            self.mesh_config['algo'] = 'direct'
        elif params['algo'] == 'lagrange' and max_probe_cnt > 6:
            # Lagrange interpolation tends to oscillate when using more
            # than 6 samples
            raise error(
                "bed_mesh: cannot exceed a probe_count of 6 when using "
                "lagrange interpolation. Configured Probe Count: %d, %d" %
                (self.mesh_config['x_count'], self.mesh_config['y_count']))
        elif params['algo'] == 'bicubic' and min_probe_cnt < 4:
            if max_probe_cnt > 6:
                raise error(
                    "bed_mesh: invalid probe_count option when using bicubic "
                    "interpolation.  Combination of 3 points on one axis with "
                    "more than 6 on another is not permitted. "
                    "Configured Probe Count: %d, %d" %
                    (self.mesh_config['x_count'], self.mesh_config['y_count']))
            else:
                logging.info(
                    "bed_mesh: bicubic interpolation with a probe_count of "
                    "less than 4 points detected.  Forcing lagrange "
                    "interpolation. Configured Probe Count: %d, %d" %
                    (self.mesh_config['x_count'], self.mesh_config['y_count']))
                params['algo'] = 'lagrange'
    def set_adaptive_mesh(self, gcmd):
        if not gcmd.get_int('ADAPTIVE', 0):
            return False
        exclude_objects = self.printer.lookup_object("exclude_object", None)
        if exclude_objects is None:
            gcmd.respond_info("Exclude objects not enabled. Using full mesh...")
            return False
        objects = exclude_objects.get_status().get("objects", [])
        if not objects:
            return False
        margin = gcmd.get_float('ADAPTIVE_MARGIN', self.adaptive_margin)

        # List all exclude_object points by axis and iterate over
        # all polygon points, and pick the min and max or each axis
        list_of_xs = []
        list_of_ys = []
        gcmd.respond_info("Found %s objects" % (len(objects)))
        for obj in objects:
            for point in obj["polygon"]:
                list_of_xs.append(point[0])
                list_of_ys.append(point[1])

        # Define bounds of adaptive mesh area
        mesh_min = [min(list_of_xs), min(list_of_ys)]
        mesh_max = [max(list_of_xs), max(list_of_ys)]
        adjusted_mesh_min = [x - margin for x in mesh_min]
        adjusted_mesh_max = [x + margin for x in mesh_max]

        # Force margin to respect original mesh bounds
        adjusted_mesh_min[0] = max(adjusted_mesh_min[0],
                                   self.orig_config["mesh_min"][0])
        adjusted_mesh_min[1] = max(adjusted_mesh_min[1],
                                   self.orig_config["mesh_min"][1])
        adjusted_mesh_max[0] = min(adjusted_mesh_max[0],
                                   self.orig_config["mesh_max"][0])
        adjusted_mesh_max[1] = min(adjusted_mesh_max[1],
                                   self.orig_config["mesh_max"][1])

        adjusted_mesh_size = (adjusted_mesh_max[0] - adjusted_mesh_min[0],
                              adjusted_mesh_max[1] - adjusted_mesh_min[1])

        # Compute a ratio between the adapted and original sizes
        ratio = (adjusted_mesh_size[0] /
                 (self.orig_config["mesh_max"][0] -
                  self.orig_config["mesh_min"][0]),
                 adjusted_mesh_size[1] /
                 (self.orig_config["mesh_max"][1] -
                  self.orig_config["mesh_min"][1]))

        gcmd.respond_info("Original mesh bounds: (%s,%s)" %
                          (self.orig_config["mesh_min"],
                           self.orig_config["mesh_max"]))
        gcmd.respond_info("Original probe count: (%s,%s)" %
                          (self.mesh_config["x_count"],
                           self.mesh_config["y_count"]))
        gcmd.respond_info("Adapted mesh bounds: (%s,%s)" %
                          (adjusted_mesh_min, adjusted_mesh_max))
        gcmd.respond_info("Ratio: (%s, %s)" % ratio)

        new_x_probe_count = int(
            math.ceil(self.mesh_config["x_count"] * ratio[0]))
        new_y_probe_count = int(
        math.ceil(self.mesh_config["y_count"] * ratio[1]))

        # There is one case, where we may have to adjust the probe counts:
        # axis0 < 4 and axis1 > 6 (see _verify_algorithm).
        min_num_of_probes = 3
        if max(new_x_probe_count, new_y_probe_count) > 6 and \
           min(new_x_probe_count, new_y_probe_count) < 4:
            min_num_of_probes = 4

        new_x_probe_count = max(min_num_of_probes, new_x_probe_count)
        new_y_probe_count = max(min_num_of_probes, new_y_probe_count)

        gcmd.respond_info("Adapted probe count: (%s,%s)" %
                          (new_x_probe_count, new_y_probe_count))

        # If the adapted mesh size is too small, adjust it to something
        # useful.
        adjusted_mesh_size = (max(adjusted_mesh_size[0], new_x_probe_count),
                              max(adjusted_mesh_size[1], new_y_probe_count))

        if self.radius is not None:
            adapted_radius = math.sqrt((adjusted_mesh_size[0] ** 2) +
                                       (adjusted_mesh_size[1] ** 2)) / 2
            adapted_origin = (adjusted_mesh_min[0] +
                              (adjusted_mesh_size[0] / 2),
                              adjusted_mesh_min[1] +
                              (adjusted_mesh_size[1] / 2))
            to_adapted_origin = math.sqrt(adapted_origin[0]**2 +
                                          adapted_origin[1]**2)
            # If the adapted mesh size is smaller than the default/full
            # mesh, adjust the parameters. Otherwise, just do the full mesh.
            if adapted_radius + to_adapted_origin < self.radius:
                self.radius = adapted_radius
                self.origin = adapted_origin
                self.mesh_min = (-self.radius, -self.radius)
                self.mesh_max = (self.radius, self.radius)
                new_probe_count = max(new_x_probe_count, new_y_probe_count)
                # Adaptive meshes require odd number of points
                new_probe_count += 1 - (new_probe_count % 2)
                self.mesh_config["x_count"] = self.mesh_config["y_count"] = \
                        new_probe_count
        else:
            self.mesh_min = adjusted_mesh_min
            self.mesh_max = adjusted_mesh_max
            self.mesh_config["x_count"] = new_x_probe_count
            self.mesh_config["y_count"] = new_y_probe_count
        self._profile_name = None
        return True
    def update_config(self, gcmd):
        # reset default configuration
        self.radius = self.orig_config['radius']
        self.origin = self.orig_config['origin']
        self.mesh_min = self.orig_config['mesh_min']
        self.mesh_max = self.orig_config['mesh_max']
        for key in list(self.mesh_config.keys()):
            self.mesh_config[key] = self.orig_config[key]

        params = gcmd.get_command_parameters()
        need_cfg_update = False
        if self.radius is not None:
            if "MESH_RADIUS" in params:
                self.radius = gcmd.get_float("MESH_RADIUS")
                self.radius = math.floor(self.radius * 10) / 10
                self.mesh_min = (-self.radius, -self.radius)
                self.mesh_max = (self.radius, self.radius)
                need_cfg_update = True
            if "MESH_ORIGIN" in params:
                self.origin = parse_gcmd_coord(gcmd, 'MESH_ORIGIN')
                need_cfg_update = True
            if "ROUND_PROBE_COUNT" in params:
                cnt = gcmd.get_int('ROUND_PROBE_COUNT', minval=3)
                self.mesh_config['x_count'] = cnt
                self.mesh_config['y_count'] = cnt
                need_cfg_update = True
        else:
            if "MESH_MIN" in params:
                self.mesh_min = parse_gcmd_coord(gcmd, 'MESH_MIN')
                need_cfg_update = True
            if "MESH_MAX" in params:
                self.mesh_max = parse_gcmd_coord(gcmd, 'MESH_MAX')
                need_cfg_update = True
            if "PROBE_COUNT" in params:
                x_cnt, y_cnt = parse_gcmd_pair(gcmd, 'PROBE_COUNT', minval=3)
                self.mesh_config['x_count'] = x_cnt
                self.mesh_config['y_count'] = y_cnt
                need_cfg_update = True

        if "MESH_PPS" in params:
            xpps, ypps = parse_gcmd_pair(gcmd, 'MESH_PPS', minval=0)
            self.mesh_config['mesh_x_pps'] = xpps
            self.mesh_config['mesh_y_pps'] = ypps
            need_cfg_update = True

        if "ALGORITHM" in params:
            self.mesh_config['algo'] = gcmd.get('ALGORITHM').strip().lower()
            need_cfg_update = True

        need_cfg_update |= self.set_adaptive_mesh(gcmd)
        probe_method = gcmd.get("METHOD", "automatic")

        if need_cfg_update:
            self._verify_algorithm(gcmd.error)
            self.probe_mgr.generate_points(
                self.mesh_config, self.mesh_min, self.mesh_max,
                self.radius, self.origin, probe_method
            )
            msg = "\n".join(["%s: %s" % (k, v)
                             for k, v in self.mesh_config.items()])
            logging.info("Updated Mesh Configuration:\n" + msg)
        else:
            self.probe_mgr.generate_points(
                self.mesh_config, self.mesh_min, self.mesh_max,
                self.radius, self.origin, probe_method
            )
    def dump_calibration(self, gcmd=None):
        if gcmd is not None and gcmd.get_command_parameters():
            self.update_config(gcmd)
        cfg = dict(self.mesh_config)
        cfg["mesh_min"] = self.mesh_min
        cfg["mesh_max"] = self.mesh_max
        cfg["origin"] = self.origin
        cfg["radius"] = self.radius
        return {
            "points": self.probe_mgr.get_base_points(),
            "config": cfg,
            "probe_path": self.probe_mgr.get_std_path(),
            "rapid_path": list(self.probe_mgr.iter_rapid_path())
        }
    cmd_BED_MESH_CALIBRATE_help = "Perform Mesh Bed Leveling"
    def cmd_BED_MESH_CALIBRATE(self, gcmd):
        self._profile_name = gcmd.get('PROFILE', "default")
        if not self._profile_name.strip():
            raise gcmd.error("Value for parameter 'PROFILE' must be specified")
        self.bedmesh.set_mesh(None)
        try:
            self.update_config(gcmd)
        except BedMeshError as e:
            raise gcmd.error(str(e))
        self.probe_mgr.start_probe(gcmd)

    def _run_fast_bed_mesh_recalibrate(self, gcmd, reason):
        logging.info(
            "bed_mesh: fast bed mesh fallback to recalibrate, reason=%s",
            reason)
        # 强制 PROFILE=runtime_active：既避免覆盖校准全床基准 default，
        # 也不再引入只写不读的中间名（原 model_leveling）。
        # probe_finalize 的自动 save_profile 只写 pending，
        # 由结尾 save_runtime_mesh_snapshot 统一 SAVE_CONFIG 落盘一次；
        # 落盘前先抬升并 M400，避免与 Z 运动重叠导致 Timer too close。
        params = dict(gcmd.get_command_parameters())
        params['PROFILE'] = RUNTIME_ACTIVE_PROFILE
        param_str = ' '.join('%s=%s' % (k.upper(), v) for k, v in params.items())
        self.gcode.run_script_from_command(
            "BED_MESH_CLEAR\n"
            f"BED_MESH_CALIBRATE {param_str}\n"
            "G1 Z10 F600\n"
            "M400\n"
        )
        self.bedmesh.save_runtime_mesh_snapshot()

    def cmd_BED_MESH_CALIBRATE_FAST(self, gcmd):
        # 1. 卸载mesh（参考cmd_BED_MESH_CALIBRATE方法）
        self._profile_name = gcmd.get('PROFILE', "default")
        if not self._profile_name.strip():
            raise gcmd.error("Value for parameter 'PROFILE' must be specified")
        active_mesh = self.bedmesh.get_mesh()
        self.bedmesh.set_mesh(None)
        try:
            self.update_config(gcmd)
        except BedMeshError as e:
            raise gcmd.error(str(e))

        mesh_source = "profile"
        if active_mesh is not None:
            z_mesh = active_mesh
            mesh_source = "active"
        else:
            z_mesh = self._load_profile_mesh(self._profile_name)
        temp_cpst = self._get_temp_comp()
        if not temp_cpst.load_model():
            self.gcode.respond_info(
                "[FAST_BED_MESH] temp model missing: "
                "action=recalibrate_mesh")
            self.temp_cpst = None
            self._run_fast_bed_mesh_recalibrate(gcmd, "temp_model_missing")
            return

        # 2. 路径规划（旅行商问题）
        toolhead = self.printer.lookup_object('toolhead')
        pt_now = toolhead.get_position()
        edge = gcmd.get("PRINT_MIN", "140.0,140.0")
        pt_min = [float(x.strip()) for x in edge.split(',')]
        edge = gcmd.get("PRINT_MAX", "160.0,160.0")
        pt_max = [float(x.strip()) for x in edge.split(',')]
        pts = [[pt_now[0], pt_now[1]], # 当前位置
               [0.5*(pt_min[0]+pt_max[0]), 0.5*(pt_min[1]+pt_max[1])], # 中心点
               [pt_min[0], pt_min[1]], # xmin, ymin
               [pt_min[0], pt_max[1]], # xmin, ymax
               [pt_max[0], pt_max[1]], # xmax, ymax
               [pt_max[0], pt_min[1]]] # xmax, ymin
        pts = calc_open_tsp(pts) # 旅行商路线规划
        pts = pts[1:] # 去除当前点

        # 3. 获取偏移（参考_move_next方法）
        pprobe = self.printer.lookup_object('probe', None) # 获取探针对象
        if pprobe is None:
            raise gcmd.error("bed_mesh: probe not configured")
        poffset = (0., 0., 0.) # 探针偏移初值
        if self.probe_mgr.probe_helper.use_offsets:
            poffset = pprobe.get_offsets() # 探针偏移更新
        ref_mesh_params = z_mesh.get_mesh_params()
        ref_mesh_min = (
            float(ref_mesh_params['min_x']),
            float(ref_mesh_params['min_y']))
        ref_mesh_max = (
            float(ref_mesh_params['max_x']),
            float(ref_mesh_params['max_y']))
        try:
            raw_pts = [pt[:] for pt in pts]
            pts, safe_min, safe_max = self._clamp_fast_bed_mesh_points(
                pts, ref_mesh_min, ref_mesh_max, poffset)
        except BedMeshError as e:
            raise gcmd.error(str(e))
        self.gcode.respond_info(
            "[FAST_BED_MESH] start: profile=%s print_min=(%.2f, %.2f) "
            "print_max=(%.2f, %.2f) toolhead=(%.2f, %.2f, %.2f) "
            "probe_offset=(%.4f, %.4f, %.4f) ref_mesh_source=%s "
            "ref_mesh_min=(%.2f, %.2f) ref_mesh_max=(%.2f, %.2f) "
            "safe_min=(%.2f, %.2f) safe_max=(%.2f, %.2f) "
            "raw_trig_pts=%s trig_pts=%s"
            % (self._profile_name, pt_min[0], pt_min[1], pt_max[0], pt_max[1],
               pt_now[0], pt_now[1], pt_now[2], poffset[0], poffset[1],
               poffset[2], mesh_source,
               ref_mesh_min[0], ref_mesh_min[1],
               ref_mesh_max[0], ref_mesh_max[1],
               safe_min[0], safe_min[1], safe_max[0], safe_max[1],
               raw_pts, pts))

        # 4. 依次探点（参考cmd_PROBE方法）
        heater_bed = self.printer.lookup_object("heater_bed", None)
        trig_pts = [[pt, None, 70.0] for pt in pts]
        for i, (pt, _, _) in enumerate(trig_pts):
            pt[0] -= poffset[0] # 消除x轴探针偏移
            pt[1] -= poffset[1] # 消除y轴探针偏移
            # TODO: pt未包含z坐标，有待观察是否存在z运动干涉
            toolhead.manual_move(pt, 100) # 移动到目标坐标
            toolhead.wait_moves() # 等待移动完成
            trig_pt = probe.run_single_probe(pprobe, gcmd) # 触发探针测量
            temp_trig = 70.0 # 当前点采样时的热床温度（基准值为70℃）
            if heater_bed is not None:
                temp_trig = heater_bed.get_status(0)['temperature']
            trig_pts[i][1] = trig_pt[2] # 保存触发时刻的目标点z坐标
            trig_pts[i][2] = temp_trig # 保存触发时刻的热床温度

        # 5. 加载mesh（参考load_profile方法）
        # 6. 计算偏差
        self.gcode.respond_info(
            "[FAST_BED_MESH] profile loaded: source=%s profile=%s "
            "mesh_params=%s"
            % (mesh_source, z_mesh.get_profile_name(), z_mesh.get_mesh_params()))

        z_deltas = []
        for pt, z_trig, temp_trig in trig_pts:
            x, y = float(pt[0]), float(pt[1])
            point_delta = calc_fast_bed_mesh_point_delta(
                z_mesh=z_mesh,
                temp_cpst=temp_cpst,
                mesh_source=mesh_source,
                x=x, y=y,
                temp_trig=temp_trig,
                z_trig=z_trig,
                probe_z_offset=poffset[2],
            )
            z_trig_normalized = point_delta['z_trig_normalized']
            z_default = point_delta['z_default']
            z_comp = point_delta['z_comp']
            z_predict = point_delta['z_predict']
            z_delta = point_delta['z_delta']
            self.gcode.respond_info(
                "[FAST_BED_MESH] point: x=%.2f y=%.2f temp_trig=%.2f "
                "z_trig=%.6f z_trig_norm=%.6f z_default=%.6f z_comp=%.6f "
                "z_predict=%.6f z_delta=%.6f"
                % (x, y, temp_trig, z_trig, z_trig_normalized,
                   z_default, z_comp,
                   z_predict, z_delta))
            z_deltas.append(z_delta)
        self.temp_cpst = None # 使用完毕及时释放内存

        # 7. 分类处理
        # 单点探测误差约 ±0.03mm，因此快速调平不能用 0.02mm 这么严
        Z_MAX_ABS_THRESHOLD = 0.06   # 单点最大绝对偏差阈值
        Z_RANGE_THRESHOLD = 0.06     # 多点偏差极差阈值，用于判断床面形状是否变化

        strategy = calc_fast_bed_mesh_strategy(
            z_deltas,
            max_abs_threshold=Z_MAX_ABS_THRESHOLD,
            range_threshold=Z_RANGE_THRESHOLD,
        )

        if strategy['action'] == "load_profile_mesh":
            self.gcode.respond_info(
                "[FAST_BED_MESH] accepted: "
                "max_abs_threshold=%.6f range_threshold=%.6f "
                "z_delta_max=%.6f z_delta_min=%.6f "
                "z_delta_range=%.6f max_abs_delta=%.6f "
                "z_shift=%.6f residual_max_abs=%.6f residual_range=%.6f "
                "action=load_profile_mesh"
                % (
                    Z_MAX_ABS_THRESHOLD, Z_RANGE_THRESHOLD,
                    strategy['raw_max'], strategy['raw_min'],
                    strategy['raw_range'], strategy['raw_max_abs'],
                    strategy['z_shift'], strategy['residual_max_abs'],
                    strategy['residual_range']
                )
            )
            self.bedmesh.set_mesh(z_mesh)
            self.bedmesh.save_runtime_mesh_snapshot(z_mesh)
        elif strategy['action'] == "apply_mesh_z_shift":
            z_mesh.set_mesh_z_shift(strategy['z_shift'])
            self.gcode.respond_info(
                "[FAST_BED_MESH] accepted: "
                "max_abs_threshold=%.6f range_threshold=%.6f "
                "z_delta_max=%.6f z_delta_min=%.6f "
                "z_delta_range=%.6f max_abs_delta=%.6f "
                "z_shift=%.6f residual_max_abs=%.6f residual_range=%.6f "
                "action=apply_mesh_z_shift"
                % (
                    Z_MAX_ABS_THRESHOLD, Z_RANGE_THRESHOLD,
                    strategy['raw_max'], strategy['raw_min'],
                    strategy['raw_range'], strategy['raw_max_abs'],
                    strategy['z_shift'], strategy['residual_max_abs'],
                    strategy['residual_range']
                )
            )
            self.bedmesh.set_mesh(z_mesh)
            self.bedmesh.save_runtime_mesh_snapshot(z_mesh)
        else:
            self.gcode.respond_info(
                "[FAST_BED_MESH] rejected: "
                "max_abs_threshold=%.6f range_threshold=%.6f "
                "z_delta_max=%.6f z_delta_min=%.6f "
                "z_delta_range=%.6f max_abs_delta=%.6f "
                "z_shift=%.6f residual_max_abs=%.6f residual_range=%.6f "
                "action=recalibrate_mesh"
                % (
                    Z_MAX_ABS_THRESHOLD, Z_RANGE_THRESHOLD,
                    strategy['raw_max'], strategy['raw_min'],
                    strategy['raw_range'], strategy['raw_max_abs'],
                    strategy['z_shift'], strategy['residual_max_abs'],
                    strategy['residual_range']
                )
            )
            self._run_fast_bed_mesh_recalibrate(gcmd, "strategy_rejected")
    def cmd_BED_MESH_TEMP_LOAD(self, gcmd):
        target_temp = gcmd.get_float('TEMP')
        profile_name = gcmd.get('PROFILE', "default")
        predicted_loaded = self.load_temperature_mesh(
            target_temp, profile_name=profile_name, save_snapshot=True)
        if predicted_loaded:
            self.gcode.respond_info(
                "[BED_MESH_TEMP_LOAD] predicted mesh loaded: profile=%s temp=%.2f"
                % (profile_name, target_temp))
        else:
            self.gcode.respond_info(
                "[BED_MESH_TEMP_LOAD] predicted mesh unavailable, fallback profile=%s temp=%.2f"
                % (profile_name, target_temp))
    def cmd_BED_MESH_TEMP(self, gcmd):
        flag = gcmd.get_int('P', 0)
        try:
            if flag == 0:
                temps = self._parse_temp_list(gcmd)
                # Force recreate so a leftover adaptive mesh_config cannot stick.
                self.temp_cpst = None
                temp_cpst = self._get_temp_comp()
                temp_cpst.reset(val_t=temps)
                self._start_temp_sampling_session(temps)
                self._start_temp_csv_session(temps)
            elif flag == 1:
                temp_now = None
                heater_bed = self.printer.lookup_object("heater_bed", None)
                if heater_bed is not None: # ["target"]期望温度, ["temperature"]实测温度
                    temp_now = heater_bed.get_status(0)["temperature"]

                z_mesh = self.bedmesh.get_mesh()
                if z_mesh is None:
                    logging.info("bed_mesh: temp sample skipped, no active mesh")
                    self._append_temp_csv_sample([], temp_now)
                    return
                probed_matrix = z_mesh.get_probed_matrix()
                self._get_temp_comp().receive(probed_matrix, temp_now)
                self._append_temp_csv_sample(probed_matrix, temp_now)
            elif flag == 2:
                # 拟合并原子落盘；成功后才 commit stash→default；同步等待完成再返回
                reactor = self.printer.get_reactor()
                completion = reactor.completion()
                result = {"error": None}
                temp_cpst = self._get_temp_comp()
                sample_shapes = []
                for sample in temp_cpst.val_z:
                    try:
                        rows = len(sample)
                        cols = len(sample[0]) if rows else 0
                        sample_shapes.append("%dx%d" % (rows, cols))
                    except Exception:
                        sample_shapes.append("unavailable")
                sample_count = len(temp_cpst.val_z)
                expected_sample_count = temp_cpst.n_t
                sample_temps = [float(temp) for temp in temp_cpst.val_t]
                logging.info(
                    "[BED_MESH_TEMP] fitting temperature model: "
                    "samples=%d/%d shapes=%s temperatures=%s",
                    sample_count, expected_sample_count, sample_shapes,
                    sample_temps)

                def on_model_saved():
                    try:
                        if not getattr(temp_cpst, "last_save_ok", False):
                            result["error"] = "temperature model save failed"
                            return
                        if not self._commit_temp_sampling_baseline():
                            result["error"] = (
                                "failed to commit stashed baseline to default")
                            return
                        self.gcode.respond_info(
                            "[BED_MESH_TEMP] temperature model saved and "
                            "baseline profile committed")
                    except Exception as e:
                        logging.exception(
                            "BED_MESH_TEMP P=2 post-save finalize failed")
                        result["error"] = str(e)
                    finally:
                        self.temp_cpst = None
                        self._reset_temp_sampling_session()
                        self._reset_temp_csv_session()
                        completion.complete(True)

                temp_cpst.fit(reactor, on_model_saved)
                completion.wait()
                if result["error"]:
                    raise gcmd.error(
                        "BED_MESH_TEMP P=2 failed: %s" % (result["error"],))
            elif flag == 3:
                # 取消/幂等清理：仅影响未提交 stash，不改已落盘的 default / npz
                abort_result = self.abort_temp_sampling_session()
                if abort_result == "discarded":
                    self.gcode.respond_info(
                        "[BED_MESH_TEMP] unfinished sampling cancelled; "
                        "uncommitted baseline discarded "
                        "(saved default / temp_comp.npz unchanged)")
                elif abort_result == "cleared":
                    self.gcode.respond_info(
                        "[BED_MESH_TEMP] sampling session cleared "
                        "(no uncommitted baseline)")
                else:
                    # idle：FINISH 正常收尾常见路径，避免误导成“校准被中止”
                    logging.info(
                        "[BED_MESH_TEMP] P=3 no-op: no active sampling session")
            else:
                raise gcmd.error("BED_MESH_TEMP unknown P=%s" % (flag,))
        except aniso_temp_compensation.error as e:
            raise gcmd.error(str(e))
        except self.printer.command_error:
            raise
        except Exception as e:
            logging.exception("BED_MESH_TEMP P=%s failed", flag)
            raise gcmd.error("BED_MESH_TEMP P=%s failed: %s" % (flag, e))

    def probe_finalize(self, offsets, positions):
        z_offset = offsets[2]
        positions = [[round(p[0], 2), round(p[1], 2), p[2]]
                     for p in positions]
        if self.probe_mgr.get_zero_ref_mode() == ZrefMode.PROBE:
            ref_pos = positions.pop()
            logging.info(
                "bed_mesh: z-offset replaced with probed z value at "
                "position (%.2f, %.2f, %.6f)"
                % (ref_pos[0], ref_pos[1], ref_pos[2])
            )
            z_offset = ref_pos[2]
        base_points = self.probe_mgr.get_base_points()
        params = dict(self.mesh_config)
        params['min_x'] = min(base_points, key=lambda p: p[0])[0]
        params['max_x'] = max(base_points, key=lambda p: p[0])[0]
        params['min_y'] = min(base_points, key=lambda p: p[1])[1]
        params['max_y'] = max(base_points, key=lambda p: p[1])[1]
        x_cnt = params['x_count']
        y_cnt = params['y_count']

        substitutes = self.probe_mgr.get_substitutes()
        probed_pts = positions
        if substitutes:
            # Replace substituted points with the original generated
            # point.  Its Z Value is the average probed Z of the
            # substituted points.
            corrected_pts = []
            idx_offset = 0
            start_idx = 0
            for i, pts in substitutes.items():
                fpt = [p - o for p, o in zip(base_points[i], offsets[:2])]
                # offset the index to account for additional samples
                idx = i + idx_offset
                # Add "normal" points
                corrected_pts.extend(positions[start_idx:idx])
                avg_z = sum([p[2] for p in positions[idx:idx+len(pts)]]) \
                    / len(pts)
                idx_offset += len(pts) - 1
                start_idx = idx + len(pts)
                fpt.append(avg_z)
                logging.info(
                    "bed_mesh: Replacing value at faulty index %d"
                    " (%.4f, %.4f): avg value = %.6f, avg w/ z_offset = %.6f"
                    % (i, fpt[0], fpt[1], avg_z, avg_z - z_offset))
                corrected_pts.append(fpt)
            corrected_pts.extend(positions[start_idx:])
            positions = corrected_pts

        # validate length of result
        if len(base_points) != len(positions):
            self._dump_points(probed_pts, positions, offsets)
            raise self.gcode.error(
                "bed_mesh: invalid position list size, "
                "generated count: %d, probed count: %d"
                % (len(base_points), len(positions))
            )

        probed_matrix = []
        row = []
        prev_pos = base_points[0]
        for pos, result in zip(base_points, positions):
            offset_pos = [p - o for p, o in zip(pos, offsets[:2])]
            if (
                not isclose(offset_pos[0], result[0], abs_tol=.5) or
                not isclose(offset_pos[1], result[1], abs_tol=.5)
            ):
                logging.info(
                    "bed_mesh: point deviation > .5mm: orig pt = (%.2f, %.2f)"
                    ", probed pt = (%.2f, %.2f)"
                    % (offset_pos[0], offset_pos[1], result[0], result[1])
                )
            z_pos = result[2] - z_offset
            if not isclose(pos[1], prev_pos[1], abs_tol=.1):
                # y has changed, append row and start new
                probed_matrix.append(row)
                row = []
            if pos[0] > prev_pos[0]:
                # probed in the positive direction
                row.append(z_pos)
            else:
                # probed in the negative direction
                row.insert(0, z_pos)
            prev_pos = pos
        # append last row
        probed_matrix.append(row)

        # make sure the y-axis is the correct length
        if len(probed_matrix) != y_cnt:
            raise self.gcode.error(
                ("bed_mesh: Invalid y-axis table length\n"
                 "Probed table length: %d Probed Table:\n%s") %
                (len(probed_matrix), str(probed_matrix)))

        if self.radius is not None:
            # round bed, extrapolate probed values to create a square mesh
            for row in probed_matrix:
                row_size = len(row)
                if not row_size & 1:
                    # an even number of points in a row shouldn't be possible
                    msg = "bed_mesh: incorrect number of points sampled on X\n"
                    msg += "Probed Table:\n"
                    msg += str(probed_matrix)
                    raise self.gcode.error(msg)
                buf_cnt = (x_cnt - row_size) // 2
                if buf_cnt == 0:
                    continue
                left_buffer = [row[0]] * buf_cnt
                right_buffer = [row[row_size-1]] * buf_cnt
                row[0:0] = left_buffer
                row.extend(right_buffer)

        #  make sure that the x-axis is the correct length
        for row in probed_matrix:
            if len(row) != x_cnt:
                raise self.gcode.error(
                    ("bed_mesh: invalid x-axis table length\n"
                        "Probed table length: %d Probed Table:\n%s") %
                    (len(probed_matrix), str(probed_matrix)))

        z_mesh = ZMesh(params, self._profile_name)
        try:
            z_mesh.build_mesh(probed_matrix)
        except BedMeshError as e:
            raise self.gcode.error(str(e))
        if self.probe_mgr.get_zero_ref_mode() == ZrefMode.IN_MESH:
            # The reference can be anywhere in the mesh, therefore
            # it is necessary to set the reference after the initial mesh
            # is generated to lookup the correct z value.
            zero_ref_pos = self.probe_mgr.get_zero_ref_pos()
            z_mesh.set_zero_reference(*zero_ref_pos)
        self.bedmesh.set_mesh(z_mesh)
        self.gcode.respond_info("Mesh Bed Leveling Complete")
        # 温补采样：首轮只 stash 基准，不写正式 default；普通调平仍立即 save_profile
        if self._temp_sampling_session is not None:
            if not self._temp_sampling_session.get("baseline_stashed", False):
                self._stash_temp_sampling_baseline(
                    z_mesh, self._profile_name)
            else:
                logging.info(
                    "bed_mesh: skip baseline stash during temperature "
                    "sampling baseline_stashed=%s csv_stage=%s",
                    self._temp_sampling_session.get("baseline_stashed", None),
                    self._temp_csv_session.get("stage_index", None)
                    if self._temp_csv_session is not None else None)
        elif should_auto_save_calibrated_profile(
                self._temp_sampling_session, self._profile_name):
            self.bedmesh.save_profile(self._profile_name)
        elif self._profile_name is not None:
            logging.info(
                "bed_mesh: skip auto-save profile [%s]",
                self._profile_name)
    def _dump_points(self, probed_pts, corrected_pts, offsets):
        # logs generated points with offset applied, points received
        # from the finalize callback, and the list of corrected points
        points = self.probe_mgr.get_base_points()
        max_len = max([len(points), len(probed_pts), len(corrected_pts)])
        logging.info(
            "bed_mesh: calibration point dump\nIndex | %-17s| %-25s|"
            " Corrected Point" % ("Generated Point", "Probed Point"))
        for i in list(range(max_len)):
            gen_pt = probed_pt = corr_pt = ""
            if i < len(points):
                off_pt = [p - o for p, o in zip(points[i], offsets[:2])]
                gen_pt = "(%.2f, %.2f)" % tuple(off_pt)
            if i < len(probed_pts):
                probed_pt = "(%.2f, %.2f, %.4f)" % tuple(probed_pts[i])
            if i < len(corrected_pts):
                corr_pt = "(%.2f, %.2f, %.4f)" % tuple(corrected_pts[i])
            logging.info(
                "  %-4d| %-17s| %-25s| %s" % (i, gen_pt, probed_pt, corr_pt))

class ProbeManager:
    def __init__(self, config, orig_config, finalize_cb):
        self.printer = config.get_printer()
        self.cfg_overshoot = config.getfloat("scan_overshoot", 0, minval=1.)
        self.orig_config = orig_config
        self.faulty_regions = []
        self.overshoot = self.cfg_overshoot
        self.zero_ref_pos = config.getfloatlist(
            "zero_reference_position", None, count=2
        )
        self.zref_mode = ZrefMode.DISABLED
        self.base_points = []
        self.substitutes = collections.OrderedDict()
        self.is_round = orig_config["radius"] is not None
        self.probe_helper = probe.ProbePointsHelper(config, finalize_cb, [])
        self.probe_helper.use_xy_offsets(True)
        self.rapid_scan_helper = RapidScanHelper(config, self, finalize_cb)
        self._init_faulty_regions(config)

    def _init_faulty_regions(self, config):
        for i in list(range(1, 100, 1)):
            start = config.getfloatlist("faulty_region_%d_min" % (i,), None,
                                        count=2)
            if start is None:
                break
            end = config.getfloatlist("faulty_region_%d_max" % (i,), count=2)
            # Validate the corners.  If necessary reorganize them.
            # c1 = min point, c3 = max point
            #  c4 ---- c3
            #  |        |
            #  c1 ---- c2
            c1 = [min([s, e]) for s, e in zip(start, end)]
            c3 = [max([s, e]) for s, e in zip(start, end)]
            c2 = [c1[0], c3[1]]
            c4 = [c3[0], c1[1]]
            # Check for overlapping regions
            for j, (prev_c1, prev_c3) in enumerate(self.faulty_regions):
                prev_c2 = [prev_c1[0], prev_c3[1]]
                prev_c4 = [prev_c3[0], prev_c1[1]]
                # Validate that no existing corner is within the new region
                for coord in [prev_c1, prev_c2, prev_c3, prev_c4]:
                    if within(coord, c1, c3):
                        raise config.error(
                            "bed_mesh: Existing faulty_region_%d %s overlaps "
                            "added faulty_region_%d %s"
                            % (j+1, repr([prev_c1, prev_c3]),
                               i, repr([c1, c3])))
                # Validate that no new corner is within an existing region
                for coord in [c1, c2, c3, c4]:
                    if within(coord, prev_c1, prev_c3):
                        raise config.error(
                            "bed_mesh: Added faulty_region_%d %s overlaps "
                            "existing faulty_region_%d %s"
                            % (i, repr([c1, c3]),
                               j+1, repr([prev_c1, prev_c3])))
            self.faulty_regions.append((c1, c3))

    def start_probe(self, gcmd):
        method = gcmd.get("METHOD", "automatic").lower()
        can_scan = False
        pprobe = self.printer.lookup_object("probe", None)
        if pprobe is not None:
            probe_name = pprobe.get_status(None).get("name", "")
            can_scan = probe_name.startswith("probe_eddy_current")
        if method == "rapid_scan" and can_scan:
            self.rapid_scan_helper.perform_rapid_scan(gcmd)
        else:
            self.probe_helper.start_probe(gcmd)

    def get_zero_ref_pos(self):
        return self.zero_ref_pos

    def get_zero_ref_mode(self):
        return self.zref_mode

    def get_substitutes(self):
        return self.substitutes

    def generate_points(
        self, mesh_config, mesh_min, mesh_max, radius, origin,
        probe_method="automatic"
    ):
        x_cnt = mesh_config['x_count']
        y_cnt = mesh_config['y_count']
        min_x, min_y = mesh_min
        max_x, max_y = mesh_max
        x_dist = (max_x - min_x) / (x_cnt - 1)
        y_dist = (max_y - min_y) / (y_cnt - 1)
        # floor distances down to next hundredth
        x_dist = math.floor(x_dist * 100) / 100
        y_dist = math.floor(y_dist * 100) / 100
        if x_dist < 1. or y_dist < 1.:
            raise BedMeshError("bed_mesh: min/max points too close together")

        if radius is not None:
            # round bed, min/max needs to be recalculated
            y_dist = x_dist
            new_r = (x_cnt // 2) * x_dist
            min_x = min_y = -new_r
            max_x = max_y = new_r
        else:
            # rectangular bed, only re-calc max_x
            max_x = min_x + x_dist * (x_cnt - 1)
        pos_y = min_y
        points = []
        for i in range(y_cnt):
            for j in range(x_cnt):
                if not i % 2:
                    # move in positive direction
                    pos_x = min_x + j * x_dist
                else:
                    # move in negative direction
                    pos_x = max_x - j * x_dist
                if radius is None:
                    # rectangular bed, append
                    points.append((pos_x, pos_y))
                else:
                    # round bed, check distance from origin
                    dist_from_origin = math.sqrt(pos_x*pos_x + pos_y*pos_y)
                    if dist_from_origin <= radius:
                        points.append(
                            (origin[0] + pos_x, origin[1] + pos_y))
            pos_y += y_dist
        if self.zero_ref_pos is None or probe_method == "manual":
            # Zero Reference Disabled
            self.zref_mode = ZrefMode.DISABLED
        elif within(self.zero_ref_pos, mesh_min, mesh_max):
            # Zero Reference position within mesh
            self.zref_mode = ZrefMode.IN_MESH
        else:
            # Zero Reference position outside of mesh
            self.zref_mode = ZrefMode.PROBE
        self.base_points = points
        self.substitutes.clear()
        # adjust overshoot
        og_min_x = self.orig_config["mesh_min"][0]
        og_max_x = self.orig_config["mesh_max"][0]
        add_ovs = min(max(0, min_x - og_min_x), max(0, og_max_x - max_x))
        self.overshoot = self.cfg_overshoot + math.floor(add_ovs)
        min_pt, max_pt = (min_x, min_y), (max_x, max_y)
        self._process_faulty_regions(min_pt, max_pt, radius)
        self.probe_helper.update_probe_points(self.get_std_path(), 3)

    def _process_faulty_regions(self, min_pt, max_pt, radius):
        if not self.faulty_regions:
            return
        # Cannot probe a reference within a faulty region
        if self.zref_mode == ZrefMode.PROBE:
            for min_c, max_c in self.faulty_regions:
                if within(self.zero_ref_pos, min_c, max_c):
                    opt = "zero_reference_position"
                    raise BedMeshError(
                        "bed_mesh: Cannot probe zero reference position at "
                        "(%.2f, %.2f) as it is located within a faulty region."
                        " Check the value for option '%s'"
                        % (self.zero_ref_pos[0], self.zero_ref_pos[1], opt,)
                    )
        # Check to see if any points fall within faulty regions
        last_y = self.base_points[0][1]
        is_reversed = False
        for i, coord in enumerate(self.base_points):
            if not isclose(coord[1], last_y):
                is_reversed = not is_reversed
            last_y = coord[1]
            adj_coords = []
            for min_c, max_c in self.faulty_regions:
                if within(coord, min_c, max_c, tol=.00001):
                    # Point lies within a faulty region
                    adj_coords = [
                        (min_c[0], coord[1]), (coord[0], min_c[1]),
                        (coord[0], max_c[1]), (max_c[0], coord[1])]
                    if is_reversed:
                        # Swap first and last points for zig-zag pattern
                        first = adj_coords[0]
                        adj_coords[0] = adj_coords[-1]
                        adj_coords[-1] = first
                    break
            if not adj_coords:
                # coord is not located within a faulty region
                continue
            valid_coords = []
            for ac in adj_coords:
                # make sure that coordinates are within the mesh boundary
                if radius is None:
                    if within(ac, min_pt, max_pt, .000001):
                        valid_coords.append(ac)
                else:
                    dist_from_origin = math.sqrt(ac[0]*ac[0] + ac[1]*ac[1])
                    if dist_from_origin <= radius:
                        valid_coords.append(ac)
            if not valid_coords:
                raise BedMeshError(
                    "bed_mesh: Unable to generate coordinates"
                    " for faulty region at index: %d" % (i)
                )
            self.substitutes[i] = valid_coords

    def get_base_points(self):
        return self.base_points

    def get_std_path(self):
        path = []
        for idx, pt in enumerate(self.base_points):
            if idx in self.substitutes:
                for sub_pt in self.substitutes[idx]:
                    path.append(sub_pt)
            else:
                path.append(pt)
        if self.zref_mode == ZrefMode.PROBE:
            path.append(self.zero_ref_pos)
        return path

    def iter_rapid_path(self):
        ascnd_x = True
        last_base_pt = last_mv_pt = self.base_points[0]
        # Generate initial move point
        if self.overshoot:
            overshoot = min(8, self.overshoot)
            last_mv_pt = (last_base_pt[0] - overshoot, last_base_pt[1])
            yield last_mv_pt, False
        for idx, pt in enumerate(self.base_points):
            # increasing Y indicates direction change
            dir_change = not isclose(pt[1], last_base_pt[1], abs_tol=1e-6)
            if idx in self.substitutes:
                fp_gen = self._gen_faulty_path(
                    last_mv_pt, idx, ascnd_x, dir_change
                )
                for sub_pt, is_smp in fp_gen:
                    yield sub_pt, is_smp
                    last_mv_pt = sub_pt
            else:
                if dir_change:
                    for dpt in self._gen_dir_change(last_mv_pt, pt, ascnd_x):
                        yield dpt, False
                yield pt, True
                last_mv_pt = pt
            last_base_pt = pt
            ascnd_x ^= dir_change
        if self.zref_mode == ZrefMode.PROBE:
            if self.overshoot:
                ovs = min(4, self.overshoot)
                ovs = ovs if ascnd_x else -ovs
                yield (last_mv_pt[0] + ovs, last_mv_pt[1]), False
            yield self.zero_ref_pos, True

    def _gen_faulty_path(self, last_pt, idx, ascnd_x, dir_change):
        subs = self.substitutes[idx]
        sub_cnt = len(subs)
        if dir_change:
            for dpt in self._gen_dir_change(last_pt, subs[0], ascnd_x):
                yield dpt, False
        if self.is_round:
            # No faulty region path handling for round beds
            for pt in subs:
                yield pt, True
            return
        # Check to see if this is the first corner
        first_corner = False
        sorted_sub_idx = sorted(self.substitutes.keys())
        if sub_cnt == 2 and idx < len(sorted_sub_idx):
            first_corner = sorted_sub_idx[idx] == idx
        yield subs[0], True
        if sub_cnt == 1:
            return
        last_pt, next_pt = subs[:2]
        if sub_cnt == 2:
            if first_corner or dir_change:
                # horizontal move first
                yield (next_pt[0], last_pt[1]), False
            else:
                yield (last_pt[0], next_pt[1]), False
            yield next_pt, True
        elif sub_cnt >= 3:
            if dir_change:
                # first move should be a vertical switch up.  If overshoot
                # is available, simulate another direction change.  Otherwise
                # move inward 2 mm, then up through the faulty region.
                if self.overshoot:
                    for dpt in self._gen_dir_change(last_pt, next_pt, ascnd_x):
                        yield dpt, False
                else:
                    shift = -2 if ascnd_x else 2
                    yield (last_pt[0] + shift, last_pt[1]), False
                    yield (last_pt[0] + shift, next_pt[1]), False
                yield next_pt, True
                last_pt, next_pt = subs[1:3]
            else:
                # vertical move
                yield (last_pt[0], next_pt[1]), False
                yield next_pt, True
                last_pt, next_pt = subs[1:3]
                if sub_cnt == 4:
                    # Vertical switch up within faulty region
                    shift = 2 if ascnd_x else -2
                    yield (last_pt[0] + shift, last_pt[1]), False
                    yield (next_pt[0] - shift, next_pt[1]), False
                    yield next_pt, True
                    last_pt, next_pt = subs[2:4]
            # horizontal move before final point
            yield (next_pt[0], last_pt[1]), False
            yield next_pt, True

    def _gen_dir_change(self, last_pt, next_pt, ascnd_x):
        if not self.overshoot:
            return
        # overshoot X beyond the outer point
        xdir = 1 if ascnd_x else -1
        overshoot = 2. if self.overshoot >= 3. else self.overshoot
        ovr_pt = (last_pt[0] + overshoot * xdir, last_pt[1])
        yield ovr_pt
        if self.overshoot < 3.:
            # No room to generate an arc, move up to next y
            yield (next_pt[0] + overshoot * xdir, next_pt[1])
        else:
            # generate arc
            STEP_ANGLE = 3
            START_ANGLE = 270
            ydiff = abs(next_pt[1] - last_pt[1])
            xdiff = abs(next_pt[0] - last_pt[0])
            max_radius = min(self.overshoot - 2, 8)
            radius = min(ydiff / 2, max_radius)
            origin = [ovr_pt[0], last_pt[1] + radius]
            next_origin_y = next_pt[1] - radius
            # determine angle
            if xdiff < .01:
                # Move is aligned on the x-axis
                angle = 90
                if next_origin_y - origin[1] < .05:
                    # The move can be completed in a single arc
                    angle = 180
            else:
                angle = int(math.degrees(math.atan(ydiff / xdiff)))
                if (
                    (ascnd_x and next_pt[0] < last_pt[0]) or
                    (not ascnd_x and next_pt[0] > last_pt[0])
                ):
                    angle = 180 - angle
            count = int(angle // STEP_ANGLE)
            # Gen first arc
            step = STEP_ANGLE * xdir
            start = START_ANGLE + step
            for arc_pt in self._gen_arc(origin, radius, start, step, count):
                yield arc_pt
            if angle == 180:
                # arc complete
                return
            # generate next arc
            origin = [next_pt[0] + overshoot * xdir, next_origin_y]
            # start at the angle where the last arc finished
            start = START_ANGLE + count * step
            # recalculate the count to make sure we generate a full 180
            # degrees.  Add a step for the repeated connecting angle
            count = 61 - count
            for arc_pt in self._gen_arc(origin, radius, start, step, count):
                yield arc_pt

    def _gen_arc(self, origin, radius, start, step, count):
        end = start + step * count
        # create a segent for every 3 degrees of travel
        for angle in range(start, end, step):
            rad = math.radians(angle % 360)
            opp = math.sin(rad) * radius
            adj = math.cos(rad) * radius
            yield (origin[0] + adj, origin[1] + opp)


MAX_HIT_DIST = 2.
MM_WIN_SPEED = 125

class RapidScanHelper:
    def __init__(self, config, probe_mgr, finalize_cb):
        self.printer = config.get_printer()
        self.probe_manager = probe_mgr
        self.speed = config.getfloat("speed", 50., above=0.)
        self.scan_height = config.getfloat("horizontal_move_z", 5.)
        self.finalize_callback = finalize_cb

    def perform_rapid_scan(self, gcmd):
        speed = gcmd.get_float("SCAN_SPEED", self.speed)
        scan_height = gcmd.get_float("HORIZONTAL_MOVE_Z", self.scan_height)
        gcmd.respond_info(
            "Beginning rapid surface scan at height %.2f..." % (scan_height)
        )
        pprobe = self.printer.lookup_object("probe")
        toolhead = self.printer.lookup_object("toolhead")
        # Calculate time window around which a sample is valid.  Current
        # assumption is anything within 2mm is usable, so:
        # window = 2 / max_speed
        #
        # TODO: validate maximum speed allowed based on sample rate of probe
        # Scale the hit distance window for speeds lower than 125mm/s.  The
        # lower the speed the less the window shrinks.
        scale = max(0, 1 - speed / MM_WIN_SPEED) + 1
        hit_dist = min(MAX_HIT_DIST, scale * speed / MM_WIN_SPEED)
        half_window = hit_dist / speed
        gcmd.respond_info(
            "Sample hit distance +/- %.4fmm, time window +/- ms %.4f"
            % (hit_dist, half_window * 1000)
        )
        gcmd_params = gcmd.get_command_parameters()
        gcmd_params["SAMPLE_TIME"] = half_window * 2
        self._raise_tool(gcmd, scan_height)
        probe_session = pprobe.start_probe_session(gcmd)
        offsets = pprobe.get_offsets()
        initial_move = True
        for pos, is_probe_pt in self.probe_manager.iter_rapid_path():
            pos = self._apply_offsets(pos[:2], offsets)
            toolhead.manual_move(pos, speed)
            if initial_move:
                initial_move = False
                self._move_to_scan_height(gcmd, scan_height)
            if is_probe_pt:
                probe_session.run_probe(gcmd)
        results = probe_session.pull_probed_results()
        toolhead.get_last_move_time()
        self.finalize_callback(offsets, results)
        probe_session.end_probe_session()

    def _raise_tool(self, gcmd, scan_height):
        # If the nozzle is below scan height raise the tool
        toolhead = self.printer.lookup_object("toolhead")
        pprobe = self.printer.lookup_object("probe")
        cur_pos = toolhead.get_position()
        if cur_pos[2] >= scan_height:
            return
        pparams = pprobe.get_probe_params(gcmd)
        lift_speed = pparams["lift_speed"]
        cur_pos[2] = self.scan_height + .5
        toolhead.manual_move(cur_pos, lift_speed)

    def _move_to_scan_height(self, gcmd, scan_height):
        time_window = gcmd.get_float("SAMPLE_TIME")
        toolhead = self.printer.lookup_object("toolhead")
        pprobe = self.printer.lookup_object("probe")
        cur_pos = toolhead.get_position()
        pparams = pprobe.get_probe_params(gcmd)
        lift_speed = pparams["lift_speed"]
        probe_speed = pparams["probe_speed"]
        cur_pos[2] = scan_height + .5
        toolhead.manual_move(cur_pos, lift_speed)
        cur_pos[2] = scan_height
        toolhead.manual_move(cur_pos, probe_speed)
        toolhead.dwell(time_window / 2 + .01)

    def _apply_offsets(self, point, offsets):
        return [(pos - ofs) for pos, ofs in zip(point, offsets)]


class MoveSplitter:
    def __init__(self, config, gcode):
        self.split_delta_z = config.getfloat(
            'split_delta_z', .025, minval=0.01)
        self.move_check_distance = config.getfloat(
            'move_check_distance', 5., minval=3.)
        self.z_mesh = None
        self.fade_offset = 0.
        self.gcode = gcode

    def initialize(self, mesh, fade_offset):
        self.z_mesh = mesh
        self.fade_offset = fade_offset
    def build_move(self, prev_pos, next_pos, factor, material_axes=None):
        self.prev_pos = tuple(prev_pos)
        self.next_pos = tuple(next_pos)
        self.current_pos = list(prev_pos)
        self.z_factor = factor
        self.z_offset = self._calc_z_offset(prev_pos)
        self.traverse_complete = False
        self.distance_checked = 0.
        axes_d = [np - pp for np, pp in zip(self.next_pos, self.prev_pos)]
        self.total_move_length = math.sqrt(sum([d*d for d in axes_d[:3]]))
        self.axis_move = [not isclose(d, 0., abs_tol=1e-10) for d in axes_d]
        # 本条指令的材料总量；按沿程比例 Δt 分摊给各切段
        self.material_axes = list(material_axes or [])
        self.prev_t = 0.
    def _calc_z_offset(self, pos):
        z = self.z_mesh.calc_z(pos[0], pos[1])
        offset = self.fade_offset
        return self.z_factor * (z - offset) + offset
    def _take_material(self, t):
        # 取出从上一段到当前沿程比例 t 之间应分摊的材料增量
        if not self.material_axes:
            return self.material_axes
        delta = t - self.prev_t
        self.prev_t = t
        return [m.scaled(delta) for m in self.material_axes]
    def _set_next_move(self, distance_from_prev):
        t = distance_from_prev / self.total_move_length
        if t > 1. or t < 0.:
            raise self.gcode.error(
                "bed_mesh: Slice distance is negative "
                "or greater than entire move length")
        for i in range(len(self.next_pos)):
            if self.axis_move[i]:
                self.current_pos[i] = lerp(
                    t, self.prev_pos[i], self.next_pos[i])
    def split(self):
        # 返回 (pos, material)；材料按沿程比例切分，终点段用 t=1 保证总量守恒
        if not self.traverse_complete:
            if self.axis_move[0] or self.axis_move[1]:
                # X and/or Y axis move, traverse if necessary
                while self.distance_checked + self.move_check_distance \
                        < self.total_move_length:
                    self.distance_checked += self.move_check_distance
                    self._set_next_move(self.distance_checked)
                    next_z = self._calc_z_offset(self.current_pos)
                    if abs(next_z - self.z_offset) >= self.split_delta_z:
                        self.z_offset = next_z
                        newpos = list(self.current_pos)
                        newpos[2] += self.z_offset
                        t = self.distance_checked / self.total_move_length
                        return newpos, self._take_material(t)
            # end of move reached
            self.current_pos[:] = self.next_pos
            self.z_offset = self._calc_z_offset(self.current_pos)
            # Its okay to add Z-Offset to the final move, since it will not be
            # used again.
            self.current_pos[2] += self.z_offset
            self.traverse_complete = True
            # 终点固定 t=1，无 XY 时也能拿到全部材料（避免 ratio=0 清零）
            return self.current_pos, self._take_material(1.)
        else:
            # Traverse complete
            return None, None


class ZMesh:
    def __init__(self, params, name):
        self.profile_name = name or "adaptive-%X" % (id(self),)
        self.probed_matrix = self.mesh_matrix = None
        self.mesh_params = params
        self.mesh_offsets = [0., 0.]
        self.mesh_z_shift = 0.
        logging.debug('bed_mesh: probe/mesh parameters:')
        for key, value in self.mesh_params.items():
            logging.debug("%s :  %s" % (key, value))
        self.mesh_x_min = params['min_x']
        self.mesh_x_max = params['max_x']
        self.mesh_y_min = params['min_y']
        self.mesh_y_max = params['max_y']
        logging.debug(
            "bed_mesh: Mesh Min: (%.2f,%.2f) Mesh Max: (%.2f,%.2f)"
            % (self.mesh_x_min, self.mesh_y_min,
               self.mesh_x_max, self.mesh_y_max))
        # Set the interpolation algorithm
        interpolation_algos = {
            'lagrange': self._sample_lagrange,
            'bicubic': self._sample_bicubic,
            'direct': self._sample_direct
        }
        self._sample = interpolation_algos.get(params['algo'])
        # Number of points to interpolate per segment
        mesh_x_pps = params['mesh_x_pps']
        mesh_y_pps = params['mesh_y_pps']
        px_cnt = params['x_count']
        py_cnt = params['y_count']
        self.mesh_x_count = (px_cnt - 1) * mesh_x_pps + px_cnt
        self.mesh_y_count = (py_cnt - 1) * mesh_y_pps + py_cnt
        self.x_mult = mesh_x_pps + 1
        self.y_mult = mesh_y_pps + 1
        logging.debug("bed_mesh: Mesh grid size - X:%d, Y:%d"
                      % (self.mesh_x_count, self.mesh_y_count))
        self.mesh_x_dist = (self.mesh_x_max - self.mesh_x_min) / \
                           (self.mesh_x_count - 1)
        self.mesh_y_dist = (self.mesh_y_max - self.mesh_y_min) / \
                           (self.mesh_y_count - 1)
    def get_mesh_matrix(self):
        if self.mesh_matrix is not None:
            return [[round(z + self.mesh_z_shift, 6) for z in line]
                    for line in self.mesh_matrix]
        return [[]]
    def get_probed_matrix(self):
        if self.probed_matrix is not None:
            return [[round(z + self.mesh_z_shift, 6) for z in line]
                    for line in self.probed_matrix]
        return [[]]
    def get_mesh_params(self):
        return self.mesh_params
    def get_profile_name(self):
        return self.profile_name
    def print_probed_matrix(self, print_func):
        if self.probed_matrix is not None:
            msg = "Mesh Leveling Probed Z positions:\n"
            for line in self.probed_matrix:
                for x in line:
                    msg += " %f" % x
                msg += "\n"
            print_func(msg)
        else:
            print_func("bed_mesh: bed has not been probed")
    def print_mesh(self, print_func, move_z=None):
        matrix = self.get_mesh_matrix()
        if matrix is not None:
            msg = "Mesh X,Y: %d,%d\n" % (self.mesh_x_count, self.mesh_y_count)
            if move_z is not None:
                msg += "Search Height: %d\n" % (move_z)
            msg += "Mesh Offsets: X=%.4f, Y=%.4f\n" % (
                self.mesh_offsets[0], self.mesh_offsets[1])
            msg += "Mesh Average: %.2f\n" % (self.get_z_average())
            rng = self.get_z_range()
            msg += "Mesh Range: min=%.4f max=%.4f\n" % (rng[0], rng[1])
            msg += "Interpolation Algorithm: %s\n" \
                   % (self.mesh_params['algo'])
            msg += "Measured points:\n"
            for y_line in range(self.mesh_y_count - 1, -1, -1):
                for z in matrix[y_line]:
                    msg += "  %f" % (z)
                msg += "\n"
            print_func(msg)
        else:
            print_func("bed_mesh: Z Mesh not generated")
    def build_mesh(self, z_matrix):
        self.probed_matrix = z_matrix
        self._sample(z_matrix)
        self.print_mesh(logging.debug)
    def set_zero_reference(self, xpos, ypos):
        offset = self.calc_z(xpos, ypos)
        logging.info(
            "bed_mesh: setting zero reference at (%.2f, %.2f, %.6f)"
            % (xpos, ypos, offset)
        )
        for matrix in [self.probed_matrix, self.mesh_matrix]:
            for yidx in range(len(matrix)):
                for xidx in range(len(matrix[yidx])):
                    matrix[yidx][xidx] -= offset
    def set_mesh_offsets(self, offsets):
        for i, o in enumerate(offsets):
            if o is not None:
                self.mesh_offsets[i] = o
    def set_mesh_z_shift(self, z_shift):
        self.mesh_z_shift = z_shift
    def get_x_coordinate(self, index):
        return self.mesh_x_min + self.mesh_x_dist * index
    def get_y_coordinate(self, index):
        return self.mesh_y_min + self.mesh_y_dist * index
    def calc_z(self, x, y):
        if self.mesh_matrix is not None:
            tbl = self.mesh_matrix
            tx, xidx = self._get_linear_index(x + self.mesh_offsets[0], 0)
            ty, yidx = self._get_linear_index(y + self.mesh_offsets[1], 1)
            z0 = lerp(tx, tbl[yidx][xidx], tbl[yidx][xidx+1])
            z1 = lerp(tx, tbl[yidx+1][xidx], tbl[yidx+1][xidx+1])
            return lerp(ty, z0, z1) + self.mesh_z_shift
        else:
            # No mesh table generated, no z-adjustment
            return 0.
    def get_z_range(self):
        if self.mesh_matrix is not None:
            mesh_min = min([min(x) for x in self.mesh_matrix]) + self.mesh_z_shift
            mesh_max = max([max(x) for x in self.mesh_matrix]) + self.mesh_z_shift
            return mesh_min, mesh_max
        else:
            return 0., 0.
    def get_z_average(self):
        if self.mesh_matrix is not None:
            avg_z = (sum([sum(x) for x in self.mesh_matrix]) /
                     sum([len(x) for x in self.mesh_matrix]))
            avg_z += self.mesh_z_shift
            # Round average to the nearest 100th.  This
            # should produce an offset that is divisible by common
            # z step distances
            return round(avg_z, 2)
        else:
            return 0.
    def _get_linear_index(self, coord, axis):
        if axis == 0:
            # X-axis
            mesh_min = self.mesh_x_min
            mesh_cnt = self.mesh_x_count
            mesh_dist = self.mesh_x_dist
            cfunc = self.get_x_coordinate
        else:
            # Y-axis
            mesh_min = self.mesh_y_min
            mesh_cnt = self.mesh_y_count
            mesh_dist = self.mesh_y_dist
            cfunc = self.get_y_coordinate
        t = 0.
        idx = int(math.floor((coord - mesh_min) / mesh_dist))
        idx = constrain(idx, 0, mesh_cnt - 2)
        t = (coord - cfunc(idx)) / mesh_dist
        return constrain(t, 0., 1.), idx
    def _sample_direct(self, z_matrix):
        self.mesh_matrix = z_matrix
    def _sample_lagrange(self, z_matrix):
        x_mult = self.x_mult
        y_mult = self.y_mult
        self.mesh_matrix = \
            [[0. if ((i % x_mult) or (j % y_mult))
             else z_matrix[j//y_mult][i//x_mult]
             for i in range(self.mesh_x_count)]
             for j in range(self.mesh_y_count)]
        xpts, ypts = self._get_lagrange_coords()
        # Interpolate X coordinates
        for i in range(self.mesh_y_count):
            # only interpolate X-rows that have probed coordinates
            if i % y_mult != 0:
                continue
            for j in range(self.mesh_x_count):
                if j % x_mult == 0:
                    continue
                x = self.get_x_coordinate(j)
                self.mesh_matrix[i][j] = self._calc_lagrange(xpts, x, i, 0)
        # Interpolate Y coordinates
        for i in range(self.mesh_x_count):
            for j in range(self.mesh_y_count):
                if j % y_mult == 0:
                    continue
                y = self.get_y_coordinate(j)
                self.mesh_matrix[j][i] = self._calc_lagrange(ypts, y, i, 1)
    def _get_lagrange_coords(self):
        xpts = []
        ypts = []
        for i in range(self.mesh_params['x_count']):
            xpts.append(self.get_x_coordinate(i * self.x_mult))
        for j in range(self.mesh_params['y_count']):
            ypts.append(self.get_y_coordinate(j * self.y_mult))
        return xpts, ypts
    def _calc_lagrange(self, lpts, c, vec, axis=0):
        pt_cnt = len(lpts)
        total = 0.
        for i in range(pt_cnt):
            n = 1.
            d = 1.
            for j in range(pt_cnt):
                if j == i:
                    continue
                n *= (c - lpts[j])
                d *= (lpts[i] - lpts[j])
            if axis == 0:
                # Calc X-Axis
                z = self.mesh_matrix[vec][i*self.x_mult]
            else:
                # Calc Y-Axis
                z = self.mesh_matrix[i*self.y_mult][vec]
            total += z * n / d
        return total
    def _sample_bicubic(self, z_matrix):
        # should work for any number of probe points above 3x3
        x_mult = self.x_mult
        y_mult = self.y_mult
        c = self.mesh_params['tension']
        self.mesh_matrix = \
            [[0. if ((i % x_mult) or (j % y_mult))
             else z_matrix[j//y_mult][i//x_mult]
             for i in range(self.mesh_x_count)]
             for j in range(self.mesh_y_count)]
        # Interpolate X values
        for y in range(self.mesh_y_count):
            if y % y_mult != 0:
                continue
            for x in range(self.mesh_x_count):
                if x % x_mult == 0:
                    continue
                pts = self._get_x_ctl_pts(x, y)
                self.mesh_matrix[y][x] = self._cardinal_spline(pts, c)
        # Interpolate Y values
        for x in range(self.mesh_x_count):
            for y in range(self.mesh_y_count):
                if y % y_mult == 0:
                    continue
                pts = self._get_y_ctl_pts(x, y)
                self.mesh_matrix[y][x] = self._cardinal_spline(pts, c)
    def _get_x_ctl_pts(self, x, y):
        # Fetch control points and t for a X value in the mesh
        x_mult = self.x_mult
        x_row = self.mesh_matrix[y]
        last_pt = self.mesh_x_count - 1 - x_mult
        if x < x_mult:
            p0 = p1 = x_row[0]
            p2 = x_row[x_mult]
            p3 = x_row[2*x_mult]
            t = x / float(x_mult)
        elif x > last_pt:
            p0 = x_row[last_pt - x_mult]
            p1 = x_row[last_pt]
            p2 = p3 = x_row[last_pt + x_mult]
            t = (x - last_pt) / float(x_mult)
        else:
            found = False
            for i in range(x_mult, last_pt, x_mult):
                if x > i and x < (i + x_mult):
                    p0 = x_row[i - x_mult]
                    p1 = x_row[i]
                    p2 = x_row[i + x_mult]
                    p3 = x_row[i + 2*x_mult]
                    t = (x - i) / float(x_mult)
                    found = True
                    break
            if not found:
                raise BedMeshError(
                    "bed_mesh: Error finding x control points")
        return p0, p1, p2, p3, t
    def _get_y_ctl_pts(self, x, y):
        # Fetch control points and t for a Y value in the mesh
        y_mult = self.y_mult
        last_pt = self.mesh_y_count - 1 - y_mult
        y_col = self.mesh_matrix
        if y < y_mult:
            p0 = p1 = y_col[0][x]
            p2 = y_col[y_mult][x]
            p3 = y_col[2*y_mult][x]
            t = y / float(y_mult)
        elif y > last_pt:
            p0 = y_col[last_pt - y_mult][x]
            p1 = y_col[last_pt][x]
            p2 = p3 = y_col[last_pt + y_mult][x]
            t = (y - last_pt) / float(y_mult)
        else:
            found = False
            for i in range(y_mult, last_pt, y_mult):
                if y > i and y < (i + y_mult):
                    p0 = y_col[i - y_mult][x]
                    p1 = y_col[i][x]
                    p2 = y_col[i + y_mult][x]
                    p3 = y_col[i + 2*y_mult][x]
                    t = (y - i) / float(y_mult)
                    found = True
                    break
            if not found:
                raise BedMeshError(
                    "bed_mesh: Error finding y control points")
        return p0, p1, p2, p3, t
    def _cardinal_spline(self, p, tension):
        t = p[4]
        t2 = t*t
        t3 = t2*t
        m1 = tension * (p[2] - p[0])
        m2 = tension * (p[3] - p[1])
        a = p[1] * (2*t3 - 3*t2 + 1)
        b = p[2] * (-2*t3 + 3*t2)
        c = m1 * (t3 - 2*t2 + t)
        d = m2 * (t3 - t2)
        return a + b + c + d


class ProfileManager:
    def __init__(self, config, bedmesh):
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.bedmesh = bedmesh
        self.profiles = {}
        self.incompatible_profiles = []
        # Fetch stored profiles from Config
        stored_profs = config.get_prefix_sections(self.name)
        stored_profs = [s for s in stored_profs
                        if s.get_name() != self.name]
        for profile in stored_profs:
            name = profile.get_name().split(' ', 1)[1]
            version = profile.getint('version', 0)
            if version != PROFILE_VERSION:
                logging.info(
                    "bed_mesh: Profile [%s] not compatible with this version\n"
                    "of bed_mesh.  Profile Version: %d Current Version: %d "
                    % (name, version, PROFILE_VERSION))
                self.incompatible_profiles.append(name)
                continue
            self.profiles[name] = {}
            zvals = profile.getlists('points', seps=(',', '\n'), parser=float)
            self.profiles[name]['points'] = zvals
            self.profiles[name]['mesh_params'] = params = \
                collections.OrderedDict()
            for key, t in PROFILE_OPTIONS.items():
                if t is int:
                    params[key] = profile.getint(key)
                elif t is float:
                    params[key] = profile.getfloat(key)
                elif t is str:
                    params[key] = profile.get(key)
            # temp = profile.getfloat('temperature', 20.0)
            # self.profiles[name]['temperature'] = temp
        # Register GCode
        self.gcode.register_command(
            'BED_MESH_PROFILE', self.cmd_BED_MESH_PROFILE,
            desc=self.cmd_BED_MESH_PROFILE_help)
    def get_profiles(self):
        return self.profiles
    def _check_incompatible_profiles(self):
        if self.incompatible_profiles:
            configfile = self.printer.lookup_object('configfile')
            for profile in self.incompatible_profiles:
                configfile.remove_section('bed_mesh ' + profile)
            self.gcode.respond_info(
                "The following incompatible profiles have been detected\n"
                "and are scheduled for removal:\n%s\n"
                "The SAVE_CONFIG command will update the printer config\n"
                "file and restart the printer" %
                (('\n').join(self.incompatible_profiles)))
    def save_profile_from_mesh(self, prof_name, z_mesh, announce=True):
        """将指定 ZMesh 写入会话 profiles 与 configfile pending（不触发落盘）。"""
        if z_mesh is None:
            self.gcode.respond_info(
                "Unable to save to profile [%s], the bed has not been probed"
                % (prof_name))
            return False
        probed_matrix = z_mesh.get_probed_matrix()
        mesh_params = z_mesh.get_mesh_params()

        configfile = self.printer.lookup_object('configfile')
        cfg_name = self.name + " " + prof_name
        z_values = ""
        for line in probed_matrix:
            z_values += "\n  "
            for p in line:
                z_values += "%.6f, " % p
            z_values = z_values[:-2]

        configfile.set(cfg_name, 'version', PROFILE_VERSION)
        configfile.set(cfg_name, 'points', z_values)
        for key, value in mesh_params.items():
            configfile.set(cfg_name, key, value)
        # 会话内副本，保证 status/profiles 不可变视图
        profiles = dict(self.profiles)
        profiles[prof_name] = profile = {}
        profile['points'] = probed_matrix
        profile['mesh_params'] = collections.OrderedDict(mesh_params)
        self.profiles = profiles
        self.bedmesh.update_status()
        if announce:
            self.gcode.respond_info(
                "Bed Mesh state has been saved to profile [%s]\n"
                "for the current session.  The SAVE_CONFIG command will\n"
                "update the printer config file and restart the printer."
                % (prof_name))
        return True
    def save_profile(self, prof_name):
        z_mesh = self.bedmesh.get_mesh()
        self.save_profile_from_mesh(prof_name, z_mesh, announce=True)
    def load_profile(self, prof_name, save_snapshot=True):
        profile = self.profiles.get(prof_name, None)
        if profile is None:
            raise self.gcode.error(
                "bed_mesh: Unknown profile [%s]" % prof_name)
        probed_matrix = profile['points']
        mesh_params = profile['mesh_params']
        z_mesh = ZMesh(mesh_params, prof_name)
        try:
            z_mesh.build_mesh(probed_matrix)
        except BedMeshError as e:
            raise self.gcode.error(str(e))
        self.bedmesh.set_mesh(z_mesh)
        # 加载 runtime_active 自身时不再二次落盘，避免续打路径重复 SAVE_CONFIG
        if save_snapshot and prof_name != RUNTIME_ACTIVE_PROFILE:
            self.bedmesh.save_runtime_mesh_snapshot(z_mesh)
    def remove_profile(self, prof_name):
        if prof_name in self.profiles:
            configfile = self.printer.lookup_object('configfile')
            configfile.remove_section('bed_mesh ' + prof_name)
            profiles = dict(self.profiles)
            del profiles[prof_name]
            self.profiles = profiles
            self.bedmesh.update_status()
            self.gcode.respond_info(
                "Profile [%s] removed from storage for this session.\n"
                "The SAVE_CONFIG command will update the printer\n"
                "configuration and restart the printer" % (prof_name))
        else:
            self.gcode.respond_info(
                "No profile named [%s] to remove" % (prof_name))
    cmd_BED_MESH_PROFILE_help = "Bed Mesh Persistent Storage management"
    def cmd_BED_MESH_PROFILE(self, gcmd):
        options = (
            ('LOAD', self.load_profile),
            ('SAVE', self.save_profile),
            ('REMOVE', self.remove_profile)
            )

        for key, func in options:
            name = gcmd.get(key, None)
            if name is not None:
                name = name.strip()
                if not name:
                    raise gcmd.error(
                        "Value for parameter '%s' must be specified" % (key)
                    )
                
                func(name)
                return
        gcmd.respond_info("Invalid syntax '%s'" % (gcmd.get_commandline(),))


def load_config(config):
    return BedMesh(config)
