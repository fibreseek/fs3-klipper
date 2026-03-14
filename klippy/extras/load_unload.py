#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
进退料流程 Python 模块
将 load_unload.cfg 和 global_variables.cfg 中的逻辑同步到 Klipper Python 模块

====== 统一状态机编号 (上位机/固件通用) ======
# 0x: 空闲
#   0  = IDLE              空闲
# 1x: 准备阶段
#   10 = PREHEATING        预热中
#   11 = WAIT_INSERT       等待插入耗材
#   12 = DETECTED          检测到耗材（过渡状态）
# 2x: 进料阶段
#   20 = LOADING          进料中
#   21 = SLOW_EXTRUSION    慢速挤出中 (P1 1mm逐毫米)
#   22 = MELT_EXTRUDING    熔融挤出中
#   23 = PREHEAT_LOW       低温预热50°C (P1内部状态)
#   24 = CLOG_DETECTION    堵料检测中 (P1内部状态)
#   25 = CLOG_RETRACTED    堵料回退完成 (P1内部状态，耗材已到达喷头)
# 3x: 确认阶段
#   30 = CONFIRM_EXTRUSION 等待确认是否挤出
#   31 = RETRYING          补料重试中
# 4x: 清理阶段
#   40 = WAIT_USER_CLEAN   等待用户清理 (F)
#   41 = AUTO_CLEANING     自动擦嘴中 (F/P2)
# 5x: 退料阶段
#   50 = PREHEATING_UNLOAD 退料预热中
#   51 = UNLOADING         退料中
#   52 = PREHEATING_UNLOAD_WAIT 预回抽等待中
# 9x: 结束状态
#   90 = FINISHED          完成
#   91 = CANCELLED         已取消
#   98 = TIMEOUT           超时
#   99 = ERROR             错误
# ================================================

进料流程说明:
  F通道 (L=1): 10(预热中) -> 22(熔融挤出) -> [30(确认)] -> 40(用户清理) -> 41(自动擦嘴) -> 90
  F通道 (L=0): 10(预热中) -> 11(等待插入) -> 20(快速进料) -> 22(熔融挤出) -> [30(确认)] -> 40(用户清理) -> 41(自动擦嘴) -> 90
  P1通道: 11(等待插入) -> 12(检测到耗材) -> 24(50°C预热+快挤出) -> 21(慢速挤出,堵料检测) -> 25(回退5mm+前进3mm) -> 90(完成)
  P2通道 (任何): 10(预热中) -> 11(等待插入) -> 22(熔融挤出) -> [30(确认)] -> 41(自动擦嘴) -> 90
  注: P1通道不设置初始温度，保持当前温度；在阶段2(24)设置50°C
  注: P1通道堵料检测仅在慢速挤出阶段(21)进行，检测到耗材停止运动=到达喷头
  注: P1堵料检测成功说明耗材已到达喷头，在阶段3(25)回退5mm后前进3mm，然后完成(90)
  注: 在22时用户可随时确认挤出，跳过30直接进入40/41
"""

import logging

# 创建模块 logger
logger = logging.getLogger(__name__)

# ============================================================
# 状态机常量定义
# ============================================================

class State:
    """
    状态机常量类
    定义所有可能的状态值，与.cfg文件中的状态机保持一致
    """
    # 空闲状态
    IDLE = 0
    # 准备阶段
    PREHEATING = 10        # 预热中（所有通道通用）
    WAIT_INSERT = 11       # 等待插入耗材
    DETECTED = 12          # 检测到耗材（过渡状态）
    # 进料阶段
    FAST_LOADING = 20      # 快速进料中 (F/P1)
    MELT_EXTRUDING = 22    # 熔融挤出中 (所有通道)
        # P1堵料检测阶段 (新增，使用2x编号)
    SLOW_EXTRUSION = 21    # 慢速逐毫米挤出中 (1mm-by-1mm with timeout)
    SLOW_EXTRUSION_CHECK_CLOG = 22 # 慢速挤出后检测堵料中
    PREHEAT_LOW = 23       # 低温预热中 (50°C)
    CLOG_DETECTION = 24    # 堵料检测中 (快速进料1000mm @ 20mm/s)
    CLOG_RETRACTED = 25    # 堵料回退完成 (回退10mm)
    WAIT_MOMENT_FOR_FAST_MOVE = 26 # 等待时机进行快速移动（P1专用，进行快速移动）
    # 确认阶段
    CONFIRM_EXTRUSION = 30 # 等待确认是否挤出
    RETRYING = 31          # 补料重试中

    # 清理阶段
    WAIT_USER_CLEAN = 40   # 等待用户清理 (F/P1)
    AUTO_CLEANING = 41     # 自动擦嘴中
    # 退料阶段
    PREHEATING_UNLOAD = 50 # 退料预热中
    UNLOADING = 51         # 退料中
    PREHEATING_UNLOAD_WAIT = 52 # 预回抽等待中（等待3秒）
    # 结束状态
    FINISHED = 90          # 完成
    CANCELLED = 91         # 已取消
    TIMEOUT = 98            # 进退料超时（整体超时）
    ERROR = 99             # 错误

class OperationMode:
    """
    操作模式常量类
    定义当前的操作模式：空闲、进料、退料
    """
    IDLE = 0      # 空闲
    LOADING = 1   # 进料中
    UNLOADING = -1 # 退料中

class Channel:
    """
    进料通道常量类
    定义三个进料通道的索引值
    """
    F = 0   # F通道：长距离快速进料 + 熔融挤出，需要纤维切割器
    P1 = 1  # P1通道：中距离快速进料 + 堵料检测 + 熔融挤出
    P2 = 2  # P2通道：直接熔融挤出，无快速进料阶段

# ============================================================
# 主处理器类
# ============================================================

class LoadUnloadHandler:
    """
    进退料流程处理器

    核心功能：
    1. 管理三个独立进料通道(F/P1/P2)的加载/卸载流程
    2. 实现统一状态机，处理温度等待、耗材检测、进料、确认、清理等阶段
    3. 提供G-code命令接口供上位机调用
    4. 通过定时器实现非阻塞的状态轮询
    """

    def __init__(self, config):
        """初始化加载处理器，注册G-code命令并加载配置参数"""
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        # ========================================================
        # 状态变量 (对应 global_variables.cfg)
        # ========================================================
        self.load_distance = 0.0       # 当前进料距离 (mm)
        self.unload_distance = 0.0     # 当前出料距离 (mm)
        self.load_unload_index = Channel.F  # 通道索引: 0=F, 1=P1, 2=P2
        self.load_unload_nozzle = 0     # 喷嘴索引: 0=T0, 1=T1
        self.load_unload_temperature = 200 # 进料/退料温度，默认200°C
        self.operation_mode = OperationMode.IDLE # 操作模式: 0=空闲, 1=进料, -1=退料
        self.load_step = State.IDLE    # 当前状态机步骤
        self.user_confirmed = 0        # 用户确认标志: 0=未确认, 1=已确认挤出, 2=继续补料
        self.was_loaded_parameter = 0  # 存储L参数，供F/P1通道温度达标后判断使用
        self.p1_clog_detected = False  # P1堵料检测标志
        self.p1_clog_retract_distance = 5.0  # P1堵料回退距离(mm)

        # P1慢速挤出超时相关
        self.p1_slow_extrusion_timeout = 120.0  # 2分钟超时(秒)
        self.p1_slow_extrusion_start_time = None  # 开始时间
        self.p1_timeout_timer = None  # 超时定时器
        self.p1_slow_extrusion_distance = 0.0  # 慢速挤出距离
        self.p1_clog_detection_initialized = False  # CLOG_DETECTION状态是否已初始化
        self.p1_slow_extrusion_entered = False  # 是否已进入SLOW_EXTRUSION状态（防止重复初始化）
        self.p1_clog_extruded = False  # CLOG_RETRACTED状态是否已处理（防止重复执行）
        self.last_reported_state = None  # 上次向上位机发送的状态码（防止重复发送）

        # ========================================================
        # F 通道参数 (长距离快速进料 + 熔融挤出)
        # ========================================================
        self.f_long_distance = config.getfloat('f_long_distance', 1100.0)      # F 快速进料距离 (mm)
        self.f_melt_distance = config.getfloat('f_melt_distance', 250.0)       # F 熔融挤出距离 (mm)
        self.f_step_length = config.getfloat('f_step_length', 20.0)            # F 快速进料步长 (mm) - 20mm @ 20mm/s = 1秒/步
        self.f_melt_step_length = config.getfloat('f_melt_step_length', 5.0)    # F 熔融挤出步长 (mm) - 5mm @ 5mm/s = 1秒/步

        # ========================================================
        # P1 通道参数 (中距离快速进料 + 熔融挤出)
        # ========================================================
        self.p1_long_distance = config.getfloat('p1_long_distance', 1035.0)      # P1 快速进料距离 (mm)
        self.p1_melt_distance = config.getfloat('p1_melt_distance', 150.0)      # P1 熔融挤出距离 (mm)
        self.p1_step_length = config.getfloat('p1_step_length', 20.0)           # P1 快速进料步长 (mm) - 20mm @ 20mm/s = 1秒/步
        self.p1_melt_step_length = config.getfloat('p1_melt_step_length', 2.0)   # P1 熔融挤出步长 (mm) - 5mm @ 5mm/s = 1秒/步
        self.p1_fast_move_speed = config.getfloat('p1_fast_move_speed', 1200.0)  # P1 快速移动速度 (mm/min) - F1200 = 20mm/s
        self.p1_slow_move_speed = config.getfloat('p1_slow_move_speed', 100.0)   # P1 慢速移动速度 (mm/min) - F100 = 2mm/s
        self.p1_clog_detection_min_length = config.getfloat('p1_clog_detection_min_length', 16.0)  # P1 堵料检测最小挤出长度 (mm)，如果挤出距离超过这个值还未检测到堵料，则认为检测失败
        # ========================================================
        # P2 通道参数 (直接熔融挤出，无快速进料阶段)
        # ========================================================
        self.p2_long_distance = config.getfloat('p2_long_distance', 112.5)      # P2 进料距离 (mm) - 2.5mm/s * 45s = 112.5mm
        self.p2_step_length = config.getfloat('p2_step_length', 2.5)            # P2 每次进料步长 (mm) - 2.5mm @ 2.5mm/s = 1秒/步

        # ========================================================
        # 通用参数
        # ========================================================
        self.retry_distance = config.getfloat('retry_distance', 50.0)          # 补料重试距离 (mm) - F/P1/P2 统一使用50mm
        self.retry_step_length = config.getfloat('retry_step_length', 5.0)      # 补料重试步长 (mm) - 5mm @ 5mm/s = 1秒/步
        self.retry_speed = config.getfloat('retry_speed', 300.0)                # 补料重试速度 (mm/min)

        # 退料速度 (不同通道使用不同速度)
        self.f_p1_unload_speed = config.getfloat('f_p1_unload_speed', 1200.0)  # F/P1 退料速度 (F1200 = 20mm/s，已限制最大速度)
        self.p2_unload_speed = config.getfloat('p2_unload_speed', 1000.0)       # P2 退料速度 (F1000 = 16.67mm/s)

        # 清理参数 (喷嘴位置和运动参数)
        self.clean_nozzle_x = config.getfloat('clean_nozzle_x', 267.0)         # 初始清理X位置，预挤出耗材进入废料盒
        self.clean_nozzle_y = config.getfloat('clean_nozzle_y', 322.0)         # 初始清理Y位置，预挤出耗材进入废料盒
        self.clean_nozzle_x1 = config.getfloat('clean_nozzle_x1', 267.0)        # 清理X位置，铁刷旁边
        self.clean_nozzle_y1 = config.getfloat('clean_nozzle_y1', 337.0)        # 清理Y位置，铁刷旁边
        self.clean_move_speed = config.getfloat('clean_move_speed', 6000.0)    # 移动喷嘴的速度 (mm/min)
        self.clean_brush_speed = config.getfloat('clean_brush_speed', 2000.0)  # 清理喷嘴的速度 (mm/min)
        self.clean_x_offset = config.getfloat('clean_x_offset', 10.0)          # 清理时左右摆动的x偏移量 (mm)
        self.clean_y_offset = config.getfloat('clean_y_offset', 20.0)          # 清理时上下摆动的y偏移量 (mm)
        self.temperature_offset = config.getfloat('temperature_offset', 80.0)  # 清理时的温度比实际打印温度的offset (°C)

        # 进料类型参数 (进料时清理喷嘴的回抽和预加载参数)
        self.load_clean_retract_speed = config.getfloat('load_clean_retract_speed', 800.0)    # 进料时清理喷嘴的回抽速度 (mm/min)
        self.load_clean_retract_e = config.getfloat('load_clean_retract_e', -8.0)             # 进料时回抽长度 (mm)
        self.load_preloading_filament_e = config.getfloat('load_preloading_filament_e', 20.0)  # 进料预加载挤出长度 (mm)
        self.load_preloading_filament_speed = config.getfloat('load_preloading_filament_speed', 300.0) # 进料预加载挤出速度 (mm/min)

        # 退料类型参数 (退料时清理喷嘴的回抽和预加载参数)
        self.unload_clean_retract_speed = config.getfloat('unload_clean_retract_speed', 1200)    # 退料时清理喷嘴的回抽速度 (mm/min)
        self.unload_clean_retract_e = config.getfloat('unload_clean_retract_e', -15.0)             # 退料时回抽长度 (mm)
        self.unload_preloading_filament_e = config.getfloat('unload_preloading_filament_e', 20.0)  # 退料预加载挤出长度 (mm) - F/P2使用
        self.unload_preloading_filament_speed = config.getfloat('unload_preloading_filament_speed', 100.0) # 退料预加载挤出速度 (mm/min)
        # P1通道退料专用参数 (预加载挤出长度较小)
        self.p1_unload_preloading_filament_e = config.getfloat('p1_unload_preloading_filament_e', 2.0)  # P1退料预加载挤出长度 (mm)
        self.p1_unload_preloading_filament_speed = config.getfloat('p1_unload_preloading_filament_speed', 100.0) # P1退料预加载挤出速度 (mm/min)
        self.p1_unload_clean_retract_speed = config.getfloat('p1_unload_clean_retract_speed', 300.0)  # P1退料回抽速度 (mm/min)

        # 进退料超时参数
        self.load_unload_timeout = config.getfloat('load_unload_timeout', 900., above=0.)  # 进退料整体超时时间（秒），默认900（15分钟）
        self.load_unload_idle_timeout_extension = config.getfloat('load_unload_idle_timeout_extension', 3600., above=0.)  # idle_timeout扩展时间（秒），默认3600（1小时）

        # ========================================================
        # 定时器引用
        # ========================================================
        self.process_timer = None  # 核心处理定时器（处理进料/退料循环）
        self.temp_timer = None     # 温度等待定时器（处理温度达标检测）
        self.load_unload_timer = None  # 进退料整体超时定时器
        self.saved_idle_timeout = None   # 保存的原始 idle_timeout 值

        # ========================================================
        # 注册 G-code 命令
        # ========================================================
        self.gcode.register_command('LOAD_FILAMENT', self.cmd_LOAD_FILAMENT, desc=self.cmd_LOAD_FILAMENT_help)
        self.gcode.register_command('UNLOAD_FILAMENT', self.cmd_UNLOAD_FILAMENT, desc=self.cmd_UNLOAD_FILAMENT_help)
        self.gcode.register_command('CONFIRM_EXTRUDED', self.cmd_CONFIRM_EXTRUDED, desc=self.cmd_CONFIRM_EXTRUDED_help)
        self.gcode.register_command('CONTINUE_LOAD', self.cmd_CONTINUE_LOAD, desc=self.cmd_CONTINUE_LOAD_help)
        self.gcode.register_command('CONFIRM_CLEANED', self.cmd_CONFIRM_CLEANED, desc=self.cmd_CONFIRM_CLEANED_help)
        self.gcode.register_command('EXIT_LOADUNLOAD', self.cmd_EXIT_LOADUNLOAD, desc=self.cmd_EXIT_LOADUNLOAD_help)
        self.gcode.register_command('PREPARE_LOAD_UNLOAD_NOZZLE', self.cmd_PREPARE_LOAD_UNLOAD_NOZZLE, desc=self.cmd_PREPARE_LOAD_UNLOAD_NOZZLE_help)
        self.gcode.register_command('FINISH_LOADUNLOAD', self.cmd_FINISH_LOADUNLOAD, desc=self.cmd_FINISH_LOADUNLOAD_help)

        # 注意：filament_motion_sensor:filament_clog 事件只在 P1 进料时动态注册
        self.printer.register_event_handler('extruder_stall_detector:stall_detected', self._handle_filament_clog)


        self.count = 0
    # ============================================================
    # G-code 命令实现
    # ============================================================

    cmd_LOAD_FILAMENT_help = "进料命令: LOAD_FILAMENT I=<0-2> T=<0-1> S=<temperature> L=<0-1>"
    def cmd_LOAD_FILAMENT(self, gcmd):
        """
        进料主命令

        功能：启动指定通道的进料流程

        参数说明:
        I: 通道索引 (0=F通道, 1=P1通道, 2=P2通道)
        T: 喷嘴选择 (0=T0左喷嘴, 1=T1右喷嘴)
        S: 目标温度 (°C)
        L: 是否已加载 (0=否, 需要等待传感器检测; 1=是, 跳过传感器检测直接熔融挤出)

        状态流程:
        1. 检查打印机是否空闲
        2. 验证目标温度是否合法
        3. 初始化变量并准备进料
        4. 设置加热温度
        5. 进入STATE:10预热状态
        6. 启动温度等待定时器
        """
        # 检查打印机是否空闲
        logger.debug(f"[LOAD_FILAMENT] Starting, operation_mode={self.operation_mode}")
        if self.operation_mode != OperationMode.IDLE:
            channel_name = self._get_channel_name()
            gcmd.respond_info(f"[{channel_name} LOAD] ERROR: Printer is busy")
            return

        # 解析命令参数
        index = gcmd.get_int('I', 0)           # 通道索引
        nozzle = gcmd.get_int('T', 0)          # 喷嘴选择
        target = gcmd.get_int('S', 0)          # 目标温度
        was_loaded = gcmd.get_int('L', 0)      # 是否已加载

        logger.debug(f"[LOAD_FILAMENT] Parameters: I={index}, T={nozzle}, S={target}, L={was_loaded}")
        channel_name = self._get_channel_name(index)

        # 验证目标温度是否合法（根据nozzle参数获取对应挤出机）
        extruder_name = 'extruder' if nozzle == 0 else 'extruder1'
        logger.debug(f"[LOAD_FILAMENT] Looking up extruder: {extruder_name}")
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None:
            logger.error(f"[LOAD_FILAMENT] Extruder '{extruder_name}' not found")
            gcmd.respond_info(f"[{channel_name} LOAD] ERROR: Extruder '{extruder_name}' not found")
            return
        min_temp = extruder.heater.min_extrude_temp
        logger.debug(f"[LOAD_FILAMENT] Extruder min_temp={min_temp}, target={target}")
        if target <= min_temp:
            gcmd.respond_info(f"[{channel_name} LOAD] ERROR: Temperature {target}C too low")
            return

        # 初始化状态变量
        self.load_unload_index = index
        self.load_unload_nozzle = nozzle
        self.load_unload_temperature = target
        self.load_step = State.IDLE
        self.user_confirmed = 0
        self.was_loaded_parameter = was_loaded  # 存储L参数供温度达标后使用
        self.p1_clog_detected = False  # 重置P1堵料检测标志
        self.p1_clog_detection_initialized = False  # 重置CLOG_DETECTION初始化标志
        self.p1_slow_extrusion_entered = False  # 重置SLOW_EXTRUSION进入标志
        self.p1_clog_extruded = False  # 重置CLOG_RETRACTED处理标志
        self.p1_slow_extrusion_distance = 0.0  # 重置慢速挤出距离
        self.p1_slow_extrusion_start_time = None  # 重置慢速挤出开始时间
        self.last_reported_state = None  # 重置上次报告状态，允许状态码重新发送
        # 停止任何残留的定时器（防止上一次操作的定时器干扰新操作）
        self._stop_temp_timer()
        self._stop_process_timer()
        self._stop_timeout_timer()

        # 停止P1堵料检测（防止上一次操作的堵料检测干扰新操作）
        self.printer.send_event("filament_clog:check_stop")
        self.gcode.respond_info("filament_clog:check_stop")

        logger.debug(f"[LOAD_FILAMENT] Calling _prepare_load()")
        # 扩展 idle_timeout 并启动超时定时器（在准备进料之前）
        self._extend_idle_timeout()
        self._start_load_unload_timer(self.load_unload_timeout)
        # 准备进料（移动到废料站、切换喷嘴等）
        self._prepare_load()

        # 只有非 P1 的情况下才设置目标温度
        # P1通道（L=0或L=1）: 初始化时不设置温度，保持当前温度，在阶段2(CLOG_DETECTION)设置50°C
        if index != Channel.P1:
            logger.debug(f"[LOAD_FILAMENT] Setting heater temperature: M104 T{nozzle} S{target}")
            self.gcode.run_script_from_command(f"M104 T{nozzle} S{target}")
        else:
            # P1通道：初始化时不设置温度，保持当前温度
            logger.debug(f"[LOAD_FILAMENT] P1: Not setting initial temperature, keeping current temperature")

        # ============================================================
        # 根据通道决定初始状态
        # ============================================================
        if index == Channel.P1:
            # P1通道，直接进入等待插入
            self.load_step = State.WAIT_INSERT
            gcmd.respond_info(f"[{channel_name} LOAD] STATE:11 WAIT_INSERT")
            # 启动核心处理定时器开始检测耗材
            self._start_process_timer(1.0)
        elif index == Channel.F:
            # F通道: 先预热到目标温度
            self.load_step = State.PREHEATING
            gcmd.respond_info(f"[{channel_name} LOAD] STATE:10 PREHEATING T:{target}")
            # 启动温度等待定时器
            self._start_temp_timer(1.0)
        else:
            # P2通道: 先预热到目标温度
            self.load_step = State.PREHEATING
            gcmd.respond_info(f"[{channel_name} LOAD] STATE:10 PREHEATING T:{target}")
            # 启动温度等待定时器
            self._start_temp_timer(1.0)

        logger.debug(f"[LOAD_FILAMENT] Command completed successfully")

    cmd_UNLOAD_FILAMENT_help = "退料命令: UNLOAD_FILAMENT I=<0-2> T=<0-1> S=<temperature>"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        """
        退料主命令

        功能：启动指定通道的退料流程

        参数说明:
        I: 通道索引 (0=F通道, 1=P1通道, 2=P2通道)
        T: 喷嘴选择 (0=T0左喷嘴, 1=T1右喷嘴)
        S: 目标温度 (°C)

        状态流程:
        - F通道：直接退料（关闭最低挤出温度限制），无需预热
        - P1/P2通道：需要预热到目标温度后再退料

        退料完成后恢复最低挤出温度限制（F通道）
        """
        # 检查打印机是否空闲
        if self.operation_mode != OperationMode.IDLE:
            channel_name = self._get_channel_name()
            gcmd.respond_info(f"[{channel_name} UNLOAD] ERROR: Printer is busy")
            return

        # 解析命令参数
        index = gcmd.get_int('I', 0)
        nozzle = gcmd.get_int('T', 0)
        target = gcmd.get_int('S', 0)

        channel_name = self._get_channel_name(index)

        # 获取最低挤出温度（根据nozzle参数获取对应挤出机）
        extruder_name = 'extruder' if nozzle == 0 else 'extruder1'
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None:
            gcmd.respond_info(f"[{channel_name} UNLOAD] ERROR: Extruder '{extruder_name}' not found")
            return
        min_temp = extruder.heater.min_extrude_temp

        # 初始化变量
        self.load_unload_nozzle = nozzle
        self.load_unload_temperature = target
        self.load_unload_index = index
        # 停止任何残留的定时器（防止上一次操作的定时器干扰新操作）
        self._stop_temp_timer()
        self._stop_process_timer()
        self._stop_timeout_timer()

        # 扩展 idle_timeout 并启动超时定时器（在准备退料之前）
        self._extend_idle_timeout()
        self._start_load_unload_timer(self.load_unload_timeout)
        # 准备退料
        self._prepare_unload()

        if index == Channel.F:
            # F通道直接退料（关闭最低挤出温度限制，允许冷挤出）
            self.gcode.run_script_from_command("SET_MIN_EXTRUDE_TEMP HEATER=extruder TEMP=0")
            self.load_step = State.UNLOADING
            gcmd.respond_info(f"[{channel_name} UNLOAD] STATE:51 UNLOADING")
            self.gcode.run_script_from_command("M2800")  # 切断纤维
            self._start_process_timer(0.1)
        else:
            # P1/P2需要预热
            if target > min_temp:
                self.gcode.run_script_from_command(f"M104 T{nozzle} S{target}")
                self.load_step = State.PREHEATING_UNLOAD
                gcmd.respond_info(f"[{channel_name} UNLOAD] STATE:50 PREHEATING_UNLOAD T:{target}")
                self._start_temp_timer(1.0)
            else:
                gcmd.respond_info(f"[{channel_name} UNLOAD] ERROR: Temperature {target}C too low")

    cmd_CONFIRM_EXTRUDED_help = "确认已出料（跳转到清理阶段）"
    def cmd_CONFIRM_EXTRUDED(self, gcmd):
        """
        用户确认已出料命令

        功能：用户确认耗材已从喷嘴挤出，跳转到清理阶段

        调用时机：在熔融挤出阶段(22)或补料确认阶段(30)都可以调用

        状态转换:
        - F/P1通道：降温 -> STATE:40等待用户清理 -> CONFIRM_CLEANED -> STATE:41自动擦嘴 -> STATE:90完成
        - P2通道：直接进入STATE:41自动擦嘴 -> STATE:90完成
        """
        load_step = self.load_step
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)

        if load_step not in [State.MELT_EXTRUDING, State.CONFIRM_EXTRUSION, State.RETRYING]:
            gcmd.respond_info(f"[{channel_name} LOAD] STATE:99 ERROR Invalid state for confirmation")
            return


        # 设置用户确认标志，让挤出循环自动停止
        self.user_confirmed = 1

        # 停止核心循环逻辑
        self._stop_process_timer()

        if index == Channel.F:
            # F 通道：切断耗材纤维，进入阶段 40（降温、开风扇、V 进挤在 CONFIRM_CLEANED 执行）
            self.gcode.run_script_from_command("M2800")
            self.load_step = State.WAIT_USER_CLEAN
            self.user_confirmed = 0
            gcmd.respond_info(f"[{channel_name} LOAD] STATE:40 WAIT_USER_CLEAN")
            gcmd.respond_info(f"ACTION:PROMPT_CONFIRM {channel_name}_CLEANED")
        else:
            # P2 通道直接结束逻辑（通常直接进入自动擦嘴流程）
            self._finish_loadunload()

    cmd_CONTINUE_LOAD_help = "继续补料（用户确认未出料）"
    def cmd_CONTINUE_LOAD(self, gcmd):
        """
        继续补料命令

        功能：用户确认未出料，进入补料重试流程

        调用时机：在STATE:30确认阶段，用户选择"继续补料"

        状态转换: STATE:30 -> STATE:31补料重试(50mm) -> STATE:30再次确认
        """
        load_step = self.load_step
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)

        # 检查是否在确认阶段
        if load_step != State.CONFIRM_EXTRUSION:
            gcmd.respond_info(f"[{channel_name} LOAD] STATE:99 ERROR Invalid state")
            return

        # 进入补料重试阶段
        self.load_step = State.RETRYING
        self.load_distance = 0.0
        gcmd.respond_info(f"[{channel_name} LOAD] STATE:31 RETRYING")
        self._start_process_timer(0.1)

    cmd_CONFIRM_CLEANED_help = "确认已清理完成（F/P1通道）"
    def cmd_CONFIRM_CLEANED(self, gcmd):
        """
        确认已清理完成命令

        功能：用户确认手动清理喷嘴完成，进入自动擦嘴流程

        调用时机：在STATE:40等待用户清理阶段，用户完成手动清理后调用

        状态转换: STATE:40 -> STATE:41自动擦嘴 -> STATE:90完成
        """
        index = self.load_unload_index
        load_step = self.load_step
        channel_name = self._get_channel_name(index)

        # 检查是否在等待清理状态
        if (index == Channel.F or index == Channel.P1) and load_step == State.WAIT_USER_CLEAN:
            if index == Channel.F:
                # F 通道：用户确认清理完成后，执行 V 进挤 2mm、降温、开风扇，再进入自动擦嘴
                self.gcode.run_script_from_command("M83")
                self.gcode.run_script_from_command("AE2")
                self.gcode.run_script_from_command("G1 V2 F100")
                nozzle = self.load_unload_nozzle
                target = self.load_unload_temperature
                cooling_temp = target - self.temperature_offset
                self.gcode.run_script_from_command("M106 P1 S255")
                self.gcode.run_script_from_command("M106 P2 S255")
                self.gcode.run_script_from_command(f"M104 T{nozzle} S{cooling_temp}")
            self._finish_loadunload()
        else:
            gcmd.respond_info(f"[{channel_name} LOAD] STATE:99 ERROR Invalid state")

    def _exit_loadunload_on_timeout(self):
        """
        超时退出进出料流程（内部方法，不输出状态码）

        功能：超时时退出当前进料/退料流程

        执行操作:
        1. 停止所有定时循环
        2. 重置状态变量
        3. 恢复最低挤出温度限制（F通道）
        4. 关闭加热器
        5. 关闭风扇
        """
        index = self.load_unload_index

        # 停止所有定时循环
        self._stop_process_timer()
        self._stop_temp_timer()
        self._stop_timeout_timer()
        self._stop_load_unload_timer()
        self._restore_idle_timeout()

        # 停止P1堵料检测（如果已启动）
        self.printer.send_event("filament_clog:check_stop")
        self.gcode.respond_info("filament_clog:check_stop")

        # 重置状态变量
        self.operation_mode = OperationMode.IDLE
        self.load_step = State.IDLE
        self.user_confirmed = 0

        # F通道需要恢复最低挤出温度限制
        if index == Channel.F:
            self.gcode.run_script_from_command("SET_MIN_EXTRUDE_TEMP HEATER=extruder TEMP=180")

        # 关闭加热器
        self.gcode.run_script_from_command("M104 T0 S0")
        self.gcode.run_script_from_command("M104 T1 S0")

        # 关闭风扇（所有通道）
        self.gcode.run_script_from_command("M106 P1 S0")
        self.gcode.run_script_from_command("M106 P2 S0")

        self._sync_print_stats_load_unload("error", "Load/unload timeout")

    cmd_EXIT_LOADUNLOAD_help = "退出进出料流程"
    def cmd_EXIT_LOADUNLOAD(self, gcmd):
        """
        退出进出料流程命令

        功能：用户主动退出当前进料/退料流程

        执行操作:
        1. 停止所有定时循环
        2. 重置状态变量
        3. 恢复最低挤出温度限制（F通道）
        4. 关闭加热器
        5. 输出STATE:91已取消状态
        """
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)
        action = 'LOAD' if self.operation_mode == OperationMode.LOADING else 'UNLOAD'

        # 停止所有定时循环
        self._stop_process_timer()
        self._stop_temp_timer()
        self._stop_timeout_timer()  # 停止超时定时器
        self._stop_load_unload_timer()  # 停止进退料超时定时器
        self._restore_idle_timeout()  # 恢复 idle_timeout

        # 停止P1堵料检测（如果已启动）
        self.printer.send_event("filament_clog:check_stop")
        self.gcode.respond_info("filament_clog:check_stop")

        # 重置状态变量
        self.operation_mode = OperationMode.IDLE
        self.load_step = State.IDLE
        self.user_confirmed = 0

        # F通道需要恢复最低挤出温度限制
        if index == Channel.F:
            self.gcode.run_script_from_command("SET_MIN_EXTRUDE_TEMP HEATER=extruder TEMP=180")

        # 关闭加热器
        self.gcode.run_script_from_command("M104 T0 S0")
        self.gcode.run_script_from_command("M104 T1 S0")

        # 关闭风扇（所有通道）
        self.gcode.run_script_from_command("M106 P1 S0")
        self.gcode.run_script_from_command("M106 P2 S0")

        self._sync_print_stats_load_unload("standby")
        gcmd.respond_info(f"[{channel_name} {action}] STATE:91 CANCELLED")

    cmd_PREPARE_LOAD_UNLOAD_NOZZLE_help = "准备进出料喷嘴"
    def cmd_PREPARE_LOAD_UNLOAD_NOZZLE(self, gcmd):
        """
        准备进出料喷嘴命令

        功能：根据通道索引激活对应的挤出机

        挤出机映射:
        - F(0)  -> extruder  (U轴)
        - P1(1) -> extruder2 (V轴)
        - P2(2) -> extruder1 (E轴)

        这个映射确保每个通道使用正确的挤出机轴
        """
        logger.debug("[PREPARE_LOAD_UNLOAD_NOZZLE] Starting")
        index = self.load_unload_index
        extruder_map = {
            Channel.F: 'extruder',
            Channel.P1: 'extruder2',
            Channel.P2: 'extruder1'
        }
        extruder_name = extruder_map.get(index, 'extruder')
        logger.debug(f"[PREPARE_LOAD_UNLOAD_NOZZLE] Channel index={index}, extruder_name={extruder_name}")

        # 检查挤出机是否存在
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None:
            logger.error(f"[PREPARE_LOAD_UNLOAD_NOZZLE] Extruder '{extruder_name}' not found")
            gcmd.respond_info(f"ERROR: Extruder '{extruder_name}' not configured")
            return
        logger.debug(f"[PREPARE_LOAD_UNLOAD_NOZZLE] Found extruder '{extruder_name}'")

        # 使用 toolhead 直接设置挤出机，避免调用可能失败的 G-code 命令
        toolhead = self.printer.lookup_object('toolhead')
        current_extruder = toolhead.get_extruder()
        logger.debug(f"[PREPARE_LOAD_UNLOAD_NOZZLE] Current extruder: {current_extruder.name if current_extruder else 'None'}")
        if current_extruder is not extruder:
            logger.debug(f"[PREPARE_LOAD_UNLOAD_NOZZLE] Switching from {current_extruder.name if current_extruder else 'None'} to {extruder_name}")
            toolhead.flush_step_generation()
            toolhead.set_extruder(extruder, extruder.last_position)
            self.printer.send_event("extruder:activate_extruder")
            gcmd.respond_info(f"Activating extruder {extruder_name}")
        else:
            logger.debug(f"[PREPARE_LOAD_UNLOAD_NOZZLE] Extruder {extruder_name} already active")
        logger.debug("[PREPARE_LOAD_UNLOAD_NOZZLE] Completed")

    cmd_FINISH_LOADUNLOAD_help = "手动完成进出料流程"
    def cmd_FINISH_LOADUNLOAD(self, gcmd):
        """
        手动完成进出料流程命令

        功能：用户主动完成当前的进料/退料流程

        执行操作：
        1. 停止所有定时循环
        2. 调用_finish_loadunload()完成流程
        """
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)
        action = 'LOAD' if self.operation_mode == OperationMode.LOADING else 'UNLOAD'

        # 检查是否有正在进行的流程
        if self.operation_mode == OperationMode.IDLE:
            gcmd.respond_info(f"[{channel_name} {action}] ERROR: No operation in progress")
            return

        # 停止所有定时循环（与 EXIT_LOADUNLOAD 一致）
        self._stop_process_timer()
        self._stop_temp_timer()
        self._stop_timeout_timer()
        self._stop_load_unload_timer()
        self._restore_idle_timeout()

        gcmd.respond_info(f"[{channel_name} {action}] User manually finished the process")

        # 完成流程
        self._finish_loadunload()

    # ============================================================
    # 定时器管理方法
    # ============================================================

    def _start_process_timer(self, duration):
        """启动处理定时器（用于进料/退料核心循环）"""
        if self.process_timer is None:
            self.process_timer = self.reactor.register_timer(
                self._process_timer_handler, self.reactor.NOW
            )
        self.reactor.update_timer(self.process_timer, self.reactor.monotonic() + duration)

    def _stop_process_timer(self):
        """停止处理定时器"""
        if self.process_timer is not None:
            self.reactor.unregister_timer(self.process_timer)
            self.process_timer = None

    def _start_temp_timer(self, duration):
        """启动温度等待定时器（用于温度达标检测）"""
        self.temp_timer = self.reactor.register_timer(
            self._temp_timer_handler, self.reactor.NOW
        )
        self.reactor.update_timer(self.temp_timer, self.reactor.monotonic() + duration)

    def _stop_temp_timer(self):
        """停止温度等待定时器"""
        if self.temp_timer is not None:
            self.reactor.unregister_timer(self.temp_timer)
            self.temp_timer = None

    def _start_timeout_timer(self, duration):
        """启动超时定时器"""
        self.p1_timeout_timer = self.reactor.register_timer(
            self._timeout_timer_handler, self.reactor.NOW
        )
        self.reactor.update_timer(self.p1_timeout_timer, self.reactor.monotonic() + duration)

    def _stop_timeout_timer(self):
        """停止超时定时器"""
        if self.p1_timeout_timer is not None:
            self.reactor.unregister_timer(self.p1_timeout_timer)
            self.p1_timeout_timer = None

    def _start_load_unload_timer(self, delay):
        """
        启动进退料超时定时器

        参数:
            delay: 超时时间（秒）
        """
        self._stop_load_unload_timer()  # 先停止现有定时器
        self.load_unload_timer = self.reactor.register_timer(
            self._load_unload_timeout_handler, self.reactor.NOW
        )
        self.reactor.update_timer(self.load_unload_timer, self.reactor.monotonic() + delay)
        logger.debug(f"[_start_load_unload_timer] Started {delay}s timeout")

    def _stop_load_unload_timer(self):
        """停止进退料超时定时器"""
        if self.load_unload_timer is not None:
            self.reactor.unregister_timer(self.load_unload_timer)
            self.load_unload_timer = None
            logger.debug("[_stop_load_unload_timer] Stopped")

    def _extend_idle_timeout(self):
        """
        扩展 idle_timeout 超时时间，防止进退料期间触发空闲超时

        功能：
        1. 保存当前 idle_timeout 的超时时间
        2. 设置新的超时时间为 load_unload_idle_timeout_extension
        3. 如果处于 Ready 状态，更新定时器
        """
        try:
            idle_timeout = self.printer.lookup_object('idle_timeout')
            # 保存原始超时时间
            if self.saved_idle_timeout is None:
                self.saved_idle_timeout = idle_timeout.idle_timeout
            # 设置扩展超时时间
            idle_timeout.idle_timeout = self.load_unload_idle_timeout_extension
            logger.info(f"[_extend_idle_timeout] Extended idle_timeout to {self.load_unload_idle_timeout_extension}s")
            # 如果当前处于 Ready 状态，更新定时器以确保新超时时间生效
            if hasattr(idle_timeout, 'state') and idle_timeout.state == "Ready":
                checktime = self.reactor.monotonic() + self.load_unload_idle_timeout_extension
                self.reactor.update_timer(idle_timeout.timeout_timer, checktime)
                logger.debug(f"[_extend_idle_timeout] Updated idle_timeout timer")
        except Exception as e:
            logger.warning(f"[_extend_idle_timeout] Failed to extend idle_timeout: {e}")

    def _restore_idle_timeout(self):
        """
        恢复 idle_timeout 原始超时时间

        功能：
        1. 检查是否保存了原始超时时间
        2. 恢复原始超时时间
        3. 清除保存的值
        """
        if self.saved_idle_timeout is not None:
            try:
                idle_timeout = self.printer.lookup_object('idle_timeout')
                idle_timeout.idle_timeout = self.saved_idle_timeout
                logger.info(f"[_restore_idle_timeout] Restored idle_timeout to {self.saved_idle_timeout}s")
            except Exception as e:
                logger.warning(f"[_restore_idle_timeout] Failed to restore idle_timeout: {e}")
            finally:
                self.saved_idle_timeout = None

    # ============================================================
    # 定时器处理函数
    # ============================================================

    def _temp_timer_handler(self, eventtime):
        """
        温度等待定时器处理函数

        功能：统一处理所有通道的温度等待逻辑

        处理流程:
        1. 检查当前温度是否达到目标温度
        2. 未达标：继续等待，每秒输出加热进度
        3. 达标后：根据通道类型和L参数进行状态转换
           - F/P1 L=1: 直接进入STATE:22熔融挤出
           - F/P1 L=0: 进入STATE:11等待插入耗材
           - P2: 进入STATE:11等待插入耗材

        进料模式温度达标后启动核心处理定时器
        退料模式温度达标后启动核心处理定时器（P1/P2需要预回抽）
        """
        logger.debug("[_temp_timer_handler] Called")
        if self.operation_mode == OperationMode.IDLE:
            logger.debug("[_temp_timer_handler] Operation mode is IDLE, stopping timer")
            return self.reactor.NEVER

        target = self.load_unload_temperature
        nozzle = self.load_unload_nozzle
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)
        action = 'LOAD' if self.operation_mode == OperationMode.LOADING else 'UNLOAD'

        logger.debug(f"[_temp_timer_handler] {channel_name} {action}: target={target}, nozzle={nozzle}, index={index}")

        # 获取当前温度
        extruder_name = 'extruder' if nozzle == 0 else 'extruder1'
        logger.debug(f"[_temp_timer_handler] Looking up extruder: {extruder_name}")
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None:
            logger.error(f"[_temp_timer_handler] Extruder '{extruder_name}' not found")
            self.gcode.respond_info(f"[{channel_name} {action}] ERROR: Extruder '{extruder_name}' not found")
            self._stop_temp_timer()
            return self.reactor.NEVER
        temperature = extruder.heater.get_temp(eventtime)[0]

        # 温度未达标或需要冷却，继续等待
        if temperature < target - 5:
            # 温度过低，需要加热
            logger.debug(f"[_temp_timer_handler] Temperature {temperature:.0f}/{target}C, continuing to wait")
            self.gcode.respond_info(f"[{channel_name} {action}] HEATING {temperature:.0f}/{target}C")
            return eventtime + 1.0
        elif temperature > target + 5:
            # 温度过高，需要冷却
            logger.debug(f"[_temp_timer_handler] Temperature {temperature:.0f}/{target}C, cooling down")
            self.gcode.respond_info(f"[{channel_name} {action}] COOLING {temperature:.0f}/{target}C")
            return eventtime + 1.0
        # 温度在目标温度±5°C范围内，认为达标

        # 温度达标，状态转换
        self.load_distance = 0.0

        if self.operation_mode == OperationMode.LOADING:
            # 进料模式：温度达标后的状态转换，根据通道类型和L参数决定下一步
            # P1通道已经在进料阶段（CLOG_DETECTION/SLOW_EXTRUSION/CLOG_RETRACTED），跳过状态转换
            # 这些状态使用process_timer直接处理温度，不通过temp_timer
            if index == Channel.P1 and (self.load_step == State.CLOG_DETECTION or
                                         self.load_step == State.SLOW_EXTRUSION or
                                         self.load_step == State.CLOG_RETRACTED):
                return self.reactor.NEVER

            if index == Channel.F:
                # F通道：温度达标，根据 L 参数决定下一步
                was_loaded = self.was_loaded_parameter
                if was_loaded == 1:
                    # L=1: 直接进入熔融挤出（耗材已在喷嘴中）
                    self.gcode.run_script_from_command("G1 Y100 F4000")
                    self.load_step = State.MELT_EXTRUDING
                    self.gcode.respond_info(f"[{channel_name} LOAD] STATE:22 MELT_EXTRUDING (L=1)")
                else:
                    # L=0: 进入等待插入状态（需要等待传感器检测）
                    self.gcode.run_script_from_command("G1 Y100 F4000")
                    self.load_step = State.WAIT_INSERT
                    self.gcode.respond_info(f"[{channel_name} LOAD] STATE:11 WAIT_INSERT (L=0)")
            else:
                # P2: 进入等待插入状态
                self.load_step = State.WAIT_INSERT
                self.gcode.respond_info(f"[{channel_name} {action}] STATE:11 WAIT_INSERT")

            # 启动核心处理定时器
            self._start_process_timer(1.0)

        elif self.operation_mode == OperationMode.UNLOADING:
            # 退料模式：P1/P2使用WAITING_TEMPERATURE等待温度，F不使用
            # T1喷嘴或P1通道需要预回抽
            if nozzle == 1 or index == Channel.P1:
                self._preloading_retract_filament()
                # 预回抽完成后，进入等待状态（3秒后才进入UNLOADING）
                self.load_step = State.PREHEATING_UNLOAD_WAIT
                self.gcode.respond_info(f"[{channel_name} {action}] STATE:52 PREHEATING_UNLOAD_WAIT")
                # 启动核心处理定时器，立即执行（0.1秒后开始等待3秒）
                self._start_process_timer(0.1)
            else:
                # T0喷嘴且非P1通道，直接进入UNLOADING状态
                self.load_step = State.UNLOADING
                self.gcode.respond_info(f"[{channel_name} {action}] STATE:51 UNLOADING")
                # 启动核心处理定时器
                self._start_process_timer(1.0)

        return self.reactor.NEVER

    def _timeout_timer_handler(self, eventtime):
        """P1慢速挤出超时定时器处理函数 - 2分钟无堵料检测进入ERROR"""
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)

        if index != Channel.P1:
            return self.reactor.NEVER

        # 新增：检查是否已经完成或错误，避免重复执行
        if self.operation_mode == OperationMode.IDLE or self.load_step in [State.FINISHED, State.ERROR, State.CLOG_RETRACTED]:
            logger.debug(f"[TIMEOUT] Already in terminal state {self.load_step}, skipping")
            return self.reactor.NEVER

        # 新增：先设置堵料检测标志，阻止 CLOG_RETRACTED 执行
        # 必须在停止定时器和设置状态之前设置此标志
        self.p1_clog_detected = True

        # 停止所有定时器和检测
        self._stop_process_timer()
        self._stop_timeout_timer()
        self.printer.send_event("filament_clog:check_stop")
        self.gcode.respond_info("filament_clog:check_stop")

        # 设置操作模式为IDLE，阻止其他操作（必须在设置load_step之前）
        self.operation_mode = OperationMode.IDLE
        self.load_step = State.ERROR
        self._sync_print_stats_load_unload("error",
            f"P1 slow extrusion timeout ({self.p1_slow_extrusion_timeout}s)")
        self.gcode.respond_info(f"[{channel_name} LOAD] STATE:99 ERROR (Timeout: {self.p1_slow_extrusion_timeout}s)")

        # P1 通道特有：关闭温度和风扇，取消注册堵料检测事件
        nozzle = self.load_unload_nozzle
        self.gcode.run_script_from_command(f"M104 T{nozzle} S0")  # 关闭对应喷嘴温度
        self.gcode.run_script_from_command("M106 P1 S0")  # 关闭侧吹/喉管风扇
        self.gcode.run_script_from_command("M106 P2 S0")

        return self.reactor.NEVER

    def _load_unload_timeout_handler(self, eventtime):
        """
        进退料超时处理函数

        功能：
        1. 发送超时状态码 98
        2. 根据当前操作模式执行不同处理：
           - P2 进料：先自动清理喷嘴，再调用退出流程
           - F/P1 进料：直接调用退出流程
           - 退料：直接调用退出流程
        """
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)
        is_loading = self.operation_mode == OperationMode.LOADING

        logger.warning(f"[_load_unload_timeout] Load/unload timeout for {channel_name}")

        # 发送超时状态码
        self.load_step = State.TIMEOUT
        action = 'LOAD' if is_loading else 'UNLOAD'
        self._send_state_code(98, 'TIMEOUT', channel_name, action)

        if is_loading and index == Channel.P2:
            # P2 进料超时：先执行自动清理喷嘴
            self.gcode.respond_info(f"[{channel_name} LOAD] Timeout - Starting auto clean")
            self._clean_nozzle()

        # 调用超时退出流程（不输出状态码）
        self._exit_loadunload_on_timeout()

        # 恢复超时状态
        self.load_step = State.TIMEOUT

        return self.reactor.NEVER

    def _process_timer_handler(self, eventtime):
        """
        核心处理定时器

        功能：处理进料/退料的核心循环逻辑

        路由分发:
        - 进料模式：调用_process_load()，根据通道索引路由到F/P1/P2处理函数
        - 退料模式：调用_process_unload()，统一处理所有通道的退料逻辑
        - 空闲模式：返回NEVER停止定时器
        """
        if self.operation_mode == OperationMode.IDLE:
            return self.reactor.NEVER

        # 设置相对坐标模式（所有进料/退料操作都需要）
        self.gcode.run_script_from_command("M83")

        if self.operation_mode == OperationMode.UNLOADING:
            return self._process_unload(eventtime)
        elif self.operation_mode == OperationMode.LOADING:
            return self._process_load(eventtime)

        return self.reactor.NEVER

    # ============================================================
    # 辅助方法
    # ============================================================

    def _sync_print_stats_load_unload(self, end_state, error_message=""):
        """
        同步 print_stats 状态，与状态机设计一致
        end_state: 'load_unload' | 'standby' | 'error'
        """
        try:
            print_stats = self.printer.lookup_object('print_stats')
            if end_state == "load_unload":
                self.gcode.run_script_from_command("SET_PRINT_LOAD_UNLOAD_STATE")
            elif end_state == "standby":
                self.gcode.run_script_from_command("SET_PRINT_STATS_INFO STATE=standby")
            elif end_state == "error":
                print_stats.state = "error"
                print_stats.error_message = error_message
                print_stats.prep_stage = None
                print_stats._stop_busy_timer()
        except Exception as e:
            logger.debug(f"[_sync_print_stats_load_unload] {e}")

    def _send_state_code(self, state_code, state_name, channel_name=None, action='LOAD'):
        """
        发送状态码给上位机（仅在状态变化时发送）

        参数:
        state_code: 状态码（如 20, 21, 90 等）
        state_name: 状态名称（如 LOADING, SLOW_EXTRUSION, FINISHED 等）
        channel_name: 通道名称（可选，默认使用当前通道）
        action: 操作类型（LOAD 或 UNLOAD）
        """
        if self.last_reported_state != state_code:
            if channel_name is None:
                channel_name = self._get_channel_name()
            self.gcode.respond_info(f"[{channel_name} {action}] STATE:{state_code} {state_name}")
            self.last_reported_state = state_code

    # ============================================================
    # 堵料检测事件处理
    # ============================================================

    def _handle_filament_clog(self, extruder_name):
        """
        堵料检测事件处理函数（耗材到达检测）

        功能：当filament_motion_sensor检测到耗材停止运动时触发
        说明：耗材停止运动 = 已到达喷头 = 检测成功

        触发条件：
        - 在P1通道SLOW_EXTRUSION状态(21)慢速挤出阶段，传感器检测到耗材停止运动
        - 注意：快速进料阶段(CLOG_DETECTION)不进行堵料检测

        处理流程：
        1. 验证当前是否在P1通道且处于慢速挤出状态
        2. 停止当前进料循环和超时定时器
        3. 标记检测成功状态
        4. 发送停止堵料检测信号
        5. 状态转换为CLOG_RETRACTED(25)，由状态机处理回退和前进操作
        """
        index = self.load_unload_index
        load_step = self.load_step
        channel_name = self._get_channel_name(index)

        # 只处理P1通道的堵料事件
        if index != Channel.P1:
            logger.debug(f"[_handle_filament_clog] Ignoring clog event for channel {channel_name}")
            return

        # 只在慢速挤出阶段才检测堵料
        if load_step != State.SLOW_EXTRUSION_CHECK_CLOG:
            logger.debug(f"[_handle_filament_clog] Not in slow extrusion state, current state={load_step}")
            return
        
        # 做一个最小长度限制
        if self.p1_slow_extrusion_distance < self.p1_clog_detection_min_length:
            logger.debug(f"[_handle_filament_clog] Extruded distance {self.p1_slow_extrusion_distance:.1f}mm is less than minimum threshold, ignoring clog event")
            return

        # 新增：检查是否已经超时或完成，避免在超时后继续执行
        if self.operation_mode == OperationMode.IDLE or self.load_step in [State.FINISHED, State.ERROR, State.CLOG_RETRACTED]:
            logger.debug(f"[_handle_filament_clog] Already in terminal state, skipping. mode={self.operation_mode}, step={self.load_step}")
            return

        logger.info(f"[_handle_filament_clog] Filament reached nozzle in {channel_name} channel, extruder={extruder_name}")

        # 首先停止定时器，防止新的运动命令被发送
        self._stop_process_timer()
        self._stop_timeout_timer()
        self._stop_temp_timer()  # 停止温度定时器

        # 标记检测成功（必须在状态转换之前设置，防止SLOW_EXTRUSION继续发送运动命令）
        self.p1_clog_detected = True

        # 重置load_distance（用于CLOG_RETRACTED状态的首次进入判断）
        self.load_distance = 0.0

        # 发送停止堵料检测信号
        self.printer.send_event("filament_clog:check_stop")
        self.gcode.respond_info("filament_clog:check_stop")

        # 状态转换：耗材到达检测成功，进入回退状态
        self.load_step = State.CLOG_RETRACTED

        # 重新启动处理定时器进入下一阶段
        self._start_process_timer(0.1)

    # ============================================================
    # 进料处理方法
    # ============================================================

    def _process_load(self, eventtime):
        """
        进料处理路由

        功能：根据通道索引路由到对应的通道处理函数
        """
        if self.operation_mode != OperationMode.LOADING:
            return self.reactor.NEVER

        index = self.load_unload_index

        if index == Channel.P1:
            return self._process_p1_load(eventtime)
        elif index == Channel.P2:
            return self._process_p2_load(eventtime)
        else:
            return self._process_f_load(eventtime)

    def _process_f_load(self, eventtime):
        """
        F通道进料处理

        流程说明:
        1. STATE:11 等待插入耗材 - 检测传感器，检测到后进入STATE:20
        2. STATE:20 快速进料 - 进料1100mm @ 30mm/s，完成后进入STATE:22
        3. STATE:22 熔融挤出 - 挤出250mm @ 5mm/s，用户可随时确认跳过
        4. STATE:31 补料重试 - 用户确认未出料时，补料50mm @ 5mm/s

        F通道特点:
        - 使用U轴进行挤出
        - 需要纤维切割器(M2800)
        - 长距离快速进料(1100mm)
        """
        load_step = self.load_step
        channel_name = 'F'

        # ------ 阶段11：等待用户插入耗材 ------
        if load_step == State.WAIT_INSERT:
            filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor', None)
            sensor_status = filament_sensor.get_status(eventtime) if filament_sensor else {}
            if filament_sensor and sensor_status.get('filament_detected', False):
                # 检测到耗材，进入快速进料阶段
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:12 DETECTED")
                self.load_step = State.FAST_LOADING
                self.load_distance = 0.0
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:20 FAST_LOADING")
                return eventtime + 0.1
            # 未检测到耗材，继续等待
            return eventtime + 0.1

        # ------ 阶段20：F远端快速进料 1100mm (30mm/s) ------
        elif load_step == State.FAST_LOADING:
            step_length = self.f_step_length  # 30mm每步，30mm/s速度 = 1秒/步
            self.gcode.run_script_from_command("M83")  # 设置相对坐标模式
            self.gcode.run_script_from_command(f"G1 U{step_length} F{step_length * 60}")  # U轴进料
            self.load_distance += step_length
            current_distance = self.load_distance

            # 实时显示进度
            self.gcode.respond_info(f"[{channel_name} LOAD] PROGRESS {current_distance:.0f}/{self.f_long_distance:.0f}mm {step_length:.1f}mm/s")

            if current_distance >= self.f_long_distance:
                # 快速进料完成，直接进入熔融挤出
                self.gcode.run_script_from_command("G1 Y100 F4000")
                self.load_step = State.MELT_EXTRUDING
                self.load_distance = 0.0
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:22 MELT_EXTRUDING")
                return eventtime + 0.1

            # 继续进料
            return eventtime + 1.0

        # ------ 阶段22：熔融挤出 (5mm/s) ------
        elif load_step == State.MELT_EXTRUDING:
            # 检查用户是否已确认挤出（备用路径，F通道主流程在cmd_CONFIRM_EXTRUDED）
            if self.user_confirmed == 1:
                self.user_confirmed = 0
                self.load_step = State.WAIT_USER_CLEAN
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:40 WAIT_USER_CLEAN")
                self.gcode.respond_info("ACTION:PROMPT_CONFIRM F_CLEANED")
                return self.reactor.NEVER

            # 继续熔融挤出
            melt_step = self.f_melt_step_length  # 5mm每步，5mm/s速度 = 1秒/步
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(f"G1 U{melt_step} F{melt_step * 60}")
            self.load_distance += melt_step
            current_distance = self.load_distance

            # 实时显示进度
            self.gcode.respond_info(f"[{channel_name} LOAD] PROGRESS {current_distance:.0f}/{self.f_melt_distance:.0f}mm {melt_step:.1f}mm/s")

            if current_distance >= self.f_melt_distance:
                # 熔融挤出完成，等待用户确认
                self.load_step = State.CONFIRM_EXTRUSION
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:30 CONFIRM_EXTRUSION")
                self.gcode.respond_info("ACTION:PROMPT_CONFIRM F_EXTRUDED")
                return self.reactor.NEVER

            # 继续挤出
            return eventtime + 1.0

        # ------ 阶段31：用户确认未出料，补料重试 50mm ------
        elif load_step == State.RETRYING:
            retry_step = self.retry_step_length  # 5mm每步
            self.gcode.run_script_from_command(f"G1 U{retry_step} F{self.retry_speed}")
            self.load_distance += retry_step
            current_distance = self.load_distance

            # 显示补料进度
            self.gcode.respond_info(f"[{channel_name} LOAD] PROGRESS {current_distance:.0f}/{self.retry_distance:.0f}mm {self.retry_speed / 60:.1f}mm/s")

            if current_distance >= self.retry_distance:
                # 补料完成，再次等待确认
                self.load_step = State.CONFIRM_EXTRUSION
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:30 CONFIRM_EXTRUSION")
                self.gcode.respond_info("ACTION:PROMPT_CONFIRM F_EXTRUDED")
                return self.reactor.NEVER

            # 继续补料
            return eventtime + 1.0

        return self.reactor.NEVER

    def _process_p1_load(self, eventtime):
        """
        P1通道进料处理

        流程说明:
        阶段1 - 等待插入耗材 (STATE:11)
            输出: STATE:11 WAIT_INSERT
            后台: 等待filament_sensor_1检测到耗材

        阶段2 - 进料中 (内部STATE:24 CLOG_DETECTION, 21 SLOW_EXTRUSION)
            后台动作:
            1. 设置温度50°C并等待达标 (如果当前温度>55°C先冷却)
            2. L=0: 快速进料 @ 20mm/s (p1_long_distance配置，不进行堵料检测)
               L=1: 跳过快挤出，直接进入慢速挤出
            3. 慢速挤出2mm逐毫米 (内部STATE:21 SLOW_EXTRUSION，此时才启动堵料检测)
            4. 2分钟超时机制，超时后进入ERROR状态
            5. 实时监控耗材运动，检测到停止运动=到达喷头=成功

        阶段3 - 堵料回退完成 (内部STATE:25 CLOG_RETRACTED)
            输出: STATE:90 FINISHED
            后台动作: 回退5mm，前进3mm，完成流程

        P1通道特点:
        - 使用V轴进行挤出(P1耗材)
        - 支持耗材到达检测和自动回退
        - 2分钟超时机制
        - 堵料检测成功=耗材到达喷头，回退后完成
        """
        load_step = self.load_step
        channel_name = 'P1'

        # ============================================================
        # 阶段1: 等待插入耗材 (STATE:11 WAIT_INSERT)
        # ============================================================
        if load_step == State.WAIT_INSERT:
            filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor_1', None)
            sensor_status = filament_sensor.get_status(eventtime) if filament_sensor else {}
            if filament_sensor and sensor_status.get('filament_detected', False):
                # 检测到耗材，等待2s再进料
                self.load_step = State.WAIT_MOMENT_FOR_FAST_MOVE
                # 启动核心处理定时器开始进料
                self._start_process_timer(3.0)
                return eventtime + 3.0
            # 未检测到耗材，继续等待
            return eventtime + 0.1
        elif load_step == State.WAIT_MOMENT_FOR_FAST_MOVE:
            # 检测到耗材，进入阶段2：进料中
            self._send_state_code(12, 'DETECTED', channel_name)
            self.load_step = State.CLOG_DETECTION
            self.load_distance = 0.0
            # 启动核心处理定时器开始进料
            self._start_process_timer(0.1)
            return eventtime + 0.1
        # ============================================================
        # 阶段2: 进料中 (内部STATE:24 CLOG_DETECTION, 21 SLOW_EXTRUSION)
        # 后台: 50°C预热 + 堵料检测下的快速进料p1_long_distance (L=0) 或 跳过进入慢速挤出 (L=1)
        # ============================================================
        elif load_step == State.CLOG_DETECTION:
            self._send_state_code(20, 'LOADING', channel_name)

            was_loaded = self.was_loaded_parameter

            # 首次进入时设置50°C
            if not self.p1_clog_detection_initialized:
                nozzle = self.load_unload_nozzle
                extruder_name = 'extruder' if nozzle == 0 else 'extruder1'
                extruder = self.printer.lookup_object(extruder_name, None)
                current_temp = extruder.heater.get_temp(eventtime)[0] if extruder else 0

                # 设置50°C
                self.gcode.run_script_from_command(f"M104 T{nozzle} S50")
                self.p1_clog_detection_initialized = True  # 标记已初始化，允许快速进料开始

                # 检查当前温度，输出提示信息（但不阻塞快速进料）
                if current_temp > 55:  # 需要冷却
                    self.gcode.run_script_from_command("M106 P1 S255")
                    self.gcode.run_script_from_command("M106 P2 S255")
                    self.gcode.respond_info(f"[{channel_name} LOAD] Cooling to 50°C... {current_temp:.0f}C (Fans ON)")
                elif current_temp < 45:  # 需要加热
                    self.gcode.run_script_from_command("M106 P1 S0")
                    self.gcode.run_script_from_command("M106 P2 S0")
                    self.gcode.respond_info(f"[{channel_name} LOAD] Heating to 50°C... {current_temp:.0f}C")
                else:  # 温度在45-55°C范围内
                    self.gcode.run_script_from_command("M106 P1 S0")
                    self.gcode.run_script_from_command("M106 P2 S0")
                    self.gcode.respond_info(f"[{channel_name} LOAD] PREHEAT_LOW (50°C)")

            # 检查温度是否适合慢速挤出（45-55°C范围）
            temp_ready_for_slow = False
            if self.p1_clog_detection_initialized:
                nozzle = self.load_unload_nozzle
                extruder_name = 'extruder' if nozzle == 0 else 'extruder1'
                extruder = self.printer.lookup_object(extruder_name, None)
                if extruder:
                    current_temp = extruder.heater.get_temp(eventtime)[0]
                    temp_ready_for_slow = (45.0 <= current_temp <= 55.0)

            # L=1: 跳过快挤出，直接进入慢速挤出
            # 温度未达标时，先进入 PREHEAT_LOW 等待 50°C
            if was_loaded == 1 and not self.p1_slow_extrusion_entered:
                if not temp_ready_for_slow:
                    # 切换到 PREHEAT_LOW 状态，由专门分支只做温度等待和状态上报
                    self.load_step = State.PREHEAT_LOW
                    return eventtime + 1.0

                # 温度已达标，直接进入慢速挤出
                self.p1_slow_extrusion_entered = True
                self.load_step = State.SLOW_EXTRUSION
                self.p1_slow_extrusion_distance = 0.0
                self.p1_slow_extrusion_start_time = eventtime
                self._start_timeout_timer(self.p1_slow_extrusion_timeout)
                return eventtime + 0.1

            # L=0: 执行快速进料（与温度调节同步，不等待温度达标）
            # 如果已经进入慢速挤出，跳过快速进料代码
            if was_loaded == 0 and self.p1_clog_detection_initialized and not self.p1_slow_extrusion_entered:
                # 先检查当前距离，如果已达到目标则等待温度后进入慢速挤出
                if self.load_distance >= self.p1_long_distance:
                    # 当前距离已达标，但温度未必达标
                    if not temp_ready_for_slow:
                        # 切换到 PREHEAT_LOW 状态，在该状态中等待 50°C
                        self.load_step = State.PREHEAT_LOW
                        return eventtime + 1.0

                    # 温度已达标，进入慢速挤出（在真正转换时才设置标志）
                    self.gcode.run_script_from_command("M400")
                    self.p1_slow_extrusion_entered = True
                    self.load_step = State.SLOW_EXTRUSION
                    self.p1_slow_extrusion_distance = 0.0
                    self.p1_slow_extrusion_start_time = eventtime
                    self._start_timeout_timer(self.p1_slow_extrusion_timeout)
                    return eventtime + 0.1


                # 检查传感器状态，如果未检测到耗材，不执行进料
                filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor_1', None)
                sensor_status = filament_sensor.get_status(eventtime) if filament_sensor else {}
                if filament_sensor and not sensor_status.get('filament_detected', False):
                    self.gcode.respond_info(f"[{channel_name} LOAD] PAUSED: Filament removed")
                    return eventtime + 1.0
                # 距离未达标，执行快速进料
                # 计算本步需要挤出的长度（防止超出目标距离）
                remaining_distance = self.p1_long_distance - self.load_distance
                step_length = min(self.p1_step_length, remaining_distance)  # 使用配置的步长

                self.gcode.run_script_from_command("M83")
                self.gcode.run_script_from_command(f"G1 V{step_length} F{self.p1_fast_move_speed}")  # V轴进料
                self.load_distance += step_length
                current_distance = self.load_distance

                # 实时显示进度
                self.gcode.respond_info(f"[{channel_name} LOAD] FAST_EXTRUSION {current_distance:.0f}/{self.p1_long_distance:.0f}mm")

                # 计算本次移动所需时间，动态调整下一次进料的时间
                speed_mm_s = self.p1_fast_move_speed / 60.0
                move_duration = step_length / speed_mm_s if speed_mm_s > 0 else 1.0
                # 继续进料循环
                return eventtime + move_duration

        # ============================================================
        # 阶段2.5: 低温预热等待 50°C (STATE:23 PREHEAT_LOW)
        # 仅等待温度达标，不执行任何挤出动作
        # ============================================================
        elif load_step == State.PREHEAT_LOW:
            self._send_state_code(23, 'PREHEAT_LOW', channel_name)

            nozzle = self.load_unload_nozzle
            extruder_name = 'extruder' if nozzle == 0 else 'extruder1'
            extruder = self.printer.lookup_object(extruder_name, None)
            if extruder is None:
                self.gcode.respond_info(f"[{channel_name} LOAD] ERROR: Extruder '{extruder_name}' not found in PREHEAT_LOW")
                self.load_step = State.ERROR
                self._sync_print_stats_load_unload("error", f"Extruder '{extruder_name}' not found")
                return self.reactor.NEVER

            current_temp = extruder.heater.get_temp(eventtime)[0]

            # 温度反馈信息
            if current_temp < 45.0:
                self.gcode.respond_info(f"[{channel_name} LOAD] HEATING {current_temp:.0f}/50C")
                return eventtime + 1.0
            if current_temp > 55.0:
                self.gcode.respond_info(f"[{channel_name} LOAD] COOLING {current_temp:.0f}/50C")
                return eventtime + 1.0

            # 温度进入 45~55°C 区间，认为已达标，进入慢速挤出阶段
            self.gcode.respond_info(f"[{channel_name} LOAD] REACHED_50C {current_temp:.0f}/50C")
            if not self.p1_slow_extrusion_entered:
                self.p1_slow_extrusion_entered = True
                self.load_step = State.SLOW_EXTRUSION
                self.p1_slow_extrusion_distance = 0.0
                self.p1_slow_extrusion_start_time = eventtime
                self._start_timeout_timer(self.p1_slow_extrusion_timeout)
            return eventtime + 0.1

        # ============================================================
        # 阶段2续: 慢速逐2毫米挤出 (内部STATE:21 SLOW_EXTRUSION)
        # 后台: 2mm逐毫米挤出，2分钟超时检测堵料
        # ============================================================
        elif load_step == State.SLOW_EXTRUSION:
            # 检查是否已检测到堵料，如果是则跳过运动命令（防止电机继续转动）
            if self.p1_clog_detected:
                logger.debug("[SLOW_EXTRUSION] Clog detected, skipping motion commands")
                return self.reactor.NEVER

            self._send_state_code(21, 'SLOW_EXTRUSION', channel_name)

            # 移除重复的 M400：在 CLOG_DETECTION 切换状态时已经等待过了
            # 这里不需要再次等待，直接开始慢速挤出

            # 2mm逐毫米挤出
            step_length = 1.0
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(f"G1 V{step_length} F{self.p1_slow_move_speed}")  # V轴进料
            self.p1_slow_extrusion_distance += step_length

            toolhead = self.printer.lookup_object('toolhead')
            toolhead.wait_moves()

            elapsed_time = eventtime - self.p1_slow_extrusion_start_time
            self.gcode.respond_info(f"[{channel_name} LOAD] SLOW_EXTRUSION {self.p1_slow_extrusion_distance:.0f}mm ({elapsed_time:.0f}s)")

            if self.p1_slow_extrusion_distance >= 5.0:
                self.load_step = State.SLOW_EXTRUSION_CHECK_CLOG
                self.printer.send_event("filament_clog:check_start")
                self.gcode.respond_info(f"filament_clog:check_start:{self.count}")
                self.count += 1
                return eventtime + 5.0

            return eventtime + 1.0
        
        elif load_step == State.SLOW_EXTRUSION_CHECK_CLOG:
            # 检查是否已检测到堵料，如果是则跳过运动命令（防止电机继续转动）
            if self.p1_clog_detected:
                logger.debug("[SLOW_EXTRUSION_CHECK_CLOG] Clog detected, skipping motion commands")
                return self.reactor.NEVER

            self._send_state_code(22, 'SLOW_EXTRUSION_CHECK_CLOG', channel_name)

            # 移除重复的 M400：在 CLOG_DETECTION 切换状态时已经等待过了
            # 这里不需要再次等待，直接开始慢速挤出

            # 2mm逐毫米挤出
            step_length = self.p1_melt_step_length
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(f"G1 V{step_length} F{self.p1_slow_move_speed}")  # V轴进料
            self.p1_slow_extrusion_distance += step_length

            toolhead = self.printer.lookup_object('toolhead')
            toolhead.wait_moves()

            elapsed_time = eventtime - self.p1_slow_extrusion_start_time
            self.gcode.respond_info(f"[{channel_name} LOAD] SLOW_EXTRUSION_CHECK_CLOG {self.p1_slow_extrusion_distance:.0f}mm ({elapsed_time:.0f}s)")

            return eventtime + 2.0

        # ============================================================
        # 阶段3: 堵料回退完成 (内部STATE:25 CLOG_RETRACTED)
        # 后台: 堵料检测成功=耗材已到达喷头，回退后前进，然后完成
        # ============================================================
        elif load_step == State.CLOG_RETRACTED:
            # 防止重复执行：如果操作模式已经变为IDLE或已处理，说明已完成，直接返回
            if self.operation_mode == OperationMode.IDLE or self.p1_clog_extruded:
                return self.reactor.NEVER
            # 新增：检查是否已超时，如果是则不执行回退前进操作
            if self.load_step == State.ERROR:
                logger.debug("[CLOG_RETRACTED] Already in ERROR state, skipping retract/advance")
                return self.reactor.NEVER
            # 立即停止定时器，防止在执行过程中再次触发
            self._stop_process_timer()
            # 标记已处理，防止重复执行
            self.p1_clog_extruded = True

            # 回退5mm后再前进3mm
            self.gcode.run_script_from_command("M83")
            retract_distance = self.p1_clog_retract_distance
            self.gcode.run_script_from_command(f"G1 V-{retract_distance} F{self.retry_speed}")
            self.gcode.run_script_from_command("M400") 
            advance_distance = 3.0
            self.gcode.run_script_from_command(f"G1 V{advance_distance} F{self.retry_speed}")
            self.gcode.respond_info(f"[{channel_name} LOAD] CLOG_DETECTED - Retract {retract_distance}mm, advance {advance_distance}mm")

            # 完成流程
            self.operation_mode = OperationMode.IDLE
            self.load_step = State.FINISHED
            self._sync_print_stats_load_unload("standby")
            self._send_state_code(90, 'FINISHED', channel_name)

            # P1 通道特有：关闭温度和风扇，取消注册堵料检测事件
            nozzle = self.load_unload_nozzle
            self.gcode.run_script_from_command(f"M104 T{nozzle} S0")  # 关闭对应喷嘴温度
            self.gcode.run_script_from_command("M106 P1 S0")  # 关闭侧吹/喉管风扇
            self.gcode.run_script_from_command("M106 P2 S0")
            self._stop_load_unload_timer()
            self._restore_idle_timeout()

            return self.reactor.NEVER

        return self.reactor.NEVER

    def _process_p2_load(self, eventtime):
        """
        P2通道进料处理

        流程说明:
        1. STATE:11 等待插入耗材 - 检测传感器，检测到后直接进入STATE:22
        2. STATE:22 熔融挤出 - 挤出300mm @ 5mm/s（P2唯一进料阶段）
           - 支持用户随时确认直接完成
           - 如果传感器丢失（用户拔出），返回STATE:11
        3. STATE:31 补料重试 - 用户确认未出料时，补料50mm @ 5mm/s

        P2通道特点:
        - 使用E轴进行挤出
        - 无快速进料阶段，直接熔融挤出
        - 支持中途拔出检测
        - 完成后直接进入自动擦嘴，无需用户手动清理
        """
        load_step = self.load_step
        channel_name = 'P2'

        # ------ 阶段11：等待插入耗材 ------
        if load_step == State.WAIT_INSERT:
            self.gcode.respond_info(f"[{channel_name} LOAD] STATE:11 WAIT_INSERT")
            filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor_3', None)
            sensor_status = filament_sensor.get_status(eventtime) if filament_sensor else {}
            if filament_sensor and sensor_status.get('filament_detected', False):
                # 检测到耗材，直接进入熔融挤出阶段（P2无快速进料）
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:12 DETECTED")
                
                # 开启冷却风扇
                self.gcode.run_script_from_command("M106 P1 S255")
                self.gcode.run_script_from_command("M106 P2 S255")
                
                self.load_step = State.MELT_EXTRUDING
                self.load_distance = 0.0
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:22 MELT_EXTRUDING")
                return eventtime + 0.1
            # 未检测到耗材，继续等待
            return eventtime + 0.1

        # ------ 阶段22：熔融挤出 (P2 唯一进料阶段) ------
        elif load_step == State.MELT_EXTRUDING:
            filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor_3', None)

            # 如果传感器丢失（用户拔出），返回等待插入状态
            sensor_status = filament_sensor.get_status(eventtime) if filament_sensor else {}
            if filament_sensor and not sensor_status.get('filament_detected', False):
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:11 WAIT_INSERT")
                self.load_step = State.WAIT_INSERT
                return eventtime + 1.0

            # 检查用户是否已确认挤出
            if self.user_confirmed == 1:
                # 用户已确认，P2 直接进入自动擦嘴
                self.user_confirmed = 0
                self._finish_loadunload()
                return self.reactor.NEVER

            # 继续熔融挤出
            step_length = self.p2_step_length  # 5mm每步
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(f"G1 E{step_length} F{step_length * 60}")  # E轴进料
            self.load_distance += step_length
            current_distance = self.load_distance

            # 实时显示进度
            self.gcode.respond_info(f"[{channel_name} LOAD] PROGRESS {current_distance:.0f}/{self.p2_long_distance:.0f}mm {step_length:.1f}mm/s")

            if current_distance >= self.p2_long_distance:
                # 熔融挤出完成，等待用户确认
                self.load_step = State.CONFIRM_EXTRUSION
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:30 CONFIRM_EXTRUSION")
                self.gcode.respond_info("ACTION:PROMPT_CONFIRM P2_EXTRUDED")
                return self.reactor.NEVER

            # 继续挤出
            return eventtime + 1.0

        # ------ 阶段31：用户确认未出料，补料重试 50mm ------
        elif load_step == State.RETRYING:
            retry_step = self.retry_step_length
            self.gcode.run_script_from_command("M83")
            self.gcode.run_script_from_command(f"G1 E{retry_step} F{self.retry_speed}")
            self.load_distance += retry_step
            current_distance = self.load_distance

            # 显示补料进度
            self.gcode.respond_info(f"[{channel_name} LOAD] PROGRESS {current_distance:.0f}/{self.retry_distance:.0f}mm {self.retry_speed / 60:.1f}mm/s")

            if current_distance >= self.retry_distance:
                # 补料完成，再次等待确认
                self.load_step = State.CONFIRM_EXTRUSION
                self.gcode.respond_info(f"[{channel_name} LOAD] STATE:30 CONFIRM_EXTRUSION")
                self.gcode.respond_info("ACTION:PROMPT_CONFIRM P2_EXTRUDED")
                return self.reactor.NEVER

            # 继续补料
            return eventtime + 1.0

        return self.reactor.NEVER

    # ============================================================
    # 退料处理方法
    # ============================================================

    def _process_unload(self, eventtime):
        """
        退料处理

        功能：统一处理所有通道的退料逻辑

        流程说明:
        1. PREHEATING_UNLOAD_WAIT状态：等待3秒后进入UNLOADING状态
        2. UNLOADING状态：根据通道索引确定退料速度（F/P1: 30mm/s, P2: 16.67mm/s）
        3. 检查传感器状态
        4. 退料距离达到200mm后，检测传感器状态
           - 如果传感器检测到料已退出，完成流程
           - 否则继续退料
        5. 每50mm发送一次进度

        退料轴映射:
        - F:  U轴 (G1 U-xxx)
        - P1: V轴 (G1 V-xxx)
        - P2: E轴 (G1 E-xxx)
        """
        if self.operation_mode != OperationMode.UNLOADING:
            return self.reactor.NEVER

        index = self.load_unload_index
        channel_name = self._get_channel_name(index)
        load_step = self.load_step

        # ------ 阶段52：预回抽等待中（等待3秒后进入UNLOADING） ------
        if load_step == State.PREHEATING_UNLOAD_WAIT:
            self.gcode.respond_info(f"[{channel_name} UNLOAD] Waiting 3 seconds before UNLOADING...")
            # 等待3秒后进入UNLOADING状态
            self.load_step = State.UNLOADING
            self.gcode.respond_info(f"[{channel_name} UNLOAD] STATE:51 UNLOADING")
            return eventtime + 3.0

        # 确定退料速度
        if index == Channel.F or index == Channel.P1:
            unload_speed = self.f_p1_unload_speed  # 1800 = 30mm/s
        else:
            unload_speed = self.p2_unload_speed  # 1000 = 16.67mm/s

        # 检查传感器状态
        if index == Channel.F:
            filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor', None)
        elif index == Channel.P1:
            filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor_1', None)
        else:
            filament_sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor_3', None)

        sensor_status = filament_sensor.get_status(eventtime) if filament_sensor else {}
        filament_detected = sensor_status.get('filament_detected', False)

        # 退料距离达到200mm后，检测传感器状态
        # 如果传感器检测到料已退出，停止退料并完成流程
        if self.load_distance >= 200 and not filament_detected:
            self.gcode.respond_info(f"[{channel_name} UNLOAD] SENSOR_DETECTED: Filament removed")
            self._finish_loadunload()
            return self.reactor.NEVER

        # 继续退料动作
        step_length = unload_speed / 60  # 计算每步的进料量(mm)
        current_speed = unload_speed

        # 根据通道选择对应的轴进行退料
        if index == Channel.F:
            self.gcode.run_script_from_command(f"G1 U-{step_length} F{current_speed}")
        elif index == Channel.P1:
            self.gcode.run_script_from_command(f"G1 V-{step_length} F{current_speed}")
        else:
            self.gcode.run_script_from_command(f"G1 E-{step_length} F{current_speed}")

        self.load_distance += step_length

        # 每50mm发送一次进度
        if int(self.load_distance) % 50 == 0:
            self.gcode.respond_info(f"[{channel_name} UNLOAD] PROGRESS {self.load_distance:.0f}mm")

        # 继续退料循环
        return eventtime + 1.0

    # ============================================================
    # 辅助方法
    # ============================================================

    def _get_channel_name(self, index=None):
        """
        获取通道名称

        参数:
        index: 通道索引（可选，默认使用当前通道）

        返回:
        通道名称字符串 ('F', 'P1', 或 'P2')
        """
        if index is None:
            index = self.load_unload_index
        return 'F' if index == Channel.F else ('P1' if index == Channel.P1 else 'P2')

    def _prepare_load(self):
        """
        准备进料宏

        功能：
        1. 设置操作模式为进料
        2. 检查打印机状态，如果处于standby模式则归零坐标
        3. 根据喷嘴选择移动到预装载位置
        4. 调用PREPARE_LOAD_UNLOAD_NOZZLE激活对应挤出机
        """
        logger.debug("[_prepare_load] Starting")
        self.operation_mode = OperationMode.LOADING
        self.load_step = State.IDLE
        self.user_confirmed = 0

        self.gcode.respond_info("[LOAD FILAMENT] Checking printer status...")

        print_stats = self.printer.lookup_object('print_stats')
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.reactor.monotonic()

        logger.debug(f"[_prepare_load] print_stats.state={print_stats.state}")
        logger.debug("[_prepare_load] In standby mode, checking homed axes")
        # 如果未归零，先归零
        if "xyz" not in toolhead.get_status(eventtime)['homed_axes']:
            logger.debug("[_prepare_load] Axes not homed, running G28")
            self.gcode.run_script_from_command("G28")

        # 根据喷嘴选择移动到预装载位置
        nozzle = self.load_unload_nozzle
        logger.debug(f"[_prepare_load] nozzle={nozzle}, moving to preload position")
        if nozzle == 0:
            # T0左喷嘴
            logger.debug("[_prepare_load] Running T0, MOVE_TO_WASTE_STATION, MOVE_LEFT_PRELOAD_POSITION")
            self.gcode.run_script_from_command("T0")
            self.gcode.run_script_from_command("MOVE_TO_WASTE_STATION")
            self.gcode.run_script_from_command("MOVE_LEFT_PRELOAD_POSITION")
        else:
            # T1右喷嘴
            logger.debug("[_prepare_load] Running T1, MOVE_TO_WASTE_STATION, MOVE_RIGHT_PRELOAD_POSITION")
            self.gcode.run_script_from_command("T1")
            self.gcode.run_script_from_command("MOVE_TO_WASTE_STATION")
            self.gcode.run_script_from_command("MOVE_RIGHT_PRELOAD_POSITION")

        # 激活对应的挤出机
        logger.debug("[_prepare_load] Calling PREPARE_LOAD_UNLOAD_NOZZLE")
        self.gcode.run_script_from_command("PREPARE_LOAD_UNLOAD_NOZZLE")
        self._sync_print_stats_load_unload("load_unload")
        logger.debug("[_prepare_load] Completed")

    def _prepare_unload(self):
        """
        准备出料宏

        功能：
        1. 设置操作模式为退料
        2. 检查打印机状态，如果处于standby模式则归零坐标
        3. 根据喷嘴选择移动到预装载位置
        4. 调用PREPARE_LOAD_UNLOAD_NOZZLE激活对应挤出机
        """
        self.operation_mode = OperationMode.UNLOADING

        print_stats = self.printer.lookup_object('print_stats')
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.reactor.monotonic()

        # 只在standby模式下移动
        if print_stats.state == "standby":
            # 如果未归零，先归零
            if "xyz" not in toolhead.get_status(eventtime)['homed_axes']:
                self.gcode.run_script_from_command("G28")

            # 根据喷嘴选择移动到预装载位置
            nozzle = self.load_unload_nozzle
            if nozzle == 0:
                # T0左喷嘴
                self.gcode.run_script_from_command("MOVE_TO_WASTE_STATION")
                self.gcode.run_script_from_command("MOVE_LEFT_PRELOAD_POSITION")
            else:
                # T1右喷嘴
                self.gcode.run_script_from_command("MOVE_TO_WASTE_STATION")
                self.gcode.run_script_from_command("MOVE_RIGHT_PRELOAD_POSITION")

        # 激活对应的挤出机
        self.gcode.run_script_from_command("PREPARE_LOAD_UNLOAD_NOZZLE")
        self._sync_print_stats_load_unload("load_unload")

    def _preloading_retract_filament(self):
        """
        预挤出回抽耗材

        功能：在进料/退料前进行预挤出和回抽操作

        操作流程:
        1. 根据操作模式选择对应的参数
        2. G92 E0: 重置挤出机位置
        3. 预挤出一小段耗材(20mm)
        4. G92 E0: 再次重置挤出机位置
        5. 回抽一段耗材(进料-8mm, 退料-40mm)
        6. 退料模式额外等待3秒

        参数:
        - 进料模式: 预挤出20mm@300mm/min, 回抽-8mm@800mm/min
        - 退料模式: 预挤出20mm@100mm/min, 回抽-40mm@300mm/min
        """
        if self.operation_mode == OperationMode.LOADING:
            # 进料模式参数
            preloading_filament_e = self.load_preloading_filament_e
            preloading_filament_speed = self.load_preloading_filament_speed
            retract_speed = self.load_clean_retract_speed
            retract_e = self.load_clean_retract_e
        else:
            # 退料模式参数，P1通道使用专门的参数
            index = self.load_unload_index
            if index == Channel.P1:
                # P1通道退料：预加载挤出2mm, 回抽速度600mm/min
                preloading_filament_e = self.p1_unload_preloading_filament_e
                preloading_filament_speed = self.p1_unload_preloading_filament_speed
                retract_speed = self.p1_unload_clean_retract_speed
            else:
                # F/P2通道退料：预加载挤出20mm, 回抽速度300mm/min
                preloading_filament_e = self.unload_preloading_filament_e
                preloading_filament_speed = self.unload_preloading_filament_speed
                retract_speed = self.unload_clean_retract_speed
            retract_e = self.unload_clean_retract_e

        # 预挤出耗材
        # self.gcode.run_script_from_command("G92 E0")
        # self.gcode.run_script_from_command(f"G1 E{preloading_filament_e} F{preloading_filament_speed}")
        # 回抽耗材
        self.gcode.run_script_from_command("G92 E0")
        self.gcode.run_script_from_command(f"G1 E{retract_e} F{retract_speed}")

        # 退料模式额外等待3秒
        if self.operation_mode == OperationMode.UNLOADING:
            self.gcode.run_script_from_command("G4 P3000")

    def _clean_nozzle(self):
        """
        自动擦嘴逻辑（Python版本，替代原cfg中的CLEAN_NOZZLE宏）

        功能：根据当前喷嘴选择执行相应的清理流程

        T0左喷嘴流程:
        1. 开启风扇
        2. 关闭T1加热器
        3. 移动到废料站
        4. 移动到左预装载位置
        5. 释放废料
        6. 移动到左冷却位置
        7. 降温T0到冷却温度
        8. 移动到左刷嘴位置
        9. 刷嘴
        10. 移回废料站
        11. 关闭风扇

        T1右喷嘴流程:
        1. 开启风扇
        2. 关闭T1加热器
        3. 释放废料
        4. 移动到右冷却位置
        5. 降温T1到冷却温度
        6. 移动到右刷嘴位置
        7. 刷嘴
        8. 移回废料站
        9. 关闭风扇
        """
        nozzle = self.load_unload_nozzle
        target = self.load_unload_temperature
        cooling_temp = target - self.temperature_offset

        # 开启风扇
        self.gcode.run_script_from_command("M106 P1 S255")
        self.gcode.run_script_from_command("M106 P2 S255")

        if nozzle == 0:
            # T0左喷嘴清理流程
            self.gcode.run_script_from_command("M104 T1 S0")
            self.gcode.run_script_from_command("MOVE_TO_WASTE_STATION")
            self.gcode.run_script_from_command("MOVE_LEFT_PRELOAD_POSITION")
            self.gcode.run_script_from_command("RELEASE_WASTE_FILAMENT")
            self.gcode.run_script_from_command("MOVE_LEFT_COOLING_POSITION")
            self.gcode.run_script_from_command(f"M109 T0 S{cooling_temp:.0f}")
            self.gcode.run_script_from_command("MOVE_LEFT_BRUSH_POSITION")
            self.gcode.run_script_from_command("BRUSH_NOZZLE2")
            self.gcode.run_script_from_command("MOVE_TO_WASTE_STATION")
        else:
            # T1右喷嘴清理流程
            self.gcode.run_script_from_command("M104 T1 S0")
            self.gcode.run_script_from_command("RELEASE_WASTE_FILAMENT")
            self.gcode.run_script_from_command("MOVE_RIGHT_COOLING_POSITION")
            self.gcode.run_script_from_command(f"M109 T1 S{cooling_temp:.0f}")
            self.gcode.run_script_from_command("MOVE_RIGHT_BRUSH_POSITION")
            self.gcode.run_script_from_command("BRUSH_NOZZLE2")
            self.gcode.run_script_from_command("MOVE_TO_WASTE_STATION")

        # 关闭风扇
        self.gcode.run_script_from_command("M106 P1 S0")
        self.gcode.run_script_from_command("M106 P2 S0")

    def _finish_loadunload(self):
        """
        进出料流程完成宏（流程完成）

        功能：处理进料/退料完成后的清理工作

        进料完成流程:
        1. 设置操作模式为空闲
        2. 停止process_timer，防止继续执行旧逻辑
        3. 进入STATE:41自动擦嘴
        4. 根据通道调用_clean_nozzle()（F/P1/P2都调用）
        5. 输出STATE:90完成状态

        退料完成流程:
        1. 设置操作模式为空闲
        2. F通道恢复最低挤出温度限制
        3. 等待所有运动完成(M400)
        4. 输出STATE:90完成状态
        5. 关闭加热器
        6. P2通道释放废料
        """
        index = self.load_unload_index
        channel_name = self._get_channel_name(index)

        if self.operation_mode == OperationMode.LOADING:
            # 进料完成
            self.operation_mode = OperationMode.IDLE
            self.load_step = State.AUTO_CLEANING
            # 停止所有定时器，防止继续执行旧逻辑
            self._stop_process_timer()
            self._stop_temp_timer()
            self._stop_timeout_timer()
            self._stop_load_unload_timer()  # 停止进退料超时定时器
            self._restore_idle_timeout()  # 恢复 idle_timeout
            self._stop_temp_timer()
            self._stop_timeout_timer()
            self.gcode.respond_info(f"[{channel_name} LOAD] STATE:41 AUTO_CLEANING")

            # 根据通道调用清理逻辑
            if index == Channel.F:
                self.gcode.run_script_from_command("G1 Y100 F4000")
                self._clean_nozzle()
            elif index == Channel.P1:
                # P1通道：调用自动清理
                self._clean_nozzle()
            else:
                # P2通道直接调用清理
                self._clean_nozzle()

            # 进料完成，输出完成状态
            self.load_step = State.FINISHED
            self._sync_print_stats_load_unload("standby")
            self.gcode.respond_info(f"[{channel_name} LOAD] STATE:90 FINISHED")

            # 进料完成后关闭加热器
            self.gcode.run_script_from_command("M104 T0 S0")
            self.gcode.run_script_from_command("M104 T1 S0")

        elif self.operation_mode == OperationMode.UNLOADING:
            # 卸料完成
            self.operation_mode = OperationMode.IDLE
            self.load_step = State.FINISHED
            # 停止所有定时器
            self._stop_process_timer()
            self._stop_temp_timer()
            self._stop_timeout_timer()
            self._stop_load_unload_timer()  # 停止进退料超时定时器
            self._restore_idle_timeout()  # 恢复 idle_timeout

            # F通道退料完成，恢复最低挤出温度限制
            if index == Channel.F:
                self.gcode.run_script_from_command("SET_MIN_EXTRUDE_TEMP HEATER=extruder TEMP=180")

            # 等待所有运动完成
            self.gcode.run_script_from_command("M400")
            self._sync_print_stats_load_unload("standby")
            self.gcode.respond_info(f"[{channel_name} UNLOAD] STATE:90 FINISHED")

            # 关闭加热器
            self.gcode.run_script_from_command("M104 T0 S0")
            self.gcode.run_script_from_command("M104 T1 S0")

            # P2通道释放废料
            if index == Channel.P2:
                self.gcode.run_script_from_command("RELEASE_WASTE_FILAMENT")


# ============================================================
# Klipper 配置加载函数
# ============================================================

def load_config(config):
    """
    Klipper 配置加载函数

    这是Klipper框架调用的入口函数
    创建并返回LoadUnloadHandler实例
    """
    return LoadUnloadHandler(config)
