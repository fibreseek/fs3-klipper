import numpy as np
from scipy.interpolate import interpn
import logging, os

class error(Exception):
    pass

class AnisoTempCompensation:
    # 热床温度补偿器，用于快速调平的跨温度对比
    def __init__(self, mesh_min, mesh_max, n_x, n_y):
        """
        初始化温度形变补偿器

        参数:
        mesh_min: 热床最小范围，如 (0.0, 0.0)
        mesh_max: 热床最大范围，如 (300.0, 300.0)
        n_x: x方向探测点数
        n_y: y方向探测点数
        """
        if n_x <= 0 or n_y <= 0:
            raise error("[TempComp] n_x and n_y must be positive integer")
        if mesh_min[0] >= mesh_max[0] or mesh_min[1] >= mesh_max[1]:
            raise error("[TempComp] range must be an increasing tuple")
        
        self.grid_x = None
        self.grid_y = None
        self.grid_t = None
        self.grid_z = None

        self.val_x = np.linspace(mesh_min[0], mesh_max[0], n_x)
        self.val_y = np.linspace(mesh_min[1], mesh_max[1], n_y)
        # TODO: 最好写成config参数，校准宏也要同步读取
        self.val_t = np.array([30.0, 70.0], dtype=np.float32)
        self.val_z = []

        self.n_x = n_x
        self.n_y = n_y
        self.n_t = len(self.val_t)

        # TODO: 需同步stepper_x模块: position_min, position_max
        self.limit_x = (-43.5, 325.0)
        # TODO: 需同步stepper_y模块: position_min, position_max
        self.limit_y = (-6.0, 339.0)
        # TODO: 需同步heater_bed模块: min_temp, max_temp
        self.limit_t = (0.0, 130.0)

        self.built = False # 网格构建完成标志位
        self.fname = "~/printer_data/config/temp_comp.npz"
        self.clean_callback = None

    def setup(self, val_t=None, limit_x=None, limit_y=None, limit_t=None):
        # TODO: 修改val_t和limit_t，可能存在干涉，需判断
        if val_t is not None:
            self.val_t = np.asarray(val_t, dtype=np.float32)
            self.n_t = len(self.val_t)
        if limit_x is not None:
            self.limit_x = limit_x
        if limit_y is not None:
            self.limit_y = limit_y
        if limit_t is not None:
            self.limit_t = limit_t

    def receive(self, matrix_p, temp_now):
        # 更新为实测热床温度
        if temp_now is not None:
            idx = len(self.val_z)
            self.val_t[idx] = temp_now

        # 注意 matrix_p 是 (ny,nx) 形状的二维数组
        self.val_z.append(matrix_p)
        self.built = False
        # logging.debug(f"cpst----\n{self.val_z}")

    def fit(self, reactor, callback=None):
        """
        拟合模型，基于预设的网格结构
        """
        # 清理回调函数
        self.clean_callback = callback

        # 评估XYT外推需求
        ext_head_x = self.limit_x[0] < self.val_x[0]
        ext_tail_x = self.limit_x[1] > self.val_x[-1]
        ext_head_y = self.limit_y[0] < self.val_y[0]
        ext_tail_y = self.limit_y[1] > self.val_y[-1]
        ext_head_t = self.limit_t[0] < self.val_t[0] 
        ext_tail_t = self.limit_t[1] > self.val_t[-1]

        # 计算XYT首偏移
        idx_head_x = int(ext_head_x)
        idx_head_y = int(ext_head_y)
        idx_head_t = int(ext_head_t)

        # 计算XYT网格完整长度
        len_x = self.n_x + idx_head_x + int(ext_tail_x)
        len_y = self.n_y + idx_head_y + int(ext_tail_y)
        len_t = self.n_t + idx_head_t + int(ext_tail_t)

        # 构建含外推的XYT网格
        self.grid_x = np.empty(len_x, dtype=np.float32)
        self.grid_y = np.empty(len_y, dtype=np.float32)
        self.grid_t = np.empty(len_t, dtype=np.float32)

        # 填充XYT网格核心数据
        self.grid_x[idx_head_x:idx_head_x + self.n_x] = self.val_x
        self.grid_y[idx_head_y:idx_head_y + self.n_y] = self.val_y
        self.grid_t[idx_head_t:idx_head_t + self.n_t] = self.val_t

        # 填充XYT网格外推数据
        if ext_head_x:
            self.grid_x[0] = self.limit_x[0]
        if ext_tail_x:
            self.grid_x[-1] = self.limit_x[1]
        if ext_head_y:
            self.grid_y[0] = self.limit_y[0]
        if ext_tail_y:
            self.grid_y[-1] = self.limit_y[1]
        if ext_head_t:
            self.grid_t[0] = self.limit_t[0]
        if ext_tail_t:
            self.grid_t[-1] = self.limit_t[1]

        # 形状调整
        # 修改为(n_x,n_y,n_t)三维数组，第三维表示不同温度下的 Z 形变
        self.val_z = np.stack(self.val_z, axis=0)  # (n_t, n_y, n_x)
        self.val_z = np.transpose(self.val_z, (2, 1, 0))  # (n_x, n_y, n_t)

        # 转为相对形变 Z 矩阵
        base_z = self.val_z[...,-1:]
        self.val_z -= base_z
        del base_z

        # 创建网格Z
        self.grid_z = np.full((len_x, len_y, len_t), np.nan, dtype=np.float32)
        self.grid_z[idx_head_x:idx_head_x + self.n_x, 
                    idx_head_y:idx_head_y + self.n_y, 
                    idx_head_t:idx_head_t + self.n_t] = self.val_z
        self.val_z = [] # 清空列表释放内存

        def process_space_z(eventtime):
            if ext_head_x:
                self.grid_z[0, :, :] = self.grid_z[1, :, :]
            if ext_tail_x:
                self.grid_z[-1, :, :] = self.grid_z[-2, :, :]
            if ext_head_y:
                self.grid_z[:, 0, :] = self.grid_z[:, 1, :]
            if ext_tail_y:
                self.grid_z[:, -1, :] = self.grid_z[:, -2, :]
            
            reactor.register_callback(lambda e: self.save_model())

        def process_temp_z(eventtime):
            nonlocal start_row, end_row # 允许对外部闭包变量写操作
            for i in range(start_row, end_row):
                for j in range(idx_head_y, idx_head_y + self.n_y):
                    known_z = self.grid_z[i, j, slice_t]
                    valid_mask = ~np.isnan(known_z)
                    if np.count_nonzero(valid_mask) < 2:
                        continue

                    known_t = self.grid_t[slice_t][valid_mask]
                    known_z_filtered = known_z[valid_mask]
                    try:
                        coeffs = np.polyfit(known_t, known_z_filtered, 1)
                        self.grid_z[i, j, :] = \
                            np.polyval(coeffs, self.grid_t).astype(np.float32)
                    except np.linalg.LinAlgError:
                        if ext_head_t:
                            if valid_mask[0]:
                                self.grid_z[i, j, 0] = known_z_filtered[0]
                            else:
                                self.grid_z[i, j, 0] = 0.0
                        if ext_tail_t:
                            if valid_mask[-1]:
                                self.grid_z[i, j, -1] = known_z_filtered[-1]
                            else:
                                self.grid_z[i, j, -1] = 0.0

            # 有剩余数据，继续回调处理
            if end_row < total_row:
                start_row = end_row
                end_row = min(start_row + batch_row, total_row)
                reactor.register_callback(process_temp_z)
            else:
                self.grid_t = self.grid_t[[0,-1]]
                self.grid_z = self.grid_z[...,[0,-1]]
                self.n_t = 2
                reactor.register_callback(process_space_z)

        # 填充网格Z温度外推数据
        if ext_head_t or ext_tail_t:
            batch_row = min(4, max(1, self.n_x // 5))  # 每次批处理的行数
            start_row = idx_head_x
            total_row = idx_head_x + self.n_x
            end_row = min(start_row + batch_row, total_row)
            slice_t = slice(idx_head_t, idx_head_t + self.n_t)
            reactor.register_callback(process_temp_z)
        else: # 填充网格Z空间外推数据
            reactor.register_callback(process_space_z)

    def predict(self, x, y, t):
        """
        预测给定x, y, t的相对z值（相对于基准温度）。
        t可以是标量或一维数组，返回对应z值。
        """
        outputs = 0.0
        isscalar_t = np.isscalar(t)
        if not isscalar_t:
            t = np.atleast_1d(t).astype(np.float32)
            n = len(t)
            x = np.full(n, x, dtype=np.float32)
            y = np.full(n, y, dtype=np.float32)
            outputs = np.full(n, outputs, dtype=np.float32)

        if self.built == False:
            if self.load_model() == False:
                return outputs

        # 执行插值
        points = (self.grid_x, self.grid_y, self.grid_t)
        inputs = np.column_stack((x, y, t))
        outputs = interpn(points, self.grid_z, inputs, method='linear', 
                          bounds_error=False, fill_value=np.nan)
        del points, inputs

        # 用全局均值填充NaN值
        nan_mask = np.isnan(outputs)
        if np.any(nan_mask):
            mean_z = np.nanmean(self.grid_z)
            if not np.isfinite(mean_z):
                mean_z = 0.0  # 回退值
            outputs[nan_mask] = mean_z

        if isscalar_t:
            return float(outputs[0])
        else:
            return outputs

    def save_model(self, filepath=None):
        """
        保存模型参数到npz文件
        """
        if filepath is None:
            filepath = self.fname
        filepath = os.path.expanduser(filepath) # 将 ~ 映射为 /home/anisoprint/

        # 离线保存时，只保留6位小数
        self.grid_x = np.round(self.grid_x, decimals=6)
        self.grid_y = np.round(self.grid_y, decimals=6)
        self.grid_t = np.round(self.grid_t, decimals=6)
        self.grid_z = np.round(self.grid_z, decimals=6)
            
        try:
            # 只保存必需的网格数据
            np.savez_compressed(
                filepath,
                grid_x=self.grid_x,
                grid_y=self.grid_y,
                grid_t=self.grid_t,
                grid_z=self.grid_z
            )
            self.built = True
            logging.debug(f"[TempComp] model save success {filepath}")
        except Exception as e:
            logging.debug(f"[TempComp] model save fail {e}")

        if self.clean_callback:
            self.clean_callback()
            self.clean_callback = None # 防止重复调用

    def load_model(self, filepath=None):
        """
        从npz文件加载模型参数
        """
        if filepath is None:
            filepath = self.fname
        filepath = os.path.expanduser(filepath) # 将 ~ 映射为 /home/anisoprint/

        if not os.path.exists(filepath):
            logging.debug("[TempComp] model path not exist")
            return False
            
        try:
            # 加载保存的数据
            with np.load(filepath) as data:
                # 恢复网格数据
                self.grid_x = data['grid_x']
                self.grid_y = data['grid_y']
                self.grid_t = data['grid_t']
                self.grid_z = data['grid_z']
            
            logging.debug(f"[TempComp] model load success {filepath}")
            self.built = True
            return True
        except Exception as e:
            logging.debug(f"[TempComp] model load fail {e}")
            return False