import numpy as np
from scipy.interpolate import interpn
import pickle, os, logging

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
            raise ValueError("n_x 和 n_y 必须为正整数")
        if mesh_min[0] >= mesh_max[0] or mesh_min[1] >= mesh_max[1]:
            raise ValueError("范围必须是递增的元组")

        self.val_x = np.linspace(mesh_min[0], mesh_max[0], n_x)
        self.val_y = np.linspace(mesh_min[1], mesh_max[1], n_y)
        # TODO: 最好写成config参数，校准宏也要同步读取
        self.val_t = np.array([30.0, 70.0], dtype=np.float32)

        self.n_x = n_x
        self.n_y = n_y
        self.n_t = len(self.val_t)

        # TODO: 需同步stepper_x模块: position_min, position_max
        self.limit_x = (-43.5, 325.0)
        # TODO: 需同步stepper_y模块: position_min, position_max
        self.limit_y = (-6.0, 339.0)
        # TODO: 需同步heater_bed模块: min_temp, max_temp
        self.limit_t = (0.0, 130.0)

        self.matrix_z = np.zeros([n_x, n_y, self.n_t])
        self.ndim_z = 0
        self.fname = "klippy/extras/aniso_temp_compensation_model.pkl"

        self.grid_t = None
        self.grid_x = None
        self.grid_y = None
        self.grid_z = None
        
    def setup(self, val_t=None, limit_x=None, limit_y=None, limit_t=None):
        # TODO: 修改val_t和limit_t，可能存在干涉，需判断
        if val_t is not None:
            self.val_t = np.asarray(val_t, dtype=np.float32)
            self.n_t = len(self.val_t)
            self.matrix_z = np.zeros([self.n_x, self.n_y, self.n_t])
        if limit_x is not None:
            self.limit_x = limit_x
        if limit_y is not None:
            self.limit_y = limit_y
        if limit_t is not None:
            self.limit_t = limit_t

    def receive(self, matrix_p):
        # matrix_p 是 (ny,nx)形状的二维数组，需转置
        matrix_p = np.asarray(matrix_p)
        self.matrix_z[...,self.ndim_z] = matrix_p.T
        self.ndim_z += 1
        # logging.info(f"cpst----\n{self.matrix_z}")

    def fit(self):
        """
        拟合模型，基于预设的网格结构

        参数:
        matrix_z: 形状为 (n_x, n_y, n_t) 的三维数组
                第三维表示不同温度下的变形数据
        """
        # 评估XYT外推需求
        ext_head_x = self.limit_x[0] < self.val_x[0]
        ext_tail_x = self.limit_x[1] > self.val_x[-1]
        ext_head_y = self.limit_y[0] < self.val_y[0]
        ext_tail_y = self.limit_y[1] > self.val_y[-1]
        ext_head_t = self.limit_t[0] < self.val_t[0] 
        ext_tail_t = self.limit_t[1] > self.val_t[-1]
        # cmd = "cpst----\n"
        # cmd += f"{ext_head_x}, {ext_tail_x}, {ext_head_y}, {ext_tail_y}, {ext_head_t}, {ext_tail_t}\n"

        # 计算XYT首偏移
        idx_head_x = int(ext_head_x)
        idx_head_y = int(ext_head_y)
        idx_head_t = int(ext_head_t)

        # 计算XYT网格完整长度
        len_x = self.n_x + idx_head_x + int(ext_tail_x)
        len_y = self.n_y + idx_head_y + int(ext_tail_y)
        len_t = self.n_t + idx_head_t + int(ext_tail_t)
        # cmd += f"{len_x}, {len_y}, {len_t}\n"

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
        
        # cmd += f"{self.grid_x}\n-----\n"
        # cmd += f"{self.grid_y}\n-----\n"
        # cmd += f"{self.grid_t}\n-----\n"

        # 转为相对形变 Z 矩阵
        base_z = self.matrix_z[...,-1:]
        self.matrix_z -= base_z
        del base_z

        # 创建网格Z
        self.grid_z = np.full((len_x, len_y, len_t), np.nan, dtype=np.float32)
        self.grid_z[idx_head_x:idx_head_x + self.n_x, 
                    idx_head_y:idx_head_y + self.n_y, 
                    idx_head_t:idx_head_t + self.n_t] = self.matrix_z
        self.matrix_z = np.zeros([self.n_x, self.n_y, self.n_t])
        self.ndim_z = 0
        # cmd += f"{self.grid_z}\n----\n"

        # 填充网格Z温度外推数据
        if ext_head_t or ext_tail_t:
            slice_t = slice(idx_head_t, idx_head_t + self.n_t)
            for i in range(idx_head_x, idx_head_x + self.n_x):
                for j in range(idx_head_y, idx_head_y + self.n_y):
                    known_z = self.grid_z[i, j, slice_t]
                    if np.all(np.isnan(known_z)):
                        continue
                    valid_mask = ~np.isnan(known_z)
                    if np.sum(valid_mask) < 2:
                        continue

                    known_t = self.grid_t[slice_t][valid_mask]
                    known_z_filtered = known_z[valid_mask]
                    try:
                        coeffs = np.polyfit(known_t, known_z_filtered, 1)
                        if ext_head_t:
                            self.grid_z[i, j, 0] = \
                                np.polyval(coeffs, self.limit_t[0])
                        if ext_tail_t:
                            self.grid_z[i, j, -1] = \
                                np.polyval(coeffs, self.limit_t[1])
                    except np.linalg.LinAlgError:
                        if ext_head_t and valid_mask[0]:
                            self.grid_z[i, j, 0] = known_z[0]
                        if ext_tail_t and valid_mask[-1]:
                            self.grid_z[i, j, -1] = known_z[-1]

        # cmd += f"{self.grid_z}\n----\n"

        # 填充网格Z空间外推数据
        if ext_head_x:
            self.grid_z[0, :, :] = self.grid_z[1, :, :]
        if ext_tail_x:
            self.grid_z[-1, :, :] = self.grid_z[-2, :, :]
        if ext_head_y:
            self.grid_z[:, 0, :] = self.grid_z[:, 1, :]
        if ext_tail_y:
            self.grid_z[:, -1, :] = self.grid_z[:, -2, :]
        
        self.save_model()
        
        # cmd += f"{self.grid_z}\n----\n"
        # logging.info(cmd)
        # cmd=""

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
            outputs = np.full(n, outputs)

        if self.grid_z is None:
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
        保存模型参数到pkl文件
        """
        if filepath is None:
            filepath = self.fname
            
        model_data = {
            'grid_x': self.grid_x,
            'grid_y': self.grid_y,
            'grid_t': self.grid_t,
            'grid_z': self.grid_z,
            'val_x': self.val_x,
            'val_y': self.val_y,
            'val_t': self.val_t,
            'n_x': self.n_x,
            'n_y': self.n_y,
            'n_t': self.n_t,
            'limit_x': self.limit_x,
            'limit_y': self.limit_y,
            'limit_t': self.limit_t
            }
        
        try:
            with open(filepath, 'wb') as f:
                pickle.dump(model_data, f)
            logging.info(f"模型已保存至 {filepath}")
        except Exception as e:
            logging.error(f"保存模型失败: {e}")

    def load_model(self, filepath=None):
        """
        从pkl文件加载模型参数
        """
        if filepath is None:
            filepath = self.fname
            
        if not os.path.exists(filepath):
            logging.info(f"模型文件不存在: {filepath}")
            return False
            
        try:
            with open(filepath, 'rb') as f:
                model_data = pickle.load(f)
            
            # 恢复模型参数
            self.grid_x = model_data['grid_x']
            self.grid_y = model_data['grid_y']
            self.grid_t = model_data['grid_t']
            self.grid_z = model_data['grid_z']
            self.val_x = model_data['val_x']
            self.val_y = model_data['val_y']
            self.val_t = model_data['val_t']
            self.n_x = model_data['n_x']
            self.n_y = model_data['n_y']
            self.n_t = model_data['n_t']
            self.limit_x = model_data['limit_x']
            self.limit_y = model_data['limit_y']
            self.limit_t = model_data['limit_t']
            
            logging.info(f"模型已从 {filepath} 加载")
            return True
        except Exception as e:
            logging.info(f"加载模型失败: {e}")
            return False
        