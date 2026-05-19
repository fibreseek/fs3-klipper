# 张力传感器力值与堵料算法

本文说明 Klipper 扩展 `klippy/extras/tension_sensor.py` 中的**力值（ADC）处理**、**基准更新**与**堵料判定**逻辑，便于理解与调参。

## 实现位置

- 源码：`klippy/extras/tension_sensor.py`
- 类：`MySerial`

## 1. 力值（原始量）

### 1.1 数据来源

- 从 **`/dev/spidev0.0`** 每次读取 **3 字节**，拼成 24 位数据后转为**有符号整数**，在后续逻辑中即「力值」（与电压换算无关）。

### 1.2 有符号解析

三字节组合为 `value` 后：

- 若 `value > 0x400000`，则 `value -= 0x800000`，得到与 24 位补码一致的有符号 ADC 计数。

### 1.3 电压（仅展示）

可选换算用于 G 码回复，**不参与**基准与堵料运算：

\[
V = \frac{\mathrm{adc}}{\mathrm{ADC\_MAX}} \times \frac{\mathrm{VREF}}{\mathrm{GAIN}}
\]

其中 `ADC_MAX = 2^23 - 1`，`VREF = 3.3`，`GAIN = 128`（见 `adc_to_voltage`）。

## 2. 采样与缓冲

| 项目 | 说明 |
|------|------|
| `sample_interval` | 默认 **0.2 s**（200 ms），轮询线程每周期读一次 |
| `detection_buffer` | 环形缓冲，最多 **30** 点，用于堵料判定 |
| `value_history` | 最多 **500** 点，用于历史记录 |

## 3. 基准力值 `force_baseline`

基准**仅**通过 G 码宏 **`TENSION_UPDATE_BASELINE`** 在后台线程更新，**不会**从日常滑动窗口自动学习。

### 3.1 采集流程

1. 先连续读 **10** 次作为 warmup（间隔 `sample_interval`）。
2. 再连续读 **10** 次作为正式样本 `samples`。
3. 更新期间 **`_skip_jam_detection = True`**，该时段**不参与**堵料判定。

### 3.2 从 10 个样本计算基准（主簇 + EMA）

1. **样本数量**：必须为 `baseline_sample_count`（默认 **10**），否则失败。
2. **异常占比**：`|v| > max_normal_value`（默认 **500000**）视为异常；若异常占比 **> 70%**，失败。
3. **有效集**：保留 `|v| ≤ max_normal_value` 的样本；若为空，失败。
4. **主簇** `_find_primary_cluster(valid, eps)`：对有效值排序，在有序序列上找**跨度 ≤ `baseline_cluster_eps`**（默认 **1200**）且**点数最多**的连续区间（一维密度簇）。
5. **点数门槛**：主簇点数 ≥ `baseline_cluster_min_points`（默认 **4**），否则失败。
6. **簇中心**：对主簇排序后取**中位数**，四舍五入为整数 `center`。
7. **EMA**：若已有旧基准  
   `baseline = α × center + (1 − α) × old_baseline`，  
   `α = baseline_ema_alpha`（默认 **0.2**）；若无旧基准，则 `baseline = center`。

失败时通过 M118 等方式报告原因，并打印 warmup / raw 样本便于排查。

## 4. 堵料判定用的平滑力值

- **`get_trigger_median()`**：取 `detection_buffer` 中**最新 3 个点**的中位数（`trigger_sample_count = 3`），记为 `smoothed`（调试输出中常称 trig median3）。
- 缓冲内点数 **< 3** 时处于收集阶段，不判堵。

## 5. 堵料判据

前提：已设置 `force_baseline`，且已有 `smoothed`。

### 5.1 相对基准的偏离（单向）

- `delta_fast = smoothed - baseline`
- `fast_deviation = max(0, −delta_fast)`

即**仅当 `smoothed < baseline`（相对基准往负方向）**时才有正的偏离量；**正向偏离不视为堵料**（该方向被忽略）。

### 5.2 阈值与饱和

- 若 `fast_deviation > jam_deviation_limit`（默认 **45000**），本帧判为堵料趋势。可通过 **`TENSION_SET_THRESHOLD MIN_DEV=<n>`** 修改（`min_dev ≥ 1000`）。
- 若 `|smoothed| > max_normal_value`，**直接**视为堵料（与方向无关）。

### 5.3 连续帧去抖

- 单帧满足堵料条件时递增 `consecutive_jam_frames`。
- 达到 **`jam_consecutive_frames`**（默认 **3**）后锁定：`jamming_triggered = True`，状态 **JAMMED**，并记录快照日志。可通过 **`TENSION_SET_CONSECUTIVE FRAMES=<n>`** 调整。
- 若本帧不满足堵料条件，**连续计数清零**。

锁定后需通过 **`TENSION_RESET_ALARM`** 等逻辑解除（会清空部分缓冲；基准通常保留，见源码）。

## 6. 主要参数默认值

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `sample_interval` | 0.2 s | 轮询与基准采样间隔 |
| `baseline_sample_count` | 10 | 基准计算用样本数 |
| `baseline_cluster_eps` | 1200 | 主簇允许的最大跨度 |
| `baseline_cluster_min_points` | 4 | 主簇最少点数 |
| `baseline_ema_alpha` | 0.2 | 新簇中心相对旧基准的权重 |
| `trigger_sample_count` | 3 | 判堵用中位数窗口长度 |
| `jam_deviation_limit` | 45000 | 负向相对偏差阈值（ADC 计数） |
| `jam_consecutive_frames` | 3 | 连续超阈帧数才锁定堵料 |
| `max_normal_value` | 500000 | 饱和/异常幅值门槛 |

部分参数可在 `printer.cfg` 对应段中通过 `config.getint` / `getfloat` 传入；运行 **`TENSION_SHOW_CONFIG`** 可查看当前值。

## 7. 相关 G 码（节选）

| 命令 | 作用 |
|------|------|
| `TENSION_UPDATE_BASELINE` | 后台更新基准（主簇 + EMA） |
| `TENSION_SHOW_BASELINE` / `TENSION_SHOW_CONFIG` | 查看基准与配置 |
| `TENSION_SET_THRESHOLD MIN_DEV=<n>` | 设置堵料偏差阈值 |
| `TENSION_SET_CONSECUTIVE FRAMES=<n>` | 设置连续帧数 |
| `TENSION_SENSOR_POLL ENABLE=<0\|1>` | 武装/解除（不开始采样） |
| `TENSION_SENSOR_START [LOGGING=…] [DETECT=…]` | 开始轮询与堵料检测 |
| `GET_FORCE_VALUE` / `GET_RAW_FORCE_DATA` | 单次读数与原始字节 |
| `TENSION_DEBUG` | 调试信息与近期统计 |
| `TENSION_RESET_ALARM` | 解除堵料锁定并清空部分缓冲 |

（完整参数与行为以源码为准。）

## 8. 算法小结

- **力值** = SPI 读出的 **24 位有符号 ADC 整数**；电压公式仅用于人机可读。
- **基准** = 10 点采样 → 过滤极端异常 → **最密一维簇的中位数** → 与历史基准 **EMA** 融合。
- **堵料** =（**最新 3 点中位数**相对基准**仅负向**超阈，或**幅值饱和**）且经**连续多帧**确认；**正向**相对基准的偏离不进入负向堵料判据。
