#!/usr/bin/env python3
"""
解析probe数据文件，计算偏差值，生成柱状图，统计相同偏差值的个数和百分比
"""

import os
import re
import locale
from collections import Counter

# 在无头（headless）环境下运行时，避免 matplotlib 自动选择需要显示服务器的交互后端。
# 必须在导入 matplotlib.pyplot 之前设置后端或环境变量。
try:
    # 优先使用环境变量，这样外部可以覆盖
    if 'MPLBACKEND' not in os.environ:
        os.environ['MPLBACKEND'] = 'Agg'
except Exception:
    pass

# 尝试设置合理的 locale，避免 "Locale not supported by C library" 警告
try:
    locale.setlocale(locale.LC_ALL, '')
except Exception:
    # 如果系统没有设置或支持当前 locale，回退到 C.UTF-8 或 C
    try:
        locale.setlocale(locale.LC_ALL, 'C.UTF-8')
    except Exception:
        try:
            locale.setlocale(locale.LC_ALL, 'C')
        except Exception:
            # 最后手段：不要设置 locale
            pass

import matplotlib.pyplot as plt
import numpy as np
import warnings

# 抑制与显示相关的 matplotlib/GTK 警告（非致命）
warnings.filterwarnings("ignore", message=".*Gdk-CRITICAL.*")
warnings.filterwarnings("ignore", message=".*Gtk-WARNING.*")

def parse_probe_data(file_path, filter_abnormal=False):
    """
    解析probe数据文件，提取z值
    如果filter_abnormal为True，过滤掉 z < -0.2 或 z > 0 的数据点，并记录异常数据
    如果filter_abnormal为False，不过滤任何数据，所有数据都作为有效数据
    """
    z_values = []
    abnormal_data = []  # 存储异常数据
    
    with open(file_path, 'r', encoding='utf-8') as file:
        for line_num, line in enumerate(file, 1):
            # 使用正则表达式匹配 z= 后面的数值
            match = re.search(r'z=(-?\d+\.\d+)', line)
            if match:
                z_value = float(match.group(1))
                
                if filter_abnormal:
                    # 过滤条件：只保留 -0.2 <= z <= 0 的数据点
                    if -0.2 <= z_value <= 0:
                        z_values.append(z_value)
                    else:
                        # 记录异常数据：行号、原始z值、异常类型
                        if z_value < -0.2:
                            abnormal_type = "过低"
                        else:
                            abnormal_type = "过高"
                        abnormal_data.append({
                            'line_number': line_num,
                            'z_value': z_value,
                            'type': abnormal_type
                        })
                else:
                    # 不过滤，所有数据都作为有效数据
                    z_values.append(z_value)
    
    return z_values, abnormal_data

def calculate_deviations(z_values, baseline=-0.1):
    """
    计算每个z值与基准值的偏差，保留两位小数
    """
    deviations = [round(z - baseline, 2) for z in z_values]
    return deviations

def analyze_deviations(deviations):
    """
    统计相同偏差值的个数和百分比
    """
    deviation_counter = Counter(deviations)
    total_count = len(deviations)
    
    # 计算百分比
    deviation_stats = {}
    for deviation, count in deviation_counter.items():
        percentage = round((count / total_count) * 100, 2)
        deviation_stats[deviation] = {'count': count, 'percentage': percentage}
    
    return deviation_stats

def plot_histogram(deviations, output_path, image_title, stats=None):
    """
    生成偏差值柱状图和所有偏差值的折线图
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # 第一个子图：柱状图
    unique_deviations = sorted(set(deviations))
    deviation_counts = [deviations.count(d) for d in unique_deviations]
    
    # 使用整数位置作为x轴，避免浮点数精度问题
    x_positions = range(len(unique_deviations))
    
    # 创建柱状图
    bars = ax1.bar(x_positions, deviation_counts, color='skyblue', alpha=0.7, edgecolor='black', width=0.6)
    
    # 在柱子上方显示数值和百分比
    total_count = len(deviations)
    # 计算一个相对偏移量，使得标签与柱顶更贴近，并对极端值进行限制
    max_count = max(deviation_counts) if deviation_counts else 1
    # 偏移按最大计数的比例计算，最小为 0.15，最大不超过 1.0 （可以避免在极小或极大数据下偏移过大）
    label_offset = min(max(max_count * 0.02, 0.15), 1.0)

    for bar, count, deviation in zip(bars, deviation_counts, unique_deviations):
        percentage = round((count / total_count) * 100, 2)
        text = f"{count} ({percentage}%)"
        # 使用计算出的偏移量而不是固定的 0.5，使标签高度自适应且更靠近柱顶
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + label_offset,
                text, ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # 设置第一个子图的标签和标题 - 使用图片名称作为标题
    ax1.set_xlabel('Deviation (mm)', fontsize=12)
    ax1.set_ylabel('Frequency', fontsize=12)
    ax1.set_title(f'{image_title} - Deviation Distribution', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # 设置x轴刻度和标签
    ax1.set_xticks(x_positions)
    ax1.set_xticklabels([f'{d:.2f}' for d in unique_deviations], rotation=0)
    
    # 设置y轴范围，留出一些空间显示数值。顶部空间缩小到 ~6% 并考虑标签偏移
    top_margin = max_count * 1.06 + label_offset
    ax1.set_ylim(0, top_margin)
    
    # 第二个子图：所有偏差值的折线图
    ax2.plot(range(len(deviations)), deviations, 'b-', linewidth=1, alpha=0.7)
    ax2.set_xlabel('Data Point Index', fontsize=12)
    ax2.set_ylabel('Deviation (mm)', fontsize=12)
    ax2.set_title(f'{image_title} - All Deviation Values', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # 设置y轴范围，基于偏差值范围
    deviation_range = max(deviations) - min(deviations)
    ax2.set_ylim(min(deviations) - deviation_range * 0.1, max(deviations) + deviation_range * 0.1)
    
    # 如果提供了统计信息，则在右上角显示（中文标签）
    if stats is not None:
        try:
            min_v = stats.get('min')
            max_v = stats.get('max')
            range_v = stats.get('range')
            mode_v = stats.get('mode')
            mode_count = stats.get('mode_count')
            info_lines = []
            if min_v is not None:
                info_lines.append(f"Min: {min_v:.2f} mm")
            if max_v is not None:
                info_lines.append(f"Max: {max_v:.2f} mm")
            if range_v is not None:
                info_lines.append(f"Range: {range_v:.2f} mm")
            if mode_v is not None:
                if mode_count is not None:
                    info_lines.append(f"Mode: {mode_v:.2f} ({mode_count})")
                else:
                    info_lines.append(f"Mode: {mode_v:.2f}")
            info_text = "\n".join(info_lines)
            # place at top-right inside ax1 with improved readability
            bbox_props = dict(facecolor='white', alpha=0.9, edgecolor='black', linewidth=0.6)
            text_obj = ax1.text(0.98, 0.98, info_text, transform=ax1.transAxes,
                                ha='right', va='top', fontsize=10,
                                bbox=bbox_props)
            # Add a light stroke for extra contrast (matplotlib >= 3.4 supports path_effects)
            try:
                import matplotlib.patheffects as pe
                text_obj.set_path_effects([pe.withStroke(linewidth=1.2, foreground='black')])
            except Exception:
                # path effects may not be available; fall back to bbox only
                pass
        except Exception:
            pass

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

def save_statistics(deviation_stats, output_path, baseline=-0.1):
    """
    保存统计结果到文本文件
    """
    with open(output_path, 'w', encoding='utf-8') as file:
        file.write("Probe数据偏差值统计分析\n")
        file.write("=" * 50 + "\n\n")
        
        file.write(f"基准值: {baseline} mm\n")
        file.write(f"总数据点数: {sum(stats['count'] for stats in deviation_stats.values())}\n\n")
        
        file.write("偏差值统计:\n")
        file.write("-" * 30 + "\n")
        file.write("偏差值(mm)\t出现次数\t百分比(%)\n")
        file.write("-" * 30 + "\n")
        
        # 按偏差值排序输出
        for deviation in sorted(deviation_stats.keys()):
            stats = deviation_stats[deviation]
            file.write(f"{deviation:>10.2f}\t{stats['count']:>8}\t{stats['percentage']:>10.2f}\n")

def analyze_abnormal_data(abnormal_data):
    """
    统计异常数据
    """
    abnormal_stats = {
        'total_abnormal': len(abnormal_data),
        'too_low': 0,
        'too_high': 0,
        'low_values': [],
        'high_values': []
    }
    
    for data in abnormal_data:
        if data['type'] == "过低":
            abnormal_stats['too_low'] += 1
            abnormal_stats['low_values'].append(data['z_value'])
        else:
            abnormal_stats['too_high'] += 1
            abnormal_stats['high_values'].append(data['z_value'])
    
    # 计算百分比
    if abnormal_stats['total_abnormal'] > 0:
        abnormal_stats['low_percentage'] = round((abnormal_stats['too_low'] / abnormal_stats['total_abnormal']) * 100, 2)
        abnormal_stats['high_percentage'] = round((abnormal_stats['too_high'] / abnormal_stats['total_abnormal']) * 100, 2)
    else:
        abnormal_stats['low_percentage'] = 0.0
        abnormal_stats['high_percentage'] = 0.0
    
    return abnormal_stats

def save_abnormal_statistics(abnormal_stats, abnormal_data, output_path):
    """
    保存异常数据统计结果到文本文件
    """
    with open(output_path, 'w', encoding='utf-8') as file:
        file.write("Probe数据异常数据统计分析\n")
        file.write("=" * 50 + "\n\n")
        
        file.write(f"总异常数据点数: {abnormal_stats['total_abnormal']}\n\n")
        
        file.write("异常数据分类统计:\n")
        file.write("-" * 40 + "\n")
        file.write("异常类型\t出现次数\t百分比(%)\n")
        file.write("-" * 40 + "\n")
        file.write(f"z值过低\t{abnormal_stats['too_low']:>8}\t{abnormal_stats['low_percentage']:>10.2f}\n")
        file.write(f"z值过高\t{abnormal_stats['too_high']:>8}\t{abnormal_stats['high_percentage']:>10.2f}\n")
        
        file.write("\n异常数据详细信息:\n")
        file.write("-" * 60 + "\n")
        file.write("行号\tz值(mm)\t异常类型\n")
        file.write("-" * 60 + "\n")
        
        # 按行号排序输出
        for data in sorted(abnormal_data, key=lambda x: x['line_number']):
            file.write(f"{data['line_number']}\t{data['z_value']:.3f}\t{data['type']}\n")
        
        # 添加异常值范围信息
        if abnormal_stats['low_values']:
            file.write(f"\n过低z值范围: {min(abnormal_stats['low_values']):.3f} 到 {max(abnormal_stats['low_values']):.3f} mm\n")
        if abnormal_stats['high_values']:
            file.write(f"过高z值范围: {min(abnormal_stats['high_values']):.3f} 到 {max(abnormal_stats['high_values']):.3f} mm\n")

def main():
    import sys
    import argparse
    
    # 设置命令行参数解析
    parser = argparse.ArgumentParser(description='分析probe数据文件')
    parser.add_argument('input_file', help='输入数据文件路径')
    parser.add_argument('-b', '--baseline', type=float, default=-0.1,
                        help='基准z值，默认 -0.1 (mm)')
    # --filter-abnormal 已移除，脚本始终不过滤输入数据
    
    # 解析命令行参数
    args = parser.parse_args()
    
    input_file = args.input_file
    baseline = args.baseline
    # 不再支持命令行开关，始终不过滤（保留所有样本）
    filter_abnormal = False
    output_dir = "/tmp/results_probe_analysis"
    
    # 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 从输入文件名提取基础名称（去掉扩展名）
    base_name = os.path.splitext(os.path.basename(input_file))[0]
    
    # 输出文件路径 - 以原数据文件名称起始
    histogram_path = os.path.join(output_dir, f"{base_name}_deviations_histogram.png")
    stats_path = os.path.join(output_dir, f"{base_name}_deviations_statistics.txt")
    abnormal_stats_path = os.path.join(output_dir, f"{base_name}_abnormal_statistics.txt")
    
    try:
        # 解析数据
        print("Parsing probe data...")
        print(f"Filter abnormal data: {'Yes' if filter_abnormal else 'No'}")
        z_values, abnormal_data = parse_probe_data(input_file, filter_abnormal=filter_abnormal)
        print(f"Successfully parsed {len(z_values)} valid data points")
        print(f"Found {len(abnormal_data)} abnormal data points")

        # 计算偏差值
        print("Calculating deviations...")
        deviations = calculate_deviations(z_values, baseline=baseline)

        # 统计分析
        deviation_stats = analyze_deviations(deviations)

        # 异常数据统计
        abnormal_stats = analyze_abnormal_data(abnormal_data)

        # 计算用于图注的统计信息：最小值、极差、出现最多的值
        stat_info = {}
        try:
            stat_info['max'] = max(deviations)
            stat_info['min'] = min(deviations)
            stat_info['range'] = max(deviations) - min(deviations)
            # 计算出现最多的值（众数）
            counter = Counter(deviations)
            mode_val, mode_count = counter.most_common(1)[0]
            stat_info['mode'] = mode_val
            stat_info['mode_count'] = mode_count
        except Exception:
            stat_info = None

        # 生成柱状图（传入统计信息以在右上角显示）
        print("Generating histogram...")
        plot_histogram(deviations, histogram_path, base_name, stats=stat_info)

        # 保存统计结果
        print("Saving statistics...")
        save_statistics(deviation_stats, stats_path, baseline=baseline)

        # 保存异常数据统计结果
        print("Saving abnormal statistics...")
        save_abnormal_statistics(abnormal_stats, abnormal_data, abnormal_stats_path)

        # 输出摘要信息
        print("\n" + "="*50)
        print("Summary:")
        print(f"Valid data points: {len(z_values)}")
        print(f"Abnormal data points: {len(abnormal_data)}")
        print(f"Baseline: {baseline} mm")
        print(f"Range: {min(deviations):.2f} to {max(deviations):.2f} mm")
        print(f"Histogram saved at: {histogram_path}")
        print(f"Statistics saved at: {stats_path}")
        print(f"Abnormal statistics saved at: {abnormal_stats_path}")
        print("="*50)

        # 显示主要统计信息
        print("\nMain Statistics:")
        print("-" * 40)
        print("Deviation (mm)\tCount\tPercentage (%)")
        print("-" * 40)
        for deviation in sorted(deviation_stats.keys()):
            stats = deviation_stats[deviation]
            print(f"{deviation:>10.2f}\t{stats['count']:>8}\t{stats['percentage']:>10.2f}")

        # Display abnormal data statistics
        if abnormal_stats['total_abnormal'] > 0:
            print("\nAbnormal Data Statistics:")
            print("-" * 40)
            print("Abnormal Type\tCount\tPercentage (%)")
            print("-" * 40)
            print(f"Z value too low\t{abnormal_stats['too_low']:>8}\t{abnormal_stats['low_percentage']:>10.2f}")
            print(f"Z value too high\t{abnormal_stats['too_high']:>8}\t{abnormal_stats['high_percentage']:>10.2f}")

        except FileNotFoundError:
        print(f"Error: File not found: {input_file}")
        except Exception as e:
        print(f"An error occurred during analysis: {str(e)}")

if __name__ == "__main__":
    main()
