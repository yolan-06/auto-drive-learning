import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

# --- 1. 参数定义 ---
V_STRAIGHT_KMH = 5.0
V_CURVE_KMH = 2.0
CURVATURE_THRESHOLD = 0.05
MAX_ACCELERATION = 0.3
MAX_DECELERATION = 0.3
GAUSSIAN_SIGMA = 5
S_CURVE_STEEPNESS = 10.0
# --- 【新增】最终抛光参数 ---
# 一个非常小的值，用于消除S型曲线生成后残余的微小数值噪声
FINAL_POLISHING_SIGMA = 1.5

# --- 路径加密功能的参数 ---
ENABLE_DENSIFICATION = True
DENSE_CURVATURE_THRESHOLD = 0.1
TARGET_DENSITY_M = 0.1

# --- 2. 单位转换 ---
V_STRAIGHT_MS = V_STRAIGHT_KMH / 3.6
V_CURVE_MS = V_CURVE_KMH / 3.6

def load_path(filename):
    """从CSV文件加载路径点"""
    try:
        df = pd.read_csv(filename)
        print(f"路径文件 '{filename}' 加载成功，包含 {len(df)} 个路径点。")
        return df
    except FileNotFoundError:
        print(f"错误: 文件 '{filename}' 未找到。")
        return None

def calculate_curvature(df):
    """使用多级高斯滤波计算平滑曲率"""
    x, y = df['x'].values, df['y'].values
    x_smooth, y_smooth = gaussian_filter1d(x, GAUSSIAN_SIGMA), gaussian_filter1d(y, GAUSSIAN_SIGMA)
    df['x'], df['y'] = x_smooth, y_smooth
    dx, dy = np.gradient(x_smooth), np.gradient(y_smooth)
    dx_smooth, dy_smooth = gaussian_filter1d(dx, GAUSSIAN_SIGMA), gaussian_filter1d(dy, GAUSSIAN_SIGMA)
    ddx, ddy = np.gradient(dx_smooth), np.gradient(dy_smooth)
    df['yaw'] = np.arctan2(dy_smooth, dx_smooth)
    denominator = (dx_smooth**2 + dy_smooth**2)**1.5 + 1e-6
    curvature = np.abs(dx_smooth * ddy - dy_smooth * ddx) / denominator
    df['curvature'] = curvature
    return df

def plan_motion_profile(df):
    """【最终版】使用Sigmoid函数构建S型曲线并进行最终抛光"""
    s = np.cumsum(np.hypot(np.diff(df['x'], prepend=df['x'][0]), np.diff(df['y'], prepend=df['y'][0])))
    df['s'] = s
    ds = np.diff(s, prepend=0)
    
    velocity_limit = np.where(df['curvature'] < CURVATURE_THRESHOLD, V_STRAIGHT_MS, V_CURVE_MS)
    v_trapezoidal = np.copy(velocity_limit)
    
    # 1. 生成梯形速度以确定斜坡的起止时间
    for i in range(1, len(df)):
        v_max_by_accel = np.sqrt(v_trapezoidal[i-1]**2 + 2 * MAX_ACCELERATION * ds[i])
        v_trapezoidal[i] = min(v_trapezoidal[i], v_max_by_accel)
    for i in range(len(df) - 2, -1, -1):
        v_max_by_decel = np.sqrt(v_trapezoidal[i+1]**2 + 2 * MAX_DECELERATION * ds[i+1])
        v_trapezoidal[i] = min(v_trapezoidal[i], v_max_by_decel)
        
    # 2. 计算时间戳
    t = np.zeros_like(s)
    if len(v_trapezoidal) > 1:
        v_avg = (v_trapezoidal[:-1] + v_trapezoidal[1:]) / 2.0
        dt_diff = np.diff(s)[1:] / (v_avg[1:] + 1e-6)
        if len(dt_diff) > 0:
            t[2:2+len(dt_diff)] = np.cumsum(dt_diff)
    
    # 3. 用Sigmoid函数替换所有斜坡
    v_s_curve = np.copy(v_trapezoidal)
    is_ramping = ~np.isclose(np.diff(v_trapezoidal), 0, atol=1e-5)
    ramp_starts = np.where(is_ramping & ~np.roll(is_ramping, 1, axis=0))[0]
    ramp_ends = np.where(is_ramping & ~np.roll(is_ramping, -1, axis=0))[0]
    
    if len(ramp_starts) > len(ramp_ends):
        ramp_ends = np.append(ramp_ends, len(v_trapezoidal) - 1)

    for start_idx, end_idx in zip(ramp_starts, ramp_ends):
        if start_idx >= end_idx: continue
        v_start, v_end = v_trapezoidal[start_idx], v_trapezoidal[end_idx]
        t_start, t_end = t[start_idx], t[end_idx]
        T = t_end - t_start
        if T <= 0: continue
        time_segment = t[start_idx:end_idx+1]
        delta_v = v_end - v_start
        t0 = (t_start + t_end) / 2
        k = S_CURVE_STEEPNESS / T
        sigmoid_segment = delta_v / (1 + np.exp(-k * (time_segment - t0))) + v_start
        v_s_curve[start_idx:end_idx+1] = sigmoid_segment
        
    # 4. 【核心修正】对生成的S型曲线进行最终的轻微平滑“抛光”，去除数值噪声
    v_final = gaussian_filter1d(v_s_curve, sigma=FINAL_POLISHING_SIGMA)
    v_final = np.minimum(v_final, velocity_limit) # 确保抛光后不超速

    # 5. 使用最终的完美曲线更新所有物理量
    df['time_s'] = t
    df['planned_velocity_ms'] = v_final
    df['planned_velocity_kmh'] = v_final * 3.6
    df['acceleration_ms2'] = np.gradient(v_final, t, edge_order=2)
    df['jerk_ms3'] = np.gradient(df['acceleration_ms2'].values, t, edge_order=2)
    
    return df

def densify_path(df):
    """对高曲率路段进行插值加密"""
    if not ENABLE_DENSIFICATION:
        return df
    df_chunks = []
    for i in range(len(df) - 1):
        p1, p2 = df.iloc[i], df.iloc[i+1]
        df_chunks.append(p1.to_frame().T)
        if p1['curvature'] > DENSE_CURVATURE_THRESHOLD or p2['curvature'] > DENSE_CURVATURE_THRESHOLD:
            dist = np.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
            num = int(np.ceil(dist / TARGET_DENSITY_M)) - 1
            if num > 0:
                interp_df = pd.DataFrame()
                ratios = np.linspace(0, 1, num + 2)[1:-1]
                for col in ['x', 'y', 'z']:
                    if col in df.columns:
                        interp_df[col] = np.interp(ratios, [0, 1], [p1[col], p2[col]])
                df_chunks.append(interp_df)
    df_chunks.append(df.iloc[-1].to_frame().T)
    df_dense = pd.concat(df_chunks).reset_index(drop=True)
    print(f"路径加密完成：原始点数 {len(df)} -> 加密后点数 {len(df_dense)}")
    return df_dense

def create_and_save_single_plot(df, config, x_range=None, x_ticks=None, tick_length=8):
    """根据单个配置，生成并保存一张独立的、出版风格的图表。"""
    plt.rcParams['font.family'] = 'STIXGeneral'
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 26
    plt.rcParams['axes.linewidth'] = 2
    fig, ax = plt.subplots(figsize=(15, 5))
    data_col, y_range, y_ticks, color, filename = [config.get(k) for k in ['data_col', 'y_range', 'y_ticks', 'color', 'filename']]
    x_data, y_data = df['time_s'], df[data_col]
    ax.plot(x_data, y_data, color=color, linewidth=2.0)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(axis='both', direction='in', which='major', length=tick_length, width=2)
    if y_range: ax.set_ylim(y_range)
    if y_ticks is not None: ax.set_yticks(y_ticks)
    if x_range: ax.set_xlim(x_range)
    if x_ticks is not None: ax.set_xticks(x_ticks)
    ax.set_xlabel('Time (s)')
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"已生成独立的图片文件: '{filename}'")
    plt.close(fig)

def main():
    """主函数"""
    input_filename = 'global_path51782.csv'
    df_path = load_path(input_filename)
    if df_path is None: return
    
    df_path = calculate_curvature(df_path)
    df_dense = densify_path(df_path)
    df_dense = calculate_curvature(df_dense)
    df_dense = plan_motion_profile(df_dense)
    
    print("\n--- 正在生成全局概览图 ---")
    shared_x_range_global = (0, 800)
    shared_x_ticks_global = np.arange(0, 801, 100)
    global_plot_configs = [
        {'filename': 'curvature.png', 'data_col': 'curvature', 'y_range': (-0.1, 0.5), 'y_ticks': np.arange(-0.1, 0.51, 0.2), 'color': 'red'},
        {'filename': 'velocity_final.png', 'data_col': 'planned_velocity_ms', 'y_range': (0.4, 1.6), 'y_ticks': np.arange(0.4, 1.61, 0.4), 'color': 'blue'},
        {'filename': 'acceleration_final.png', 'data_col': 'acceleration_ms2', 'y_range': (-0.4, 0.4), 'y_ticks': np.arange(-0.4, 0.41, 0.2), 'color': 'green'}
    ]
    for config in global_plot_configs:
        create_and_save_single_plot(df_dense, config=config, x_range=shared_x_range_global, x_ticks=shared_x_ticks_global, tick_length=6)
        
    print("\n--- 正在查找加速事件并生成S型曲线放大图 ---")
    accel = df_dense['acceleration_ms2'].values
    time = df_dense['time_s'].values
    v_dense = df_dense['planned_velocity_ms'].values
    positive_accel_indices = np.where(accel > 0.05)[0]
    
    if len(positive_accel_indices) > 0:
        start_idx = positive_accel_indices[0]
        start_time = time[start_idx]
        end_idx = start_idx
        while end_idx + 1 < len(v_dense) and v_dense[end_idx] < V_STRAIGHT_MS - 0.01:
             end_idx +=1
        end_time = time[end_idx]
        
        zoom_start, zoom_end = start_time - 2.0, end_time + 4.0
        print(f"找到加速事件，时间窗口为: ({zoom_start:.2f}s, {zoom_end:.2f}s)")
        
        zoomed_x_range = (zoom_start, zoom_end)
        zoomed_plot_configs = [
            {'filename': 'velocity_final_zoomed.png', 'data_col': 'planned_velocity_ms', 'y_range': (0.5, 1.5), 'y_ticks': np.arange(0.5, 1.51, 0.2), 'color': 'blue'},
            {'filename': 'acceleration_final_zoomed.png', 'data_col': 'acceleration_ms2', 'y_range': (-0.05, 0.35), 'y_ticks': np.arange(0, 0.31, 0.1), 'color': 'green'}
        ]
        
        for config in zoomed_plot_configs:
            create_and_save_single_plot(df_dense, config=config, x_range=zoomed_x_range, tick_length=8)
    else:
        print("未在数据中找到明显的加速事件。")

if __name__ == '__main__':
    main()
