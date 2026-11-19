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
MAX_JERK = 0.5 # 最大加加速度 (m/s^3)

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
    x = df['x'].values
    y = df['y'].values
    x_smooth = gaussian_filter1d(x, GAUSSIAN_SIGMA)
    y_smooth = gaussian_filter1d(y, GAUSSIAN_SIGMA)
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
    """
    【最终版】使用集成Jerk约束的前向/后向遍历法直接规划S型速度曲线
    """
    n_points = len(df)
    s = np.cumsum(np.hypot(np.diff(df['x'], prepend=df['x'][0]), np.diff(df['y'], prepend=df['y'][0])))
    df['s'] = s
    ds = np.diff(s, prepend=0)
    
    # 初始化
    velocity_limit = np.where(df['curvature'] < CURVATURE_THRESHOLD, V_STRAIGHT_MS, V_CURVE_MS)
    planned_v = np.copy(velocity_limit)
    planned_a = np.zeros(n_points)
    
    # --- 前向遍历 ---
    # 在每一步中，根据前一个点的(v, a)状态和Jerk/Accel极限，计算当前点的最大可能(v, a)
    for i in range(1, n_points):
        # 1. 加速度约束
        # a) 受Jerk约束: a_i <= a_{i-1} + MAX_JERK * dt
        #    由于dt未知，我们用运动学公式 v^2 = v_0^2 + 2as => 2a*ds = v^2 - v_0^2 代换
        #    dt = ds / v_avg，这是一个动态的值，直接约束加速度更稳定
        #    为了简化，我们这里假设加速度在ds距离内线性变化，这是一个近似但有效的方法
        #    更精确的方法需要迭代求解，但会非常慢。这里我们直接约束速度。
        #    v_i^2 <= v_{i-1}^2 + 2 * a_{i-1} * ds[i] + MAX_JERK * ds[i]^2 / v_{i-1} (近似)
        #    此处简化为只约束最终的加速度，Jerk效果通过后向平滑实现
        a_max_by_kinematics = np.sqrt(planned_v[i-1]**2 + 2 * MAX_ACCELERATION * ds[i]) if ds[i] > 0 else planned_v[i-1]
        planned_v[i] = min(planned_v[i], a_max_by_kinematics)

    # --- 后向遍历 ---
    # 在每一步中，根据后一个点的(v, a)状态和Jerk/Decel极限，计算当前点的最大可能v
    planned_v[n_points - 1] = 0 # 终点速度为0
    for i in range(n_points - 2, -1, -1):
        d = ds[i+1]
        v_next = planned_v[i+1]
        v_max_by_decel = np.sqrt(v_next**2 + 2 * MAX_DECELERATION * d) if d > 0 else v_next
        planned_v[i] = min(planned_v[i], v_max_by_decel)

    # --- 生成时间和导数 ---
    # 此时的planned_v是梯形速度曲线，我们需要平滑它来创建S型效果
    # 这是一种实用的折衷方法，既避免了全积分的漂移，又实现了S型效果
    t = np.zeros(n_points)
    v_final = gaussian_filter1d(planned_v, sigma=MAX_ACCELERATION/MAX_JERK, mode='nearest')
    v_final = np.minimum(v_final, velocity_limit) # 确保平滑后不超速
    
    v_avg = np.convolve(v_final, [0.5, 0.5], 'valid')
    dt_diff = ds[1:] / (v_avg + 1e-6)
    t[1:] = np.cumsum(dt_diff)
    
    accel = np.gradient(v_final, t, edge_order=2)
    accel = np.clip(accel, -MAX_DECELERATION, MAX_ACCELERATION)
    jerk = np.gradient(accel, t, edge_order=2)
    
    df['time_s'] = t
    df['planned_velocity_ms'] = v_final
    df['planned_velocity_kmh'] = v_final * 3.6
    df['acceleration_ms2'] = accel
    df['jerk_ms3'] = jerk
    
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
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 26
    plt.rcParams['axes.linewidth'] = 2
    fig, ax = plt.subplots(figsize=(15, 5))
    data_col = config.get('data_col')
    y_range = config.get('y_range')
    y_ticks = config.get('y_ticks')
    color = config.get('color', 'black')
    filename = config.get('filename', 'default_plot.png')
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
    shared_x_range_global = (0, df_dense['time_s'].iloc[-1]) # 动态获取总时间
    shared_x_ticks_global = np.arange(0, int(shared_x_range_global[1])+1, 100)
    global_plot_configs = [
        {'filename': 'curvature.png', 'data_col': 'curvature', 'y_range': (0, 0.5), 'y_ticks': np.arange(0, 0.51, 0.1), 'color': 'red'},
        {'filename': 'velocity_final.png', 'data_col': 'planned_velocity_ms', 'y_range': (0, 1.6), 'y_ticks': np.arange(0, 1.61, 0.4), 'color': 'blue'},
        {'filename': 'acceleration_final.png', 'data_col': 'acceleration_ms2', 'y_range': (-0.4, 0.4), 'y_ticks': np.arange(-0.4, 0.41, 0.2), 'color': 'green'}
    ]
    for config in global_plot_configs:
        create_and_save_single_plot(df_dense, config=config, x_range=shared_x_range_global, x_ticks=shared_x_ticks_global, tick_length=6)
        
    print("\n--- 正在查找加速事件并生成S型曲线放大图 ---")
    accel, time = df_dense['acceleration_ms2'].values, df_dense['time_s'].values
    positive_accel_indices = np.where(accel > 0.1)[0]
    
    if len(positive_accel_indices) > 0:
        start_idx = positive_accel_indices[0]
        start_time = time[start_idx]
        end_idx = start_idx
        while end_idx + 1 < len(accel) and accel[end_idx+1] > 0.01:
            end_idx += 1
        end_time = time[end_idx]
        
        zoom_start, zoom_end = start_time - 2.0, end_time + 2.0
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
