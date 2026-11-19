import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

# --- 1. 参数定义 ---
V_STRAIGHT_KMH = 4.0
V_CURVE_KMH = 1
CURVATURE_THRESHOLD = 0.05
MAX_ACCELERATION = 0.1
MAX_DECELERATION = 0.1
GAUSSIAN_SIGMA = 3

# --- 【新增】路径加密功能的参数 ---
ENABLE_DENSIFICATION = True      # 功能总开关
DENSE_CURVATURE_THRESHOLD = 0.1  # 触发加密的曲率阈值
TARGET_DENSITY_M = 0.1           # 加密后弯道中的目标点间距(米)

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
    """使用多级高斯滤波计算平滑曲率，并更新路径点"""
    x = df['x'].values
    y = df['y'].values
    x_smooth = gaussian_filter1d(x, GAUSSIAN_SIGMA)
    y_smooth = gaussian_filter1d(y, GAUSSIAN_SIGMA)
    df['x'] = x_smooth
    df['y'] = y_smooth
    dx = np.gradient(x_smooth)
    dy = np.gradient(y_smooth)
    dx_smooth = gaussian_filter1d(dx, GAUSSIAN_SIGMA)
    dy_smooth = gaussian_filter1d(dy, GAUSSIAN_SIGMA)
    ddx = np.gradient(dx_smooth)
    ddy = np.gradient(dy_smooth)
    df['yaw'] = np.arctan2(dy_smooth, dx_smooth)
    denominator = (dx_smooth**2 + dy_smooth**2)**1.5 + 1e-6
    curvature = np.abs(dx_smooth * ddy - dy_smooth * ddx) / denominator
    df['curvature'] = curvature
    return df

def plan_motion_profile(df):
    """使用前向/后向遍历约束法规划速度"""
    # (此函数无需修改，为简洁省略)
    s = np.cumsum(np.hypot(np.diff(df['x'], prepend=df['x'][0]), np.diff(df['y'], prepend=df['y'][0])))
    df['s'] = s
    ds = np.diff(s, prepend=0)
    velocity_limit = np.where(df['curvature'] < CURVATURE_THRESHOLD, V_STRAIGHT_MS, V_CURVE_MS)
    planned_velocity = np.copy(velocity_limit)
    for i in range(1, len(df)):
        v_max_by_accel = np.sqrt(planned_velocity[i-1]**2 + 2 * MAX_ACCELERATION * ds[i])
        if planned_velocity[i] > v_max_by_accel:
            planned_velocity[i] = v_max_by_accel
    for i in range(len(df) - 2, -1, -1):
        v_max_by_decel = np.sqrt(planned_velocity[i+1]**2 + 2 * MAX_DECELERATION * ds[i+1])
        if planned_velocity[i] > v_max_by_decel:
            planned_velocity[i] = v_max_by_decel
    df['planned_velocity_ms'] = planned_velocity
    df['planned_velocity_kmh'] = planned_velocity * 3.6
    v = df['planned_velocity_ms'].values
    s = df['s'].values
    ds_diff = np.diff(s)
    v_avg = (v[:-1] + v[1:]) / 2.0
    dt_diff = ds_diff / (v_avg + 1e-6)
    t = np.zeros_like(s)
    t[1:] = np.cumsum(dt_diff)
    df['time_s'] = t
    acceleration = np.gradient(v, t, edge_order=2)
    df['acceleration_ms2'] = acceleration
    jerk = np.gradient(acceleration, t, edge_order=2)
    df['jerk_ms3'] = jerk
    return df

def densify_path(df):
    """【新增】对高曲率路段进行插值加密"""
    if not ENABLE_DENSIFICATION:
        print("路径加密功能已禁用。")
        return df

    # 用于存储新路径的Dataframe片段
    df_chunks = []
    
    # 遍历所有路径段
    for i in range(len(df) - 1):
        p1 = df.iloc[i]
        p2 = df.iloc[i+1]

        # 将段的起点加入列表
        df_chunks.append(p1.to_frame().T)

        # 检查该段是否需要加密 (起点或终点的曲率大于阈值)
        if p1['curvature'] > DENSE_CURVATURE_THRESHOLD or p2['curvature'] > DENSE_CURVATURE_THRESHOLD:
            distance = np.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
            # 计算需要插入多少个点
            num_new_points = int(np.ceil(distance / TARGET_DENSITY_M)) - 1

            if num_new_points > 0:
                interp_df = pd.DataFrame()
                # 创建插值比例数组，例如[0.33, 0.66]代表插入2个点
                interp_ratios = np.linspace(0, 1, num_new_points + 2)[1:-1]

                # 对所有数值列进行线性插值
                for col in df.columns:
                    if col != 'yaw': # yaw需要单独处理
                         interp_df[col] = np.interp(interp_ratios, [0, 1], [p1[col], p2[col]])
                
                # 对偏航角进行特殊插值，处理-PI到PI的跳变问题
                delta_yaw = p2['yaw'] - p1['yaw']
                if delta_yaw > np.pi: delta_yaw -= 2 * np.pi
                elif delta_yaw < -np.pi: delta_yaw += 2 * np.pi
                
                interp_df['yaw'] = p1['yaw'] + delta_yaw * interp_ratios
                # 将插值后的yaw归一化到[-pi, pi]范围
                interp_df['yaw'] = (interp_df['yaw'] + np.pi) % (2 * np.pi) - np.pi

                df_chunks.append(interp_df)
    
    # 加入原始路径的最后一个点
    df_chunks.append(df.iloc[-1].to_frame().T)

    # 将所有片段拼接成一个新的DataFrame
    df_dense = pd.concat(df_chunks).reset_index(drop=True)
    print(f"路径加密完成：原始点数 {len(df)} -> 加密后点数 {len(df_dense)}")
    return df_dense


def visualize_results_vs_distance(df):
    """将结果可视化 (X轴为距离)"""
    # (此函数无需修改，为简洁省略)
    plt.figure(figsize=(18, 20))
    plt.subplot(4, 1, 1); plt.plot(df['s'], df['curvature'], label='Curvature', color='r'); plt.axhline(y=CURVATURE_THRESHOLD, color='gray', linestyle='--', label=f'Threshold ({CURVATURE_THRESHOLD})'); plt.title('Path Curvature vs. Distance'); plt.ylabel('Curvature'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 2); plt.plot(df['s'], df['planned_velocity_kmh'], label='Planned Velocity (km/h)', color='b'); plt.title('Velocity vs. Distance'); plt.ylabel('Velocity (km/h)'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 3); plt.plot(df['s'], df['acceleration_ms2'], label='Acceleration (m/s^2)', color='g'); plt.axhline(y=MAX_ACCELERATION, color='gray', linestyle='--', label=f'Max Accel/Decel ({MAX_ACCELERATION})'); plt.axhline(y=-MAX_DECELERATION, color='gray', linestyle='--'); plt.title('Acceleration vs. Distance'); plt.ylabel('Acceleration (m/s^2)'); plt.legend(); plt.grid(True); plt.ylim(bottom=-MAX_ACCELERATION*1.5, top=MAX_ACCELERATION*1.5)
    plt.subplot(4, 1, 4); plt.plot(df['s'], df['jerk_ms3'], label='Jerk (m/s^3)', color='purple'); plt.title('Jerk vs. Distance'); plt.xlabel('Distance along path (m)'); plt.ylabel('Jerk (m/s^3)'); plt.legend(); plt.grid(True)
    plt.tight_layout()
    output_image_path = 'motion_profile_vs_distance.png'
    plt.savefig(output_image_path)
    print(f"距离-运动曲线图已保存至 '{output_image_path}'")

def visualize_results_vs_time(df):
    """将结果可视化 (X轴为时间)"""
    # (此函数无需修改，为简洁省略)
    plt.figure(figsize=(18, 20))
    plt.subplot(4, 1, 1); plt.plot(df['time_s'], df['curvature'], label='Curvature', color='r'); plt.axhline(y=CURVATURE_THRESHOLD, color='gray', linestyle='--', label=f'Threshold ({CURVATURE_THRESHOLD})'); plt.title('Path Curvature vs. Time'); plt.ylabel('Curvature'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 2); plt.plot(df['time_s'], df['planned_velocity_kmh'], label='Planned Velocity (km/h)', color='b'); plt.title('Velocity vs. Time'); plt.ylabel('Velocity (km/h)'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 3); plt.plot(df['time_s'], df['acceleration_ms2'], label='Acceleration (m/s^2)', color='g'); plt.axhline(y=MAX_ACCELERATION, color='gray', linestyle='--', label=f'Max Accel/Decel ({MAX_ACCELERATION})'); plt.axhline(y=-MAX_DECELERATION, color='gray', linestyle='--'); plt.title('Acceleration vs. Time'); plt.ylabel('Acceleration (m/s^2)'); plt.legend(); plt.grid(True); plt.ylim(bottom=-MAX_ACCELERATION*1.5, top=MAX_ACCELERATION*1.5)
    plt.subplot(4, 1, 4); plt.plot(df['time_s'], df['jerk_ms3'], label='Jerk (m/s^3)', color='purple'); plt.title('Jerk vs. Time'); plt.xlabel('Time (s)'); plt.ylabel('Jerk (m/s^3)'); plt.legend(); plt.grid(True)
    plt.tight_layout()
    output_image_path = 'motion_profile_vs_time.png'
    plt.savefig(output_image_path)
    print(f"时间-运动曲线图已保存至 '{output_image_path}'")

def main():
    """主函数"""
    input_filename = 'global_path51092.csv'
    output_analysis_filename = 'planned_path_with_analysis.csv'
    output_path_filename = 'global_path_with_velocity.csv'
    
    df_path = load_path(input_filename)
    if df_path is None: return
    
    original_columns = df_path.columns.tolist()
    
    df_path = calculate_curvature(df_path)
    df_path = plan_motion_profile(df_path)
    
    # --- 【核心修改】在所有规划完成后，进行路径加密 ---
    df_dense = densify_path(df_path)
    
    # --- 【核心修改】后续的保存和可视化，都使用加密后的df_dense ---
    df_dense['velocity'] = df_dense['planned_velocity_kmh']
    if all(col in df_dense.columns for col in original_columns):
        df_path_new = df_dense[original_columns]
        df_path_new.to_csv(output_path_filename, index=False)
        print(f"已生成带速度的加密路径文件 (单位: km/h): '{output_path_filename}'")
    else:
        print("警告: 原始文件的列不完整，无法生成新路径文件。")

    output_analysis_columns = ['time_s', 'x', 'y', 'z', 'yaw', 's', 'planned_velocity_ms', 
                               'planned_velocity_kmh', 'curvature', 'acceleration_ms2', 'jerk_ms3']
    df_dense[output_analysis_columns].to_csv(output_analysis_filename, index=False)
    print(f"运动规划分析文件(加密后)已保存至 '{output_analysis_filename}'")
    
    visualize_results_vs_distance(df_dense)
    visualize_results_vs_time(df_dense)

if __name__ == '__main__':
    main()
