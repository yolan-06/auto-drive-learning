import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
# 【新增】导入scipy的高斯滤波器
from scipy.ndimage import gaussian_filter1d

# --- 1. 参数定义 ---
V_STRAIGHT_KMH = 5.0
V_CURVE_KMH = 2.0
CURVATURE_THRESHOLD = 0.05
MAX_ACCELERATION = 0.5
MAX_DECELERATION = 0.5
# 【新增】高斯滤波器的Sigma值。这是现在控制曲率平滑度的关键参数。
# 值越大，平滑效果越强。您可以从较小的值开始尝试，如3, 5, 7...
GAUSSIAN_SIGMA = 3 # 可调参数

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
    """【最终修正】使用多级高斯滤波计算平滑曲率"""
    x = df['x'].values
    y = df['y'].values
    
    # --- 【核心修改】多级高斯滤波 ---
    # 1. 对原始坐标进行第一次平滑
    x_smooth = gaussian_filter1d(x, GAUSSIAN_SIGMA)
    y_smooth = gaussian_filter1d(y, GAUSSIAN_SIGMA)

    # 2. 基于平滑后的坐标计算一阶导数
    dx = np.gradient(x_smooth)
    dy = np.gradient(y_smooth)
    
    # 3. 对一阶导数进行第二次平滑，为计算二阶导数做准备
    dx_smooth = gaussian_filter1d(dx, GAUSSIAN_SIGMA)
    dy_smooth = gaussian_filter1d(dy, GAUSSIAN_SIGMA)

    # 4. 基于二次平滑后的一阶导数计算二阶导数
    ddx = np.gradient(dx_smooth)
    ddy = np.gradient(dy_smooth)
    
    # 5. 使用平滑后的一阶和二阶导数计算最终曲率
    #    注意：分母使用平滑后的一阶导数，保证一致性
    denominator = (dx_smooth**2 + dy_smooth**2)**1.5 + 1e-6
    curvature = np.abs(dx_smooth * ddy - dy_smooth * ddx) / denominator
    
    df['curvature'] = curvature
    return df

def plan_motion_profile(df):
    """
    使用前向/后向遍历约束法规划速度
    """
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
    input_filename = 'global_path34494.csv'
    # (main函数无需修改，为简洁省略)
    output_analysis_filename = 'planned_path_with_analysis.csv'
    output_path_filename = 'global_path_with_velocity.csv'
    df_path = load_path(input_filename)
    if df_path is None: return
    original_columns = df_path.columns.tolist()
    df_path = calculate_curvature(df_path)
    df_path = plan_motion_profile(df_path)
    df_path['velocity'] = df_path['planned_velocity_kmh']
    if all(col in df_path.columns for col in original_columns):
        df_path_new = df_path[original_columns]
        df_path_new.to_csv(output_path_filename, index=False)
        print(f"已生成带速度的新路径文件 (单位: km/h): '{output_path_filename}'")
    else:
        print("警告: 原始文件的列不完整，无法生成新路径文件。")
    output_analysis_columns = ['time_s', 'x', 'y', 'z', 'yaw', 's', 'planned_velocity_ms', 
                               'planned_velocity_kmh', 'curvature', 'acceleration_ms2', 'jerk_ms3']
    df_path[output_analysis_columns].to_csv(output_analysis_filename, index=False)
    print(f"运动规划分析文件已保存至 '{output_analysis_filename}'")
    visualize_results_vs_distance(df_path)
    visualize_results_vs_time(df_path)

if __name__ == '__main__':
    main()
