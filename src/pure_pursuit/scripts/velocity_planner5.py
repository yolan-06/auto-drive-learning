import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- 1. 参数定义 ---
V_STRAIGHT_KMH = 5.0
V_CURVE_KMH = 1.0
CURVATURE_THRESHOLD = 0.05
MAX_ACCELERATION = 0.5
MAX_DECELERATION = 0.5

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
    """计算路径上每个点的曲率"""
    x = df['x'].values
    y = df['y'].values
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denominator = (dx**2 + dy**2)**1.5 + 1e-6
    curvature = np.abs(dx * ddy - dy * ddx) / denominator
    df['curvature'] = curvature
    return df

def plan_motion_profile(df):
    """
    【决定版】使用前向/后向遍历约束法规划速度，并使用精确的时间戳计算导数
    """
    s = np.cumsum(np.hypot(np.diff(df['x'], prepend=df['x'][0]), np.diff(df['y'], prepend=df['y'][0])))
    df['s'] = s
    ds = np.diff(s, prepend=0)

    velocity_limit = np.where(df['curvature'] < CURVATURE_THRESHOLD, V_STRAIGHT_MS, V_CURVE_MS)
    planned_velocity = np.copy(velocity_limit)

    # 前向遍历
    for i in range(1, len(df)):
        v_max_by_accel = np.sqrt(planned_velocity[i-1]**2 + 2 * MAX_ACCELERATION * ds[i])
        if planned_velocity[i] > v_max_by_accel:
            planned_velocity[i] = v_max_by_accel

    # 后向遍历
    for i in range(len(df) - 2, -1, -1):
        v_max_by_decel = np.sqrt(planned_velocity[i+1]**2 + 2 * MAX_DECELERATION * ds[i+1])
        if planned_velocity[i] > v_max_by_decel:
            planned_velocity[i] = v_max_by_decel
            
    df['planned_velocity_ms'] = planned_velocity
    df['planned_velocity_kmh'] = planned_velocity * 3.6

    # --- 【核心修正】使用更精确的平均速度来计算时间戳t ---
    v = df['planned_velocity_ms'].values
    s = df['s'].values
    
    # 1. 计算每两点之间的距离增量 ds
    ds_diff = np.diff(s)
    # 2. 计算每两点之间的平均速度 v_avg
    v_avg = (v[:-1] + v[1:]) / 2.0
    # 3. 计算每两点之间的时间间隔 dt = ds / v_avg
    dt_diff = ds_diff / (v_avg + 1e-6) # 加极小值防止除以零
    
    # 4. 计算累积时间戳 t
    t = np.zeros_like(s)
    t[1:] = np.cumsum(dt_diff) # t[0]默认为0
    
    # 5. 使用精确的t，通过中心差分法(np.gradient)计算加速度和加加速度
    acceleration = np.gradient(v, t, edge_order=2)
    df['acceleration_ms2'] = acceleration

    jerk = np.gradient(acceleration, t, edge_order=2)
    df['jerk_ms3'] = jerk
    
    return df

def visualize_results(df):
    """将结果可视化"""
    plt.figure(figsize=(18, 20))
    # ... (代码与上一版相同，为简洁省略) ...
    plt.subplot(4, 1, 1)
    plt.plot(df['s'], df['curvature'], label='Curvature', color='r')
    plt.axhline(y=CURVATURE_THRESHOLD, color='gray', linestyle='--', label=f'Threshold ({CURVATURE_THRESHOLD})')
    plt.title('Path Curvature'); plt.ylabel('Curvature'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 2)
    plt.plot(df['s'], df['planned_velocity_kmh'], label='Planned Velocity (km/h)', color='b')
    plt.title('Velocity Planning'); plt.ylabel('Velocity (km/h)'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 3)
    plt.plot(df['s'], df['acceleration_ms2'], label='Acceleration (m/s^2)', color='g')
    plt.axhline(y=MAX_ACCELERATION, color='gray', linestyle='--', label=f'Max Accel/Decel ({MAX_ACCELERATION})')
    plt.axhline(y=-MAX_DECELERATION, color='gray', linestyle='--')
    plt.title('Acceleration Profile'); plt.ylabel('Acceleration (m/s^2)'); plt.legend(); plt.grid(True)
    plt.ylim(bottom=-MAX_ACCELERATION*1.5, top=MAX_ACCELERATION*1.5) # 设定Y轴范围，让观察更清晰
    plt.subplot(4, 1, 4)
    plt.plot(df['s'], df['jerk_ms3'], label='Jerk (m/s^3)', color='purple')
    plt.title('Jerk Profile'); plt.xlabel('Distance along path (m)'); plt.ylabel('Jerk (m/s^3)'); plt.legend(); plt.grid(True)
    plt.tight_layout()
    output_image_path = 'motion_profile_plot_final.png'
    plt.savefig(output_image_path)
    print(f"可视化图表已保存至 '{output_image_path}'")

def main():
    """主函数"""
    input_filename = 'global_path36412.csv'
    output_filename = 'planned_path_final.csv'
    df_path = load_path(input_filename)
    if df_path is None: return
    df_path = calculate_curvature(df_path)
    df_path = plan_motion_profile(df_path)
    output_columns = ['x', 'y', 'z', 'yaw', 'planned_velocity_ms', 'planned_velocity_kmh', 
                      'curvature', 'acceleration_ms2', 'jerk_ms3']
    df_path[output_columns].to_csv(output_filename, index=False)
    print(f"运动规划完成，结果已保存至 '{output_filename}'")
    visualize_results(df_path)

if __name__ == '__main__':
    main()
