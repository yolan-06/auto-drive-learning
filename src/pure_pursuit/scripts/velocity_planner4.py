import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- 1. 参数定义 ---
V_STRAIGHT_KMH = 5.0
V_CURVE_KMH = 1.0
CURVATURE_THRESHOLD = 0.05
MAX_ACCELERATION = 0.2
MAX_JERK = 0.5

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

def generate_s_curve_profile(v_start, v_end, max_a, max_j):
    """根据给定的起止速度和运动学约束，生成一个S型速度曲线的模板。"""
    delta_v = abs(v_end - v_start)
    
    if delta_v < 1e-6:
        return np.array([0]), np.array([v_start])

    t_j = max_a / max_j
    delta_v_j = max_a * t_j

    if delta_v >= delta_v_j:
        t_a = (delta_v / max_a) - t_j
        total_time = 2 * t_j + t_a
    else:
        t_j = np.sqrt(delta_v / max_j)
        t_a = 0
        max_a = max_j * t_j
        total_time = 2 * t_j
        
    if total_time == 0:
        return np.array([0]), np.array([v_start])

    t_steps = np.linspace(0, total_time, 200)
    v_base_profile = np.zeros_like(t_steps)
    s_base_profile = np.zeros_like(t_steps)
    
    v_p1_end = 0.5 * max_j * t_j**2
    s_p1_end = (1/6) * max_j * t_j**3
    v_p2_end = v_p1_end + max_a * t_a
    s_p2_end = s_p1_end + v_p1_end * t_a + 0.5 * max_a * t_a**2

    for i, t in enumerate(t_steps):
        if t <= t_j:
            v_base_profile[i] = 0.5 * max_j * t**2
            s_base_profile[i] = (1/6) * max_j * t**3
        elif t <= t_j + t_a:
            t_rem = t - t_j
            v_base_profile[i] = v_p1_end + max_a * t_rem
            s_base_profile[i] = s_p1_end + v_p1_end * t_rem + 0.5 * max_a * t_rem**2
        else:
            t_rem = t - (t_j + t_a)
            v_base_profile[i] = v_p2_end + max_a * t_rem - 0.5 * max_j * t_rem**2
            s_base_profile[i] = s_p2_end + v_p2_end * t_rem + 0.5 * max_a * t_rem**2 - (1/6) * max_j * t_rem**3
    
    if v_end > v_start:
        v_final = v_base_profile + v_start
        s_final = s_base_profile
    else:
        v_final = v_start - v_base_profile
        s_final = v_start * t_steps - s_base_profile

    return s_final, v_final

def plan_motion_profile(df):
    """规划路径的速度、加速度和加加速度"""
    df['s'] = np.cumsum(np.hypot(np.diff(df['x'], prepend=df['x'][0]), np.diff(df['y'], prepend=df['y'][0])))
    df['target_velocity'] = np.where(df['curvature'] < CURVATURE_THRESHOLD, V_STRAIGHT_MS, V_CURVE_MS)
    
    planned_velocity = np.copy(df['target_velocity'].values)
    
    s_profile_decel, v_profile_decel = generate_s_curve_profile(V_STRAIGHT_MS, V_CURVE_MS, MAX_ACCELERATION, MAX_JERK)
    s_profile_accel, v_profile_accel = generate_s_curve_profile(V_CURVE_MS, V_STRAIGHT_MS, MAX_ACCELERATION, MAX_JERK)
    
    if s_profile_decel.size > 1:
        transition_dist_decel = s_profile_decel[-1]
        for i in range(1, len(df) - 1):
            if planned_velocity[i-1] == V_STRAIGHT_MS and planned_velocity[i] == V_CURVE_MS:
                curve_start_s = df['s'][i]
                transition_start_s = curve_start_s - transition_dist_decel
                mask = (df['s'] >= transition_start_s) & (df['s'] < curve_start_s)
                s_in_transition = df['s'][mask].values
                if len(s_in_transition) > 0:
                    planned_velocity[mask] = np.interp(s_in_transition, s_profile_decel + transition_start_s, v_profile_decel)

    if s_profile_accel.size > 1:
        transition_dist_accel = s_profile_accel[-1]
        for i in range(1, len(df) - 1):
            if planned_velocity[i] == V_CURVE_MS and planned_velocity[i+1] == V_STRAIGHT_MS:
                curve_end_s = df['s'][i]
                mask = (df['s'] > curve_end_s) & (df['s'] <= curve_end_s + transition_dist_accel)
                s_in_transition = df['s'][mask].values
                if len(s_in_transition) > 0:
                    planned_velocity[mask] = np.interp(s_in_transition, s_profile_accel + curve_end_s, v_profile_accel)
    
    df['planned_velocity_ms'] = planned_velocity
    df['planned_velocity_kmh'] = planned_velocity * 3.6

    # --- 【核心修正】采用更鲁棒的np.gradient方法计算导数 ---
    v = df['planned_velocity_ms'].values
    s = df['s'].values
    
    # 1. 首先，根据每个点的速度，反算出每个点对应的时间戳t
    #    (使用np.gradient计算距离增量ds，以提高精度)
    ds = np.gradient(s)
    #    避免除以零
    v_for_dt = np.where(v > 1e-4, v, 1e-4)
    dt = ds / v_for_dt
    t = np.cumsum(dt) # 得到累积时间数组
    
    # 2. 使用np.gradient计算速度对时间的变化率，即加速度
    #    edge_order=2可以提高边界点的计算精度
    acceleration = np.gradient(v, t, edge_order=2)
    df['acceleration_ms2'] = acceleration

    # 3. 同样地，计算加速度对时间的变化率，即加加速度
    jerk = np.gradient(acceleration, t, edge_order=2)
    df['jerk_ms3'] = jerk
    
    return df

def visualize_results(df):
    """将结果可视化"""
    # (此函数无需修改)
    plt.figure(figsize=(18, 20))
    # ... (代码与上一版相同，为简洁省略) ...
    plt.subplot(4, 1, 1)
    plt.plot(df['s'], df['curvature'], label='Curvature', color='r')
    plt.axhline(y=CURVATURE_THRESHOLD, color='gray', linestyle='--', label=f'Threshold ({CURVATURE_THRESHOLD})')
    plt.title('Path Curvature'); plt.ylabel('Curvature'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 2)
    plt.plot(df['s'], df['target_velocity'] * 3.6, '--', label='Target Velocity (km/h)', color='orange')
    plt.plot(df['s'], df['planned_velocity_kmh'], label='S-Curve Planned Velocity (km/h)', color='b')
    plt.title('Velocity Planning'); plt.ylabel('Velocity (km/h)'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 3)
    plt.plot(df['s'], df['acceleration_ms2'], label='Acceleration (m/s^2)', color='g')
    plt.axhline(y=MAX_ACCELERATION, color='gray', linestyle='--', label=f'Max Accel ({MAX_ACCELERATION})')
    plt.axhline(y=-MAX_ACCELERATION, color='gray', linestyle='--')
    plt.title('Acceleration Profile'); plt.ylabel('Acceleration (m/s^2)'); plt.legend(); plt.grid(True)
    plt.subplot(4, 1, 4)
    plt.plot(df['s'], df['jerk_ms3'], label='Jerk (m/s^3)', color='purple')
    plt.title('Jerk Profile'); plt.xlabel('Distance along path (m)'); plt.ylabel('Jerk (m/s^3)'); plt.legend(); plt.grid(True)
    plt.tight_layout()
    output_image_path = 'motion_profile_plot.png'
    plt.savefig(output_image_path)
    print(f"可视化图表已保存至 '{output_image_path}'")


def main():
    """主函数"""
    # (此函数无需修改)
    input_filename = 'global_path36412.csv'
    output_filename = 'planned_path_with_7segment_profile.csv'
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
