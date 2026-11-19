import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- 1. 参数定义 ---
V_STRAIGHT_KMH = 5.0
V_CURVE_KMH = 1.0
CURVATURE_THRESHOLD = 0.05
TRANSITION_DISTANCE = 2.0

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

def sigmoid(x):
    """标准的Sigmoid函数"""
    return 1 / (1 + np.exp(-x))

def plan_motion_profile(df):
    """规划路径的速度、加速度和加加速度"""
    # 步骤1: 计算累积距离s
    s = np.zeros(len(df))
    for i in range(1, len(df)):
        dist = np.hypot(df['x'][i] - df['x'][i-1], df['y'][i] - df['y'][i-1])
        s[i] = s[i-1] + dist
    df['s'] = s
    
    # 步骤2: 根据曲率初步分配目标速度
    df['target_velocity'] = np.where(df['curvature'] < CURVATURE_THRESHOLD, V_STRAIGHT_MS, V_CURVE_MS)
    
    # 步骤3: 应用S型曲线平滑速度过渡
    planned_velocity = np.copy(df['target_velocity'].values)
    v_diff = V_STRAIGHT_MS - V_CURVE_MS
    
    for i in range(1, len(df) - 1):
        is_entering_curve = df['target_velocity'][i-1] == V_STRAIGHT_MS and df['target_velocity'][i] == V_CURVE_MS
        if is_entering_curve:
            curve_start_s = df['s'][i]
            transition_start_s = curve_start_s - TRANSITION_DISTANCE
            mask = (df['s'] >= transition_start_s) & (df['s'] < curve_start_s)
            s_in_transition = df['s'][mask].values
            if len(s_in_transition) == 0: continue
            x_norm = (s_in_transition - transition_start_s) / TRANSITION_DISTANCE
            x_scaled = 12 * (x_norm - 0.5)
            decay = v_diff * sigmoid(x_scaled)
            planned_velocity[mask] = V_STRAIGHT_MS - decay

        is_exiting_curve = df['target_velocity'][i] == V_CURVE_MS and df['target_velocity'][i+1] == V_STRAIGHT_MS
        if is_exiting_curve:
            curve_end_s = df['s'][i]
            transition_end_s = curve_end_s + TRANSITION_DISTANCE
            mask = (df['s'] > curve_end_s) & (df['s'] <= transition_end_s)
            s_in_transition = df['s'][mask].values
            if len(s_in_transition) == 0: continue
            x_norm = (s_in_transition - curve_end_s) / TRANSITION_DISTANCE
            x_scaled = 12 * (x_norm - 0.5)
            growth = v_diff * sigmoid(x_scaled)
            planned_velocity[mask] = V_CURVE_MS + growth

    df['planned_velocity_ms'] = planned_velocity
    df['planned_velocity_kmh'] = planned_velocity * 3.6

    # 步骤4: 计算加速度
    dv = np.diff(df['planned_velocity_ms'].values)
    ds = np.diff(df['s'].values)
    v_avg = (df['planned_velocity_ms'].values[:-1] + df['planned_velocity_ms'].values[1:]) / 2.0
    dt = ds / (v_avg + 1e-6)
    acceleration = dv / (dt + 1e-6) # 同样加极小值防止除以零
    acceleration = np.insert(acceleration, 0, 0)
    df['acceleration_ms2'] = acceleration

    # --- 新增步骤5: 计算加加速度 (Jerk) ---
    da = np.diff(df['acceleration_ms2'].values) # 加速度变化量
    # dt 数组的长度是N-1，da的长度是N-1，正好可以计算
    jerk = da / (dt + 1e-6)
    # 加加速度数组比路径点少一个，在开头补一个0
    jerk = np.insert(jerk, 0, 0)
    df['jerk_ms3'] = jerk
    
    return df

def visualize_results(df):
    """将结果可视化 (修改为4个子图)"""
    plt.figure(figsize=(18, 20)) # 画布再次调高

    # 子图1: 曲率
    plt.subplot(4, 1, 1)
    plt.plot(df['s'], df['curvature'], label='Curvature', color='r')
    plt.axhline(y=CURVATURE_THRESHOLD, color='gray', linestyle='--', label=f'Threshold ({CURVATURE_THRESHOLD})')
    plt.title('Path Curvature')
    plt.ylabel('Curvature')
    plt.legend()
    plt.grid(True)

    # 子图2: 速度
    plt.subplot(4, 1, 2)
    plt.plot(df['s'], df['target_velocity'] * 3.6, '--', label='Target Velocity (km/h)', color='orange')
    plt.plot(df['s'], df['planned_velocity_kmh'], label='S-Curve Planned Velocity (km/h)', color='b')
    plt.title('Velocity Planning')
    plt.ylabel('Velocity (km/h)')
    plt.legend()
    plt.grid(True)

    # 子图3: 加速度
    plt.subplot(4, 1, 3)
    plt.plot(df['s'], df['acceleration_ms2'], label='Acceleration (m/s^2)', color='g')
    plt.title('Acceleration Profile')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()
    plt.grid(True)

    # --- 新增子图4: 加加速度 (Jerk) ---
    plt.subplot(4, 1, 4)
    plt.plot(df['s'], df['jerk_ms3'], label='Jerk (m/s^3)', color='purple')
    plt.title('Jerk Profile')
    plt.xlabel('Distance along path (m)') # 只在最下面的图标注x轴
    plt.ylabel('Jerk (m/s^3)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    output_image_path = 'motion_profile_plot.png'
    plt.savefig(output_image_path)
    print(f"可视化图表已保存至 '{output_image_path}'")

def main():
    """主函数"""
    input_filename = 'global_path36412.csv'
    output_filename = 'planned_path_with_motion_profile.csv'
    
    df_path = load_path(input_filename)
    if df_path is None:
        return
        
    df_path = calculate_curvature(df_path)
    
    # 函数名已更新
    df_path = plan_motion_profile(df_path)
    
    # 输出列中新增jerk
    output_columns = ['x', 'y', 'z', 'yaw', 'planned_velocity_ms', 'planned_velocity_kmh', 
                      'curvature', 'acceleration_ms2', 'jerk_ms3']
    df_path[output_columns].to_csv(output_filename, index=False)
    print(f"运动规划完成，结果已保存至 '{output_filename}'")
    
    visualize_results(df_path)

if __name__ == '__main__':
    main()
