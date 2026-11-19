import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- 1. 参数定义 ---
# 可以根据实际效果调整这些参数
V_STRAIGHT_KMH = 5.0  # 直线速度 (km/h)
V_CURVE_KMH = 1.0    # 弯道速度 (km/h)
CURVATURE_THRESHOLD = 0.05  # 曲率阈值，大于此值为弯道
TRANSITION_DISTANCE = 2.0 # 速度过渡区的长度（米）

# --- 2. 单位转换 ---
V_STRAIGHT_MS = V_STRAIGHT_KMH / 3.6  # m/s
V_CURVE_MS = V_CURVE_KMH / 3.6      # m/s

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

    # 使用numpy的gradient函数计算一阶和二阶导数，更稳定
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    # 曲率计算公式: k = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    # 添加一个极小值 epsilon 防止分母为零
    denominator = (dx**2 + dy**2)**1.5 + 1e-6
    curvature = np.abs(dx * ddy - dy * ddx) / denominator
    
    df['curvature'] = curvature
    return df

def sigmoid(x):
    """标准的Sigmoid函数"""
    return 1 / (1 + np.exp(-x))

def plan_velocity(df):
    """规划路径速度"""
    # 步骤1: 计算累积距离s
    s = np.zeros(len(df))
    for i in range(1, len(df)):
        dist = np.hypot(df['x'][i] - df['x'][i-1], df['y'][i] - df['y'][i-1])
        s[i] = s[i-1] + dist
    df['s'] = s
    
    # 步骤2: 根据曲率初步分配目标速度
    df['target_velocity'] = np.where(df['curvature'] < CURVATURE_THRESHOLD, V_STRAIGHT_MS, V_CURVE_MS)
    
    # 步骤3: 应用S型曲线平滑速度过渡
    # 创建最终的速度规划列
    planned_velocity = np.copy(df['target_velocity'].values)
    
    v_diff = V_STRAIGHT_MS - V_CURVE_MS
    
    for i in range(1, len(df) - 1):
        # 检测到进入弯道（减速区）
        is_entering_curve = df['target_velocity'][i-1] == V_STRAIGHT_MS and df['target_velocity'][i] == V_CURVE_MS
        if is_entering_curve:
            curve_start_s = df['s'][i]
            transition_start_s = curve_start_s - TRANSITION_DISTANCE
            
            # 在过渡区内应用S型减速曲线
            mask = (df['s'] >= transition_start_s) & (df['s'] < curve_start_s)
            s_in_transition = df['s'][mask].values
            
            # 将过渡区距离归一化到[-6, 6]之间，以利用sigmoid函数的主要变化范围
            x_norm = (s_in_transition - transition_start_s) / TRANSITION_DISTANCE # 0 to 1
            x_scaled = 12 * (x_norm - 0.5) # -6 to 6
            
            # 计算速度衰减
            decay = v_diff * sigmoid(x_scaled)
            planned_velocity[mask] = V_STRAIGHT_MS - decay

        # 检测到离开弯道（加速区）
        is_exiting_curve = df['target_velocity'][i] == V_CURVE_MS and df['target_velocity'][i+1] == V_STRAIGHT_MS
        if is_exiting_curve:
            curve_end_s = df['s'][i]
            transition_end_s = curve_end_s + TRANSITION_DISTANCE

            # 在过渡区内应用S型加速曲线
            mask = (df['s'] > curve_end_s) & (df['s'] <= transition_end_s)
            s_in_transition = df['s'][mask].values
            
            x_norm = (s_in_transition - curve_end_s) / TRANSITION_DISTANCE # 0 to 1
            x_scaled = 12 * (x_norm - 0.5) # -6 to 6

            # 计算速度增长
            growth = v_diff * sigmoid(x_scaled)
            planned_velocity[mask] = V_CURVE_MS + growth
            
    df['planned_velocity_ms'] = planned_velocity
    df['planned_velocity_kmh'] = planned_velocity * 3.6
    return df

def visualize_results(df):
    """将结果可视化"""
    plt.figure(figsize=(18, 10))

    # 绘制曲率
    plt.subplot(2, 1, 1)
    plt.plot(df['s'], df['curvature'], label='Curvature', color='r')
    plt.axhline(y=CURVATURE_THRESHOLD, color='gray', linestyle='--', label=f'Threshold ({CURVATURE_THRESHOLD})')
    plt.title('Path Curvature')
    plt.xlabel('Distance along path (m)')
    plt.ylabel('Curvature')
    plt.legend()
    plt.grid(True)

    # 绘制速度
    plt.subplot(2, 1, 2)
    plt.plot(df['s'], df['target_velocity'] * 3.6, '--', label='Target Velocity (km/h)', color='orange')
    plt.plot(df['s'], df['planned_velocity_kmh'], label='S-Curve Planned Velocity (km/h)', color='b')
    plt.title('Velocity Planning')
    plt.xlabel('Distance along path (m)')
    plt.ylabel('Velocity (km/h)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    output_image_path = 'velocity_planning_plot.png'
    plt.savefig(output_image_path)
    print(f"可视化图表已保存至 '{output_image_path}'")
    # plt.show()


def main():
    """主函数"""
    input_filename = 'global_path36412.csv'
    output_filename = 'planned_path_with_velocity.csv'
    
    # 1. 加载路径
    df_path = load_path(input_filename)
    if df_path is None:
        return
        
    # 2. 计算曲率
    df_path = calculate_curvature(df_path)
    
    # 3. 规划速度
    df_path = plan_velocity(df_path)
    
    # 4. 保存结果
    # 选择需要保存的列
    output_columns = ['x', 'y', 'z', 'yaw', 'planned_velocity_ms', 'planned_velocity_kmh', 'curvature']
    df_path[output_columns].to_csv(output_filename, index=False)
    print(f"速度规划完成，结果已保存至 '{output_filename}'")
    
    # 5. 可视化结果
    visualize_results(df_path)

if __name__ == '__main__':
    main()
