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
    """对高曲率路段进行插值加密"""
    if not ENABLE_DENSIFICATION:
        print("路径加密功能已禁用。")
        return df
    df_chunks = []
    for i in range(len(df) - 1):
        p1 = df.iloc[i]
        p2 = df.iloc[i+1]
        df_chunks.append(p1.to_frame().T)
        if p1['curvature'] > DENSE_CURVATURE_THRESHOLD or p2['curvature'] > DENSE_CURVATURE_THRESHOLD:
            distance = np.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
            num_new_points = int(np.ceil(distance / TARGET_DENSITY_M)) - 1
            if num_new_points > 0:
                interp_df = pd.DataFrame()
                interp_ratios = np.linspace(0, 1, num_new_points + 2)[1:-1]
                columns_to_interpolate = ['x', 'y', 'z']
                for col in columns_to_interpolate:
                    if col in df.columns:
                        interp_df[col] = np.interp(interp_ratios, [0, 1], [p1[col], p2[col]])
                df_chunks.append(interp_df)
    df_chunks.append(df.iloc[-1].to_frame().T)
    df_dense = pd.concat(df_chunks).reset_index(drop=True)
    print(f"路径加密完成：原始点数 {len(df)} -> 加密后点数 {len(df_dense)}")
    return df_dense

# ==============================================================================
# --- 【单图版】生成并保存单个出版级图表 ---
# ==============================================================================
def create_and_save_single_plot(df, config, x_range=None, x_ticks=None, tick_length=8):
    """
    根据单个配置，生成并保存一张独立的、出版风格的图表。
    
    :param df: 包含绘图数据的DataFrame。
    :param config: 包含此图所有配置的字典。
    :param x_range: (optional) 共享的X轴范围, e.g., (0, 800)。
    :param x_ticks: (optional) 共享的X轴刻度位置列表。
    :param tick_length: (optional) 所有坐标轴主刻度线的长度。
    """
    # 设置全局字体和字号
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 26
    plt.rcParams['axes.linewidth'] = 2
    
    # 每次调用都创建一个全新的图
    fig, ax = plt.subplots(figsize=(15, 5))

    # 从配置中获取信息
    data_col = config.get('data_col')
    y_range = config.get('y_range')
    y_ticks = config.get('y_ticks')
    color = config.get('color', 'black')
    filename = config.get('filename', 'default_plot.png') # 获取独立的文件名
    
    # 绘制数据
    x_data = df['time_s']
    y_data = df[data_col]
    ax.plot(x_data, y_data, color=color, linewidth=1.5)

    # --- 应用统一的风格 ---
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(axis='both', direction='in', which='major', length=tick_length, width=2)

    # 应用Y轴的自定义设置
    if y_range:
        ax.set_ylim(y_range)
    if y_ticks is not None:
        ax.set_yticks(y_ticks)
    
    # 应用X轴的自定义设置
    if x_range:
        ax.set_xlim(x_range)
    if x_ticks is not None:
        ax.set_xticks(x_ticks)
    
    # 设置X轴标签
    ax.set_xlabel('Time (s)')

    # 保存高清图
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"已生成独立的图片文件: '{filename}'")
    
    # 关闭图形，防止在循环中互相干扰并释放内存
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
    
    # --- 【核心配置】在这里定义所有独立的图表 ---
    
    # 1. 定义共享的X轴样式
    shared_x_range = (0, 800)
    shared_x_ticks = np.arange(0, 801, 100)
    
    # 2. 定义每个图表的独立样式，包括文件名
    plot_configurations = [
        {
            'filename': 'curvature.png', # <--- 文件名1
            'data_col': 'curvature',
            'y_range': (-0.1, 0.5),
            'y_ticks': np.arange(-0.1, 0.51, 0.2),
            'color': 'red'
        },
        {
            'filename': 'velocity.png', # <--- 文件名2
            'data_col': 'planned_velocity_ms',
            'y_range': (0.4, 1.6),
            'y_ticks': np.arange(0.4, 1.61, 0.4),
            'color': 'blue'
        },
        {
            'filename': 'acceleration.png', # <--- 文件名3
            'data_col': 'acceleration_ms2',
            'y_range': (-0.4, 0.4),
            'y_ticks': np.arange(-0.4, 0.41, 0.2),
            'color': 'green'
        }
    ]
    
    # 3. 循环调用单图生成函数，创建所有独立的图片
    for config in plot_configurations:
        create_and_save_single_plot(
            df_dense,
            config=config,
            x_range=shared_x_range,
            x_ticks=shared_x_ticks,
            tick_length=6
        )

if __name__ == '__main__':
    main()
