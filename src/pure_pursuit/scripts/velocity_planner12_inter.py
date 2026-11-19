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
# --- 【最终版】模仿Origin风格的绘图函数，功能最全 ---
# ==============================================================================
def visualize_for_publication(df, y_label='Curvature', tick_length=8, x_range=None, y_range=None, x_ticks=None, y_ticks=None):
    """
    使用Matplotlib生成一个模仿Origin/出版风格的简洁图表。
    最终版：支持自定义标签、刻度长度、坐标轴范围和刻度位置。
    
    :param df: 包含绘图数据的DataFrame。
    :param y_label: (optional) Y轴的标签文本。传入None或空字符串可隐藏标签。
    :param tick_length: (optional) 主刻度线的长度。
    :param x_range: (optional) A tuple (min, max) for the x-axis range.
    :param y_range: (optional) A tuple (min, max) for the y-axis range.
    :param x_ticks: (optional) A list of specific tick locations for the x-axis.
    :param y_ticks: (optional) A list of specific tick locations for the y-axis.
    """
    # 设置全局字体和字号
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 24
    plt.rcParams['axes.linewidth'] = 2

    fig, ax = plt.subplots(figsize=(15, 5))

    # 绘制数据
    x_data = df['time_s']
    y_data = df['curvature']
    ax.plot(x_data, y_data, color='black', linewidth=1.5)

    # --- 风格定制 ---
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    
    # 【核心修改2】使用 tick_length 参数
    ax.tick_params(axis='both', direction='in', which='major', length=tick_length, width=2)

    # 应用自定义范围和刻度
    if x_range:
        ax.set_xlim(x_range)
    else:
        ax.set_xlim(left=0, right=max(x_data) * 1.01)

    if y_range:
        ax.set_ylim(y_range)
    else:
        ax.set_ylim(bottom=0, top=max(y_data) * 1.1)

    if x_ticks is not None:
        ax.set_xticks(x_ticks)
    
    if y_ticks is not None:
        ax.set_yticks(y_ticks)

    # 【核心修改1】根据 y_label 参数决定是否显示标签
    if y_label:
        ax.set_ylabel(y_label, rotation=0, ha='right')
        ax.yaxis.set_label_coords(0.01, 1.02)

    # 保存高清图
    output_image_path = 'final_publication_style_plot.png'
    plt.savefig(output_image_path, dpi=300, bbox_inches='tight')
    print(f"已生成最终版风格图: '{output_image_path}'")
    plt.show()


def main():
    """主函数"""
    input_filename = 'global_path51782.csv'
    
    df_path = load_path(input_filename)
    if df_path is None: return
    
    df_path = calculate_curvature(df_path)
    df_dense = densify_path(df_path)
    df_dense = calculate_curvature(df_dense)
    df_dense = plan_motion_profile(df_dense)
    
    # --- 【使用示例】最终精修 ---
    custom_x_range = (0, 800)
    custom_y_range = (-0.05, 0.5)
    custom_x_ticks = np.arange(0, 801, 100)
    custom_y_ticks = np.arange(0, 0.51, 0.1)
    
    # 调用最终版绘图函数
    visualize_for_publication(
        df_dense,
        y_label=None,             # <--- 不显示Y轴标签
        tick_length=6,           # <--- 将刻度线长度设为10
        x_range=custom_x_range,
        y_range=custom_y_range,
        x_ticks=custom_x_ticks,
        y_ticks=custom_y_ticks
    )

if __name__ == '__main__':
    main()
