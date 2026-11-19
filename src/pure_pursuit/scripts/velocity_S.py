import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def generate_and_save_s_curve_data():
    """
    生成S型速度曲线的数值数据，并将其保存到CSV文件中。
    同时，也会生成曲线图用于可视化。
    """
    # --- 1. 定义运动参数 ---
    J_MAX = 2.0  # 最大加加速度 (Jerk) m/s^3
    A_MAX = 1.5  # 最大加速度 m/s^2
    V_MAX = 3.0  # 最大速度 m/s
    T_CRUISE = 2.0 # 匀速阶段的持续时间 (s)

    # --- 参数有效性检查 ---
    if V_MAX * J_MAX < A_MAX**2:
        print("警告: V_MAX太小，无法达到A_MAX。自动调整A_MAX以满足约束条件。")
        A_MAX = np.sqrt(V_MAX * J_MAX)
        print(f"A_MAX 已自动调整为: {A_MAX:.2f} m/s^2")

    # --- 2. 计算各个阶段的时间点 ---
    t1 = A_MAX / J_MAX
    t2 = V_MAX / A_MAX
    t3 = t2 + t1
    t4 = t3 + T_CRUISE
    t5 = t4 + t1
    t6 = t5 + (t2 - t1)
    t7 = t6 + t1

    # --- 3. 生成时间和数据数组 ---
    # 创建一个高分辨率的时间轴，用于精确计算
    t = np.linspace(0, t7, 1000)
    jerk = np.zeros_like(t)
    accel = np.zeros_like(t)
    vel = np.zeros_like(t)

    # --- 4. 分段计算Jerk, 加速度和速度 ---
    # 计算各阶段结束时的速度
    v_t1 = 0.5 * J_MAX * t1**2
    v_t2 = v_t1 + A_MAX * (t2 - t1)
    v_t4 = V_MAX
    v_t5 = v_t4 - 0.5 * J_MAX * (t5 - t4)**2
    v_t6 = v_t5 - A_MAX * (t6 - t5)

    # 使用布尔索引进行矢量化计算
    mask1 = (t >= 0) & (t < t1); mask2 = (t >= t1) & (t < t2); mask3 = (t >= t2) & (t < t3)
    mask4 = (t >= t3) & (t < t4); mask5 = (t >= t4) & (t < t5); mask6 = (t >= t5) & (t < t6)
    mask7 = (t >= t6) & (t <= t7)

    jerk[mask1] = J_MAX; jerk[mask3] = -J_MAX; jerk[mask5] = -J_MAX; jerk[mask7] = J_MAX
    
    accel[mask1] = J_MAX * t[mask1]
    accel[mask2] = A_MAX
    accel[mask3] = A_MAX - J_MAX * (t[mask3] - t2)
    accel[mask5] = -J_MAX * (t[mask5] - t4)
    accel[mask6] = -A_MAX
    accel[mask7] = -A_MAX + J_MAX * (t[mask7] - t6)

    vel[mask1] = 0.5 * J_MAX * t[mask1]**2
    vel[mask2] = v_t1 + A_MAX * (t[mask2] - t1)
    vel[mask3] = v_t2 + A_MAX * (t[mask3] - t2) - 0.5 * J_MAX * (t[mask3] - t2)**2
    vel[mask4] = V_MAX
    vel[mask5] = v_t4 - 0.5 * J_MAX * (t[mask5] - t4)**2
    vel[mask6] = v_t5 - A_MAX * (t[mask6] - t5)
    vel[mask7] = v_t6 - A_MAX * (t[mask7] - t6) + 0.5 * J_MAX * (t[mask7] - t6)**2
    
    # --- 5. 将数据保存到CSV文件 ---
    output_csv_filename = 's_curve_data.csv'
    
    # 创建一个字典，键为列名，值为数据
    data_dict = {
        'time_s': t,
        'jerk_m_s3': jerk,
        'acceleration_m_s2': accel,
        'velocity_m_s': vel
    }
    
    # 使用pandas创建DataFrame并保存
    df = pd.DataFrame(data_dict)
    df.to_csv(output_csv_filename, index=False, float_format='%.4f')
    
    print(f"成功生成数据并保存到 '{output_csv_filename}'")

    # --- (可选) 仍然生成图片用于快速预览 ---
    fig, axes = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
    axes[0].plot(t, vel, label='Velocity'); axes[0].set_ylabel('Velocity (m/s)'); axes[0].legend()
    axes[1].plot(t, accel, label='Acceleration'); axes[1].set_ylabel('Acceleration (m/s^2)'); axes[1].legend()
    axes[2].plot(t, jerk, label='Jerk', drawstyle='steps-post'); axes[2].set_ylabel('Jerk (m/s^3)'); axes[2].set_xlabel('Time (s)'); axes[2].legend()
    for ax in axes: ax.grid(True)
    plt.tight_layout()
    plt.savefig('s_curve_numerical_profile.png')
    print("同时生成了预览图 's_curve_numerical_profile.png'")


# --- 运行脚本 ---
if __name__ == '__main__':
    generate_and_save_s_curve_data()
