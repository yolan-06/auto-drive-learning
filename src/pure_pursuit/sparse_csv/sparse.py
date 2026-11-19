import pandas as pd
import os

def sparsify_data(input_file, output_file, interval=1.0):
    """
    读取一个CSV文件，根据's'列中指定的间隔稀疏化数据，
    并将其保存到一个新的CSV文件中。

    参数:
        input_file (str): 输入CSV文件的路径。
        output_file (str): 输出CSV文件的路径。
        interval (float): 's'列所需的数据间隔。
    """
    try:
        # 读取数据，假设第一行是表头
        df = pd.read_csv(input_file)

        # 清理列名中可能存在的空格
        df.columns = [col.strip() for col in df.columns]

        # 验证's'和'l'列是否存在
        if 's' not in df.columns or 'l' not in df.columns:
            print(f"错误: 文件 {input_file} 必须包含 's' 和 'l' 列。")
            return

        # 根据's'列对数据进行排序，以确保处理顺序正确
        df_sorted = df.sort_values(by='s').reset_index(drop=True)

        # 用于存放稀疏数据的列表
        sparse_data = []

        # 总是包含第一个数据点
        if not df_sorted.empty:
            sparse_data.append(df_sorted.iloc[0])
            last_s = df_sorted.iloc[0]['s']
        else:
            print(f"警告: 文件 {input_file} 是空的。")
            return

        # 遍历数据帧的其余部分
        for index, row in df_sorted.iloc[1:].iterrows():
            if row['s'] >= last_s + interval:
                sparse_data.append(row)
                last_s = row['s']

        # 从稀疏数据列表创建一个新的数据帧
        df_sparse = pd.DataFrame(sparse_data)

        # 将新的数据帧保存为CSV文件
        df_sparse.to_csv(output_file, index=False)
        print(f"成功处理 {input_file} 并将稀疏数据保存至 {output_file}")

    except FileNotFoundError:
        print(f"错误: 文件 {input_file} 未找到。")
    except Exception as e:
        print(f"处理文件 {input_file} 时发生错误: {e}")

# --- 主执行部分 ---

# --- 1. 在这里定义您的文件和设置 ---

# 需要处理的输入CSV文件列表
# 您可以在这里添加更多文件，例如: ["文件1.csv", "文件2.csv"]
input_files = ["sl_data_run4_results2_200.csv","sl_data_run4_result2_200_none.csv"]

# 定义所需的距离间隔（单位：米）。您可以修改这个值。
distance_interval = 1.0

# --- 2. 脚本将处理以下文件 ---

for file_path in input_files:
    # 为输出文件创建一个新名称，例如："sl_data_run3_sparse.csv"
    base_name, ext = os.path.splitext(file_path)
    output_file_path = f"{base_name}_sparse.csv"

    # 调用函数处理文件
    sparsify_data(file_path, output_file_path, distance_interval)
