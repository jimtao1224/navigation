import pandas as pd
from env_test import Env
import sys
import os
# from dstar_test import d_star_main
# from Astar_test import AStar
# from Dijkstra_test import main
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")
from Search_2D import Dijkstra_test
from Search_2D import Astar_test
from Search_2D import dstar_test
# from Search_2D import dstar_test
def load_data(file_path):
    """從 Excel 文件加載數據。"""
    return pd.read_excel(file_path)


def calculate_percentile_entropy_ranges(df, column):
    """基於百分位數計算熵值等級範圍。"""
    low_threshold = df[column].quantile(0.33)
    mid_threshold = df[column].quantile(0.67)
    max_val = df[column].max()
    min_val = df[column].min()
    return {
        '低': (min_val, low_threshold),
        '中': (low_threshold, mid_threshold),
        '高': (mid_threshold, max_val)
    }

def recommend_algorithm(df, entropy_value, ranges):
    """根據地圖熵值推薦最適合的路徑規劃演算法。"""
    for level, (min_val, max_val) in ranges.items():
        if min_val <= entropy_value <= max_val:
            highest_efficiency_algo = df[(df['熵值等級'] == level) & 
                                         (df['地圖熵值'] >= min_val) & 
                                         (df['地圖熵值'] <= max_val)].nlargest(1, '轉換效率 (%)')
            return highest_efficiency_algo.iloc[0]['演算法'], highest_efficiency_algo.iloc[0]['轉換效率 (%)'], level
    return None, None, None

    
def select_algorithm(algo_name):
    algo_name = algo_name.strip().lower()

    if algo_name is None:
        raise ValueError("Algorithm name cannot be None")
    algo_name = algo_name.lower()
    if algo_name == 'dstar_lite':
        return dstar_test.d_star_main()
    elif algo_name == 'dijkstra':
        return Dijkstra_test.Dijkstra_main()
    elif algo_name == 'astar':
        return Astar_test.a_star_main()
    else:
        raise ValueError("Unknown algorithm name.")

# 讀取數據
file_path = '/home/jim/Desktop/data_base.xlsx'
df = load_data(file_path)
map_entropy = Env(scale='A')

# 計算熵值等級範圍
entropy_ranges = calculate_percentile_entropy_ranges(df, '地圖熵值')
# 為數據分配熵值等級
df['熵值等級'] = df['地圖熵值'].apply(
    lambda x: '低' if x <= entropy_ranges['低'][1] else ('中' if x <= entropy_ranges['中'][1] else '高'))

# 獲取輸入
# entropy_value = float(input("請輸入地圖熵值:"))
entropy_value = float(map_entropy.entropy)
# 推薦系統
recommended_algo, efficiency, level = recommend_algorithm(df, entropy_value, entropy_ranges)

if recommended_algo:
    print(f"對於地圖熵值 {entropy_value}，推薦使用的路徑規劃演算法為 {recommended_algo}，熵值等級為 {level}，預期轉換效率為 {efficiency}%。")
    algorithm_choose = input("請輸入要使用的演算法:Dijkstra,astar,dstar_lite:")
    algorithm_instance = select_algorithm(algorithm_choose)
    # algorithm_instance.d_star_main()
else:
    if entropy_value < df['地圖熵值'].min():
        print("地圖熵值太低，沒有找到合適的演算法推薦。")
        algorithm_instance = select_algorithm('dstar_lite')
    elif entropy_value > df['地圖熵值'].max():
        print("地圖熵值太高，沒有找到合適的演算法推薦。")
        algorithm_instance = select_algorithm('dstar_lite')
