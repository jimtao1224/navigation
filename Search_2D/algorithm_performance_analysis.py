import pandas as pd

def load_data(file_path, algorithm_name):
    """從 Excel 文件加載數據，並添加演算法名稱標記。"""
    df = pd.read_excel(file_path)
    df['演算法'] = algorithm_name
    return df

def prepare_and_calculate_efficiency(df):
    """準備數據並計算效率，給定數據分為尺度A和尺度B，計算轉換效率。"""
    df_a = df[df['尺度'] == 'A'].sort_values(by='地圖熵值').reset_index(drop=True)
    df_b = df[df['尺度'] == 'B'].sort_values(by='地圖熵值').reset_index(drop=True)
    
    df_a['運行時間數值'] = df_a['系統運行時間'].str.extract('(\d+\.\d+)').astype(float)
    df_b['運行時間數值'] = df_b['系統運行時間'].str.extract('(\d+\.\d+)').astype(float)
    
    efficiency = ((df_a['運行時間數值'] - df_b['運行時間數值']) / df_a['運行時間數值']) * 100
    df_a['轉換效率 (%)'] = efficiency
    
    # 將地圖熵值分為三個層級
    df_a['熵值等級'] = pd.qcut(df_a['地圖熵值'], 3, labels=["低", "中", "高"])
    return df_a[['地圖熵值', '轉換效率 (%)', '演算法', '熵值等級']]

# 指定文件路徑
path_astar = '/home/jim/output_astar.xlsx'
path_dijkstra = '/home/jim/output_dijkstra.xlsx'
path_dlite = '/home/jim/output2.xlsx'

# 處理每個演算法的數據
efficiency_astar = prepare_and_calculate_efficiency(load_data(path_astar, "A*"))
efficiency_dijkstra = prepare_and_calculate_efficiency(load_data(path_dijkstra, "Dijkstra"))
efficiency_dlite = prepare_and_calculate_efficiency(load_data(path_dlite, "D* Lite"))

# 合併數據並排序
all_efficiencies = pd.concat([efficiency_astar, efficiency_dijkstra, efficiency_dlite])

# 輸出每個熵值等級的前三個最高效率的條目
output = all_efficiencies.groupby('熵值等級').apply(lambda x: x.nlargest(3, '轉換效率 (%)')).reset_index(drop=True)

# 儲存數據到Excel檔案
output_file_path = '/home/jim/Desktop/data_base.xlsx'
output.to_excel(output_file_path, index=False)

print("數據已保存到", output_file_path)
