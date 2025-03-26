import pandas as pd
from math import radians, sin, cos, sqrt, atan2
from global_land_mask import globe

# Haversine公式计算距离
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.393 # 地球半径近似值
    lat1_rad, lon1_rad = radians(lat1), radians(lon1)
    lat2_rad, lon2_rad = radians(lat2), radians(lon2)
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    a = sin(dlat/2)**2 + cos(lat1_rad)*cos(lat2_rad)*sin(dlon/2)**2
    return R * 2 * atan2(sqrt(a), sqrt(1-a))

# 读取数据
df = pd.read_csv('amazon_delivery.csv')

# 步骤1: 筛选顾客节点>=10的仓库
valid_warehouses = df.groupby(['Store_Latitude', 'Store_Longitude']).filter(lambda x: len(x) >= 10)

# 步骤2: 剔除顾客-仓库经纬度差>=1的订单
valid_warehouses = valid_warehouses[
    (abs(valid_warehouses['Store_Latitude'] - valid_warehouses['Drop_Latitude']) < 1) &
    (abs(valid_warehouses['Store_Longitude'] - valid_warehouses['Drop_Longitude']) < 1)
]

# 步骤3: 计算距离并筛选<=50公里的订单
valid_warehouses['Distance'] = valid_warehouses.apply(
    lambda row: haversine(row['Store_Latitude'], row['Store_Longitude'],
                          row['Drop_Latitude'], row['Drop_Longitude']),
    axis=1
)
valid_warehouses = valid_warehouses[valid_warehouses['Distance'] <= 50]

# 步骤4: 去重同一仓库下的重复顾客
valid_warehouses = valid_warehouses.drop_duplicates(
    subset=['Store_Latitude', 'Store_Longitude', 'Drop_Latitude', 'Drop_Longitude']
)

# 步骤5: 检查仓库是否在陆地
# 提取唯一仓库坐标
warehouse_coords = valid_warehouses[['Store_Latitude', 'Store_Longitude']].drop_duplicates()

# 使用global_land_mask检查陆地
warehouse_coords['Is_Land'] = warehouse_coords.apply(
    lambda row: globe.is_land(row['Store_Latitude'], row['Store_Longitude']),
    axis=1
)

# 合并陆地标记到原始数据
valid_warehouses = valid_warehouses.merge(
    warehouse_coords[['Store_Latitude', 'Store_Longitude', 'Is_Land']],
    on=['Store_Latitude', 'Store_Longitude'],
    how='left'
)

# 步骤6: 剔除位于海里的仓库数据
final_data = valid_warehouses[valid_warehouses['Is_Land']]

# 输出结果
final_data.to_excel('amazon_delivery_filtered_data.xlsx', index=False)
warehouse_stats = final_data.groupby(['Store_Latitude', 'Store_Longitude']).size().reset_index(name='Count')
warehouse_stats.sort_values(by='Count', ascending=False).to_excel('warehouse_stats.xlsx', index=False)