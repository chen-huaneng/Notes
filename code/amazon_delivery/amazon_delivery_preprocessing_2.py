import pandas as pd
import numpy as np
from sklearn.cluster import KMeans
import os

# 1. 读取Excel文件并提取需要的列
df = pd.read_excel("amazon_delivery_filtered_data.xlsx")
filtered_df = df[["Store_Latitude", "Store_Longitude", "Drop_Latitude", "Drop_Longitude"]]

# 2. 提取唯一的仓库坐标用于聚类
store_coords = filtered_df[["Store_Latitude", "Store_Longitude"]].drop_duplicates()

# 3. 使用KMeans进行聚类（已知聚类数=22）
kmeans = KMeans(n_clusters=22, random_state=42, n_init=10)
store_coords["Cluster"] = kmeans.fit_predict(store_coords[["Store_Latitude", "Store_Longitude"]])

# 4. 将聚类标签合并回原始数据
merged_df = filtered_df.merge(
    store_coords,
    how="left",
    on=["Store_Latitude", "Store_Longitude"]
)

# 5. 创建保存结果的文件夹
output_dir = "clustered_nodes"
os.makedirs(output_dir, exist_ok=True)

# 6. 按聚类分组处理数据
for cluster_id in range(22):
    # 提取当前聚类的数据
    cluster_data = merged_df[merged_df["Cluster"] == cluster_id]
    
    # 分离仓库节点和顾客节点
    store_nodes = cluster_data[["Store_Latitude", "Store_Longitude"]].drop_duplicates()
    customer_nodes = cluster_data[["Drop_Latitude", "Drop_Longitude"]].drop_duplicates()
    
    # 生成唯一ID
    store_nodes["ID"] = ["store_" + str(i) for i in range(len(store_nodes))]
    customer_nodes["ID"] = ["customer_" + str(i) for i in range(len(customer_nodes))]
    
    # 重命名列以匹配
    store_nodes.rename(columns={"Store_Latitude": "latitude", "Store_Longitude": "longitude"}, inplace=True)
    customer_nodes.rename(columns={"Drop_Latitude": "latitude", "Drop_Longitude": "longitude"}, inplace=True)
    
    # 合并节点并整理列顺序
    combined_nodes = pd.concat([store_nodes, customer_nodes], ignore_index=True)
    combined_nodes = combined_nodes[["ID", "latitude", "longitude"]]
    
    # 输出每个聚类的节点数量
    total_nodes = len(store_nodes) + len(customer_nodes)
    print(f"聚类 {cluster_id} 有 {total_nodes} 个节点（仓库节点: {len(store_nodes)}，顾客节点: {len(customer_nodes)}）")
    
    # 保存到Excel文件
    output_path = os.path.join(output_dir, f"cluster_{cluster_id}.xlsx")
    combined_nodes.to_excel(output_path, index=False)