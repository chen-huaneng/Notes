import gurobipy as gp
from gurobipy import GRB
import math

# ============================================================
# 集合定义
# ============================================================

# 顾客集合
# N = [1, 2, 3, 4]

# 候选卫星仓库
# S = [5, 6]

# 顾客集合
N = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]

# 候选卫星仓库
S = [1, 2, 3, 4]

# 主仓库
depot = 0

# 一级网络
V1 = [depot] + S

# 二级网络
V2 = S + N

# 所有节点
V = [depot] + S + N

# ============================================================
# 参数
# ============================================================

# 顾客需求
# q = {
#     1: 2,
#     2: 3,
#     3: 2,
#     4: 4
# }

# 顾客需求
q = {
    5: 4.4, 6: 3.3, 7: 3.8, 8: 1.2, 9: 1.9,
    10: 2.2, 11: 1.3, 12: 1.9, 13: 1.4, 14: 2.1,
    15: 3.5, 16: 2.5, 17: 2.5, 18: 1.8, 19: 2.1
}

# 卫星仓库建设成本
# f = {
#     5: 100,
#     6: 120
# }

# 卫星仓库建设成本
f = {
    1: 190.5, 2: 147.2, 3: 141.4, 4: 75.7
}

# 所有节点的二维坐标(x, y)
# coords = {
#     0: (0, 0),
#     1: (-10, -10),
#     2: (5, 5),
#     3: (15, 0),
#     4: (15, 5),
#     5: (5, 0),
#     6: (0, -10)
# }

# 所有节点的二维坐标(x, y)
coords = {
    0: (0.0, 0.0),
    1: (31.97, 1.25),
    2: (13.75, 11.16),
    3: (36.82, 33.83),
    4: (44.61, 4.35),
    5: (21.1, 1.49),
    6: (10.93, 25.27),
    7: (1.33, 9.94),
    8: (32.49, 27.25),
    9: (11.02, 29.46),
    10: (40.47, 0.32),
    11: (40.29, 34.91),
    12: (17.01, 7.77),
    13: (47.86, 16.83),
    14: (4.64, 4.84),
    15: (42.37, 30.19),
    16: (40.36, 36.49),
    17: (26.81, 48.66),
    18: (18.93, 27.6),
    19: (41.47, 30.93)
}

# 距离矩阵
d = {}
for i in V:
    for j in V:
        if i != j:
            d[i, j] = math.dist(coords[i], coords[j])

# 成本因子
cL_factor = 2.0
cT_factor = 1.5
cD_factor = 0.5
e_factor = 1.0

# 一级运输成本
cL = {(i, j): cL_factor * d[i, j] for i, j in d}

# 二级卡车运输成本
cT = {(i, j): cT_factor * d[i, j] for i, j in d}

# 无人机运输成本
cD = {(i, j): cD_factor * d[i, j] for i, j in d}

# 无人机能耗
e = {(i, j): e_factor * d[i, j] for i, j in d}

# 速度
vT = 40
vD = 60

# 容量
QL = 50
QT = 15
QD = 5

# 无人机自重
w = 1

# 电量
E = 20

# 大M
M = 1000000

# ============================================================
# 创建模型
# ============================================================

model = gp.Model("2E_Truck_Drone")

# ============================================================
# 决策变量
# ============================================================

# 卫星仓库选址
y = model.addVars(S, vtype=GRB.BINARY, name="y")

# 一级大型卡车路径
xL = model.addVars(V1, V1,
                   vtype=GRB.BINARY,
                   name="xL")

# 二级协同卡车路径
xT = model.addVars(V2, V2, S,
                   vtype=GRB.BINARY,
                   name="xT")

# 无人机配送变量
xD = model.addVars(V2, N, V2, S,
                   vtype=GRB.BINARY,
                   name="xD")

# 一级载重变量
uL = model.addVars(V1,
                   lb=0,
                   vtype=GRB.CONTINUOUS,
                   name="uL")

# 二级载重变量
uT = model.addVars(V2, S,
                   lb=0,
                   vtype=GRB.CONTINUOUS,
                   name="uT")

# 卫星仓库需求量
psi = model.addVars(S,
                    lb=0,
                    vtype=GRB.CONTINUOUS,
                    name="psi")

# 时间变量
# tauT = model.addVars(V2, S,
#                      lb=0,
#                      vtype=GRB.CONTINUOUS,
#                      name="tauT")

# tauD = model.addVars(V2, S,
#                      lb=0,
#                      vtype=GRB.CONTINUOUS,
#                      name="tauD")

# rho = model.addVars(V2, S,
#                     lb=0,
#                     vtype=GRB.CONTINUOUS,
#                     name="rho")

# ============================================================
# 目标函数
# ============================================================

obj1 = gp.quicksum(f[s] * y[s] for s in S)

obj2 = gp.quicksum(
    cL[i, j] * xL[i, j]
    for i in V1 for j in V1 if i != j
)

obj3 = gp.quicksum(
    cT[i, j] * xT[i, j, s]
    for s in S
    for i in V2
    for j in V2
    if i != j
)

obj4 = gp.quicksum(
    (cD[i, j] + cD[j, k]) * xD[i, j, k, s]
    for s in S
    for i in V2
    for j in N
    for k in V2
    if i != j and j != k and i != k
)

model.setObjective(obj1 + obj2 + obj3 + obj4, GRB.MINIMIZE)

# ============================================================
# 一级网络约束
# ============================================================

# 主仓库出发返回
model.addConstr(
    gp.quicksum(xL[depot, i] for i in S) == 1
)

model.addConstr(
    gp.quicksum(xL[i, depot] for i in S) == 1
)

# 一级流平衡
for j in S:

    model.addConstr(
        gp.quicksum(xL[i, j]
                    for i in V1 if i != j)
        == y[j]
    )

    model.addConstr(
        gp.quicksum(xL[j, k]
                    for k in V1 if k != j)
        == y[j]
    )

# 一级MTZ
for i in V1:
    for j in S:
        if i != j:

            model.addConstr(
                uL[j] >= uL[i] + psi[j]
                - M * (1 - xL[i, j])
            )

# 一级容量
for s in S:

    model.addConstr(uL[s] >= psi[s])

    model.addConstr(uL[s] <= QL)

# ============================================================
# 顾客唯一服务
# ============================================================

for j in N:

    model.addConstr(

        gp.quicksum(
            xT[i, j, s]
            for s in S
            for i in N + [s]
            if i != j
        )

        +

        gp.quicksum(
            xD[i, j, k, s]
            for s in S
            for i in N + [s]
            for k in N + [s]
            if i != j and j != k and i != k
        )

        == 1
    )

# ============================================================
# 二级卡车路径
# ============================================================

for s in S:

    # 从卫星仓库出发
    model.addConstr(
        gp.quicksum(
            xT[s, i, s]
            for i in N
        ) == y[s]
    )

    # 返回卫星仓库
    model.addConstr(
        gp.quicksum(
            xT[j, s, s]
            for j in N
        ) == y[s]
    )

# 流平衡
for s in S:
    Ns = N + [s]
    for j in N:

        model.addConstr(
            gp.quicksum(
                xT[i, j, s]
                for i in Ns if i != j
            )
            ==
            gp.quicksum(
                xT[j, k, s]
                for k in Ns if k != j
            )
        )

# ============================================================
# 仓库激活约束
# ============================================================

for s in S:
    Ns = N + [s]
    model.addConstr(
        gp.quicksum(
            xT[i, j, s]
            for i in Ns
            for j in Ns
            if i != j
        ) <= M * y[s]
    )

    model.addConstr(
        gp.quicksum(
            xD[i, j, k, s]
            for i in Ns
            for j in N
            for k in Ns
            if i != j and k != i and k != j
        ) <= M * y[s]
    )

# ============================================================
# 二级MTZ
# ============================================================

for s in S:
    for i in N:
        for j in N:

            if i != j:

                model.addConstr(
                    uT[i, s] - uT[j, s] + 1
                    <=
                    (len(N) + 1) * (1 - xT[i, j, s])
                )

# ============================================================
# 无人机起降约束
# ============================================================

for s in S:
    Ns = N + [s]
    for i in Ns:

        model.addConstr(

            gp.quicksum(
                xD[i, j, k, s]
                for j in N
                for k in Ns
                if i != j and j != k and i != k
            )

            <=

            gp.quicksum(
                xT[i, h, s]
                for h in Ns if h != i
            )
        )

    for k in Ns:

        model.addConstr(

            gp.quicksum(
                xD[i, j, k, s]
                for i in Ns
                for j in N
                if i != j and j != k and i != k
            )

            <=

            gp.quicksum(
                xT[h, k, s]
                for h in Ns if h != k
            )
        )

# ============================================================
# 卡车容量
# ============================================================

for s in S:
    Ns = N + [s]

    model.addConstr(

        gp.quicksum(

            q[j] * (
                gp.quicksum(
                    xT[i, j, s]
                    for i in Ns if i != j
                )

                +

                gp.quicksum(
                    xD[i, j, k, s]
                    for i in Ns
                    for k in Ns
                    if i != j and j != k and i != k
                )
            )

            for j in N
        )

        + w

        <= QT
    )

# ============================================================
# 一级二级需求关联
# ============================================================

for s in S:
    Ns = N + [s]

    model.addConstr(

        psi[s]

        ==

        gp.quicksum(

            q[j] * (

                gp.quicksum(
                    xT[i, j, s]
                    for i in Ns if i != j
                )

                +

                gp.quicksum(
                    xD[i, j, k, s]
                    for i in Ns
                    for k in Ns
                    if i != j and j != k and i != k
                )
            )

            for j in N
        )
    )

# ============================================================
# 无人机容量
# ============================================================

for s in S:
    Ns = N + [s]
    for j in N:

        model.addConstr(

            gp.quicksum(
                q[j] * xD[i, j, k, s]
                for i in Ns
                for k in Ns
                if i != j and j != k and i != k
            )

            <= QD
        )

# ============================================================
# 无人机电量
# ============================================================

for s in S:
    Ns = N + [s]
    for j in N:

        model.addConstr(

            gp.quicksum(
                (e[i, j] + e[j, k])
                * xD[i, j, k, s]

                for i in Ns
                for k in Ns
                if i != j and j != k and i != k
            )

            <= E
        )

# ============================================================
# 时间同步约束
# ============================================================

# for s in S:
#     Ns = N + [s]
#     for i in Ns:
#         for j in Ns:

#             if i != j:

#                 model.addConstr(
#                     tauT[j, s]
#                     >=
#                     rho[i, s]
#                     + d[i, j] / vT
#                     - M * (1 - xT[i, j, s])
#                 )

# for s in S:
#     Ns = N + [s]
#     for i in Ns:
#         for j in N:
#             for k in Ns:

#                 if i != j and j != k and i != k:

#                     model.addConstr(
#                         tauD[k, s]
#                         >=
#                         rho[i, s]
#                         + (d[i, j] + d[j, k]) / vD
#                         - M * (1 - xD[i, j, k, s])
#                     )

# 等待同步
# for s in S:
#     Ns = N + [s]
#     for k in Ns:

#         model.addConstr(
#             rho[k, s] >= tauT[k, s]
#         )

#         model.addConstr(
#             rho[k, s] >= tauD[k, s]
#         )

# ============================================================
# 初始条件
# ============================================================

# for s in S:
#     model.addConstr(uT[s, s] == 0)

    # model.addConstr(tauT[s, s] == 0)

    # model.addConstr(tauD[s, s] == 0)

    # model.addConstr(rho[s, s] == 0)

model.addConstr(uL[0] == 0)

# ============================================================
# 求解参数
# ============================================================

model.Params.TimeLimit = 3600 # seconds
model.Params.MIPGap = 0.0001

# ============================================================
# 求解
# ============================================================

model.optimize()

# ============================================================
# 输出结果
# ============================================================
def print_section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

def var_val(v):
    """安全获取变量值"""
    try:
        return v.X
    except:
        return None

def build_routes_from_edges(edges, start_node):
    """
    从边列表重建完整路径（回路）。
    例如 edges = [(0,5),(5,6),(6,0)] → [[0, 5, 6, 0]]
    支持多条回路。
    """
    if not edges:
        return []

    adj = {}
    for i, j in edges:
        adj.setdefault(i, []).append(j)

    routes = []
    visited_edges = set()

    while True:
        start_edge = None
        for nxt in adj.get(start_node, []):
            if (start_node, nxt) not in visited_edges:
                start_edge = (start_node, nxt)
                break
        if start_edge is None:
            break

        route = [start_node]
        current = start_node

        while True:
            next_node = None
            for nxt in adj.get(current, []):
                if (current, nxt) not in visited_edges:
                    next_node = nxt
                    break
            if next_node is None:
                break
            visited_edges.add((current, next_node))
            route.append(next_node)
            current = next_node
            if current == start_node:
                break

        if len(route) > 1:
            routes.append(route)

    return routes

# ---------- 模型状态 ----------
print_section("模型求解状态")
print(f"Status: {model.status}  (2=OPTIMAL, 9=TIME_LIMIT, 3=INFEASIBLE)")
if model.SolCount > 0:
    print(f"目标值: {model.objVal}")
    print(f"MIPGap: {model.MIPGap}")
    print(f"求解时间: {model.Runtime:.2f}s")
else:
    print("未找到可行解！")

if model.status == GRB.OPTIMAL or model.status == GRB.TIME_LIMIT:

    # ---------- 目标函数各项 ----------
    print_section("目标函数分解")
    print(f"obj1 (选址成本)  = {obj1.getValue():.4f}")
    print(f"obj2 (一级运输)  = {obj2.getValue():.4f}")
    print(f"obj3 (二级卡车)  = {obj3.getValue():.4f}")
    print(f"obj4 (无人机)    = {obj4.getValue():.4f}")
    print(f"总计             = {model.objVal:.4f}")

    # ---------- 卫星仓库选址 y ----------
    print_section("选址变量 y[s]")
    for s in S:
        val = var_val(y[s])
        print(f"  y[{s}] = {val:.4f}  {'✓ 建设' if val > 0.5 else '✗ 不建'}")

    # ---------- 顾客需求 ----------
    print_section("顾客需求")
    total = 0.0
    for j in N:
        demand = q.get(j, 0.0)
        total += demand
        print(f"  顾客 {j:>3d}  需求 = {demand:.2f}")
    print(f"  {'─' * 30}")
    print(f"  顾客总数: {len(N)}")
    print(f"  需求总和: {total:.2f}")

    # ---------- 一级卡车路径 xL ----------
    print_section("一级卡车路径")

    # 提取一级边
    first_level_edges = []
    for i in V1:
        for j in V1:
            if i != j:
                val = var_val(xL[i, j])
                if val is not None and val > 0.5:
                    first_level_edges.append((i, j))

    # 重建一级路径
    first_level_routes = build_routes_from_edges(first_level_edges, depot)

    if first_level_routes:
        print("\n  重建路径:")
        for idx, route in enumerate(first_level_routes):
            route_str = " -> ".join(str(n) for n in route)
            print(f"    Route {idx + 1}: {route_str}")

        print("\n  原始边:")
        for i, j in first_level_edges:
            print(f"    xL[{i},{j}] = 1  ✓  {i} -> {j}")
    else:
        print("  (无路径)")

    # ---------- 一级载重 uL ----------
    print_section("一级载重 uL[i]")
    for i in V1:
        val = var_val(uL[i])
        print(f"  uL[{i}] = {val:.4f}")

    # ---------- 卫星仓库需求 psi ----------
    print_section("卫星仓库需求 psi[s]")
    for s in S:
        val = var_val(psi[s])
        print(f"  psi[{s}] = {val:.4f}")

    # ---------- 二级卡车路径 xT ----------
    print_section("二级卡车路径")
    for s in S:
        print(f"\n  --- 卫星仓库 {s} ---")

        # 提取二级边
        edges = []
        for i in V2:
            for j in V2:
                if i != j:
                    val = var_val(xT[i, j, s])
                    if val is not None and val > 0.5:
                        edges.append((i, j))

        # 重建二级路径
        routes = build_routes_from_edges(edges, s)

        if routes:
            print("  重建路径:")
            for idx, route in enumerate(routes):
                route_str = " -> ".join(str(n) for n in route)
                print(f"    Route {idx + 1}: {route_str}")

            print("  原始边:")
            for i, j in edges:
                print(f"    xT[{i},{j},{s}] = 1  ✓  {i} -> {j}")
        else:
            print("    (无路径)")

    # ---------- 二级载重 uT ----------
    print_section("二级网络访问顺序 uT[i,s]")
    for s in S:
        print(f"\n  --- 卫星仓库 {s} ---")
        for i in V2:
            val = var_val(uT[i, s])
            if val > 0.5:
                print(f"    uT[{i},{s}] = {val:.2f}")

    # ---------- 无人机配送 xD ----------
    print_section("无人机配送 xD[i,j,k,s]")
    for s in S:
        print(f"\n  --- 卫星仓库 {s} ---")
        found = False
        for i in V2:
            for j in N:
                for k in V2:
                    if i != j and j != k and i != k:
                        val = var_val(xD[i, j, k, s])
                        if val > 0.5:
                            cost = (cD[i,j] + cD[j,k])
                            energy = (e[i,j] + e[j,k])
                            print(f"    xD[{i},{j},{k},{s}] = {val:.4f}  ✓  "
                                  f"{i} → 顾客{j} → {k}  "
                                  f"cost={cost:.2f}  energy={energy:.2f}")
                            found = True
        if not found:
            print("    (无无人机配送)")

    # ---------- 时间变量 ----------
    # print_section("卡车到达时间 tauT[i,s]")
    # for s in S:
    #     print(f"\n  --- 卫星仓库 {s} ---")
    #     for i in V2:
    #         val = var_val(tauT[i, s])
    #         print(f"    tauT[{i},{s}] = {val:.4f}")

    # print_section("无人机到达时间 tauD[i,s]")
    # for s in S:
    #     print(f"\n  --- 卫星仓库 {s} ---")
    #     for i in V2:
    #         val = var_val(tauD[i, s])
    #         print(f"    tauD[{i},{s}] = {val:.4f}")

    # print_section("节点出发时间 rho[i,s]")
    # for s in S:
    #     print(f"\n  --- 卫星仓库 {s} ---")
    #     for i in V2:
    #         val = var_val(rho[i, s])
    #         print(f"    rho[{i},{s}] = {val:.4f}")

else:
    print("\n模型无可行解，请检查约束。")
