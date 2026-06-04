import gurobipy as gp
from gurobipy import GRB
import math
from datetime import datetime
import json
from pathlib import Path

# ============================================================
# 从 JSON 加载算例
# ============================================================

def load_instance(filepath):
    """从 JSON 文件加载算例参数"""
    filepath = Path(filepath)
    with open(filepath, "r", encoding="utf-8") as fp:
        data = json.load(fp)

    inst = {}
    inst["name"] = data["name"]
    inst["depot"] = data["depot"]
    inst["N"] = data["N"]
    inst["S"] = data["S"]
    inst["coords"] = {int(k): tuple(v) for k, v in data["coords"].items()}
    inst["q"] = {int(k): float(v) for k, v in data["q"].items()}
    inst["f"] = {int(k): float(v) for k, v in data["f"].items()}
    inst["vT"] = data["vT"]
    inst["vD"] = data["vD"]
    inst["QL"] = data["QL"]
    inst["QT"] = data["QT"]
    inst["QD"] = data["QD"]
    inst["E"] = data["E"]
    inst["w"] = data["w"]
    inst["cL_factor"] = data["cL_factor"]
    inst["cT_factor"] = data["cT_factor"]
    inst["cD_factor"] = data["cD_factor"]
    inst["e_factor"] = data["e_factor"]
    return inst

# ============================================================
# 集合定义
# ============================================================

INSTANCE_FILE = "./data/random_15C_4S_seed42.json"

inst = load_instance(INSTANCE_FILE)

N = inst["N"]
S = inst["S"]
depot = inst["depot"]
V1 = [depot] + S
V2 = S + N
V = [depot] + S + N

# ============================================================
# 参数
# ============================================================

q = inst["q"]
f = inst["f"]
coords = inst["coords"]

# 距离矩阵
d = {}
for i in V:
    for j in V:
        if i != j:
            d[i, j] = math.dist(coords[i], coords[j])

# 成本因子
cL_factor = inst["cL_factor"]
cT_factor = inst["cT_factor"]
cD_factor = inst["cD_factor"]
e_factor = inst["e_factor"]

# 一级运输成本
cL = {(i, j): cL_factor * d[i, j] for i, j in d}

# 二级卡车运输成本
cT = {(i, j): cT_factor * d[i, j] for i, j in d}

# 无人机运输成本
cD = {(i, j): cD_factor * d[i, j] for i, j in d}

# 无人机能耗
e = {(i, j): e_factor * d[i, j] for i, j in d}

# 速度
vT = inst["vT"]
vD = inst["vD"]

# 容量
QL = inst["QL"]
QT = inst["QT"]
QD = inst["QD"]

# 无人机自重
w = inst["w"]

# 电量
E = inst["E"]

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
xL = model.addVars(V1, V1, vtype=GRB.BINARY, name="xL")

# 二级协同卡车路径
xT = model.addVars(V2, V2, S, vtype=GRB.BINARY, name="xT")

# 无人机配送变量
xD = model.addVars(V2, N, V2, S, vtype=GRB.BINARY, name="xD")

# 无人机出动顺序变量
delta = model.addVars(V2, V2, S, vtype=GRB.BINARY, name="delta")

# 一级载重变量
uL = model.addVars(V1, lb=0, vtype=GRB.CONTINUOUS, name="uL")

# 二级访问顺序变量
uT = model.addVars(V2, S, lb=0, vtype=GRB.INTEGER, name="uT")

# 卫星仓库需求量
psi = model.addVars(S, lb=0, vtype=GRB.CONTINUOUS, name="psi")

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

obj2 = gp.quicksum(cL[i, j] * xL[i, j] for i in V1 for j in V1 if i != j)

obj3 = gp.quicksum(cT[i, j] * xT[i, j, s] for s in S for i in V2 for j in V2 if i != j)

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
model.addConstr(gp.quicksum(xL[depot, i] for i in S) == 1)

model.addConstr(gp.quicksum(xL[i, depot] for i in S) == 1)

# 一级流平衡
for j in S:
    model.addConstr(gp.quicksum(xL[i, j] for i in V1 if i != j) == y[j])

    model.addConstr(gp.quicksum(xL[j, k] for k in V1 if k != j) == y[j])

# 一级MTZ
for i in V1:
    for j in S:
        if i != j:
            model.addConstr(uL[i] - uL[j] + psi[j] <= M * (1 - xL[i, j]))

# 一级容量
for s in S:
    model.addConstr(uL[s] >= psi[s])

    model.addConstr(uL[s] <= QL)

# ============================================================
# 顾客唯一服务
# ============================================================

for j in N:
    model.addConstr(
        gp.quicksum(xT[i, j, s] for s in S for i in N + [s] if i != j)
        + gp.quicksum(
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
    model.addConstr(gp.quicksum(xT[s, i, s] for i in N) == y[s])

    # 返回卫星仓库
    model.addConstr(gp.quicksum(xT[j, s, s] for j in N) == y[s])

# 流平衡
for s in S:
    Ns = N + [s]
    for j in N:
        model.addConstr(
            gp.quicksum(xT[i, j, s] for i in Ns if i != j)
            == gp.quicksum(xT[j, k, s] for k in Ns if k != j)
        )

# ============================================================
# 仓库激活约束
# ============================================================

for s in S:
    Ns = N + [s]
    model.addConstr(
        gp.quicksum(xT[i, j, s] for i in Ns for j in Ns if i != j) <= M * y[s]
    )

    model.addConstr(
        gp.quicksum(
            xD[i, j, k, s]
            for i in Ns
            for j in N
            for k in Ns
            if i != j and k != i and k != j
        )
        <= M * y[s]
    )

# ============================================================
# 二级MTZ
# ============================================================

for s in S:
    for i in N:
        for j in N:
            if i != j:
                model.addConstr(
                    uT[i, s] - uT[j, s] + 1 <= (len(N) + 1) * (1 - xT[i, j, s])
                )

# ============================================================
# 无人机起降约束
# ============================================================

# 无人机起降点必须有卡车经过
for s in S:
    Ns = N + [s]
    for i in Ns:
        model.addConstr(
            gp.quicksum(
                xD[i, j, k, s] for j in N for k in Ns if i != j and j != k and i != k
            )
            <= gp.quicksum(xT[i, h, s] for h in Ns if h != i)
        )

    for k in Ns:
        model.addConstr(
            gp.quicksum(
                xD[i, j, k, s] for i in Ns for j in N if i != j and j != k and i != k
            )
            <= gp.quicksum(xT[h, k, s] for h in Ns if h != k)
        )

# 无人机在顾客节点和卫星仓库最多起飞一次
for i in V2:
    model.addConstr(
        gp.quicksum(
            xD[i, j, k, s]
            for s in S
            for j in N
            for k in V2
            if i != j and j != k and k != i
        )
        <= 1
    )

# 无人机在顾客节点和卫星仓库最多降落一次
for k in V2:
    model.addConstr(
        gp.quicksum(
            xD[i, j, k, s]
            for s in S
            for i in V2
            for j in N
            if i != j and j != k and i != k
        )
        <= 1
    )

# ============================================================
# 无人机服务顺序约束（无人机的服务顺序不能和卡车相反）
# ============================================================

for s in S:
    Ns = N + [s]
    for i in Ns:
        for k in N:
            if i == k:
                continue
            model.addConstr(
                uT[i, s] - uT[k, s] + 1
                <= (len(N) + 1)
                * (1 - gp.quicksum(xD[i, j, k, s] for j in N if j != i and j != k))
            )

# ============================================================
# 无人机出动非重叠约束
# ============================================================

for s in S:
    Ns = N + [s]
    for i in Ns:
        for k in Ns:
            if i == k:
                continue
            for l in Ns:
                for n in Ns:
                    if l == n:
                        continue

                    model.addConstr(
                        uT[k, s] - uT[l, s]
                        <= len(N) * (
                            3
                            - gp.quicksum(xD[i, j, k, s] for j in N if j != i and j != k)
                            - gp.quicksum(xD[l, m, n, s] for m in N if m != l and m != n)
                            - delta[i, l, s]
                        )
                    )

                    model.addConstr(
                        uT[n, s] - uT[i, s]
                        <= len(N) * (
                            2
                            - gp.quicksum(xD[i, j, k, s] for j in N if j != i and j != k)
                            - gp.quicksum(xD[l, m, n, s] for m in N if m != l and m != n)
                            + delta[i, l, s]
                        )
                    )

# ============================================================
# 卡车容量
# ============================================================

for s in S:
    Ns = N + [s]

    model.addConstr(
        gp.quicksum(
            q[j]
            * (
                gp.quicksum(xT[i, j, s] for i in Ns if i != j)
                + gp.quicksum(
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
        == gp.quicksum(
            q[j]
            * (
                gp.quicksum(xT[i, j, s] for i in Ns if i != j)
                + gp.quicksum(
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
                (e[i, j] + e[j, k]) * xD[i, j, k, s]
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

model.addConstr(uL[0] == 0)

# ============================================================
# 求解参数
# ============================================================

model.Params.TimeLimit = 3600  # seconds
model.Params.MIPGap = 0.00001

# ============================================================
# 求解
# ============================================================

model.optimize()

# ============================================================
# 输出结果
# ============================================================

def print_section(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")

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
    print(f"obj1 (选址成本)  = {obj1.getValue():.5f}")
    print(f"obj2 (一级运输)  = {obj2.getValue():.5f}")
    print(f"obj3 (二级卡车)  = {obj3.getValue():.5f}")
    print(f"obj4 (无人机)    = {obj4.getValue():.5f}")
    print(f"总计             = {model.objVal:.5f}")

    # ---------- 卫星仓库选址 y ----------
    # print_section("选址变量 y[s]")
    # for s in S:
    #     val = var_val(y[s])
    #     print(
    #         f"  y[{s}] = {val:.4f}  {'✓ 建设' if val is not None and val > 0.5 else '✗ 不建'}"
    #     )

    # ---------- 顾客需求 ----------
    # print_section("顾客需求")
    # total = 0.0
    # for j in N:
    #     demand = q.get(j, 0.0)
    #     total += demand
    #     print(f"  顾客 {j:>3d}  需求 = {demand:.2f}")
    # print(f"  {'─' * 30}")
    # print(f"  顾客总数: {len(N)}")
    # print(f"  需求总和: {total:.2f}")

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

        # print("\n  原始边:")
        # for i, j in first_level_edges:
        #     print(f"    xL[{i},{j}] = 1  ✓  {i} -> {j}")
    else:
        print("  (无路径)")

    # ---------- 一级载重 uL ----------
    # print_section("一级载重 uL[i]")
    # for i in V1:
    #     val = var_val(uL[i])
    #     print(f"  uL[{i}] = {val:.4f}")

    # ---------- 卫星仓库需求 psi ----------
    # print_section("卫星仓库需求 psi[s]")
    # for s in S:
    #     val = var_val(psi[s])
    #     print(f"  psi[{s}] = {val:.4f}")

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

            # print("  原始边:")
            # for i, j in edges:
            #     print(f"    xT[{i},{j},{s}] = 1  ✓  {i} -> {j}")
        else:
            print("    (无路径)")

    # ---------- 二级网络访问顺序 uT ----------
    # print_section("二级网络访问顺序 uT[i,s]")
    # for s in S:
    #     print(f"\n  --- 卫星仓库 {s} ---")
    #     for i in V2:
    #         val = var_val(uT[i, s])
    #         if val is not None and val > 0.5:
    #             print(f"    uT[{i},{s}] = {val:.2f}")

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
                        if val is not None and val > 0.5:
                            cost = cD[i, j] + cD[j, k]
                            energy = e[i, j] + e[j, k]
                            print(
                                f"    xD[{i},{j},{k},{s}] = {val:.4f}  ✓  "
                                f"{i} -> 顾客{j} -> {k}  "
                                f"cost={cost:.2f}  energy={energy:.2f}"
                            )
                            found = True
        if not found:
            print("    (无无人机配送)")

    # ---------- 无人机出动顺序 delta ----------
    # print_section("无人机出动顺序 delta[i,l,s]")
    # for s in S:
    #     print(f"\n  --- 卫星仓库 {s} ---")
    #     found = False
    #     for i in V2:
    #         for l in V2:
    #             val = var_val(delta[i, l, s])
    #             if val is not None and val > 0.5:
    #                 print(f"    delta[{i},{l},{s}] = {val:.4f}  ✓  出动({i},...) 先于 出动({l},...)")
    #                 found = True
    #     if not found:
    #         print("    (无delta=1的变量)")

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

# 输出当前的算例，方便复现结果
print(f"算例名称: {inst['name']}")

# 输出当前时间，方便后续回顾数据输出
print("当前时间是:", datetime.now())
