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

N = inst["N"]                    # 顾客节点 1..n
S = inst["S"]                    # 卫星仓库
depot = inst["depot"]            # 郊区主仓库 0
depot_p = -2026                     # 虚拟仓库 0'

# 虚拟卫星仓库 s' 用负号表示
S_p = [-s for s in S]

# 一级网络集合
V1_minus = [depot] + S           # {0} ∪ S
V1_plus  = S + [depot_p]        # S ∪ {0'}

# 二级网络集合 (与卫星s相关)
sat_prime = {s: -s for s in S}   # s -> s'

V_s = {}
Vs_minus = {}
Vs_plus = {}
for s in S:
    sp = sat_prime[s]
    V_s[s] = [s] + N + [sp]
    Vs_minus[s] = [s] + N
    Vs_plus[s] = N + [sp]

# 所有节点
V = [depot, depot_p] + S + S_p + N

# ============================================================
# 参数
# ============================================================

q = inst["q"]
f = inst["f"]
coords = inst["coords"]

# 虚拟节点坐标与对应实体相同
coords[depot_p] = coords[depot]
for s, sp in zip(S, S_p):
    coords[sp] = coords[s]

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

# 运输成本
cL = {(i, j): cL_factor * d[i, j] for (i, j) in d}
cT = {(i, j): cT_factor * d[i, j] for (i, j) in d}
cD = {(i, j): cD_factor * d[i, j] for (i, j) in d}

# 无人机能耗
e = {(i, j): e_factor * d[i, j] for (i, j) in d}

# 速度
vT = inst["vT"]
vD = inst["vD"]

# 容量
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
xL = model.addVars(V1_minus, V1_plus, vtype=GRB.BINARY, name="xL")

# 二级协同卡车路径
xT = model.addVars(
    {(i, j, s) for s in S for i in Vs_minus[s] for j in Vs_plus[s] if i != j},
    vtype=GRB.BINARY,
    name="xT"
)

# 无人机配送变量
xD = model.addVars(
    {(i, j, k, s)
     for s in S
     for i in Vs_minus[s]
     for j in N
     for k in Vs_plus[s]
     if i != j and j != k and i != k},
    vtype=GRB.BINARY,
    name="xD"
)

# 服务顺序辅助变量 δ_{ijs}  (定义在 V_s × V_s 上)
delta = model.addVars(
    {(i, j, s) for s in S for i in V_s[s] for j in V_s[s] if i != j},
    vtype=GRB.BINARY,
    name="delta"
)

# 一级访问顺序变量 (MTZ)
uL = model.addVars(V1_minus, vtype=GRB.INTEGER, lb=1, ub=len(S)+1, name="uL")

# 二级访问顺序变量 (仅用于 Vs^- 中的节点)
uT = model.addVars(
    {(i, s) for s in S for i in Vs_minus[s]},
    vtype=GRB.INTEGER,
    lb=1,
    ub=len(N)+1,
    name="uT"
)

# 时间变量 (定义在 V_s 上)
tauT = model.addVars(
    {(i, s) for s in S for i in V_s[s]},
    lb=0,
    vtype=GRB.CONTINUOUS,
    name="tauT"
)

tauD = model.addVars(
    {(i, s) for s in S for i in V_s[s]},
    lb=0,
    vtype=GRB.CONTINUOUS,
    name="tauD"
)

rho = model.addVars(
    {(i, s) for s in S for i in V_s[s]},
    lb=0,
    vtype=GRB.CONTINUOUS,
    name="rho"
)

# ============================================================
# 目标函数
# ============================================================

obj1 = gp.quicksum(f[s] * y[s] for s in S)

obj2 = gp.quicksum(cL[i, j] * xL[i, j] for i in V1_minus for j in V1_plus if i != j)

obj3 = gp.quicksum(cT[i, j] * xT[i, j, s]
                   for s in S
                   for i in Vs_minus[s]
                   for j in Vs_plus[s] if i != j)

obj4 = gp.quicksum(
    (cD[i, j] + cD[j, k]) * xD[i, j, k, s]
    for s in S
    for i in Vs_minus[s]
    for j in N
    for k in Vs_plus[s]
    if i != j and j != k and i != k
)

model.setObjective(obj1 + obj2 + obj3 + obj4, GRB.MINIMIZE)

# ============================================================
# 一级网络约束
# ============================================================

# 大型卡车从 0 出发并返回 0' 各一次
model.addConstr(gp.quicksum(xL[depot, j] for j in S) == 1, "L_depart")
model.addConstr(gp.quicksum(xL[i, depot_p] for i in S) == 1, "L_arrive")

# 一级流守恒
for j in S:
    inflow = gp.quicksum(xL[i, j] for i in V1_minus if i != j)
    outflow = gp.quicksum(xL[j, k] for k in V1_plus if k != j)
    model.addConstr(inflow == outflow, f"L_flow_conservation_{j}")
    model.addConstr(outflow == y[j], f"L_flow_open_{j}")

# 一级子回路消除 (MTZ)
for i in V1_minus:
    for j in S:
        if i != j:
            model.addConstr(
                uL[i] - uL[j] + 1 <= (len(S) + 1) * (1 - xL[i, j]),
                f"L_MTZ_{i}_{j}"
            )

# ============================================================
# 二级网络约束
# ============================================================

# 每个顾客恰好被服务一次 (卡车直达或无人机)
for j in N:
    truck = gp.quicksum(xT[i, j, s] for s in S for i in Vs_minus[s] if i != j)
    drone = gp.quicksum(xD[i, j, k, s] for s in S
                        for i in Vs_minus[s]
                        for k in Vs_plus[s]
                        if i != j and j != k and i != k)
    model.addConstr(truck + drone == 1, f"customer_service_{j}")

# 卡车从卫星出发，回到虚拟卫星 (若卫星被建设)
for s in S:
    model.addConstr(gp.quicksum(xT[s, j, s] for j in N) == y[s], f"T_depart_{s}")
    model.addConstr(gp.quicksum(xT[i, sat_prime[s], s] for i in N) == y[s], f"T_arrive_{s}")

# 二级流守恒 (顾客节点)
for s in S:
    for j in N:
        inflow = gp.quicksum(xT[i, j, s] for i in Vs_minus[s] if i != j)
        outflow = gp.quicksum(xT[j, k, s] for k in Vs_plus[s] if k != j)
        model.addConstr(inflow == outflow, f"T_flow_conservation_{s}_{j}")
        model.addConstr(outflow <= y[s], f"T_flow_open_{s}_{j}")

# 卡车/无人机启用与卫星选址耦合
for s in S:
    model.addConstr(
        gp.quicksum(xT[i, j, s] for i in Vs_minus[s] for j in Vs_plus[s] if i != j)
        <= M * y[s],
        f"T_activation_{s}"
    )
    model.addConstr(
        gp.quicksum(xD[i, j, k, s] for i in Vs_minus[s] for j in N for k in Vs_plus[s]
                    if i != j and j != k and i != k)
        <= M * y[s],
        f"D_activation_{s}"
    )

# 二级子回路消除 (MTZ)
for s in S:
    for i in Vs_minus[s]:
        for j in N:
            if i != j:
                model.addConstr(
                    uT[i, s] - uT[j, s] + 1 <= (len(N) + 2) * (1 - xT[i, j, s]),
                    f"T_MTZ_{s}_{i}_{j}"
                )

# 卡车必须经过无人机起飞/降落点
for s in S:
    for i in Vs_minus[s]:
        model.addConstr(
            gp.quicksum(xD[i, j, k, s] for j in N for k in Vs_plus[s]
                        if i != j and j != k and i != k)
            <= gp.quicksum(xT[i, h, s] for h in Vs_plus[s] if h != i),
            f"D_takeoff_{s}_{i}"
        )
    for k in Vs_plus[s]:
        model.addConstr(
            gp.quicksum(xD[i, j, k, s] for i in Vs_minus[s] for j in N
                        if i != j and j != k and i != k)
            <= gp.quicksum(xT[h, k, s] for h in Vs_minus[s] if h != k),
            f"D_landing_{s}_{k}"
        )

# 每个节点 (针对每个卫星) 最多作为无人机起飞/降落点一次
for s in S:
    for i in Vs_minus[s]:
        model.addConstr(
            gp.quicksum(xD[i, j, k, s] for j in N for k in Vs_plus[s]
                        if i != j and j != k and i != k) <= 1,
            f"D_takeoff_once_{s}_{i}"
        )
    for k in Vs_plus[s]:
        model.addConstr(
            gp.quicksum(xD[i, j, k, s] for i in Vs_minus[s] for j in N
                        if i != j and j != k and i != k) <= 1,
            f"D_landing_once_{s}_{k}"
        )

# 无人机服务方向与卡车顺序一致 (k ∈ N)
for s in S:
    for i in Vs_minus[s]:
        for k in N:
            if i != k:
                model.addConstr(
                    uT[i, s] - uT[k, s] + 1
                    <= (len(N) + 2) * (1 - gp.quicksum(xD[i, j, k, s] for j in N if i != j and j != k)),
                    f"D_order_{s}_{i}_{k}"
                )

# ============================================================
# 禁止无人机路径 s → j → s'（卫星仓库直达顾客后直接返回虚拟仓库）
# ============================================================

for s in S:
    sp = sat_prime[s]  # s 的虚拟仓库（即 -s）
    for j in N:
        # 禁止路径：s -> j -> sp
        model.addConstr(
            xD[s, j, sp, s] == 0,
            f"drone_no_direct_return_{s}_{j}"
        )

# ============================================================
# 载重与电量约束
# ============================================================

for s in S:
    # 协同卡车载重 (含无人机自重)
    model.addConstr(
        gp.quicksum(
            q[j] * (
                gp.quicksum(xT[i, j, s] for i in Vs_minus[s] if i != j)
                + gp.quicksum(xD[i, j, k, s] for i in Vs_minus[s] for k in Vs_plus[s]
                              if i != j and j != k and i != k)
            )
            for j in N
        ) + w <= QT,
        f"T_weight_{s}"
    )

    for j in N:
        # 无人机载重
        model.addConstr(
            q[j] * gp.quicksum(xD[i, j, k, s] for i in Vs_minus[s] for k in Vs_plus[s]
                               if i != j and j != k and i != k)
            <= QD,
            f"D_weight_{s}_{j}"
        )

        # 无人机电量
        model.addConstr(
            gp.quicksum((e[i, j] + e[j, k]) * xD[i, j, k, s]
                        for i in Vs_minus[s] for k in Vs_plus[s]
                        if i != j and j != k and i != k)
            <= E,
            f"D_battery_{s}_{j}"
        )

# ============================================================
# 时间约束
# ============================================================

for s in S:
    # 初始条件
    model.addConstr(tauT[s, s] == 0, f"tauT_init_{s}")
    model.addConstr(tauD[s, s] == 0, f"tauD_init_{s}")
    model.addConstr(rho[s, s] == 0, f"rho_init_{s}")

    # 卡车到达时间
    for i in Vs_minus[s]:
        for j in Vs_plus[s]:
            if i != j:
                model.addConstr(
                    tauT[j, s] >= rho[i, s] + d[i, j] / vT - M * (1 - xT[i, j, s]),
                    f"tauT_{s}_{i}_{j}"
                )
        # 无人机到达时间
        for j in N:
            for k in Vs_plus[s]:
                if i != j and j != k and i != k:
                    model.addConstr(
                        tauD[k, s] >= rho[i, s] + (d[i, j] + d[j, k]) / vD
                                      - M * (1 - xD[i, j, k, s]),
                        f"tauD_{s}_{i}_{j}_{k}"
                    )

    # 卡车组到达时间 = max{τ^T, τ^D}
    for k in Vs_plus[s]:
        model.addConstr(rho[k, s] >= tauT[k, s], f"rho_T_{s}_{k}")
        model.addConstr(rho[k, s] >= tauD[k, s], f"rho_D_{s}_{k}")

# ============================================================
# 连续两次无人机飞行不能重叠
# ============================================================

# δ 变量与卡车访问边绑定 (初始化)
for s in S:
    # δ_{s, j, s} = 1 如果卡车直接从 s 驶向 j
    for j in Vs_plus[s]:
        model.addConstr(
            delta[s, j, s] == gp.quicksum(xT[i, j, s] for i in Vs_minus[s] if i != j),
            f"delta_init_depart_{s}_{j}"
        )
    # δ_{i, s', s} = 1 如果卡车直接从 i 驶向 s'
    for i in Vs_minus[s]:
        model.addConstr(
            delta[i, sat_prime[s], s] == gp.quicksum(xT[i, j, s] for j in Vs_plus[s] if j != i),
            f"delta_init_arrive_{s}_{i}"
        )

# δ 的顺序关系 (针对顾客节点对)
for s in S:
    for i in N:
        for j in N:
            if i != j:
                model.addConstr(
                    uT[j, s] - uT[i, s] <= (len(N) + 2) * delta[i, j, s],
                    f"delta_order1_{s}_{i}_{j}"
                )
                model.addConstr(
                    uT[i, s] - uT[j, s] + 1 <= (len(N) + 2) * (1 - delta[i, j, s]),
                    f"delta_order2_{s}_{i}_{j}"
                )

# 非重叠约束 (连续无人机飞行)
for s in S:
    for i in Vs_minus[s]:
        for k in Vs_plus[s]:
            if i == k:
                continue
            for l in N:
                if l == i or l == k:
                    continue

                # 第一次无人机飞行 i -> j -> k
                first_flight = gp.quicksum(
                    xD[i, j, k, s] for j in N if j != i and j != k and j != l
                )
                # 第二次无人机飞行 l -> m -> n
                second_flight = gp.quicksum(
                    xD[l, m, n, s]
                    for m in N
                    for n in Vs_plus[s]
                    if (m != l and m != i and m != k and
                        n != l and n != m and n != i and n != k)
                )

                model.addConstr(
                    rho[l, s] >= rho[k, s] - M * (3 - first_flight - second_flight - delta[i, l, s]),
                    f"non_overlap_{s}_{i}_{k}_{l}"
                )

# ============================================================
# 求解参数
# ============================================================

model.Params.TimeLimit = 3600 # 1 小时
model.Params.MIPGap = 1e-5
# 判定整数变量是否"足够接近整数"的阈值。若变量值与最近整数的差距 ≤ IntFeasTol，则视为整数可行。
model.Params.IntFeasTol = 1e-5
# 判定约束是否满足的阈值。若约束左端与右端的差距 ≤ FeasibilityTol，则视为可行。
model.Params.FeasibilityTol = 1e-09

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

# ---------- 模型状态 ----------
print_section("模型求解状态")
print(f"Status: {model.status}  (2=OPTIMAL, 9=TIME_LIMIT, 3=INFEASIBLE)")
if model.SolCount > 0:
    print(f"目标值: {model.objVal}")
    print(f"MIPGap: {model.MIPGap}")
    print(f"求解时间: {model.Runtime:.2f}s")
else:
    print("未找到可行解！")

if model.status in (GRB.OPTIMAL, GRB.TIME_LIMIT):
    # ---------- 目标函数分解 ----------
    print_section("目标函数分解")
    print(f"obj1 (选址成本)  = {obj1.getValue():.5f}")
    print(f"obj2 (一级运输)  = {obj2.getValue():.5f}")
    print(f"obj3 (二级卡车)  = {obj3.getValue():.5f}")
    print(f"obj4 (无人机)    = {obj4.getValue():.5f}")
    print(f"总计             = {model.objVal:.5f}")

    # ---------- 卫星仓库选址 y[s] ----------
    print_section("选址变量 y[s]")
    for s in S:
        val = var_val(y[s])
        print(
            f"  y[{s}] = {val:.4f}  {'✓ 建设' if val is not None and val > 0.5 else '✗ 不建'}"
        )

    # ---------- 一级卡车路径 xL[i,j] ----------
    print_section("一级卡车路径xL[i,j]")
    active_edges = [(i, j) for i in V1_minus for j in V1_plus if i != j and var_val(xL[i, j]) > 0.5]
    if active_edges:
        print("  激活边:")
        for i, j in active_edges:
            print(f"    ({i}, {j})")

        # 构建路径
        current_node = depot
        path = [current_node]
        visited_nodes = set([current_node])

        while True:
            next_node = None
            for j in V1_plus:
                if j not in visited_nodes and var_val(xL[current_node, j]) > 0.5:
                    next_node = j
                    break

            if next_node is None:
                break

            path.append(next_node)
            visited_nodes.add(next_node)
            current_node = next_node

            # 如果到达虚拟终点，则退出
            if current_node == depot_p:
                break

        print(f"  路径: {' -> '.join(map(str, path))}")
    else:
        print("  未找到活跃路径")

    # ---------- 二级卡车路径 xT[i,j,s] ----------
    print_section("二级卡车路径xT[i,j,s]")
    for s in S:
        print(f"\n  卫星仓库 {s}:")
        active_edges = [(i, j) for i in Vs_minus[s] for j in Vs_plus[s]
                       if i != j and var_val(xT[i, j, s]) > 0.5]
        if active_edges:
            print(f"    激活边: {active_edges}")

            # 构建二级路径
            current_node = s
            path = [current_node]
            visited_nodes = set([current_node])

            while True:
                next_node = None
                for j in Vs_plus[s]:
                    if j not in visited_nodes and var_val(xT[current_node, j, s]) > 0.5:
                        next_node = j
                        break

                if next_node is None:
                    break

                path.append(next_node)
                visited_nodes.add(next_node)
                current_node = next_node

                # 如果到达虚拟卫星，则退出
                if current_node == sat_prime[s]:
                    break

            print(f"    路径: {' -> '.join(map(str, path))}")
        else:
            print(f"    卫星 {s} 未被激活或无活跃路径")

    # ---------- 无人机配送 xD[i,j,k,s] ----------
    print_section("无人机配送 xD[i,j,k,s]")
    drone_routes = []
    for s in S:
        print(f"\n  卫星仓库 {s}:")
        active_flights = [(i, j, k) for i in Vs_minus[s] for j in N for k in Vs_plus[s]
                         if i != j and j != k and i != k and var_val(xD[i, j, k, s]) > 0.5]
        if active_flights:
            print(f"    无人机飞行路线: {active_flights}")
            for i, j, k in active_flights:
                print(f"      ({i}) -> ({j}) -> ({k})")
                drone_routes.append((s, i, j, k))
        else:
            print(f"    卫星 {s} 无无人机配送")

    # ---------- 无人机出动顺序 delta[i,j,s] ----------
    # print_section("无人机出动顺序 delta[i,j,s]")
    # for s in S:
    #     print(f"\n  卫星仓库 {s}:")
    #     active_deltas = [(i, j) for i in V_s[s] for j in V_s[s]
    #                     if i != j and var_val(delta[i, j, s]) > 0.5]
    #     if active_deltas:
    #         print(f"    活跃顺序: {active_deltas}")
    #     else:
    #         print(f"    卫星 {s} 无活跃顺序关系")

    # ---------- 时间变量 ----------
    # print_section("卡车到达时间 tauT[i,s]")
    # for s in S:
    #     print(f"\n  卫星仓库 {s}:")
    #     for i in V_s[s]:
    #         val = var_val(tauT[i, s])
    #         if val is not None:
    #             print(f"    tauT[{i},{s}] = {val:.2f}")

    # print_section("无人机到达时间 tauD[i,s]")
    # for s in S:
    #     print(f"\n  卫星仓库 {s}:")
    #     for i in V_s[s]:
    #         val = var_val(tauD[i, s])
    #         if val is not None:
    #             print(f"    tauD[{i},{s}] = {val:.2f}")

    # print_section("节点出发时间 rho[i,s]")
    # for s in S:
    #     print(f"\n  卫星仓库 {s}:")
    #     for i in V_s[s]:
    #         val = var_val(rho[i, s])
    #         if val is not None:
    #             print(f"    rho[{i},{s}] = {val:.2f}")

else:
    print("\n模型无可行解，请检查约束。")

print(f"\n算例名称: {inst['name']}")
print("当前时间:", datetime.now())
