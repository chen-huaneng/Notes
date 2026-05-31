import gurobipy as gp
from gurobipy import GRB
import math

# ============================================================
# 集合定义
# ============================================================

# 顾客集合
N = [1, 2, 3, 4]

# 候选卫星仓库
S = [5, 6]

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
q = {
    1: 2,
    2: 3,
    3: 2,
    4: 4
}

# 卫星仓库建设成本
f = {
    5: 100,
    6: 120
}

# 所有节点的二维坐标(x, y)
coords = {
    0: (0, 0),
    1: (-10, -10),
    2: (5, 5),
    3: (15, 0),
    4: (15, 5),
    5: (5, 0),
    6: (0, -10)
}

# 距离矩阵
d = {}
for i in V:
    for j in V:
        if i != j:
            d[i, j] = math.dist(coords[i], coords[j])

# 一级运输成本
cL = {(i, j): 2 * d[i, j] for i, j in d}

# 二级卡车运输成本
cT = {(i, j): 1.5 * d[i, j] for i, j in d}

# 无人机运输成本
cD = {(i, j): 0.5 * d[i, j] for i, j in d}

# 无人机能耗
e = {(i, j): d[i, j] for i, j in d}

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
            xT[s, j, s]
            for j in N
        ) == y[s]
    )

    # 返回卫星仓库
    model.addConstr(
        gp.quicksum(
            xT[i, s, s]
            for i in N
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

for s in S:
    Ns = N + [s]
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

for s in S:
    Ns = N + [s]
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

for s in S:
    model.addConstr(uT[s, s] == 0)

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

    # ---------- 一级卡车路径 xL ----------
    print_section("一级卡车路径 xL[i,j]")
    found = False
    for i in V1:
        for j in V1:
            if i != j:
                val = var_val(xL[i, j])
                if val > 0.5:
                    print(f"  xL[{i},{j}] = {val:.4f}  ✓  {i} -> {j}")
                    found = True
    if not found:
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
    print_section("二级卡车路径 xT[i,j,s]")
    for s in S:
        print(f"\n  --- 卫星仓库 {s} ---")
        found = False
        for i in V2:
            for j in V2:
                if i != j:
                    val = var_val(xT[i, j, s])
                    if val > 0.5:
                        print(f"    xT[{i},{j},{s}] = {val:.4f}  ✓  {i} -> {j}")
                        found = True
        if not found:
            print("    (无路径)")

    # ---------- 二级载重 uT ----------
    print_section("二级网络访问顺序 uT[i,s]")
    for s in S:
        print(f"\n  --- 卫星仓库 {s} ---")
        for i in V2:
            val = var_val(uT[i, s])
            print(f"    uT[{i},{s}] = {val:.4f}")

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
