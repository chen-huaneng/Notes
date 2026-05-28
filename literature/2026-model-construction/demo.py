import gurobipy as gp
from gurobipy import GRB
from itertools import product
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
tauT = model.addVars(V2, S,
                     lb=0,
                     vtype=GRB.CONTINUOUS,
                     name="tauT")

tauD = model.addVars(V2, S,
                     lb=0,
                     vtype=GRB.CONTINUOUS,
                     name="tauD")

rho = model.addVars(V2, S,
                    lb=0,
                    vtype=GRB.CONTINUOUS,
                    name="rho")

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
                    for i in S if i != j)
        <= y[j]
    )

    model.addConstr(
        gp.quicksum(xL[j, k]
                    for k in S if k != j)
        <= y[j]
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
            for i in V2
            if i != j
        )

        +

        gp.quicksum(
            xD[i, j, k, s]
            for s in S
            for i in V2
            for k in V2
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
    for j in N:

        model.addConstr(
            gp.quicksum(
                xT[i, j, s]
                for i in N if i != j
            )
            ==
            gp.quicksum(
                xT[j, k, s]
                for k in N if k != j
            )
        )

# ============================================================
# 仓库激活约束
# ============================================================

for s in S:
    for i in V2:
        for j in V2:
            if i != j:

                model.addConstr(
                    xT[i, j, s] <= y[s]
                )

for s in S:
    for i in V2:
        for j in N:
            for k in V2:

                if i != j and j != k and i != k:

                    model.addConstr(
                        xD[i, j, k, s] <= y[s]
                    )

# ============================================================
# 二级MTZ
# ============================================================

for s in S:
    for i in N:
        for j in N:

            if i != j:

                model.addConstr(
                    uT[j, s]
                    >=
                    uT[i, s] + 1
                    - M * (1 - xT[i, j, s])
                )

# ============================================================
# 无人机起降约束
# ============================================================

for s in S:
    for i in V2:

        model.addConstr(

            gp.quicksum(
                xD[i, j, k, s]
                for j in N
                for k in V2
                if i != j and j != k and i != k
            )

            <=

            gp.quicksum(
                xT[h, i, s]
                for h in V2 if h != i
            )
        )

for s in S:
    for k in V2:

        model.addConstr(

            gp.quicksum(
                xD[i, j, k, s]
                for i in V2
                for j in N
                if i != j and j != k and i != k
            )

            <=

            gp.quicksum(
                xT[h, k, s]
                for h in V2 if h != k
            )
        )

# ============================================================
# 卡车容量
# ============================================================

for s in S:

    model.addConstr(

        gp.quicksum(

            q[j] * (
                gp.quicksum(
                    xT[i, j, s]
                    for i in V2 if i != j
                )

                +

                gp.quicksum(
                    xD[i, j, k, s]
                    for i in V2
                    for k in V2
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

    model.addConstr(

        psi[s]

        ==

        gp.quicksum(

            q[j] * (

                gp.quicksum(
                    xT[i, j, s]
                    for i in V2 if i != j
                )

                +

                gp.quicksum(
                    xD[i, j, k, s]
                    for i in V2
                    for k in V2
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
    for j in N:

        model.addConstr(

            gp.quicksum(
                q[j] * xD[i, j, k, s]
                for i in V2
                for k in V2
                if i != j and j != k and i != k
            )

            <= QD
        )

# ============================================================
# 无人机电量
# ============================================================

for s in S:
    for j in N:

        model.addConstr(

            gp.quicksum(
                (e[i, j] + e[j, k])
                * xD[i, j, k, s]

                for i in V2
                for k in V2
                if i != j and j != k and i != k
            )

            <= E
        )

# ============================================================
# 时间同步约束
# ============================================================

for s in S:
    for i in V2:
        for j in V2:

            if i != j:

                model.addConstr(
                    tauT[j, s]
                    >=
                    rho[i, s]
                    + d[i, j] / vT
                    - M * (1 - xT[i, j, s])
                )

for s in S:
    for i in V2:
        for j in N:
            for k in V2:

                if i != j and j != k and i != k:

                    model.addConstr(
                        tauD[k, s]
                        >=
                        rho[i, s]
                        + (d[i, j] + d[j, k]) / vD
                        - M * (1 - xD[i, j, k, s])
                    )

# 等待同步
for s in S:
    for k in V2:

        model.addConstr(
            rho[k, s] >= tauT[k, s]
        )

        model.addConstr(
            rho[k, s]
            >=
            tauD[k, s]
            - M * (1 - gp.quicksum(
                           xD[i, j, k, s]
                           for i in V2
                           for j in N
                           if i != j and j != k and i != k
                        ))
        )

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
model.Params.MIPGap = 0.01

# ============================================================
# 求解
# ============================================================

model.optimize()

# ============================================================
# 输出结果
# ============================================================

if model.status == GRB.OPTIMAL or model.status == GRB.TIME_LIMIT:

    print("\n最优目标值:", model.objVal)

    print("\n建设的卫星仓库:")
    for s in S:
        if y[s].X > 0.5:
            print(f"卫星仓库 {s}")

    print("\n一级卡车路径:")
    for i in V1:
        for j in V1:
            if i != j and xL[i, j].X > 0.5:
                print(i, "->", j)

    print("\n二级卡车路径:")
    for s in S:
        for i in V2:
            for j in V2:
                # if i != j:
                    # print(i, j, xT[i, j, s].X)
                if i != j and xT[i, j, s].X > 0.5:
                    print(f"仓库{s}: {i} -> {j}")

    print("\n无人机配送:")
    for s in S:
        for i in V2:
            for j in N:
                for k in V2:

                    if i != j and j != k and i != k:

                        if xD[i, j, k, s].X > 0.5:

                            print(
                                f"仓库{s}: "
                                f"{i} -> 无人机服务顾客{j} -> {k}"
                            )

else:
    print("模型无可行解")
