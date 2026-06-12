from __future__ import annotations
from dataclasses import dataclass
import gurobipy as gp
from gurobipy import GRB
from instance import Instance

# ================================================================
# 变量容器
# ================================================================

@dataclass
class ModelVars:
    """存储模型所有决策变量的引用"""

    # 选址
    y: dict  # y[s]

    # 一级路径
    xL: dict  # xL[i, j]

    # 二级卡车路径
    xT: dict  # xT[i, j, s]

    # 无人机配送
    xD: dict  # xD[i, j, k, s]

    # 无人机出动顺序变量
    delta: dict  # delta[i, j, s]

    # 一级访问顺序
    uL: dict  # uL[i]

    # 二级访问顺序
    uT: dict  # uT[i, s]

    # 时间变量
    tauT: dict  # tauT[i, s]
    tauD: dict  # tauD[i, s]
    rho: dict   # rho[i, s]

    # 目标函数分量（供结果提取）
    obj_components: dict | None = None

# ================================================================
# 模型构建器
# ================================================================

class ModelBuilder:
    """
    Two-Echelon Truck-Drone MILP 模型构建器

    Parameters
    ----------
    instance : Instance
        算例数据
    big_m : float
        Big-M 常数
    gurobi_params : dict | None
        传递给 Gurobi 的参数，例如 {"TimeLimit": 3600, "MIPGap": 0.0001}
    """

    def __init__(
        self,
        instance: Instance,
        big_m: float = 1e6,
        gurobi_params: dict | None = None,
    ):
        self.inst = instance
        self.big_m = big_m
        self.gurobi_params = gurobi_params or {}

        self.model: gp.Model | None = None
        self.vars: ModelVars | None = None

        # 目标函数各分量（供结果提取使用）
        self.obj1 = None
        self.obj2 = None
        self.obj3 = None
        self.obj4 = None

    # ================================================================
    # 公共接口
    # ================================================================

    def build(self) -> tuple[gp.Model, ModelVars]:
        """构建完整模型并返回 (model, vars)"""

        self._create_model()
        self._create_variables()
        self._add_objective()
        self._add_first_level_constraints()
        self._add_customer_service_constraints()
        self._add_second_level_truck_constraints()
        self._add_warehouse_activation_constraints()
        self._add_second_level_mtz_constraints()
        self._add_drone_constraints()
        self._add_drone_takeoff_landing_limit_constraints()
        self._add_drone_service_order_constraints()
        self._add_drone_no_direct_return_constraint()
        self._add_capacity_constraints()
        self._add_time_constraints()
        self._add_non_overlapping_drone_constraints()

        self._apply_params()

        return self.model, self.vars

    # ================================================================
    # 创建模型
    # ================================================================

    def _create_model(self):
        self.model = gp.Model("2E_Truck_Drone")

    # ================================================================
    # 创建变量
    # ================================================================

    def _create_variables(self):
        m = self.model
        inst = self.inst

        # --- 选址 ---
        y = m.addVars(inst.S, vtype=GRB.BINARY, name="y")

        # --- 一级大型卡车路径 ---
        xL = m.addVars(inst.V1_minus, inst.V1_plus, vtype=GRB.BINARY, name="xL")

        # --- 二级协同卡车路径 ---
        xT = {}
        for s in inst.S:
            for i in inst.V_s_minus[s]:
                for j in inst.V_s_plus[s]:
                    if i != j:
                        xT[i, j, s] = m.addVar(vtype=GRB.BINARY, name=f"xT[{i},{j},{s}]")

        # --- 无人机配送变量 ---
        xD = {}
        for s in inst.S:
            for i in inst.V_s_minus[s]:
                for j in inst.N:
                    for k in inst.V_s_plus[s]:
                        if i != j and j != k and i != k:
                            xD[i, j, k, s] = m.addVar(vtype=GRB.BINARY, name=f"xD[{i},{j},{k},{s}]")

        # --- 无人机出动顺序变量 ---
        delta = {}
        for s in inst.S:
            for i in inst.V_s[s]:
                for j in inst.V_s[s]:
                    if i != j:
                        delta[i, j, s] = m.addVar(vtype=GRB.BINARY, name=f"delta[{i},{j},{s}]")

        # --- 一级访问顺序 / MTZ ---
        uL = m.addVars(inst.V1_minus, vtype=GRB.INTEGER, lb=1, ub=len(inst.S)+1, name="uL")

        # --- 二级访问顺序 / MTZ ---
        uT = {}
        for s in inst.S:
            for i in inst.V_s_minus[s]:
                uT[i, s] = m.addVar(vtype=GRB.INTEGER, lb=1, ub=len(inst.N)+1, name=f"uT[{i},{s}]")

        # --- 时间变量 ---
        tauT = {}
        tauD = {}
        rho = {}
        for s in inst.S:
            for i in inst.V_s[s]:
                tauT[i, s] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"tauT[{i},{s}]")
                tauD[i, s] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"tauD[{i},{s}]")
                rho[i, s] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"rho[{i},{s}]")

        self.vars = ModelVars(
            y=y,
            xL=xL,
            xT=xT,
            xD=xD,
            delta=delta,
            uL=uL,
            uT=uT,
            tauT=tauT,
            tauD=tauD,
            rho=rho,
        )

    # ================================================================
    # 目标函数
    # ================================================================

    def _add_objective(self):
        m = self.model
        inst = self.inst
        v = self.vars

        # obj1: 卫星仓库建设成本
        self.obj1 = gp.quicksum(inst.f[s] * v.y[s] for s in inst.S)

        # obj2: 一级运输成本
        self.obj2 = gp.quicksum(
            inst.cL[i, j] * v.xL[i, j] for i in inst.V1_minus for j in inst.V1_plus if i != j
        )

        # obj3: 二级卡车运输成本
        self.obj3 = gp.quicksum(
            inst.cT[i, j] * v.xT[i, j, s]
            for s in inst.S
            for i in inst.V_s_minus[s]
            for j in inst.V_s_plus[s]
            if i != j
        )

        # obj4: 无人机运输成本
        self.obj4 = gp.quicksum(
            (inst.cD[i, j] + inst.cD[j, k]) * v.xD[i, j, k, s]
            for s in inst.S
            for i in inst.V_s_minus[s]
            for j in inst.N
            for k in inst.V_s_plus[s]
            if i != j and j != k and i != k
        )

        m.setObjective(
            self.obj1 + self.obj2 + self.obj3 + self.obj4,
            GRB.MINIMIZE,
        )

        self.vars.obj_components = {
            "选址成本": self.obj1,
            "一级运输": self.obj2,
            "二级卡车": self.obj3,
            "无人机": self.obj4,
        }

    # ================================================================
    # 一级网络约束
    # ================================================================

    def _add_first_level_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        # ---- 主仓库出发 & 返回虚拟仓库 ----
        m.addConstr(
            gp.quicksum(v.xL[inst.depot, j] for j in inst.S) == 1,
            name="L_depart",
        )
        m.addConstr(
            gp.quicksum(v.xL[i, inst.virtual_depot] for i in inst.S) == 1,
            name="L_arrive",
        )

        # ---- 一级网络流守恒 ----
        for j in inst.S:
            inflow = gp.quicksum(v.xL[i, j] for i in inst.V1_minus if i != j)
            outflow = gp.quicksum(v.xL[j, k] for k in inst.V1_plus if k != j)
            m.addConstr(inflow == outflow, name=f"L_flow_conservation_{j}")
            m.addConstr(outflow == v.y[j], name=f"L_flow_open_{j}")

        # ---- 一级 MTZ 子回路消除 ----
        for i in inst.V1_minus:
            for j in inst.S:
                if i != j:
                    m.addConstr(
                        v.uL[i] - v.uL[j] + 1 <= (len(inst.S) + 1) * (1 - v.xL[i, j]),
                        name=f"L_MTZ_{i}_{j}",
                    )

    # ================================================================
    # 顾客唯一服务约束
    # ================================================================

    def _add_customer_service_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for j in inst.N:
            truck = gp.quicksum(
                v.xT[i, j, s] for s in inst.S for i in inst.V_s_minus[s] if i != j
            )
            drone = gp.quicksum(
                v.xD[i, j, k, s]
                for s in inst.S
                for i in inst.V_s_minus[s]
                for k in inst.V_s_plus[s]
                if i != j and j != k and i != k
            )
            m.addConstr(truck + drone == 1, name=f"customer_service_{j}")

    # ================================================================
    # 二级卡车路径约束
    # ================================================================

    def _add_second_level_truck_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            # ---- 从卫星出发，回到虚拟卫星 ----
            m.addConstr(
                gp.quicksum(v.xT[s, j, s] for j in inst.N) == v.y[s],
                name=f"T_depart_{s}",
            )
            m.addConstr(
                gp.quicksum(v.xT[i, -s, s] for i in inst.N) == v.y[s],
                name=f"T_arrive_{s}",
            )

            # ---- 二级流守恒 (顾客节点) ----
            for j in inst.N:
                inflow = gp.quicksum(v.xT[i, j, s] for i in inst.V_s_minus[s] if i != j)
                outflow = gp.quicksum(v.xT[j, k, s] for k in inst.V_s_plus[s] if k != j)
                m.addConstr(inflow == outflow, name=f"T_flow_conservation_{s}_{j}")
                m.addConstr(outflow <= v.y[s], name=f"T_flow_open_{s}_{j}")

    # ================================================================
    # 仓库激活约束
    # ================================================================

    def _add_warehouse_activation_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            # 卡车路径只在激活的仓库存在
            m.addConstr(
                gp.quicksum(v.xT[i, j, s] for i in inst.V_s_minus[s] for j in inst.V_s_plus[s] if i != j)
                <= self.big_m * v.y[s],
                name=f"T_activation_{s}",
            )

            # 无人机路径只在激活的仓库存在
            m.addConstr(
                gp.quicksum(
                    v.xD[i, j, k, s] for i in inst.V_s_minus[s] for j in inst.N for k in inst.V_s_plus[s]
                    if i != j and j != k and i != k
                )
                <= self.big_m * v.y[s],
                name=f"D_activation_{s}",
            )

    # ================================================================
    # 二级 MTZ 子回路消除
    # ================================================================

    def _add_second_level_mtz_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            for i in inst.V_s_minus[s]:
                for j in inst.N:
                    if i != j:
                        m.addConstr(
                            v.uT[i, s] - v.uT[j, s] + 1
                            <= (len(inst.N) + 2) * (1 - v.xT[i, j, s]),
                            name=f"T_MTZ_{s}_{i}_{j}",
                        )

    # ================================================================
    # 无人机约束
    # ================================================================

    def _add_drone_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            # ---- 无人机起飞约束：起飞节点必须在卡车路径上 ----
            for i in inst.V_s_minus[s]:
                m.addConstr(
                    gp.quicksum(
                        v.xD[i, j, k, s]
                        for j in inst.N
                        for k in inst.V_s_plus[s]
                        if i != j and j != k and i != k
                    )
                    <= gp.quicksum(v.xT[i, h, s] for h in inst.V_s_plus[s] if h != i),
                    name=f"D_takeoff_{s}_{i}",
                )

            # ---- 无人机降落约束：降落节点必须在卡车路径上 ----
            for k in inst.V_s_plus[s]:
                m.addConstr(
                    gp.quicksum(
                        v.xD[i, j, k, s]
                        for i in inst.V_s_minus[s]
                        for j in inst.N
                        if i != j and j != k and i != k
                    )
                    <= gp.quicksum(v.xT[h, k, s] for h in inst.V_s_minus[s] if h != k),
                    name=f"D_landing_{s}_{k}",
                )

        # ---- 无人机容量 ----
        for s in inst.S:
            for j in inst.N:
                m.addConstr(
                    inst.q[j] * gp.quicksum(
                        v.xD[i, j, k, s]
                        for i in inst.V_s_minus[s]
                        for k in inst.V_s_plus[s]
                        if i != j and j != k and i != k
                    )
                    <= inst.QD,
                    name=f"D_weight_{s}_{j}",
                )

        # ---- 无人机电量 ----
        for s in inst.S:
            for j in inst.N:
                m.addConstr(
                    gp.quicksum(
                        (inst.e[i, j] + inst.e[j, k]) * v.xD[i, j, k, s]
                        for i in inst.V_s_minus[s]
                        for k in inst.V_s_plus[s]
                        if i != j and j != k and i != k
                    )
                    <= inst.E,
                    name=f"D_battery_{s}_{j}",
                )

    def _add_drone_takeoff_landing_limit_constraints(self):
        """无人机起降次数限制约束"""
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            # 每个节点（针对每个卫星）最多作为无人机起飞/降落点一次
            for i in inst.V_s_minus[s]:
                m.addConstr(
                    gp.quicksum(
                        v.xD[i, j, k, s]
                        for j in inst.N
                        for k in inst.V_s_plus[s]
                        if i != j and j != k and i != k
                    )
                    <= 1,
                    name=f"D_takeoff_once_{s}_{i}",
                )

            for k in inst.V_s_plus[s]:
                m.addConstr(
                    gp.quicksum(
                        v.xD[i, j, k, s]
                        for i in inst.V_s_minus[s]
                        for j in inst.N
                        if i != j and j != k and i != k
                    )
                    <= 1,
                    name=f"D_landing_once_{s}_{k}",
                )

    def _add_drone_service_order_constraints(self):
        """无人机服务顺序约束（无人机的服务顺序与卡车一致）"""
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            for i in inst.V_s_minus[s]:
                for k in inst.N:
                    if i != k:
                        m.addConstr(
                            v.uT[i, s] - v.uT[k, s] + 1
                            <= (len(inst.N) + 2) * (
                                1
                                - gp.quicksum(
                                    v.xD[i, j, k, s] for j in inst.N if i != j and j != k
                                )
                            ),
                            name=f"D_order_{s}_{i}_{k}",
                        )

    def _add_drone_no_direct_return_constraint(self):
        """禁止无人机路径 s → j → s'（卫星仓库直达顾客后直接返回虚拟仓库）"""
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            sp = -s  # 虚拟卫星仓库
            for j in inst.N:
                # 禁止路径：s -> j -> sp
                m.addConstr(
                    v.xD[s, j, sp, s] == 0,
                    name=f"drone_no_direct_return_{s}_{j}",
                )

    # ================================================================
    # 容量约束
    # ================================================================

    def _add_capacity_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            # 协同卡车载重 (含无人机自重)
            m.addConstr(
                gp.quicksum(
                    inst.q[j] * (
                        gp.quicksum(v.xT[i, j, s] for i in inst.V_s_minus[s] if i != j)
                        + gp.quicksum(
                            v.xD[i, j, k, s] for i in inst.V_s_minus[s] for k in inst.V_s_plus[s]
                            if i != j and j != k and i != k
                        )
                    )
                    for j in inst.N
                ) + inst.w <= inst.QT,
                name=f"T_weight_{s}",
            )

    # ================================================================
    # 时间约束
    # ================================================================

    def _add_time_constraints(self):
        """添加时间约束"""
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            # 初始条件
            m.addConstr(v.tauT[s, s] == 0, f"tauT_init_{s}")
            m.addConstr(v.tauD[s, s] == 0, f"tauD_init_{s}")
            m.addConstr(v.rho[s, s] == 0, f"rho_init_{s}")

            # 卡车到达时间
            for i in inst.V_s_minus[s]:
                for j in inst.V_s_plus[s]:
                    if i != j:
                        m.addConstr(
                            v.tauT[j, s] >= v.rho[i, s] + inst.d[i, j] / inst.vT - self.big_m * (1 - v.xT[i, j, s]),
                            f"tauT_{s}_{i}_{j}"
                        )

                # 无人机到达时间
                for j in inst.N:
                    for k in inst.V_s_plus[s]:
                        if i != j and j != k and i != k:
                            m.addConstr(
                                v.tauD[k, s] >= v.rho[i, s] + (inst.d[i, j] + inst.d[j, k]) / inst.vD
                                              - self.big_m * (1 - v.xD[i, j, k, s]),
                                f"tauD_{s}_{i}_{j}_{k}"
                            )

            # 卡车组到达时间 = max{τ^T, τ^D}
            for k in inst.V_s_plus[s]:
                m.addConstr(v.rho[k, s] >= v.tauT[k, s], f"rho_T_{s}_{k}")
                m.addConstr(v.rho[k, s] >= v.tauD[k, s], f"rho_D_{s}_{k}")

    # ================================================================
    # 非重叠无人机约束
    # ================================================================

    def _add_non_overlapping_drone_constraints(self):
        """添加非重叠无人机约束"""
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            sat_prime = inst.sat_prime[s]

            # δ 变量与卡车访问边绑定 (初始化)
            # δ_{s, j, s} = 1 如果卡车直接从 s 驶向 j
            for j in inst.V_s_plus[s]:
                m.addConstr(
                    v.delta[s, j, s] == gp.quicksum(v.xT[i, j, s] for i in inst.V_s_minus[s] if i != j),
                    f"delta_init_depart_{s}_{j}"
                )

            # δ_{i, s', s} = 1 如果卡车直接从 i 驶向 s'
            for i in inst.V_s_minus[s]:
                m.addConstr(
                    v.delta[i, sat_prime, s] == gp.quicksum(v.xT[i, j, s] for j in inst.V_s_plus[s] if j != i),
                    f"delta_init_arrive_{s}_{i}"
                )

            # δ 的顺序关系 (针对顾客节点对)
            for i in inst.N:
                for j in inst.N:
                    if i != j:
                        m.addConstr(
                            v.uT[j, s] - v.uT[i, s] <= (len(inst.N) + 2) * v.delta[i, j, s],
                            f"delta_order1_{s}_{i}_{j}"
                        )
                        m.addConstr(
                            v.uT[i, s] - v.uT[j, s] + 1 <= (len(inst.N) + 2) * (1 - v.delta[i, j, s]),
                            f"delta_order2_{s}_{i}_{j}"
                        )

            # 非重叠约束 (连续无人机飞行)
            for i in inst.V_s_minus[s]:
                for k in inst.V_s_plus[s]:
                    if i == k:
                        continue
                    for l in inst.N:
                        if l == i or l == k:
                            continue

                        # 第一次无人机飞行 i -> j -> k
                        first_flight = gp.quicksum(
                            v.xD[i, j, k, s] for j in inst.N if j != i and j != k and j != l
                        )
                        # 第二次无人机飞行 l -> m -> n
                        second_flight = gp.quicksum(
                            v.xD[l, m, n, s]
                            for m in inst.N
                            for n in inst.V_s_plus[s]
                            if (m != l and m != i and m != k and
                                n != l and n != m and n != i and n != k)
                        )

                        m.addConstr(
                            v.rho[l, s] >= v.rho[k, s] - self.big_m * (3 - first_flight - second_flight - v.delta[i, l, s]),
                            f"non_overlap_{s}_{i}_{k}_{l}"
                        )

    # ================================================================
    # 求解参数
    # ================================================================

    def _apply_params(self):
        for key, val in self.gurobi_params.items():
            self.model.setParam(key, val)
