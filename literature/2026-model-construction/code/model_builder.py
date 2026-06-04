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
    # delta: dict                      # delta[i, j, s]

    # 一级载重
    uL: dict  # uL[i]

    # 二级载重 / MTZ
    uT: dict  # uT[i, s]

    # 卫星需求量
    psi: dict  # psi[s]

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
    enable_time_sync : bool
        是否启用时间同步约束
    big_m : float
        Big-M 常数
    gurobi_params : dict | None
        传递给 Gurobi 的参数，例如 {"TimeLimit": 3600, "MIPGap": 0.0001}
    """

    def __init__(
        self,
        instance: Instance,
        # enable_time_sync: bool = False,
        big_m: float = 1e6,
        gurobi_params: dict | None = None,
    ):
        self.inst = instance
        # self.enable_time_sync = enable_time_sync
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
        # self._add_drone_non_overlap_constraints()
        self._add_capacity_constraints()
        self._add_demand_linking_constraints()

        self._set_initial_conditions()
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
        xL = m.addVars(inst.V1, inst.V1, vtype=GRB.BINARY, name="xL")

        # --- 二级协同卡车路径 ---
        xT = m.addVars(inst.V2, inst.V2, inst.S, vtype=GRB.BINARY, name="xT")

        # --- 无人机配送变量 ---
        xD = m.addVars(inst.V2, inst.N, inst.V2, inst.S, vtype=GRB.BINARY, name="xD")

        # --- 无人机出动顺序变量 ---
        # delta = m.addVars(inst.V2, inst.V2, inst.S, vtype=GRB.BINARY, name="delta")

        # --- 一级载重 ---
        uL = m.addVars(inst.V1, lb=0, vtype=GRB.CONTINUOUS, name="uL")

        # --- 二级访问顺序 / MTZ ---
        uT = m.addVars(inst.V2, inst.S, lb=0, vtype=GRB.CONTINUOUS, name="uT")

        # --- 卫星仓库需求量 ---
        psi = m.addVars(inst.S, lb=0, vtype=GRB.CONTINUOUS, name="psi")

        self.vars = ModelVars(
            y=y,
            xL=xL,
            xT=xT,
            xD=xD,
            # delta=delta,
            uL=uL,
            uT=uT,
            psi=psi,
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
            inst.cL[i, j] * v.xL[i, j] for i in inst.V1 for j in inst.V1 if i != j
        )

        # obj3: 二级卡车运输成本
        self.obj3 = gp.quicksum(
            inst.cT[i, j] * v.xT[i, j, s]
            for s in inst.S
            for i in inst.V2
            for j in inst.V2
            if i != j
        )

        # obj4: 无人机运输成本
        self.obj4 = gp.quicksum(
            (inst.cD[i, j] + inst.cD[j, k]) * v.xD[i, j, k, s]
            for s in inst.S
            for i in inst.V2
            for j in inst.N
            for k in inst.V2
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

        # ---- 主仓库出发 & 返回 ----
        m.addConstr(
            gp.quicksum(v.xL[inst.depot, i] for i in inst.S) == 1,
            name="depot_depart",
        )
        m.addConstr(
            gp.quicksum(v.xL[i, inst.depot] for i in inst.S) == 1,
            name="depot_return",
        )

        # ---- 一级流平衡 ----
        for j in inst.S:
            m.addConstr(
                gp.quicksum(v.xL[i, j] for i in inst.V1 if i != j) == v.y[j],
                name=f"flow_in_L_{j}",
            )
            m.addConstr(
                gp.quicksum(v.xL[j, k] for k in inst.V1 if k != j) == v.y[j],
                name=f"flow_out_L_{j}",
            )

        # ---- 一级 MTZ 子回路消除 ----
        for i in inst.V1:
            for j in inst.S:
                if i != j:
                    m.addConstr(
                        v.uL[j] >= v.uL[i] + v.psi[j] - self.big_m * (1 - v.xL[i, j]),
                        name=f"mtz_L_{i}_{j}",
                    )

        # ---- 一级容量 ----
        for s in inst.S:
            m.addConstr(v.uL[s] >= v.psi[s], name=f"cap_L_lb_{s}")
            m.addConstr(v.uL[s] <= inst.QL, name=f"cap_L_ub_{s}")

    # ================================================================
    # 顾客唯一服务约束
    # ================================================================

    def _add_customer_service_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for j in inst.N:
            m.addConstr(
                # 卡车服务
                gp.quicksum(
                    v.xT[i, j, s] for s in inst.S for i in inst.N + [s] if i != j
                )
                +
                # 无人机服务
                gp.quicksum(
                    v.xD[i, j, k, s]
                    for s in inst.S
                    for i in inst.N + [s]
                    for k in inst.N + [s]
                    if i != j and j != k and i != k
                )
                == 1,
                name=f"service_{j}",
            )

    # ================================================================
    # 二级卡车路径约束
    # ================================================================

    def _add_second_level_truck_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            Ns = inst.N + [s]

            # ---- 从卫星仓库出发 ----
            m.addConstr(
                gp.quicksum(v.xT[s, i, s] for i in inst.N) == v.y[s],
                name=f"truck_depart_{s}",
            )

            # ---- 返回卫星仓库 ----
            m.addConstr(
                gp.quicksum(v.xT[j, s, s] for j in inst.N) == v.y[s],
                name=f"truck_return_{s}",
            )

            # ---- 流平衡 ----
            for j in inst.N:
                m.addConstr(
                    gp.quicksum(v.xT[i, j, s] for i in Ns if i != j)
                    == gp.quicksum(v.xT[j, k, s] for k in Ns if k != j),
                    name=f"flow_T_{j}_{s}",
                )

    # ================================================================
    # 仓库激活约束
    # ================================================================

    def _add_warehouse_activation_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            Ns = inst.N + [s]

            # 卡车路径只在激活的仓库存在
            m.addConstr(
                gp.quicksum(v.xT[i, j, s] for i in Ns for j in Ns if i != j)
                <= self.big_m * v.y[s],
                name=f"activate_T_{s}",
            )

            # 无人机路径只在激活的仓库存在
            m.addConstr(
                gp.quicksum(
                    v.xD[i, j, k, s]
                    for i in Ns
                    for j in inst.N
                    for k in Ns
                    if i != j and k != i and k != j
                )
                <= self.big_m * v.y[s],
                name=f"activate_D_{s}",
            )

    # ================================================================
    # 二级 MTZ 子回路消除
    # ================================================================

    def _add_second_level_mtz_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            for i in inst.N:
                for j in inst.N:
                    if i != j:
                        m.addConstr(
                            v.uT[i, s] - v.uT[j, s] + 1
                            <= (len(inst.N) + 1) * (1 - v.xT[i, j, s]),
                            name=f"mtz_T_{i}_{j}_{s}",
                        )

    # ================================================================
    # 无人机约束
    # ================================================================

    def _add_drone_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            Ns = inst.N + [s]

            # ---- 无人机起飞约束：起飞节点必须在卡车路径上 ----
            for i in Ns:
                m.addConstr(
                    gp.quicksum(
                        v.xD[i, j, k, s]
                        for j in inst.N
                        for k in Ns
                        if i != j and j != k and i != k
                    )
                    <= gp.quicksum(v.xT[i, h, s] for h in Ns if h != i),
                    name=f"drone_launch_{i}_{s}",
                )

            # ---- 无人机降落约束：降落节点必须在卡车路径上 ----
            for k in Ns:
                m.addConstr(
                    gp.quicksum(
                        v.xD[i, j, k, s]
                        for i in Ns
                        for j in inst.N
                        if i != j and j != k and i != k
                    )
                    <= gp.quicksum(v.xT[h, k, s] for h in Ns if h != k),
                    name=f"drone_land_{k}_{s}",
                )

        # ---- 无人机容量 ----
        for s in inst.S:
            Ns = inst.N + [s]
            for j in inst.N:
                m.addConstr(
                    gp.quicksum(
                        inst.q[j] * v.xD[i, j, k, s]
                        for i in Ns
                        for k in Ns
                        if i != j and j != k and i != k
                    )
                    <= inst.QD,
                    name=f"drone_cap_{j}_{s}",
                )

        # ---- 无人机电量 ----
        for s in inst.S:
            Ns = inst.N + [s]
            for j in inst.N:
                m.addConstr(
                    gp.quicksum(
                        (inst.e[i, j] + inst.e[j, k]) * v.xD[i, j, k, s]
                        for i in Ns
                        for k in Ns
                        if i != j and j != k and i != k
                    )
                    <= inst.E,
                    name=f"drone_energy_{j}_{s}",
                )

    def _add_drone_takeoff_landing_limit_constraints(self):
        """无人机起降次数限制约束"""
        m = self.model
        inst = self.inst
        v = self.vars

        # 无人机在顾客节点和卫星仓库最多起飞一次
        for i in inst.V2:
            m.addConstr(
                gp.quicksum(
                    v.xD[i, j, k, s]
                    for s in inst.S
                    for j in inst.N
                    for k in inst.V2
                    if i != j and j != k and k != i
                )
                <= 1,
                name=f"drone_takeoff_limit_{i}",
            )

        # 无人机在顾客节点和卫星仓库最多降落一次
        for k in inst.V2:
            m.addConstr(
                gp.quicksum(
                    v.xD[i, j, k, s]
                    for s in inst.S
                    for i in inst.V2
                    for j in inst.N
                    if i != j and j != k and i != k
                )
                <= 1,
                name=f"drone_landing_limit_{k}",
            )

    def _add_drone_service_order_constraints(self):
        """无人机服务顺序约束（无人机的服务顺序不能和卡车相反）"""
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            Ns = inst.N + [s]
            for i in Ns:
                for k in inst.N:
                    if i == k:
                        continue
                    m.addConstr(
                        v.uT[i, s] - v.uT[k, s] + 1
                        <= (len(inst.N) + 1)
                        * (
                            1
                            - gp.quicksum(
                                v.xD[i, j, k, s] for j in inst.N if j != i and j != k
                            )
                        ),
                        name=f"drone_order_{i}_{k}_{s}",
                    )

    # def _add_drone_non_overlap_constraints(self):
    #     """无人机出动非重叠约束"""
    #     m = self.model
    #     inst = self.inst
    #     v = self.vars

    #     for s in inst.S:
    #         Ns = inst.N + [s]
    #         for i in Ns:
    #             for k in Ns:
    #                 if i == k:
    #                     continue
    #                 for l in Ns:
    #                     for n in Ns:
    #                         if l == n:
    #                             continue

    #                         m.addConstr(
    #                             v.uT[k, s] - v.uT[l, s]
    #                             <= len(inst.N) * (
    #                                 3
    #                                 - gp.quicksum(v.xD[i, j, k, s] for j in inst.N if j != i and j != k)
    #                                 - gp.quicksum(v.xD[l, m, n, s] for m in inst.N if m != l and m != n)
    #                                 - v.delta[i, l, s]
    #                             ),
    #                             name=f"drone_no_overlap1_{i}_{k}_{l}_{n}_{s}",
    #                         )

    #                         m.addConstr(
    #                             v.uT[n, s] - v.uT[i, s]
    #                             <= len(inst.N) * (
    #                                 2
    #                                 - gp.quicksum(v.xD[i, j, k, s] for j in inst.N if j != i and j != k)
    #                                 - gp.quicksum(v.xD[l, m, n, s] for m in inst.N if m != l and m != n)
    #                                 + v.delta[i, l, s]
    #                             ),
    #                             name=f"drone_no_overlap2_{i}_{k}_{l}_{n}_{s}",
    #                         )

    # ================================================================
    # 容量约束
    # ================================================================

    def _add_capacity_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            Ns = inst.N + [s]

            m.addConstr(
                gp.quicksum(
                    inst.q[j]
                    * (
                        gp.quicksum(v.xT[i, j, s] for i in Ns if i != j)
                        + gp.quicksum(
                            v.xD[i, j, k, s]
                            for i in Ns
                            for k in Ns
                            if i != j and j != k and i != k
                        )
                    )
                    for j in inst.N
                )
                + inst.w
                <= inst.QT,
                name=f"truck_cap_{s}",
            )

    # ================================================================
    # 需求关联约束
    # ================================================================

    def _add_demand_linking_constraints(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            Ns = inst.N + [s]

            m.addConstr(
                v.psi[s]
                == gp.quicksum(
                    inst.q[j]
                    * (
                        gp.quicksum(v.xT[i, j, s] for i in Ns if i != j)
                        + gp.quicksum(
                            v.xD[i, j, k, s]
                            for i in Ns
                            for k in Ns
                            if i != j and j != k and i != k
                        )
                    )
                    for j in inst.N
                ),
                name=f"demand_link_{s}",
            )

    # ================================================================
    # 初始条件
    # ================================================================

    def _set_initial_conditions(self):
        m = self.model
        inst = self.inst
        v = self.vars

        for s in inst.S:
            m.addConstr(v.uT[s, s] == 0, name=f"init_uT_{s}")

        m.addConstr(v.uL[inst.depot] == 0, name="init_uL_depot")

    # ================================================================
    # 求解参数
    # ================================================================

    def _apply_params(self):
        for key, val in self.gurobi_params.items():
            self.model.setParam(key, val)
