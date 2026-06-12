from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import gurobipy as gp
from instance import Instance
from model_builder import ModelVars

# ================================================================
# 求解结果数据类
# ================================================================

@dataclass
class SolutionResult:
    """存储模型求解后的全部结果"""

    # ---- 求解状态 ----
    status: int
    obj_val: Optional[float]
    obj_breakdown: dict[str, float]
    mip_gap: Optional[float]
    runtime: float

    # ---- 选址 ----
    satellites_opened: list[int]
    satellite_demand: dict[int, float]

    # ---- 一级网络 ----
    first_level_edges: list[tuple[int, int]]
    first_level_routes: list[list[int]]
    first_level_loads: dict[int, float]

    # ---- 二级网络 ----
    second_level_truck_edges: dict[int, list[tuple[int, int]]]
    second_level_truck_routes: dict[int, list[list[int]]]
    second_level_mtz: dict[int, dict[int, float]]

    # ---- 无人机 ----
    drone_trips: list[dict]

    # ---- 时间变量 ----
    time_variables: dict[str, dict]

# ================================================================
# 结果提取器
# ================================================================

class SolutionExtractor:
    """从 Gurobi 模型中提取求解结果"""

    def __init__(
        self,
        model: gp.Model,
        variables: ModelVars,
        instance: Instance,
    ):
        self.model = model
        self.vars = variables
        self.inst = instance

    # ---- 安全读取 ----

    @staticmethod
    def _val(v) -> Optional[float]:
        """安全获取变量值"""
        try:
            return v.X
        except Exception:
            return None

    # ---- 主提取接口 ----

    def extract(self) -> SolutionResult:
        m = self.model

        # 无可行解时返回空结果
        if m.SolCount == 0:
            return SolutionResult(
                status=m.status,
                obj_val=None,
                obj_breakdown={},
                mip_gap=None,
                runtime=m.Runtime,
                satellites_opened=[],
                satellite_demand={},
                first_level_edges=[],
                first_level_routes=[],
                first_level_loads={},
                second_level_truck_edges={},
                second_level_truck_routes={},
                second_level_mtz={},
                drone_trips=[],
                time_variables={}
            )

        # ---- 基本状态 ----
        obj_val = m.objVal
        mip_gap = m.MIPGap
        runtime = m.Runtime

        # ---- 目标函数分解 ----
        obj_breakdown = self._extract_obj_breakdown()

        # ---- 选址 ----
        satellites_opened = self._extract_satellites()
        satellite_demand = self._extract_satellite_demand()

        # ---- 一级网络 ----
        first_level_edges = self._extract_first_level_edges()
        first_level_routes = self._build_first_level_routes(first_level_edges)
        first_level_loads = self._extract_first_level_loads()

        # ---- 二级网络 ----
        second_level_truck_edges = self._extract_second_level_edges()
        second_level_truck_routes = {}
        for s, edges in second_level_truck_edges.items():
            second_level_truck_routes[s] = self._build_second_level_routes(
                edges, s
            )
        second_level_mtz = self._extract_second_level_mtz()

        # ---- 无人机 ----
        drone_trips = self._extract_drone_trips()

        # ---- 时间变量 ----
        time_variables = self._extract_time_variables()

        return SolutionResult(
            status=m.status,
            obj_val=obj_val,
            obj_breakdown=obj_breakdown,
            mip_gap=mip_gap,
            runtime=runtime,
            satellites_opened=satellites_opened,
            satellite_demand=satellite_demand,
            first_level_edges=first_level_edges,
            first_level_routes=first_level_routes,
            first_level_loads=first_level_loads,
            second_level_truck_edges=second_level_truck_edges,
            second_level_truck_routes=second_level_truck_routes,
            second_level_mtz=second_level_mtz,
            drone_trips=drone_trips,
            time_variables=time_variables
        )

    # ---- 目标函数分解 ----
    def _extract_obj_breakdown(self) -> dict[str, float]:
        result = {}
        components = self.vars.obj_components
        if components is None:
            return result
        for label, expr in components.items():
            try:
                result[label] = expr.getValue()
            except Exception:
                result[label] = None
        return result

    # ---- 选址 ----

    def _extract_satellites(self) -> list[int]:
        opened = []
        for s in self.inst.S:
            val = self._val(self.vars.y[s])
            if val is not None and val > 0.5:
                opened.append(s)
        return opened

    def _extract_satellite_demand(self) -> dict[int, float]:
        demand = {}
        for s in self.inst.S:
            # 计算卫星的需求量（由分配给它的客户决定）
            demand[s] = sum(
                self.inst.q[j] for j in self.inst.N
                if any(self._val(self.vars.xT[i, j, s]) > 0.5 for i in self.inst.V_s_minus[s] if i != j) or
                   any(self._val(self.vars.xD[i, j, k, s]) > 0.5
                       for i in self.inst.V_s_minus[s]
                       for k in self.inst.V_s_plus[s]
                       if i != j and j != k and i != k)
            )
        return demand

    # ---- 一级网络 ----

    def _extract_first_level_edges(self) -> list[tuple[int, int]]:
        edges = []
        for i in self.inst.V1_minus:
            for j in self.inst.V1_plus:
                if i != j:
                    val = self._val(self.vars.xL[i, j])
                    if val is not None and val > 0.5:
                        edges.append((i, j))
        return edges

    def _build_first_level_routes(self, edges: list[tuple[int, int]]) -> list[list[int]]:
        """构建一级网络路径"""
        if not edges:
            return []

        # 构建邻接表
        adj = {}
        for i, j in edges:
            adj.setdefault(i, []).append(j)

        routes = []
        current_node = self.inst.depot
        path = [current_node]

        while current_node != self.inst.virtual_depot:
            next_nodes = adj.get(current_node, [])
            if not next_nodes:
                break
            # 找到下一个节点
            next_node = next_nodes[0]
            path.append(next_node)
            current_node = next_node

        routes.append(path)
        return routes

    def _extract_first_level_loads(self) -> dict[int, float]:
        loads = {}
        for i in self.inst.V1_minus:
            val = self._val(self.vars.uL[i])
            if val is not None:
                loads[i] = val
        return loads

    # ---- 二级网络 ----

    def _extract_second_level_edges(self) -> dict[int, list[tuple[int, int]]]:
        result = {}
        for s in self.inst.S:
            edges = []
            for i in self.inst.V_s_minus[s]:
                for j in self.inst.V_s_plus[s]:
                    if i != j:
                        val = self._val(self.vars.xT[i, j, s])
                        if val is not None and val > 0.5:
                            edges.append((i, j))
            result[s] = edges
        return result

    def _build_second_level_routes(self, edges: list[tuple[int, int]], start_node: int) -> list[list[int]]:
        """构建二级网络路径"""
        if not edges:
            return []

        # 构建邻接表
        adj = {}
        for i, j in edges:
            adj.setdefault(i, []).append(j)

        routes = []
        current_node = start_node
        path = [current_node]

        while True:
            next_nodes = adj.get(current_node, [])
            if not next_nodes:
                break
            # 找到下一个节点
            next_node = next_nodes[0]
            path.append(next_node)
            current_node = next_node

            # 如果到达虚拟卫星仓库，则停止
            if current_node == -start_node:
                break

        routes.append(path)
        return routes

    def _extract_second_level_mtz(self) -> dict[int, dict[int, float]]:
        result = {}
        for s in self.inst.S:
            result[s] = {}
            for i in self.inst.V_s_minus[s]:
                val = self._val(self.vars.uT[i, s])
                if val is not None:
                    result[s][i] = val
        return result

    # ---- 无人机 ----

    def _extract_drone_trips(self) -> list[dict]:
        trips = []
        inst = self.inst

        for s in inst.S:
            for i in inst.V_s_minus[s]:
                for j in inst.N:
                    for k in inst.V_s_plus[s]:
                        if i != j and j != k and i != k:
                            val = self._val(self.vars.xD[i, j, k, s])
                            if val is not None and val > 0.5:
                                trips.append({
                                    "satellite": s,
                                    "launch_node": i,
                                    "customer": j,
                                    "land_node": k,
                                    "cost": inst.cD[i, j] + inst.cD[j, k],
                                    "energy": inst.e[i, j] + inst.e[j, k],
                                })
        return trips

    # ---- 时间变量 ----
    def _extract_time_variables(self) -> dict[str, dict]:
        result = {
            "tauT": {},
            "tauD": {},
            "rho": {}
        }

        for s in self.inst.S:
            result["tauT"][s] = {}
            result["tauD"][s] = {}
            result["rho"][s] = {}

            for i in self.inst.V_s[s]:
                val = self._val(self.vars.tauT[i, s])
                if val is not None:
                    result["tauT"][s][i] = val

                val = self._val(self.vars.tauD[i, s])
                if val is not None:
                    result["tauD"][s][i] = val

                val = self._val(self.vars.rho[i, s])
                if val is not None:
                    result["rho"][s][i] = val

        return result

# ================================================================
# 结果打印器
# ================================================================

class ResultPrinter:
    """终端美化打印求解结果"""

    # 分隔线
    _SEP = "=" * 60

    def __init__(self, result: SolutionResult, instance: Instance):
        self.res = result
        self.inst = instance

    def print_all(self):
        self._print_status()

        if self.res.obj_val is None:
            print("\n未找到可行解！")
            return

        self._print_objective()
        self._print_satellites()
        self._print_customer_demands()
        self._print_first_level()
        self._print_second_level()
        self._print_drones()
        self._print_time_variables()

    # ---- 内部方法 ----

    def _section(self, title: str):
        print(f"\n{self._SEP}")
        print(f"  {title}")
        print(f"{self._SEP}")

    def _print_status(self):
        self._section("模型求解状态")
        status_map = {2: "OPTIMAL", 9: "TIME_LIMIT", 3: "INFEASIBLE"}
        status_str = status_map.get(self.res.status, str(self.res.status))
        print(f"Status: {self.res.status}  ({status_str})")

        if self.res.obj_val is not None:
            print(f"目标值:   {self.res.obj_val}")
            if self.res.mip_gap is not None:
                print(f"MIPGap:   {self.res.mip_gap}")
            print(f"求解时间: {self.res.runtime:.2f}s")

    def _print_objective(self):
        self._section("目标函数分解")
        for label, val in self.res.obj_breakdown.items():
            if val is not None:
                print(f"  {label:12s} = {val:.5f}")
        print(f"  {'总计':12s} = {self.res.obj_val:.5f}")

    def _print_satellites(self):
        self._section("选址变量 y[s]")
        for s in self.inst.S:
            opened = s in self.res.satellites_opened
            mark = "✓ 建设" if opened else "✗ 不建"
            demand = self.res.satellite_demand.get(s, 0.0)
            print(f"  y[{s}] = {'1' if opened else '0'}  {mark}  "
                  f"demand={demand:.2f}")

    def _print_customer_demands(self):
        self._section("顾客需求")
        total = 0.0
        for j in self.inst.N:
            demand = self.inst.q.get(j, 0.0)
            total += demand
            print(f"  顾客 {j:>3d}  需求 = {demand:.2f}")
        print(f"  {'─' * 30}")
        print(f"  顾客总数: {len(self.inst.N)}")
        print(f"  需求总和: {total:.2f}")

    def _print_first_level(self):
        # 路径
        self._section("一级卡车路径")
        if self.res.first_level_routes:
            for idx, route in enumerate(self.res.first_level_routes):
                route_str = " -> ".join(str(n) for n in route)
                print(f"  Route {idx + 1}: {route_str}")
        else:
            print("  (无路径)")

        # 边
        if self.res.first_level_edges:
            print()
            for i, j in self.res.first_level_edges:
                print(f"  xL[{i},{j}] = 1  ✓  {i} -> {j}")

        # 访问顺序
        self._section("一级访问顺序 uL[i]")
        for i in self.inst.V1_minus:
            val = self.res.first_level_loads.get(i)
            if val is not None:
                print(f"  uL[{i}] = {val:.4f}")

    def _print_second_level(self):
        self._section("二级卡车路径")
        for s in self.inst.S:
            print(f"\n  --- 卫星仓库 {s} ---")
            routes = self.res.second_level_truck_routes.get(s, [])
            if routes:
                for idx, route in enumerate(routes):
                    route_str = " -> ".join(str(n) for n in route)
                    print(f"    Route {idx + 1}: {route_str}")
            else:
                print("    (无路径)")

            edges = self.res.second_level_truck_edges.get(s, [])
            if edges:
                for i, j in edges:
                    print(f"    xT[{i},{j},{s}] = 1  ✓  {i} -> {j}")

        # MTZ
        self._section("二级网络访问顺序 uT[i,s]")
        for s in self.inst.S:
            print(f"\n  --- 卫星仓库 {s} ---")
            mtz = self.res.second_level_mtz.get(s, {})
            for i, val in sorted(mtz.items()):
                if val > 0.5:
                    print(f"    uT[{i},{s}] = {val:.2f}")

    def _print_drones(self):
        self._section("无人机配送")
        if not self.res.drone_trips:
            print("  (无无人机配送)")
            return

        for trip in self.res.drone_trips:
            s = trip["satellite"]
            i = trip["launch_node"]
            j = trip["customer"]
            k = trip["land_node"]
            c = trip["cost"]
            e = trip["energy"]
            print(
                f"  [{s}] {i} → 顾客{j} → {k}  "
                f"cost={c:.2f}  energy={e:.2f}"
            )

    def _print_time_variables(self):
        self._section("时间变量")
        if not self.res.time_variables["tauT"]:
            print("  (无时间变量)")
            return

        for s in self.inst.S:
            print(f"\n  --- 卫星仓库 {s} ---")
            tauT = self.res.time_variables["tauT"].get(s, {})
            tauD = self.res.time_variables["tauD"].get(s, {})
            rho = self.res.time_variables["rho"].get(s, {})

            for i in self.inst.V_s[s]:
                tauT_val = tauT.get(i, 0)
                tauD_val = tauD.get(i, 0)
                rho_val = rho.get(i, 0)
                if tauT_val > 0 or tauD_val > 0 or rho_val > 0:
                    print(f"    Node {i}: tauT={tauT_val:.2f}, tauD={tauD_val:.2f}, rho={rho_val:.2f}")

# ================================================================
# 可视化数据准备
# ================================================================

@dataclass
class VisualizationData:
    """为可视化层准备的结构化数据"""

    instance: Instance
    solution: SolutionResult

    @property
    def node_positions(self) -> dict[int, tuple[float, float]]:
        """所有节点坐标"""
        coords = self.instance.coords.copy()
        # 添加虚拟节点坐标
        coords[self.instance.virtual_depot] = self.instance.coords[self.instance.depot]
        for s in self.instance.S:
            coords[-s] = self.instance.coords[s]
        return coords

    @property
    def node_types(self) -> dict[int, str]:
        """节点类型标记"""
        types = {}
        types[self.instance.depot] = "depot"
        types[self.instance.virtual_depot] = "virtual_depot"
        for s in self.instance.S:
            types[s] = (
                "satellite_open"
                if s in self.solution.satellites_opened
                else "satellite_closed"
            )
            types[-s] = "virtual_satellite"
        for j in self.instance.N:
            types[j] = "customer"
        return types

    @property
    def first_level_edges(self) -> list[tuple[int, int]]:
        return self.solution.first_level_edges

    @property
    def second_level_edges(self) -> dict[int, list[tuple[int, int]]]:
        return self.solution.second_level_truck_edges

    @property
    def drone_trips(self) -> list[dict]:
        return self.solution.drone_trips

    @property
    def route_sequences(self) -> dict:
        return {
            "first_level": self.solution.first_level_routes,
            "second_level": self.solution.second_level_truck_routes,
            "drone": self.solution.drone_trips,
        }
