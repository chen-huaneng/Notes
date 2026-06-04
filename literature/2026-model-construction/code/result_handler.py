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
        first_level_routes = self._build_routes_from_edges(
            first_level_edges,
            self.inst.depot,
        )
        first_level_loads = self._extract_first_level_loads()

        # ---- 二级网络 ----
        second_level_truck_edges = self._extract_second_level_edges()
        second_level_truck_routes = {}
        for s, edges in second_level_truck_edges.items():
            second_level_truck_routes[s] = self._build_routes_from_edges(
                edges, s
            )
        second_level_mtz = self._extract_second_level_mtz()

        # ---- 无人机 ----
        drone_trips = self._extract_drone_trips()

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
            val = self._val(self.vars.psi[s])
            if val is not None:
                demand[s] = val
        return demand

    # ---- 一级网络 ----

    def _extract_first_level_edges(self) -> list[tuple[int, int]]:
        edges = []
        for i in self.inst.V1:
            for j in self.inst.V1:
                if i != j:
                    val = self._val(self.vars.xL[i, j])
                    if val is not None and val > 0.5:
                        edges.append((i, j))
        return edges

    def _extract_first_level_loads(self) -> dict[int, float]:
        loads = {}
        for i in self.inst.V1:
            val = self._val(self.vars.uL[i])
            if val is not None:
                loads[i] = val
        return loads

    # ---- 二级网络 ----

    def _extract_second_level_edges(self) -> dict[int, list[tuple[int, int]]]:
        result = {}
        for s in self.inst.S:
            edges = []
            for i in self.inst.V2:
                for j in self.inst.V2:
                    if i != j:
                        val = self._val(self.vars.xT[i, j, s])
                        if val is not None and val > 0.5:
                            edges.append((i, j))
            result[s] = edges
        return result

    def _extract_second_level_mtz(self) -> dict[int, dict[int, float]]:
        result = {}
        for s in self.inst.S:
            result[s] = {}
            for i in self.inst.V2:
                val = self._val(self.vars.uT[i, s])
                if val is not None:
                    result[s][i] = val
        return result

    # ---- 无人机 ----

    def _extract_drone_trips(self) -> list[dict]:
        trips = []
        inst = self.inst

        for s in inst.S:
            for i in inst.V2:
                for j in inst.N:
                    for k in inst.V2:
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

    # ================================================================
    # 路径重建
    # ================================================================

    @staticmethod
    def _build_routes_from_edges(
        edges: list[tuple[int, int]],
        start_node: int,
    ) -> list[list[int]]:
        """
        从边列表重建完整路径

        例如 edges = [(0,5),(5,6),(6,0)]
        返回 [[0, 5, 6, 0]]

        支持多条路径（多条从 start_node 出发的回路）。
        """
        if not edges:
            return []

        # 构建邻接表
        adj: dict[int, list[int]] = {}
        for i, j in edges:
            adj.setdefault(i, []).append(j)

        routes = []
        visited_edges: set[tuple[int, int]] = set()

        # 从 start_node 出发找回路
        while True:
            # 找到一条未使用的从 start_node 出发的边
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
                print(f"MIPGap:   {self.res.mip_gap:.6f}")
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
        print(f"  需求总和: {total:.1f}")

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

        # 载重
        self._section("一级载重 uL[i]")
        for i in self.inst.V1:
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
        return self.instance.coords

    @property
    def node_types(self) -> dict[int, str]:
        """节点类型标记"""
        types = {}
        types[self.instance.depot] = "depot"
        for s in self.instance.S:
            types[s] = (
                "satellite_open"
                if s in self.solution.satellites_opened
                else "satellite_closed"
            )
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
