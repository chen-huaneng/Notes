from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Union
import math
import json
import random

@dataclass
class Instance:
    """
    Two-Echelon Truck-Drone Instance

    所有原始参数存储为字段，派生属性（集合、距离、成本）
    通过 @property 动态计算，不持久化。
    """

    # ---- 基本信息 ----
    name: str

    # ---- 节点 ----
    depot: int
    N: list[int]           # 顾客
    S: list[int]           # 候选卫星仓库

    # ---- 坐标 ----
    coords: dict[int, tuple[float, float]]

    # ---- 需求 & 建设成本 ----
    q: dict[int, float]    # 顾客需求
    f: dict[int, float]    # 卫星仓库建设成本

    # ---- 速度 ----
    vT: float              # 卡车速度
    vD: float              # 无人机速度

    # ---- 容量 ----
    QT: float              # 二级卡车容量
    QD: float              # 无人机容量

    # ---- 无人机参数 ----
    E: float               # 无人机最大电量/续航
    w: float               # 无人机自重

    # ---- 成本系数 ----
    cL_factor: float       # 一级运输成本系数
    cT_factor: float       # 二级卡车运输成本系数
    cD_factor: float       # 无人机运输成本系数
    e_factor: float        # 能耗系数

    # ================================================================
    # 派生集合
    # ================================================================

    @property
    def V1_minus(self) -> list[int]:
        """一级网络起点集合: depot + satellites"""
        return [self.depot] + self.S

    @property
    def V1_plus(self) -> list[int]:
        """一级网络终点集合: satellites + virtual_depot"""
        return self.S + [self.virtual_depot]

    @property
    def virtual_depot(self) -> int:
        """虚拟终点仓库"""
        return -2026

    @property
    def S_prime(self) -> list[int]:
        """虚拟卫星仓库列表"""
        return [-s for s in self.S]

    @property
    def V(self) -> list[int]:
        """全部节点"""
        return [self.depot, self.virtual_depot] + self.S + self.S_prime + self.N

    @property
    def V_s(self) -> dict[int, list[int]]:
        """每个卫星相关的节点集合"""
        result = {}
        for s in self.S:
            sp = -s
            result[s] = [s] + self.N + [sp]
        return result

    @property
    def V_s_minus(self) -> dict[int, list[int]]:
        """每个卫星相关的起点集合"""
        result = {}
        for s in self.S:
            result[s] = [s] + self.N
        return result

    @property
    def V_s_plus(self) -> dict[int, list[int]]:
        """每个卫星相关的终点集合"""
        result = {}
        for s in self.S:
            result[s] = self.N + [-s]
        return result

    @property
    def sat_prime(self) -> dict[int, int]:
        """卫星与其虚拟卫星的映射"""
        return {s: -s for s in self.S}

    # ================================================================
    # 距离矩阵
    # ================================================================

    @property
    def extended_coords(self) -> dict[int, tuple[float, float]]:
        """扩展坐标字典，包括虚拟节点"""
        coords = self.coords.copy()
        coords[self.virtual_depot] = self.coords[self.depot]
        for s in self.S:
            coords[-s] = self.coords[s]
        return coords

    @property
    def d(self) -> dict[tuple[int, int], float]:
        """欧氏距离矩阵"""
        dist = {}
        coords = self.extended_coords
        for i in self.V:
            for j in self.V:
                if i != j:
                    dist[i, j] = math.dist(
                        coords[i],
                        coords[j],
                    )
        return dist

    # ================================================================
    # 成本矩阵
    # ================================================================

    @property
    def cL(self) -> dict[tuple[int, int], float]:
        """一级运输成本"""
        return {k: self.cL_factor * v for k, v in self.d.items()}

    @property
    def cT(self) -> dict[tuple[int, int], float]:
        """二级卡车运输成本"""
        return {k: self.cT_factor * v for k, v in self.d.items()}

    @property
    def cD(self) -> dict[tuple[int, int], float]:
        """无人机运输成本"""
        return {k: self.cD_factor * v for k, v in self.d.items()}

    @property
    def e(self) -> dict[tuple[int, int], float]:
        """无人机能耗"""
        return {k: self.e_factor * v for k, v in self.d.items()}

    # ================================================================
    # 默认算例
    # ================================================================

    @classmethod
    def default(cls) -> Instance:
        """返回与原始代码完全一致的默认算例"""
        return cls(
            name="default_4C_2S",
            depot=0,
            N=[1, 2, 3, 4],
            S=[5, 6],
            coords={
                0: (0, 0),
                1: (-10, -10),
                2: (5, 5),
                3: (15, 0),
                4: (15, 5),
                5: (5, 0),
                6: (0, -10),
            },
            q={1: 2, 2: 3, 3: 2, 4: 4},
            f={5: 100, 6: 120},
            vT=40,
            vD=60,
            QT=15,
            QD=5,
            E=20,
            w=1,
            cL_factor=2.0,
            cT_factor=1.5,
            cD_factor=0.5,
            e_factor=1.0,
        )

    # ================================================================
    # 随机算例
    # ================================================================

    @classmethod
    def generate_random(
        cls,
        n_customers: int = 10,
        n_satellites: int = 3,
        grid_size: float = 50.0,
        seed: int | None = None,
    ) -> Instance:
        """
        随机生成算例

        Parameters
        ----------
        n_customers : 顾客数量
        n_satellites : 候选卫星仓库数量
        grid_size : 坐标范围 [0, grid_size]
        seed : 随机种子
        """
        rng = random.Random(seed)

        depot = 0
        S = list(range(1, n_satellites + 1))
        N = list(range(n_satellites + 1, n_satellites + 1 + n_customers))
        V = [depot] + S + N

        # 随机坐标，depot 固定在 (0, 0)
        coords = {depot: (0.0, 0.0)}
        used = {(0.0, 0.0)}
        for v in V:
            if v == depot:
                continue
            while True:
                x = round(rng.uniform(0, grid_size), 2)
                y = round(rng.uniform(0, grid_size), 2)
                if (x, y) not in used:
                    coords[v] = (x, y)
                    used.add((x, y))
                    break

        # 随机需求 [1, 5]
        q = {j: round(rng.uniform(1, 5), 1) for j in N}

        # 随机建设成本 [50, 200]
        f = {s: round(rng.uniform(50, 200), 1) for s in S}

        return cls(
            name=f"random_{n_customers}C_{n_satellites}S_seed{seed}",
            depot=depot,
            N=N,
            S=S,
            coords=coords,
            q=q,
            f=f,
            vT=40,
            vD=60,
            QT=15,
            QD=5,
            E=20,
            w=1,
            cL_factor=2.0,
            cT_factor=1.5,
            cD_factor=0.5,
            e_factor=1.0,
        )

    # ================================================================
    # JSON 接口
    # ================================================================

    @classmethod
    def from_json(cls, filepath: Union[str, Path]) -> Instance:
        """从 JSON 文件加载算例"""
        filepath = Path(filepath)
        with open(filepath, "r", encoding="utf-8") as fp:
            data = json.load(fp)

        # JSON 的 key 是字符串，需要转换
        coords = {int(k): tuple(v) for k, v in data["coords"].items()}
        q = {int(k): float(v) for k, v in data["q"].items()}
        f = {int(k): float(v) for k, v in data["f"].items()}

        return cls(
            name=data["name"],
            depot=data["depot"],
            N=data["N"],
            S=data["S"],
            coords=coords,
            q=q,
            f=f,
            vT=data["vT"],
            vD=data["vD"],
            QT=data["QT"],
            QD=data["QD"],
            E=data["E"],
            w=data["w"],
            cL_factor=data["cL_factor"],
            cT_factor=data["cT_factor"],
            cD_factor=data["cD_factor"],
            e_factor=data["e_factor"],
        )

    def to_json(self, filepath: Union[str, Path]) -> None:
        """将算例保存为 JSON 文件"""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)

        # 转换 dict key 为字符串（JSON 要求）
        data = {
            "name": self.name,
            "depot": self.depot,
            "N": self.N,
            "S": self.S,
            "coords": {str(k): list(v) for k, v in self.coords.items()},
            "q": {str(k): v for k, v in self.q.items()},
            "f": {str(k): v for k, v in self.f.items()},
            "vT": self.vT,
            "vD": self.vD,
            "QT": self.QT,
            "QD": self.QD,
            "E": self.E,
            "w": self.w,
            "cL_factor": self.cL_factor,
            "cT_factor": self.cT_factor,
            "cD_factor": self.cD_factor,
            "e_factor": self.e_factor,
        }

        with open(filepath, "w", encoding="utf-8") as fp:
            json.dump(data, fp, indent=2, ensure_ascii=False)

    # ================================================================
    # Pretty Print
    # ================================================================

    def __str__(self) -> str:
        return (
            f"Instance: {self.name}\n"
            f"Customers:   {len(self.N)}\n"
            f"Satellites:  {len(self.S)}\n"
            f"Nodes:       {len(self.V)}\n"
            f"QT={self.QT}, QD={self.QD}\n"
            f"vT={self.vT}, vD={self.vD}, E={self.E}"
        )
