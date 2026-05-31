from __future__ import annotations
from dataclasses import dataclass
import math

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
    QL: float              # 一级卡车容量
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
    def V1(self) -> list[int]:
        """一级网络节点: depot + satellites"""
        return [self.depot] + self.S

    @property
    def V2(self) -> list[int]:
        """二级网络节点: satellites + customers"""
        return self.S + self.N

    @property
    def V(self) -> list[int]:
        """全部节点"""
        return [self.depot] + self.S + self.N

    # ================================================================
    # 距离矩阵
    # ================================================================

    @property
    def d(self) -> dict[tuple[int, int], float]:
        """欧氏距离矩阵"""
        dist = {}
        for i in self.V:
            for j in self.V:
                if i != j:
                    dist[i, j] = math.dist(
                        self.coords[i],
                        self.coords[j],
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
            QL=50,
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
    # Pretty Print
    # ================================================================

    def __str__(self) -> str:
        return (
            f"Instance: {self.name}\n"
            f"Customers:   {len(self.N)}\n"
            f"Satellites:  {len(self.S)}\n"
            f"Nodes:       {len(self.V)}\n"
            f"QL={self.QL}, QT={self.QT}, QD={self.QD}\n"
            f"vT={self.vT}, vD={self.vD}, E={self.E}"
        )
