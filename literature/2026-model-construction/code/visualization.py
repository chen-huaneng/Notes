from __future__ import annotations
from typing import Optional
from pathlib import Path
from result_handler import VisualizationData

class NetworkVisualizer:
    """
    Two-Echelon Truck-Drone 网络可视化

    节点样式:
        - Depot          → 浅蓝方块  (blue square)
        - Open Satellite  → 浅绿三角  (green triangle)
        - Closed Satellite→ 灰三角    (gray triangle)
        - Customer        → 橙圆      (orange circle)

    路径样式:
        - 一级线路        → 蓝色虚线
        - 二级卡车        → 黑色实线
        - 无人机          → 红色点划线
    """

    # 节点样式配置
    NODE_STYLES = {
        "depot": {
            "marker": "s",
            "color": "#4A90D9",
            "size": 200,
            "label": "Depot",
            "zorder": 5,
        },
        "satellite_open": {
            "marker": "^",
            "color": "#5CB85C",
            "size": 150,
            "label": "Satellite (Open)",
            "zorder": 4,
        },
        "satellite_closed": {
            "marker": "^",
            "color": "#AAAAAA",
            "size": 100,
            "label": "Satellite (Closed)",
            "zorder": 3,
        },
        "customer": {
            "marker": "o",
            "color": "#F0AD4E",
            "size": 80,
            "label": "Customer",
            "zorder": 3,
        },
    }

    # 路径样式配置
    EDGE_STYLES = {
        "first_level": {
            "color": "#4A90D9",
            "linestyle": "--",
            "linewidth": 2.0,
            "label": "1st Level (Truck)",
            "alpha": 0.8,
        },
        "second_level": {
            "color": "#000000",
            "linestyle": "-",
            "linewidth": 1.5,
            "label": "2nd Level (Truck)",
            "alpha": 0.7,
        },
        "drone": {
            "color": "#D9534F",
            "linestyle": "-.",
            "linewidth": 1.5,
            "label": "Drone",
            "alpha": 0.9,
        },
    }

    @classmethod
    def plot_solution(
        cls,
        vis_data: VisualizationData,
        save_path: Optional[str | Path] = None,
        show: bool = True,
        figsize: tuple[float, float] = (12, 9),
        title: Optional[str] = None,
    ):
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches
            import matplotlib.lines as mlines
        except ImportError:
            print("[Visualization] matplotlib 未安装，跳过绘图。")
            return

        fig, ax = plt.subplots(1, 1, figsize=figsize)

        inst = vis_data.instance
        node_types = vis_data.node_types
        positions = vis_data.node_positions

        # ---- 绘制路径（annotate 不参与图例） ----

        # 一级路径：如果路径连接到虚拟仓库，则改为连接到真实仓库
        style = cls.EDGE_STYLES["first_level"]
        for i, j in vis_data.first_level_edges:
            # 将虚拟仓库替换为真实仓库
            real_j = j if j != inst.virtual_depot else inst.depot
            ax.annotate(
                "",
                xy=positions[real_j],
                xytext=positions[i],
                arrowprops=dict(
                    arrowstyle="-|>",
                    color=style["color"],
                    linestyle=style["linestyle"],
                    lw=style["linewidth"],
                    alpha=style["alpha"],
                ),
            )

        # 二级卡车路径：如果路径涉及虚拟卫星，则移除虚拟卫星，连接真实节点
        for s, edges in vis_data.second_level_edges.items():
            style = cls.EDGE_STYLES["second_level"]
            for i, j in edges:
                # 将虚拟卫星替换为真实卫星
                real_i = i if i != -s else s
                real_j = j if j != -s else s
                # 只绘制非虚拟节点之间的路径
                if real_i != s or real_j != s:  # 不绘制卫星到自己的路径
                    ax.annotate(
                        "",
                        xy=positions[real_j],
                        xytext=positions[real_i],
                        arrowprops=dict(
                            arrowstyle="-|>",
                            color=style["color"],
                            linestyle=style["linestyle"],
                            lw=style["linewidth"],
                            alpha=style["alpha"],
                        ),
                    )

        # 无人机路径：如果路径涉及虚拟卫星，则改为连接真实卫星
        style = cls.EDGE_STYLES["drone"]
        for trip in vis_data.drone_trips:
            i = trip["launch_node"]
            j = trip["customer"]
            k = trip["land_node"]

            # 替换虚拟节点为真实节点
            real_i = i if i != -trip["satellite"] else trip["satellite"]
            real_k = k if k != -trip["satellite"] else trip["satellite"]

            for src, dst in [(real_i, j), (j, real_k)]:
                ax.annotate(
                    "",
                    xy=positions[dst],
                    xytext=positions[src],
                    arrowprops=dict(
                        arrowstyle="-|>",
                        color=style["color"],
                        linestyle=style["linestyle"],
                        lw=style["linewidth"],
                        alpha=style["alpha"],
                    ),
                )

        # ---- 绘制节点（排除虚拟节点） ----
        for ntype, style in cls.NODE_STYLES.items():
            # 找出对应类型的非虚拟节点
            nodes = []
            for n, t in node_types.items():
                if t == ntype:
                    # 排除虚拟仓库和虚拟卫星
                    if n != inst.virtual_depot and not (isinstance(n, int) and -n in inst.S):
                        nodes.append(n)

            if not nodes:
                continue

            xs = [positions[n][0] for n in nodes]
            ys = [positions[n][1] for n in nodes]

            ax.scatter(
                xs, ys,
                c=style["color"],
                marker=style["marker"],
                s=style["size"],
                zorder=style["zorder"],
                edgecolors="black",
                linewidths=0.8,
            )

            for n in nodes:
                ax.annotate(
                    str(n),
                    positions[n],
                    textcoords="offset points",
                    xytext=(5, 5),
                    fontsize=8,
                    fontweight="bold",
                    zorder=6,
                )

        # ---- 构建图例句柄（全部用代理艺术家） ----
        legend_handles = []

        # 节点图例
        for ntype, style in cls.NODE_STYLES.items():
            # 检查此类型是否有节点
            nodes = []
            for n, t in node_types.items():
                if t == ntype:
                    # 排除虚拟仓库和虚拟卫星
                    if n != inst.virtual_depot and not (isinstance(n, int) and -n in inst.S):
                        nodes.append(n)

            if not nodes:
                continue

            legend_handles.append(
                mpatches.Patch(
                    facecolor=style["color"],
                    edgecolor="black",
                    label=style["label"],
                )
            )

        # 边图例
        for key in ["first_level", "second_level", "drone"]:
            style = cls.EDGE_STYLES[key]
            legend_handles.append(
                mlines.Line2D(
                    [], [],
                    color=style["color"],
                    linestyle=style["linestyle"],
                    linewidth=style["linewidth"],
                    label=style["label"],
                )
            )

        ax.legend(handles=legend_handles, loc="upper right", fontsize=9)

        # ---- 标题 ----
        if title:
            ax.set_title(title, fontsize=14, fontweight="bold")
        else:
            obj_str = (
                f"obj={vis_data.solution.obj_val:.4f}"
                if vis_data.solution.obj_val is not None
                else "No Solution"
            )
            ax.set_title(
                f"2E-Truck-Drone Solution ({inst.name}) — {obj_str}",
                fontsize=14,
                fontweight="bold",
            )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.set_aspect("equal")
        fig.tight_layout()

        if save_path is None:
            save_path = Path("results") / f"{inst.name}.png"

        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[Visualization] 图片已保存至: {save_path}")

        if show:
            plt.show()

        plt.close(fig)
