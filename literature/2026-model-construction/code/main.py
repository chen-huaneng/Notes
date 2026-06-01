from __future__ import annotations
from instance import Instance
from model_builder import ModelBuilder
from result_handler import (
    SolutionExtractor,
    ResultPrinter,
    VisualizationData,
)
from visualization import NetworkVisualizer

def main():
    # ============================================================
    # 1. 加载 / 生成算例
    # ============================================================

    # 默认加载算例
    inst = Instance.default()

    # 也可以从 JSON 加载：
    # inst = Instance.from_json("data/default.json")

    # 或生成随机算例：
    n_customers = 15
    n_satellites = 4
    seed = 42
    inst = Instance.generate_random(n_customers=n_customers, n_satellites=n_satellites, seed=seed)

    # 保存默认算例到 JSON（供后续复用）：
    # inst.to_json("data/default.json")
    inst.to_json(f"data/random_{n_customers}C_{n_satellites}S_seed{seed}.json")
    print(inst)

    # ============================================================
    # 2. 构建模型
    # ============================================================

    builder = ModelBuilder(
        instance=inst,
        big_m=1e6,
        gurobi_params={
            "TimeLimit": 3600,
            "MIPGap": 0.0001,
        },
    )

    model, variables = builder.build()

    # ============================================================
    # 3. 求解
    # ============================================================

    model.optimize()

    # ============================================================
    # 4. 提取结果
    # ============================================================

    result = SolutionExtractor(
        model=model,
        variables=variables,
        instance=inst,
    ).extract()

    # ============================================================
    # 5. 打印结果
    # ============================================================

    ResultPrinter(
        result=result,
        instance=inst,
    ).print_all()

    # ============================================================
    # 6. 可视化（可选）
    # ============================================================

    vis_data = VisualizationData(
        instance=inst,
        solution=result,
    )

    NetworkVisualizer.plot_solution(
        vis_data=vis_data,
        save_path=f"results/{inst.name}.png",
        show=True,
    )

if __name__ == "__main__":
    main()
