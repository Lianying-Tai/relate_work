# 城市低空无人机对地风险栅格评估

这个工作区实现了论文《城市低空场景下无人机运行对地风险量化评估》的核心算法思想，用于把城市区域离散成栅格，并判断每个栅格是否适合作为无人机安全空域。

## 算法流程

1. 设定无人机参数、飞行高度、风速风向不确定性。
2. 通过蒙特卡洛弹道下降仿真，生成无人机失效坠地的地面撞击概率核。
3. 将撞击概率核与人口密度层卷积，得到“至少撞击一人”的概率。
4. 结合撞击动能和遮蔽因子，估计致死概率。
5. 计算每个栅格的人员伤亡风险值：

```text
R(x, y) = P_event * P_collision(x, y) * P_fatality(x, y)
```

6. 融合禁飞区、障碍物高度和特殊区域层，生成最终风险等级图。
7. 根据风险等级或风险阈值输出安全空域掩膜、路径规划代价地图。

## 图层输入

所有图层都用二维数组表达，格式为 `grid[y][x]`。

- `population_density`：人口密度，单位 `人/m^2`。
- `shelter_factor`：遮蔽因子，范围 `[0, 1]`。0 表示无遮蔽，1 表示完全遮蔽。
- `no_fly`：禁飞区布尔层，`True` 表示禁止飞行。
- `obstacle_height_m`：障碍物高度层，单位 `m`。
- `special_area_level`：特殊区域风险等级层，可用 `RiskLevel.SAFE/LOW/MEDIUM/HIGH`。

## 最小使用示例

```python
from uav_ground_risk import (
    UAVParams,
    HumanParams,
    RiskLevel,
    build_impact_kernel,
    assess_ground_risk,
    safe_airspace_mask,
    path_planning_cost,
    write_result_visualization_html,
)

height = width = 20
population = [[0.002 for _ in range(width)] for _ in range(height)]  # 2000 人/km^2
shelter = [[0.27 for _ in range(width)] for _ in range(height)]
no_fly = [[False for _ in range(width)] for _ in range(height)]
obstacles = [[0.0 for _ in range(width)] for _ in range(height)]
special = [[RiskLevel.SAFE for _ in range(width)] for _ in range(height)]

no_fly[5][6] = True
obstacles[10][10] = 120.0
special[12][8] = RiskLevel.HIGH

uav = UAVParams(
    mass_kg=1.4,
    max_radius_m=0.3,
    cruise_speed_mps=16.0,
    failure_rate_per_hour=1e-5,
    drag_coeff=0.6,
    frontal_area_m2=0.08,
)
human = HumanParams()

kernel, impact_speed, descent_angle = build_impact_kernel(
    uav=uav,
    flight_height_m=100.0,
    cell_size_m=100.0,
    samples=1000,
    wind_speed_mean_mps=3.0,
    wind_speed_std_mps=1.0,
)

result = assess_ground_risk(
    population_density=population,
    shelter_factor=shelter,
    impact_kernel=kernel,
    uav=uav,
    human=human,
    impact_speed_mps=impact_speed,
    descent_angle_rad=descent_angle,
    no_fly=no_fly,
    obstacle_height_m=obstacles,
    flight_height_m=100.0,
    special_area_level=special,
)

safe_mask = safe_airspace_mask(result, max_allowed_level=RiskLevel.LOW)
cost_map = path_planning_cost(result)
write_result_visualization_html(result, "risk_visualization.html", title="城市低空无人机对地风险图")
```

运行后会生成 `risk_visualization.html`，其中包含风险值热力图、风险等级图、安全空域掩膜和路径规划代价图。

## 接入 CSV 栅格

如果已经把 GIS、人口、建筑物或管控区域数据栅格化为同尺寸 CSV，可以用内置读取函数接入：

```python
from uav_ground_risk import (
    read_bool_grid_csv,
    read_level_grid_csv,
    read_number_grid_csv,
)

population = read_number_grid_csv("population_density.csv")
shelter = read_number_grid_csv("shelter_factor.csv")
no_fly = read_bool_grid_csv("no_fly.csv")
obstacles = read_number_grid_csv("obstacle_height_m.csv")
special = read_level_grid_csv("special_area_level.csv")
```

CSV 中空行会被忽略；布尔层支持 `1/0`、`true/false`、`yes/no`；特殊区域等级使用 `RiskLevel` 的整数值 `0` 到 `4`。

## 与路径规划的衔接

- A* / Dijkstra：把 `cost_map[y][x]` 作为栅格通行代价，`inf` 代表不可通行。
- RRT / RRT*：采样点落入 `safe_mask == False` 的栅格时拒绝，边检查时沿线采样多个栅格。
- 三维航路：对不同飞行高度重复生成 `result`，堆叠为 `risk[z][y][x]`，再做三维路径搜索。

## 可视化输出

`write_result_visualization_html()` 使用纯标准库生成可直接打开的 HTML 插图，不依赖 Matplotlib：

```python
write_result_visualization_html(
    result,
    "risk_visualization.html",
    title="城市低空无人机对地风险图",
    cell_size=28,
)
```

输出文件包含四个面板：

- 人员伤亡风险值 `R(x, y)`：按对数尺度渲染热力图。
- 融合后的风险等级：按 `SAFE/LOW/MEDIUM/HIGH/FORBIDDEN` 分类着色。
- 安全空域掩膜：绿色表示可飞，红色表示不可飞。
- 路径规划代价图：`inf` 代表不可通行。

也可以运行示例脚本生成演示图：

```bash
python examples/generate_visualization_example.py
```

## 验证

项目只依赖 Python 标准库，克隆后可以直接运行：

```bash
python -m py_compile uav_ground_risk.py tests/test_uav_ground_risk.py examples/generate_visualization_example.py
python -m unittest discover -s tests
```

## 跨设备交接

项目包含 `.agent-handoff/` 交接状态。换电脑克隆仓库后，先阅读：

```text
.agent-handoff/START-HERE.md
```

如果目标环境安装了 Python 3.8+ 和 PyYAML，可以运行：

```bash
python .agent-handoff/runtime/agent-handoff/scripts/handoff.py resume .
```

也可以让 Codex 使用 `agent-handoff` skill 读取当前仓库的交接状态。

## 风险等级

默认阈值按论文表 2 的量级设置：

- `R < 1e-8`：安全或无风险。
- `1e-8 <= R < 1e-7`：低风险。
- `1e-7 <= R < 1e-6`：中风险。
- `R >= 1e-6`：高风险。
- 禁飞区或障碍物高于飞行高度：`FORBIDDEN`。
