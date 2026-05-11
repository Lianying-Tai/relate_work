# 已定决策

- DEC-001：先实现纯标准库 Python 版本
  时间：2026-05-11 11:40:00 +0800
  原因：当前项目需要跨设备可用，纯标准库实现便于迁移；后续可按真实数据规模再引入 NumPy、GeoPandas 或 Rasterio。

- DEC-002：以二维栅格 `grid[y][x]` 作为统一数据结构
  时间：2026-05-11 11:40:00 +0800
  原因：论文方法基于栅格地图，二维数组便于接入 A*、Dijkstra、RRT* 等规划算法。

- DEC-003：保留 agent-handoff 仓库内 runtime 副本
  时间：2026-05-11 11:40:00 +0800
  原因：方便另一台电脑克隆仓库后直接使用 `.agent-handoff/runtime/agent-handoff/scripts/handoff.py` 恢复上下文。

