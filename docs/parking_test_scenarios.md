# 记忆泊车测试地图

HPASim 当前使用一张完整的 OpenDRIVE 停车场地图：

| 场景 | 目标车位 | 用途 |
| --- | --- | --- |
| `parking_lot_full` | `central_upper_angled_row_13` | 用于完整停车场中的端到端记忆泊车仿真，覆盖入口、出口、行车通道、混合车位、已占车辆、减速带、静态障碍物和目标泊车位。 |

## 地图内容

- 三条纵向行车通道：入口通道、记忆泊车通道和出口通道。
- 三条横向连接通道，用于连通纵向通道。
- 混合车位布局，包括密集垂直车位、斜列车位和边界侧平行车位。
- 已占用车辆、无障碍车位、充电车位、保留车位和一个目标记忆泊车车位。
- 边界墙、入口坡道、出口坡道、道闸、人行通道、减速带、配送车辆障碍和施工区域障碍。

## 建模说明

- 地图从零生成，是一个完整停车场场景，不是多个独立泊车动作场景的拼接。
- 道路使用直线 OpenDRIVE 参考线，适合低速泊车仿真和路径规划验证。
- 车位和静态设施使用 OpenDRIVE `object` 元素表达，并带有矩形 `outline` 多边形。
- matplotlib 渲染器只绘制几何图形，不在地图上绘制文字标签。
- 交互式可视化工具使用 Canvas 绘制地图，点击车位后调用路线规划接口并显示路径。

## 常用命令

生成地图：

```powershell
uv run python scripts/generate_maps.py
```

渲染静态地图：

```powershell
uv run python hpasim/plot_opendrive.py
```

启动交互式可视化：

```powershell
uv run python scripts/serve_viewer.py
```

运行测试：

```powershell
uv run python -m unittest discover -s tests
```
