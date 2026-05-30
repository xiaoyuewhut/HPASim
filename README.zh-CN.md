# HPASim

[English README](README.md)

HPASim 是一个记忆泊车仿真项目脚手架。当前工作区聚焦于一张完整的
OpenDRIVE 停车场地图，以及用于快速检查地图效果的 matplotlib 渲染器。

## 当前地图

当前场景为 `parking_lot_full`。

- 目标车位：`central_upper_angled_row_10`
- OpenDRIVE 输出：`maps/opendrive/parking_lot_full.xodr`
- 预览图输出：`outputs/parking_lot_full.png`

地图包含入口和出口通道、三条纵向行车通道、三条横向连接通道、密集垂直
车位、斜列车位、边界侧平行车位、已占车辆、充电车位、无障碍车位、保留
车位、绿化岛、人行横道、减速带和静态障碍物。

## 生成地图

```powershell
uv run python scripts/generate_maps.py
```

## 渲染地图

```powershell
uv run python hpasim/plot_opendrive.py
```

渲染器只绘制几何图形：不显示对象文字标签，不显示标题，也不显示坐标轴文字。

## 关键文件

- `hpasim/parking_scenarios.py`：场景定义和 OpenDRIVE 生成逻辑。
- `hpasim/plot_opendrive.py`：matplotlib 渲染器。
- `docs/parking_test_scenarios.md`：地图细节说明。
