# HPASim

[English README](README.md)

HPASim 是一个记忆泊车仿真项目脚手架。当前工作区聚焦于一张完整的
OpenDRIVE 地下停车场风格地图、车辆运动学模型、路线规划、matplotlib 静态渲染，
以及可交互的浏览器可视化工具。

## 当前地图

当前场景是 `parking_lot_full`。

- 目标车位：`central_upper_angled_row_13`
- OpenDRIVE 输出：`maps/opendrive/parking_lot_full.xodr`
- 预览图输出：`outputs/parking_lot_full.png`

地图包含入口和出口通道、三条纵向行车通道、三条横向连接通道、密集垂直车位、
斜列车位、边界侧平行车位、已占用车辆、充电车位、无障碍车位、保留车位、
人行通道、入口和出口坡道、道闸、减速带和静态障碍物。

## 生成地图

```powershell
uv run python scripts/generate_maps.py
```

## 渲染地图

```powershell
uv run python hpasim/plot_opendrive.py
```

渲染器只绘制几何图形：不显示对象文字标签，不显示标题，也不显示坐标轴文字。

## 交互式可视化

启动本地浏览器可视化工具：

```powershell
uv run python scripts/serve_viewer.py
```

然后打开 `http://127.0.0.1:8000`。该工具使用 Canvas 做高性能交互：
滚轮缩放、拖拽平移，点击任意车位后会从自车起点向该车位请求路线规划并显示路径。

## 车辆模型

车辆模型采用前轮转向运动学自行车模型，状态为 `x, y, yaw, v`，控制量为
`steer, acceleration`。

## 路线规划

路线规划器会读取生成的 OpenDRIVE 地图，根据行车通道和静态障碍物构建栅格占用地图，
并使用 A* 从自车起点规划到选中的车位附近。

```powershell
uv run python scripts/plan_route.py
```

运行测试：

```powershell
uv run python -m unittest discover -s tests
```

## 关键文件

- `hpasim/parking_scenarios.py`：场景定义和 OpenDRIVE 生成逻辑。
- `hpasim/plot_opendrive.py`：matplotlib 渲染器。
- `hpasim/vehicle.py`：前轮转向车辆运动学模型。
- `hpasim/planner.py`：基于 OpenDRIVE 地图的栅格路线规划器。
- `hpasim/map_payload.py`：供交互式可视化使用的 JSON 地图数据。
- `scripts/serve_viewer.py`：Canvas 可视化工具的本地 HTTP 服务。
- `viewer/index.html`：交互式 Canvas 可视化页面。
