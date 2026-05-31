# HPASim

HPASim 是一个记忆泊车仿真项目脚手架。当前版本围绕地下停车场场景展开，包含
OpenDRIVE 地图生成、静态地图渲染、车辆运动学模型、路线规划，以及基于浏览器
Canvas 的交互式可视化。

## 当前场景

当前地图场景是 `parking_lot_full`。

- 目标车位：`central_upper_angled_row_13`
- OpenDRIVE 地图：`maps/opendrive/parking_lot_full.xodr`
- 静态预览图：`outputs/parking_lot_full.png`

地图包含入口和出口通道、三条纵向行车通道、三条横向连接通道、密集垂直车位、
斜列车位、边界侧平行车位、已占用车辆、充电车位、无障碍车位、保留车位、
人行通道、入口和出口坡道、道闸、减速带和静态障碍物。地图渲染时不显示文字标签、
标题或坐标轴文字。

## 生成地图

```powershell
uv run python scripts/generate_maps.py
```

## 静态渲染

```powershell
uv run python hpasim/plot_opendrive.py
```

如果直接用系统 Python 运行脚本而缺少 `matplotlib`，请使用上面的 `uv run` 命令，
它会自动使用项目虚拟环境。

## 交互式可视化

启动本地可视化服务：

```powershell
uv run python scripts/serve_viewer.py
```

然后在浏览器打开 `http://127.0.0.1:8000`。页面使用 Canvas 绘制地图，支持滚轮缩放、
拖拽平移。点击任意车位后，前端会请求后端路线规划接口，并把从自车起点到该车位的
路径画在地图上。

## 车辆模型

车辆模型采用前轮转向运动学自行车模型：

- 状态量：`x, y, yaw, v`
- 控制量：`steer, acceleration`
- 约束：最大前轮转角、最大加速度、最大减速度、速度上下限

相关代码位于 `hpasim/vehicle.py`。

## 路线规划

路线规划器读取生成的 OpenDRIVE 地图，根据行车通道和静态障碍物构建栅格占用地图，
然后使用 A* 从自车起点规划到选中的车位附近。当前代价函数会偏好右侧车道中心，
减少贴边行驶；当目标是车位时，规划结果会拆成前进段和末端倒车入库段，使车头更偏向朝外。

```powershell
uv run python scripts/plan_route.py
```

相关代码位于 `hpasim/planner.py` 和 `hpasim/map_payload.py`。

## 测试

```powershell
uv run python -m unittest discover -s tests
```

## 关键文件

- `hpasim/parking_scenarios.py`：停车场场景定义和 OpenDRIVE 生成逻辑。
- `hpasim/plot_opendrive.py`：matplotlib 静态渲染器。
- `hpasim/vehicle.py`：前轮转向车辆运动学模型。
- `hpasim/planner.py`：基于 OpenDRIVE 地图的栅格路线规划器。
- `hpasim/map_payload.py`：供交互式可视化使用的 JSON 地图数据。
- `scripts/serve_viewer.py`：Canvas 可视化工具的本地 HTTP 服务。
- `viewer/index.html`：交互式 Canvas 可视化页面。
- `docs/parking_test_scenarios.md`：停车场测试场景说明。
