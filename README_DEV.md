# Axis Rotation Viewer 开发与可视化参数调节指南

本工具用于可视化刚体变换（旋转和平移），帮助开发者直观理解 4x4 齐次变换矩阵。

## 1. 可视化参数调节 (开发者指南)

如果你需要修改界面的视觉效果，可以调整 `axis_rotation_viewer.py` 中的以下参数：

### 3D 场景参数 (在 `_setup_scene` 和相关方法中)

- **坐标轴长度**: 修改 `self.axis_length = 0.12` 可调整 X/Y/Z 轴箭头的总长度。
- **坐标轴粗细**: 在 `_create_arrow` 方法中调整：
  - `source.SetShaftRadius(0.004)`: 箭杆半径。
  - `source.SetTipRadius(0.012)`: 箭头底座半径。
- **原点球体大小**: 在 `_create_mesh_actor` 中调整 `source.SetRadius(0.006)`。
- **背景颜色**: 修改 `self.renderer.SetBackground(0.96, 0.97, 0.98)` 及其 `SetBackground2` 实现渐变背景。
- **透明度**: 
  - 修改 `_create_fixed_axis` 中的 `actor.GetProperty().SetOpacity(0.2)` 调整参考系透明度。

### UI 交互参数

- **平移步长/范围**: 
  - 在 `_setup_sidebar` 中，`factor=100` 表示滑条数值映射到 0.01 米单位。
  - `pos_x_slider.setRange(-50, 50)` 配合 `factor=100` 对应 -0.5m 到 0.5m 的范围。

## 2. 功能说明

- **RX/RY/RZ**: 绕自身坐标轴旋转（Euler XYZ 顺序）。
- **X/Y/Z**: 在世界坐标系中的平移位置。
- **Set as Fixed Pose**: 将当前位置锁定为参考系（半透明显示），用于对比变换前后的差异。
- **Fetch from Scene**: 从当前的 3D 状态读取数值并填充到 4x4 矩阵输入框中。
- **Apply Matrix**: 将 4x4 矩阵输入框中的精确数值应用到 3D 场景中。

## 3. 依赖库
- `numpy`: 矩阵运算。
- `vtk`: 3D 渲染引擎。
- `PyQt5`: GUI 框架。
