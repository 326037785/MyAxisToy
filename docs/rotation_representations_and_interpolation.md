# 旋转表示与插值方法 — 数学技术参考文档

> 本文档系统推导三维旋转的四种主要表示（旋转矩阵、欧拉角、轴角、四元数）之间的转换关系，详述 Rodrigues 旋转定理的推导与应用，并介绍 Lerp/Slerp 插值方法在路径规划与建模中的使用。

---

## 目录

1. [概述](#1-概述)
2. [符号定义](#2-符号定义)
3. [旋转矩阵](#3-旋转矩阵)
4. [欧拉角](#4-欧拉角)
5. [轴角表示](#5-轴角表示)
6. [四元数](#6-四元数)
7. [Rodrigues 旋转定理](#7-rodrigues-旋转定理)
8. [表示之间的转换](#8-表示之间的转换)
9. [四种表示的比较](#9-四种表示的比较)
10. [插值方法：Lerp 与 Slerp](#10-插值方法lerp-与-slerp)
11. [在路径规划与建模中的应用](#11-在路径规划与建模中的应用)

---

## 1. 概述

三维旋转是刚体运动学、机器人学、计算机图形学的核心概念。同一旋转可以用不同数学对象表示，各有优劣：

| 表示方法 | 自由度 | 是否冗余 | 奇异性 | 插值友好性 |
|----------|--------|----------|--------|------------|
| 旋转矩阵 | 9（约束至 6） | 是 | 无 | 差 |
| 欧拉角 | 3 | 否 | 万向锁 | 差 |
| 轴角 | 3 | 否 | 原点退化 | 中 |
| 四元数 | 4（约束至 3） | 是 | 无 | 优 |

本文档的目标：给出每种表示的严格定义，完整推导它们之间的转换公式，并说明 Rodrigues 定理如何统一轴角与旋转矩阵。

---

## 2. 符号定义

| 符号 | 含义 |
|------|------|
| $R \in SO(3)$ | 3×3 正交旋转矩阵，$\det(R) = 1$，$R^T R = I$ |
| $\hat{\mathbf{n}} \in \mathbb{R}^3$ | 单位旋转轴，$\|\hat{\mathbf{n}}\| = 1$ |
| $\theta \in \mathbb{R}$ | 旋转角度（弧度） |
| $\mathbf{r} = \theta \hat{\mathbf{n}} \in \mathbb{R}^3$ | 旋转向量（轴角的紧凑形式） |
| $\alpha, \beta, \gamma$ | 欧拉角（按约定顺序的三个旋转角） |
| $\mathbf{q} = w + xi + yj + zk$ | 四元数，$w$ 为实部，$(x, y, z)$ 为虚部 |
| $\mathbf{q} = (w, \mathbf{v})$ | 四元数的标量-向量形式，$\mathbf{v} = (x, y, z)$ |
| $\mathbf{q}^*$ | 四元数共轭 |
| $\|\mathbf{q}\|$ | 四元数模 |
| $\mathbf{p} \in \mathbb{R}^3$ | 三维空间点（齐次坐标为 $(\mathbf{p}, 1)$） |
| $[\mathbf{v}]_\times$ | 向量 $\mathbf{v}$ 的反对称矩阵（叉积矩阵） |
| $c_\theta, s_\theta$ | $\cos\theta, \sin\theta$ 的简写 |

---

## 3. 旋转矩阵

### 3.1 定义

旋转矩阵 $R \in SO(3)$ 是满足以下条件的 3×3 实矩阵：

$$R^T R = I, \quad \det(R) = +1$$

$SO(3)$ 是三维特殊正交群，包含所有保持方向的正交变换。

### 3.2 基本旋转矩阵

绕坐标轴旋转角度 $\theta$ 的基本矩阵：

**绕 X 轴：**

$$R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\theta & -s_\theta \\ 0 & s_\theta & c_\theta \end{bmatrix}$$

**绕 Y 轴：**

$$R_y(\theta) = \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix}$$

**绕 Z 轴：**

$$R_z(\theta) = \begin{bmatrix} c_\theta & -s_\theta & 0 \\ s_\theta & c_\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

### 3.3 旋转矩阵的性质

- **正交性**：$R^{-1} = R^T$
- **保距性**：$\|R\mathbf{p}\| = \|\mathbf{p}\|$，旋转不改变向量长度
- **保角性**：$(R\mathbf{a}) \cdot (R\mathbf{b}) = \mathbf{a} \cdot \mathbf{b}$
- **群封闭性**：$R_1 R_2 \in SO(3)$，连续旋转仍是旋转
- **迹与旋转角**：$\text{tr}(R) = 1 + 2\cos\theta$

---

## 4. 欧拉角

### 4.1 定义

欧拉角用三个角度参数化旋转，通过三次基本旋转的复合实现。不同的旋转顺序产生不同的约定。

### 4.2 旋转顺序约定

**内旋（Intrinsic，动轴）**：绕自身的轴旋转，记为 $Z$-$Y'$-$Z''$（或 $Z$-$Y$-$Z$）。

$$R = R_z(\alpha) \cdot R_y(\beta) \cdot R_z(\gamma)$$

**外旋（Extrinsic，固定轴）**：绕固定的世界轴旋转，等价于内旋的逆序。

$Z$-$Y$-$Z$ 外旋等价于 $Z$-$Y$-$Z$ 内旋。

### 4.3 ZYZ 欧拉角到旋转矩阵的展开

$$R_{ZYZ}(\alpha, \beta, \gamma) = R_z(\alpha) \cdot R_y(\beta) \cdot R_z(\gamma)$$

$$= \begin{bmatrix} c_\alpha & -s_\alpha & 0 \\ s_\alpha & c_\alpha & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} c_\beta & 0 & s_\beta \\ 0 & 1 & 0 \\ -s_\beta & 0 & c_\beta \end{bmatrix} \begin{bmatrix} c_\gamma & -s_\gamma & 0 \\ s_\gamma & c_\gamma & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

展开结果：

$$R_{ZYZ} = \begin{bmatrix} c_\alpha c_\beta c_\gamma - s_\alpha s_\gamma & -c_\alpha c_\beta s_\gamma - s_\alpha c_\gamma & c_\alpha s_\beta \\ s_\alpha c_\beta c_\gamma + c_\alpha s_\gamma & -s_\alpha c_\beta s_\gamma + c_\alpha c_\gamma & s_\alpha s_\beta \\ -s_\beta c_\gamma & s_\beta s_\gamma & c_\beta \end{bmatrix}$$

### 4.4 万向锁（Gimbal Lock）

当中间旋转角 $\beta = 0$ 或 $\beta = \pi$ 时：

$$R_{ZYZ}(\alpha, 0, \gamma) = \begin{bmatrix} c_{\alpha+\gamma} & -s_{\alpha+\gamma} & 0 \\ s_{\alpha+\gamma} & c_{\alpha+\gamma} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

$\alpha$ 和 $\gamma$ 退化为一个自由度，丢失一个旋转自由度。这是欧拉角表示的本质缺陷。

---

## 5. 轴角表示

### 5.1 定义

**轴角（Axis-Angle）**表示将旋转分解为：绕单位轴 $\hat{\mathbf{n}}$ 旋转角度 $\theta$。

紧凑形式为旋转向量：

$$\mathbf{r} = \theta \hat{\mathbf{n}} \in \mathbb{R}^3$$

其中 $\|\hat{\mathbf{n}}\| = 1$，$\theta = \|\mathbf{r}\|$。

### 5.2 几何含义

对于空间中任意点 $\mathbf{p}$，绕 $\hat{\mathbf{n}}$ 旋转 $\theta$ 后的位置 $\mathbf{p}'$ 满足：

- $\hat{\mathbf{n}}$ 方向的分量不变
- 垂直于 $\hat{\mathbf{n}}$ 的平面内发生角度 $\theta$ 的旋转

将 $\mathbf{p}$ 分解为平行分量和垂直分量：

$$\mathbf{p} = \mathbf{p}_\parallel + \mathbf{p}_\perp = (\mathbf{p} \cdot \hat{\mathbf{n}})\hat{\mathbf{n}} + (\mathbf{p} - (\mathbf{p} \cdot \hat{\mathbf{n}})\hat{\mathbf{n}})$$

旋转后：

$$\mathbf{p}' = \mathbf{p}_\parallel + \cos\theta \cdot \mathbf{p}_\perp + \sin\theta \cdot (\hat{\mathbf{n}} \times \mathbf{p})$$

这正是 Rodrigues 旋转公式的几何形式（§7 详述）。

### 5.3 从旋转矩阵提取轴角

给定 $R$，旋转角：

$$\theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right)$$

当 $\theta \ne 0$ 且 $\theta \ne \pi$ 时，旋转轴：

$$\hat{\mathbf{n}} = \frac{1}{2\sin\theta} \begin{pmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{pmatrix}$$

**奇异情况处理：**

- $\theta \approx 0$：旋转接近恒等，轴不确定，约定取 $\hat{\mathbf{n}} = (0, 0, 1)$
- $\theta \approx \pi$：从 $R + I$ 的列中提取轴（对角线元素最大的列方向）

---

## 6. 四元数

### 6.1 定义

四元数 $\mathbf{q}$ 是实数域的四维扩张：

$$\mathbf{q} = w + xi + yj + zk$$

其中 $i, j, k$ 为虚数单位，满足：

$$i^2 = j^2 = k^2 = ijk = -1$$

等价的标量-向量表示：

$$\mathbf{q} = (w, \mathbf{v}), \quad \mathbf{v} = (x, y, z)$$

### 6.2 基本运算

**共轭：**

$$\mathbf{q}^* = w - xi - yj - zk = (w, -\mathbf{v})$$

**模：**

$$\|\mathbf{q}\| = \sqrt{w^2 + x^2 + y^2 + z^2}$$

**逆：**

$$\mathbf{q}^{-1} = \frac{\mathbf{q}^*}{\|\mathbf{q}\|^2}$$

**乘法（Hamilton 积）：**

$$\mathbf{q}_1 \mathbf{q}_2 = (w_1 w_2 - \mathbf{v}_1 \cdot \mathbf{v}_2,\ w_1 \mathbf{v}_2 + w_2 \mathbf{v}_1 + \mathbf{v}_1 \times \mathbf{v}_2)$$

展开为标量形式：

$$\begin{aligned}
w &= w_1 w_2 - x_1 x_2 - y_1 y_2 - z_1 z_2 \\
x &= w_1 x_2 + x_1 w_2 + y_1 z_2 - z_1 y_2 \\
y &= w_1 y_2 - x_1 z_2 + y_1 w_2 + z_1 x_2 \\
z &= w_1 z_2 + x_1 y_2 - y_1 x_2 + z_1 w_2
\end{aligned}$$

**注意**：四元数乘法不可交换，$\mathbf{q}_1 \mathbf{q}_2 \ne \mathbf{q}_2 \mathbf{q}_1$。

### 6.3 单位四元数与旋转

单位四元数 $\|\mathbf{q}\| = 1$ 表示旋转。用单位四元数旋转点 $\mathbf{p}$：

$$\mathbf{p}' = \mathbf{q} \cdot \mathbf{p} \cdot \mathbf{q}^*$$

其中 $\mathbf{p}$ 被视为零实部的纯四元数 $\mathbf{p} = (0, \mathbf{p})$。

展开后等价于旋转矩阵作用：

$$\mathbf{p}' = (w^2 - \mathbf{v} \cdot \mathbf{v})\mathbf{p} + 2(\mathbf{v} \cdot \mathbf{p})\mathbf{v} + 2w(\mathbf{v} \times \mathbf{p})$$

### 6.4 双覆盖性

$\mathbf{q}$ 和 $-\mathbf{q}$ 表示相同的旋转。这是 $SO(3)$ 被 $S^3$（单位四元数球面）双覆盖的体现。

---

## 7. Rodrigues 旋转定理

### 7.1 定理陈述

给定单位旋转轴 $\hat{\mathbf{n}}$ 和旋转角 $\theta$，对应的旋转矩阵为：

$$R = I + \sin\theta \cdot [\hat{\mathbf{n}}]_\times + (1 - \cos\theta) \cdot [\hat{\mathbf{n}}]_\times^2$$

其中 $[\hat{\mathbf{n}}]_\times$ 是 $\hat{\mathbf{n}} = (n_x, n_y, n_z)$ 的反对称矩阵：

$$[\hat{\mathbf{n}}]_\times = \begin{bmatrix} 0 & -n_z & n_y \\ n_z & 0 & -n_x \\ -n_y & n_x & 0 \end{bmatrix}$$

### 7.2 推导

**目标**：推导绕任意单位轴 $\hat{\mathbf{n}}$ 旋转角度 $\theta$ 的矩阵表达式。

**Step 1：分解向量**

对于空间中任意点 $\mathbf{p}$，将其分解为平行于旋转轴和垂直于旋转轴的两个分量：

$$\mathbf{p} = \mathbf{p}_\parallel + \mathbf{p}_\perp$$

平行分量（投影到轴上）：

$$\mathbf{p}_\parallel = (\mathbf{p} \cdot \hat{\mathbf{n}})\hat{\mathbf{n}} = \hat{\mathbf{n}}\hat{\mathbf{n}}^T \mathbf{p}$$

垂直分量：

$$\mathbf{p}_\perp = \mathbf{p} - \mathbf{p}_\parallel = (I - \hat{\mathbf{n}}\hat{\mathbf{n}}^T)\mathbf{p}$$

**Step 2：垂直平面内的旋转**

在垂直于 $\hat{\mathbf{n}}$ 的平面内，$\mathbf{p}_\perp$ 旋转角度 $\theta$ 后变为：

$$\mathbf{p}_\perp' = \cos\theta \cdot \mathbf{p}_\perp + \sin\theta \cdot (\hat{\mathbf{n}} \times \mathbf{p}_\perp)$$

由于 $\hat{\mathbf{n}} \times \mathbf{p}_\parallel = \mathbf{0}$，有 $\hat{\mathbf{n}} \times \mathbf{p}_\perp = \hat{\mathbf{n}} \times \mathbf{p}$，因此：

$$\mathbf{p}_\perp' = \cos\theta \cdot \mathbf{p}_\perp + \sin\theta \cdot (\hat{\mathbf{n}} \times \mathbf{p})$$

**Step 3：合并**

旋转后的总向量：

$$\mathbf{p}' = \mathbf{p}_\parallel + \mathbf{p}_\perp' = \mathbf{p}_\parallel + \cos\theta \cdot \mathbf{p}_\perp + \sin\theta \cdot (\hat{\mathbf{n}} \times \mathbf{p})$$

将 $\mathbf{p}_\parallel$ 和 $\mathbf{p}_\perp$ 代入：

$$\mathbf{p}' = \hat{\mathbf{n}}\hat{\mathbf{n}}^T \mathbf{p} + \cos\theta \cdot (I - \hat{\mathbf{n}}\hat{\mathbf{n}}^T)\mathbf{p} + \sin\theta \cdot [\hat{\mathbf{n}}]_\times \mathbf{p}$$

提取公因子 $\mathbf{p}$：

$$R = \cos\theta \cdot I + (1 - \cos\theta) \cdot \hat{\mathbf{n}}\hat{\mathbf{n}}^T + \sin\theta \cdot [\hat{\mathbf{n}}]_\times$$

**Step 4：利用 $[\hat{\mathbf{n}}]_\times^2$ 化简**

反对称矩阵的平方满足：

$$[\hat{\mathbf{n}}]_\times^2 = \hat{\mathbf{n}}\hat{\mathbf{n}}^T - I$$

因此 $\hat{\mathbf{n}}\hat{\mathbf{n}}^T = I + [\hat{\mathbf{n}}]_\times^2$，代入上式：

$$R = \cos\theta \cdot I + (1 - \cos\theta)(I + [\hat{\mathbf{n}}]_\times^2) + \sin\theta \cdot [\hat{\mathbf{n}}]_\times$$

$$R = I + \sin\theta \cdot [\hat{\mathbf{n}}]_\times + (1 - \cos\theta) \cdot [\hat{\mathbf{n}}]_\times^2$$

证毕。

### 7.3 展开为矩阵元素

$$R = \begin{bmatrix} c_\theta + n_x^2(1 - c_\theta) & n_x n_y(1 - c_\theta) - n_z s_\theta & n_x n_z(1 - c_\theta) + n_y s_\theta \\ n_y n_x(1 - c_\theta) + n_z s_\theta & c_\theta + n_y^2(1 - c_\theta) & n_y n_z(1 - c_\theta) - n_x s_\theta \\ n_z n_x(1 - c_\theta) - n_y s_\theta & n_z n_y(1 - c_\theta) + n_x s_\theta & c_\theta + n_z^2(1 - c_\theta) \end{bmatrix}$$

### 7.4 逆问题：从旋转矩阵到轴角

由 Rodrigues 公式的迹：

$$\text{tr}(R) = 3\cos\theta + (1 - \cos\theta)(n_x^2 + n_y^2 + n_z^2) = 1 + 2\cos\theta$$

因此：

$$\theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right)$$

当 $\sin\theta \ne 0$ 时，从 $R - R^T = 2\sin\theta \cdot [\hat{\mathbf{n}}]_\times$ 提取轴：

$$\hat{\mathbf{n}} = \frac{1}{2\sin\theta} \begin{pmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{pmatrix}$$

### 7.5 无穷小旋转与角速度

当 $\theta$ 很小时，$\cos\theta \approx 1$，$\sin\theta \approx \theta$，Rodrigues 公式退化为：

$$R \approx I + \theta [\hat{\mathbf{n}}]_\times$$

这正是反对称矩阵生成 $SO(3)$ 的李代数关系。对于时间连续的旋转，角速度 $\boldsymbol{\omega} = \omega \hat{\mathbf{n}}$ 与旋转矩阵的关系：

$$\dot{R} = [\boldsymbol{\omega}]_\times R$$

---

## 8. 表示之间的转换

### 8.1 轴角 → 旋转矩阵

直接使用 Rodrigues 公式（§7.1）：

$$R = I + \sin\theta \cdot [\hat{\mathbf{n}}]_\times + (1 - \cos\theta) \cdot [\hat{\mathbf{n}}]_\times^2$$

### 8.2 旋转矩阵 → 轴角

$$\theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right)$$

$$\hat{\mathbf{n}} = \frac{1}{2\sin\theta} \begin{pmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{pmatrix}$$

奇异处理：
- $\theta \approx 0$：恒等旋转，$\hat{\mathbf{n}}$ 不确定
- $\theta \approx \pi$：令 $u_i = \sqrt{R_{ii} + 1}/2$，选择最大分量的轴

### 8.3 轴角 → 四元数

单位四元数：

$$\mathbf{q} = \left(\cos\frac{\theta}{2},\ \sin\frac{\theta}{2} \cdot \hat{\mathbf{n}}\right)$$

即：

$$w = \cos\frac{\theta}{2}, \quad x = n_x \sin\frac{\theta}{2}, \quad y = n_y \sin\frac{\theta}{2}, \quad z = n_z \sin\frac{\theta}{2}$$

**推导依据**：四元数旋转公式 $\mathbf{p}' = \mathbf{q}\mathbf{p}\mathbf{q}^*$ 展开后等价于 Rodrigues 公式，当 $w = \cos(\theta/2)$，$\mathbf{v} = \sin(\theta/2)\hat{\mathbf{n}}$ 时成立。

### 8.4 四元数 → 轴角

$$\theta = 2 \arccos(w) = 2 \arctan2(\|\mathbf{v}\|, w)$$

$$\hat{\mathbf{n}} = \frac{\mathbf{v}}{\|\mathbf{v}\|} = \frac{(x, y, z)}{\sqrt{x^2 + y^2 + z^2}}$$

当 $\|\mathbf{v}\| \approx 0$ 时（接近恒等旋转），约定 $\hat{\mathbf{n}} = (0, 0, 1)$。

### 8.5 旋转矩阵 → 四元数

**Shepperd 方法**（数值稳定）：

计算四个候选值：

$$\begin{aligned}
s &= \sqrt{\text{tr}(R) + 1} \cdot 2 \\
w &= s / 4 \\
x &= (R_{32} - R_{23}) / s \\
y &= (R_{13} - R_{31}) / s \\
z &= (R_{21} - R_{12}) / s
\end{aligned}$$

当 $\text{tr}(R) > 0$ 时上述方法数值稳定。否则选择 $R_{ii}$ 最大的分量作为起始：

若 $R_{11} > R_{22}$ 且 $R_{11} > R_{33}$：

$$s = \sqrt{1 + R_{11} - R_{22} - R_{33}} \cdot 2$$

$$w = (R_{32} - R_{23}) / s, \quad x = s/4, \quad y = (R_{12} + R_{21}) / s, \quad z = (R_{13} + R_{31}) / s$$

其余两个分支类似。

### 8.6 四元数 → 旋转矩阵

由四元数旋转公式 $\mathbf{p}' = \mathbf{q}\mathbf{p}\mathbf{q}^*$ 展开：

$$R = \begin{bmatrix} 1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\ 2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\ 2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2) \end{bmatrix}$$

**推导**：将 $\mathbf{q}\mathbf{p}\mathbf{q}^*$ 展开，收集 $\mathbf{p}$ 的各分量系数，即得上述矩阵。详细步骤如下：

设 $\mathbf{p} = (0, p_x, p_y, p_z)$，$\mathbf{q} = (w, x, y, z)$：

$$\mathbf{q}\mathbf{p} = (-xp_x - yp_y - zp_z,\ wp_x + yp_z - zp_y,\ wp_y + zx_p - xp_z,\ wp_z + xp_y - yp_x)$$

$$\mathbf{q}\mathbf{p}\mathbf{q}^* = (\ldots,\ R_{11}p_x + R_{12}p_y + R_{13}p_z,\ \ldots)$$

逐一整理系数即得 $R$ 的各元素。

### 8.7 欧拉角 → 旋转矩阵

按约定的旋转顺序，连乘基本旋转矩阵。以 ZYZ 为例（§4.3）：

$$R_{ZYZ}(\alpha, \beta, \gamma) = R_z(\alpha) \cdot R_y(\beta) \cdot R_z(\gamma)$$

### 8.8 旋转矩阵 → 欧拉角

以 ZYZ 约定为例，从 $R_{ZYZ}$ 的元素反解：

$$\beta = \arccos(R_{33})$$

当 $\sin\beta \ne 0$ 时：

$$\alpha = \arctan2(R_{23}, R_{13})$$

$$\gamma = \arctan2(R_{32}, -R_{31})$$

当 $\beta = 0$（万向锁）：$\gamma$ 不确定，约定 $\gamma = 0$，$\alpha = \arctan2(R_{21}, R_{11})$。

### 8.9 轴角 ↔ 欧拉角

不直接转换。标准路径为：

$$\text{轴角} \xrightarrow{\text{Rodrigues}} \text{旋转矩阵} \xrightarrow{\text{反解}} \text{欧拉角}$$

$$\text{欧拉角} \xrightarrow{\text{连乘}} \text{旋转矩阵} \xrightarrow{\text{提取}} \text{轴角}$$

### 8.10 四元数 ↔ 欧拉角

同理，通过旋转矩阵中转：

$$\text{四元数} \xrightarrow{\S 8.6} \text{旋转矩阵} \xrightarrow{\S 8.8} \text{欧拉角}$$

$$\text{欧拉角} \xrightarrow{\S 8.7} \text{旋转矩阵} \xrightarrow{\S 8.5} \text{四元数}$$

---

## 9. 四种表示的比较

### 9.1 存储与计算开销

| 表示 | 存储 | 矩阵化开销 | 归一化开销 |
|------|------|-----------|-----------|
| 旋转矩阵 | 9 floats | $O(1)$（已就是矩阵） | $O(1)$（正交化） |
| 欧拉角 | 3 floats | $O(1)$（3 次基本旋转乘法） | 不需要 |
| 轴角 | 3 floats | $O(1)$（Rodrigues 公式） | 不需要 |
| 四元数 | 4 floats | $O(1)$（展开公式） | $O(1)$（除以模） |

### 9.2 插值行为

| 方法 | 等速性 | 最短路径 | 光滑性 |
|------|--------|----------|--------|
| 矩阵 Lerp | 否 | 否 | 否 |
| 欧拉角 Lerp | 否 | 否 | 否 |
| 轴角 Lerp | 近似 | 是 | 近似 |
| 四元数 Slerp | 是 | 是 | 是 |

### 9.3 奇异性总结

| 表示 | 奇异性 | 触发条件 | 后果 |
|------|--------|----------|------|
| 旋转矩阵 | 无 | — | — |
| 欧拉角 | 万向锁 | 中间角 $= 0$ 或 $\pi$ | 丢失一个自由度 |
| 轴角 | 原点退化 | $\theta = 0$ | 轴不确定 |
| 四元数 | 无 | — | 双覆盖（$\pm\mathbf{q}$） |

### 9.4 推荐使用场景

| 场景 | 推荐表示 | 原因 |
|------|----------|------|
| 存储/传输 | 四元数 | 4 floats，无冗余 |
| 用户交互 | 欧拉角 | 直观易理解 |
| 碰撞检测中的姿态比较 | 轴角 | 距离度量直接 |
| 矩阵运算/渲染管线 | 旋转矩阵 | 与齐次变换兼容 |
| 动画插值 | 四元数 Slerp | 等速、最短路径 |
| 数值积分（姿态动力学） | 轴角或四元数 | 无万向锁 |

---

## 10. 插值方法：Lerp 与 Slerp

### 10.1 问题定义

给定两个旋转状态 $\mathbf{q}_A$ 和 $\mathbf{q}_B$（用单位四元数表示），求参数 $t \in [0, 1]$ 处的中间旋转 $\mathbf{q}(t)$，使得：

- $\mathbf{q}(0) = \mathbf{q}_A$
- $\mathbf{q}(1) = \mathbf{q}_B$
- 插值路径在旋转空间中尽可能"自然"

### 10.2 线性插值（Lerp）

**定义：**

$$\text{Lerp}(\mathbf{q}_A, \mathbf{q}_B, t) = (1 - t)\mathbf{q}_A + t\mathbf{q}_B$$

**归一化 Lerp（Nlerp）：**

$$\text{Nlerp}(\mathbf{q}_A, \mathbf{q}_B, t) = \frac{(1 - t)\mathbf{q}_A + t\mathbf{q}_B}{\|(1 - t)\mathbf{q}_A + t\mathbf{q}_B\|}$$

**性质：**

- 计算简单：一次向量加法 + 一次归一化
- **非等速**：角速度在 $t$ 上不均匀
- **非最短路径**：当 $\mathbf{q}_A \cdot \mathbf{q}_B < 0$（夹角 $> 90°$）时，Nlerp 会绕远路
- **$t = 0.5$ 处最均匀**，两端处速度偏差最大

**双覆盖处理：**

$$\text{Nlerp}(\mathbf{q}_A, \mathbf{q}_B, t) = \text{Nlerp}(\mathbf{q}_A, -\mathbf{q}_B, t)$$

选择 $\mathbf{q}_A \cdot \mathbf{q}_B \ge 0$ 的符号对，确保走最短弧。

### 10.3 球面线性插值（Slerp）

**定义：** 在单位四元数球面 $S^3$ 上，沿大圆弧（测地线）等速插值。

$$\text{Slerp}(\mathbf{q}_A, \mathbf{q}_B, t) = \frac{\sin((1-t)\Omega)}{\sin\Omega} \mathbf{q}_A + \frac{\sin(t\Omega)}{\sin\Omega} \mathbf{q}_B$$

其中 $\Omega$ 为两四元数之间的夹角：

$$\cos\Omega = \mathbf{q}_A \cdot \mathbf{q}_B = w_A w_B + x_A x_B + y_A y_B + z_A z_B$$

$$\Omega = \arccos(\cos\Omega)$$

**推导依据：**

$S^3$ 是单位四元数构成的三维球面。球面上两点间的最短路径是大圆弧（测地线）。Slerp 参数化这条弧线，使得角位移 $\Delta\theta(t) = t \cdot \Omega$ 线性增长，从而保证等速性。

**双覆盖处理：**

若 $\mathbf{q}_A \cdot \mathbf{q}_B < 0$，取 $\mathbf{q}_B \leftarrow -\mathbf{q}_B$（等价于同一旋转的另一表示），确保走最短弧。

### 10.4 Slerp 的数值稳定性

当 $|\cos\Omega| \approx 1$（夹角很小）时，$\sin\Omega \approx 0$，直接计算会引入数值误差。

**退化为 Nlerp：**

$$\text{Slerp}(\mathbf{q}_A, \mathbf{q}_B, t) \approx \text{Nlerp}(\mathbf{q}_A, \mathbf{q}_B, t), \quad \text{当 } |\cos\Omega| > 1 - \epsilon$$

阈值通常取 $\epsilon = 10^{-3}$。当夹角很小时，Slerp 与 Nlerp 的差异可以忽略。

### 10.5 三者对比

| 性质 | Lerp | Nlerp | Slerp |
|------|------|-------|-------|
| 公式 | $(1-t)\mathbf{q}_A + t\mathbf{q}_B$ | 归一化 Lerp | 球面大圆弧插值 |
| 结果是否单位四元数 | 否 | 是 | 是 |
| 等速性 | 否 | 否（$t=0.5$ 处最优） | 是 |
| 最短路径 | 不保证 | 需符号修正 | 需符号修正 |
| 计算成本 | 最低 | 低（+归一化） | 中（+三角函数） |
| 数值稳定性 | 好 | 好 | 小角度退化为 Nlerp |
| 适用场景 | 快速近似 | 实时动画 | 精确路径规划 |

### 10.6 多段插值（Catmull-Rom Spline on $S^3$）

给定四元数序列 $\mathbf{q}_{i-1}, \mathbf{q}_i, \mathbf{q}_{i+1}, \mathbf{q}_{i+2}$，在 $\mathbf{q}_i$ 和 $\mathbf{q}_{i+1}$ 之间构造 Catmull-Rom 样条：

$$\mathbf{q}(t) = \text{Slerp}\left(\text{Slerp}(\mathbf{q}_i, \mathbf{q}_{i+1}, t),\ \text{Slerp}(\mathbf{q}_{i-1}, \mathbf{q}_{i+2}, t),\ 2t(1-t)\right)$$

这是一种四元数上的 Catmull-Rom 插值，提供 $C^1$ 连续性。

---

## 11. 在路径规划与建模中的应用

### 11.1 笛卡尔空间路径规划中的姿态插值

在机器人路径规划中，末端执行器的路径通常在笛卡尔空间中定义为位姿序列。位置部分可用多项式样条插值，姿态部分使用四元数 Slerp：

$$T(t) = \begin{bmatrix} R(t) & \mathbf{p}(t) \\ \mathbf{0}^T & 1 \end{bmatrix}$$

其中 $\mathbf{p}(t)$ 为位置样条（如三次 B 样条），$R(t)$ 为通过 Slerp 插值四元数序列得到的旋转矩阵。

### 11.2 关节空间路径的姿态分量

当路径规划在关节空间中进行时，姿态信息隐含在正运动学的输出中。若需在两个位姿之间做笛卡尔空间插值再逆解回关节空间：

**Step 1**：提取两端姿态的四元数 $\mathbf{q}_A, \mathbf{q}_B$

**Step 2**：Slerp 插值得到中间姿态四元数序列 $\{\mathbf{q}(t_k)\}$

**Step 3**：将每个四元数转换为旋转矩阵 $R(t_k)$

**Step 4**：构造齐次变换矩阵 $T(t_k)$

**Step 5**：对每个 $T(t_k)$ 求逆运动学解

### 11.3 姿态距离度量

在 RRT 等采样式规划器中，需要度量两个姿态的"距离"。常用方法：

**四元数内积距离：**

$$d_{\text{quat}}(\mathbf{q}_A, \mathbf{q}_B) = 1 - |\mathbf{q}_A \cdot \mathbf{q}_B|$$

其中取绝对值处理双覆盖。范围 $[0, 1]$，$0$ 表示相同旋转，$1$ 表示相反旋转（$180°$）。

**轴角距离：**

$$d_{\text{axis-angle}}(\mathbf{q}_A, \mathbf{q}_B) = \|\text{axis\_angle}(\mathbf{q}_A^{-1} \mathbf{q}_B)\|$$

即计算相对旋转的旋转角。

**旋转向量 L2 范数：**

$$d_{\text{rv}}(\mathbf{q}_A, \mathbf{q}_B) = \|\mathbf{r}_A - \mathbf{r}_B\|$$

其中 $\mathbf{r}$ 为旋转向量。此方法计算简单但非测地距离。

### 11.4 关节空间与笛卡尔空间的混合插值

在实际路径规划中，常采用混合策略：

1. **关节空间**中使用欧氏距离度量（适用于 RRT 采样和最近邻搜索）
2. **笛卡尔空间**中使用 Slerp 插值（用于路径后处理和姿态修正）
3. **碰撞检测**时使用旋转矩阵（与齐次变换兼容）

这种混合策略兼顾了规划效率（关节空间搜索快）和运动学合理性（笛卡尔空间插值平滑）。

### 11.5 数值积分中的姿态更新

在基于速度的运动控制中，姿态需要通过角速度积分更新：

$$\dot{R} = [\boldsymbol{\omega}]_\times R$$

使用四元数形式：

$$\dot{\mathbf{q}} = \frac{1}{2} \boldsymbol{\omega}_q \cdot \mathbf{q}$$

其中 $\boldsymbol{\omega}_q = (0, \boldsymbol{\omega})$ 为纯四元数角速度。

离散更新（一阶近似）：

$$\mathbf{q}(t + \Delta t) = \text{normalize}\left(\mathbf{q}(t) + \frac{\Delta t}{2} \boldsymbol{\omega}_q \cdot \mathbf{q}(t)\right)$$

更精确的方法是将角速度转换为旋转向量 $\mathbf{r} = \boldsymbol{\omega} \Delta t$，再构造增量四元数 $\Delta\mathbf{q} = (\cos\frac{\|\mathbf{r}\|}{2},\ \sin\frac{\|\mathbf{r}\|}{2} \cdot \hat{\mathbf{r}})$，然后 $\mathbf{q}(t + \Delta t) = \Delta\mathbf{q} \cdot \mathbf{q}(t)$。

---

## 附录：转换路径速查表

```
                    ┌──────────┐
                    │ 旋转矩阵 │
                    └────┬─────┘
                         │
          ┌──────────────┼──────────────┐
          │              │              │
    ┌─────▼─────┐  ┌─────▼─────┐  ┌─────▼─────┐
    │   欧拉角   │  │   轴角    │  │   四元数   │
    └───────────┘  └───────────┘  └───────────┘

欧拉角 ←──→ 旋转矩阵 ←──→ 轴角 ←──→ 四元数
              ↑              ↑
              └──────────────┘
           （欧拉角↔轴角不直接转换）
```

所有转换的最短路径均经过旋转矩阵中转。实际实现中可直接推导端到端公式以避免两次矩阵化。
