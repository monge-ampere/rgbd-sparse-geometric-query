---
layout: post
title: "从 RGB-D 标定到稀疏几何查询：一个未充分展开的底层 3D 视觉方案"
date: 2026-04-05 20:30:00 +0800
categories: [3d-vision, calibration, geometry]
tags: [RGB-D, calibration, depth-camera, geometric-query, 3d-reconstruction]
math: true
---

# From RGB-D Calibration to Sparse Geometric Query: A Low-Level 3D Vision Scheme I Never Fully Developed

> **English Abstract**  
> This post reconstructs a low-level RGB-D vision scheme that I built around 2019 for calibration, cross-sensor mapping, and 3D recovery. The key point is that I did **not** treat RGB-to-depth alignment as a dense full-frame mapping problem. Instead, for the actual industrial task, I reformulated it as a **sparse geometric query problem**: given a small number of RGB feature points, recover their corresponding depth-image locations and 3D coordinates efficiently by searching along a calibrated viewing ray and validating depth consistency. Although the work was implemented quickly and never expanded into a full project or paper, it remains a representative example of a method I have repeatedly used later: when an existing API solves the wrong problem, the right move is often not to patch the API, but to rewrite the task at the correct geometric level.

这篇文章整理的是我在 2019 年前后做过的一套 RGB-D 底层视觉方案。它的原始形态并不完整：实现时间很短，大约一周左右，后来也没有继续展开成正式项目或论文。但现在回头看，它仍然值得写下来，因为它代表了一种我后来反复使用的方法：

> **当现成 API 的问题定义并不真正匹配任务时，关键并不是围绕 API 修修补补，而是把问题改写成更贴合任务本体的几何问题。**

这套方案的核心不是“做一张 RGB-D 对齐图”，而是：

- 建立 RGB、Depth、IR 之间的几何关系；
- 完成跨传感器标定与三维恢复；
- 把在线问题写成 **少量 RGB 特征点到深度图的稀疏几何查询**；
- 在任务需要时进一步结合 IR / 双目恢复更稳的 3D 参数。

---

## 1. 问题本体：我要的不是整幅稠密映射，而是少量关键点的快速 3D 恢复

很多 RGB-D 设备都提供从 RGB 到 Depth、或从 Depth 到 RGB 的官方映射 API。表面上看，这似乎已经足够；但如果进一步看工业任务，就会发现它解决的并不一定是你真正关心的问题。

对很多场景而言，真正需要的不是：

\[
\text{build a dense correspondence field over the whole image}
\]

而是：

\[
\text{given a few RGB feature points, recover their depth correspondences and 3D coordinates quickly.}
\]

也就是说，任务本体是一个 **稀疏关键点查询问题**，而不是一个 **整图稠密映射问题**。

如果问题本体本来就是稀疏的，那么先生成整幅 RGB-D 对齐场，再从里面取几个点，往往是明显的过度计算。

---

## 2. 为什么我没有直接采用厂商 API

当时使用的是 PICO Zense 相机。官方 SDK 提供了 RGB 与深度图互映射 API。其思路大体可以概括为：

1. 将深度图像素恢复到三维空间；
2. 通过相对位姿变换到另一相机坐标系；
3. 再投影到目标像平面；
4. 必要时再做一次反向拉回，完成对应建立。

如果写成符号形式，它更接近：

\[
(u_d, v_d, z_d)
\rightarrow X_d
\rightarrow X_{rgb}
\rightarrow (u_{rgb}, v_{rgb})
\rightarrow \text{dense mapping field}.
\]

这条链条当然通用，但也有三个直接问题：

### 2.1 它是整图思维，不是关键点思维

API 更接近在做

\[
\Omega_{rgb} \leftrightarrow \Omega_d
\]

上的稠密场构造，而我的任务只关心少量点

\[
\{p_i^{rgb}\}_{i=1}^n.
\]

### 2.2 它过度依赖深度图自身的稳定性

如果深度图在边界、遮挡、反光、低纹理区域出现缺失或不稳定，那么通用映射场本身就会受到影响。

### 2.3 它在计算上是“先做大，再取小”

为了得到少量点的答案，先做整图映射，这在任务上并不经济。

所以这里的关键判断不是“API 好不好”，而是：

> **API 解的是一个通用稠密问题，而我真正关心的是一个任务导向的稀疏问题。**

---

## 3. 几何基础：RGB、Depth、IR 本来就应该放在同一条几何链里

这套方案的基础不是某个特定 SDK，而是一个比较标准的多传感器几何模型。

### 3.1 RGB 相机模型

RGB 相机采用针孔模型。设三维点在 RGB 相机坐标系下为

\[
X_{rgb} = (X,Y,Z)^\top,
\]

则其归一化像平面坐标为

\[
x_n = \frac{X}{Z}, \qquad y_n = \frac{Y}{Z}.
\]

若内参矩阵记为

\[
K_{rgb} =
\begin{bmatrix}
 f_x & 0 & c_x \\
 0 & f_y & c_y \\
 0 & 0 & 1
\end{bmatrix},
\]

则理想无畸变投影为

\[
\tilde p_{rgb} = K_{rgb}
\begin{bmatrix}
 x_n \\
 y_n \\
 1
\end{bmatrix}.
\]

### 3.2 径向畸变模型

代码里采用的是简化径向畸变模型。设

\[
r^2 = x_n^2 + y_n^2,
\]

畸变因子写成

\[
\delta(r) = k_1 r^2 + k_2 r^4 + k_3 r^6.
\]

则从理想点到实际像点的近似关系可写成

\[
u = u_0 + (u_0-c_x)\,\delta(r),
\qquad
v = v_0 + (v_0-c_y)\,\delta(r),
\]

其中 \((u_0,v_0)\) 是无畸变投影后的像素坐标。

严格地说，逆畸变一般需要迭代求解；但在这份快速原型中，我采用了足够简单且可运行的近似处理。这种取舍本身也反映了当时的工程目标：**不是把局部模型做到最精，而是让整条几何链在短时间内可运行、可验证。**

### 3.3 RGB 与 D 的外参关系

设三维点在 RGB 相机坐标系和 D 相机坐标系中的坐标分别为 \(X_{rgb}\) 与 \(X_d\)，则二者满足刚体变换：

\[
X_d = R X_{rgb} + t,
\]

其中

\[
R \in SO(3), \qquad t \in \mathbb{R}^3.
\]

反变换则为

\[
X_{rgb} = R^{-1}(X_d - t).
\]

代码中分别保留了 `relaMatrix`, `relaTrans` 以及它们的逆。

### 3.4 为什么 IR 很重要

Depth 图像往往不适合直接拿来做棋盘格角点观测，而 IR 图像常常更自然地呈现标定板。因此在实际方案里，我并没有执着于“必须硬从 RGB-D 直接标定”，而是明确把 RGB–IR 双目标定看成更成熟、也更稳的入口，再通过 IR 与 D 的天然关联间接约束 D 相关参数。

这个判断在今天回头看仍然是对的：

> **不是所有传感器通道都应该被一视同仁地使用。几何上更适合做观测入口的通道，往往应该优先承担标定任务。**

---

## 4. 标定链条：先得到可靠几何关系，再谈在线查询

### 4.1 RGB 相机内参

RGB 相机内参可通过常规棋盘格方法拟合，即在多幅平面标定板图像上估计单应关系，再恢复内参与外参。若以张正友标定法的语言表达，就是从

\[
H_i = K [r_1^{(i)}\; r_2^{(i)}\; t^{(i)}]
\]

中恢复内参矩阵 \(K\)。

### 4.2 D 相机相关参数

对于深度相机，直接在深度图上提取高质量棋盘格角点并不自然，因此更可行的路线通常是：

- 利用对应点或平面模型做初始化；
- 再通过几何一致性做联合优化。

### 4.3 平面模型为何合适

设标定板所在平面为

\[
\Pi: n^\top X + d = 0,
\]

其中 \(n\) 为单位法向量。平面之所以合适，是因为：

1. 它是最简单的全局几何对象；
2. 它避免了深度边界跳变对观测稳定性的干扰；
3. 它为后续最小二乘或 LM 优化提供了自然约束。

### 4.4 初始化与细化

当时方案的思路并不是“一上来就全量大优化”，而是分成两步：

1. 先得到一个足够可靠的初始化；
2. 再通过联合残差做细化。

这一步在工程里很重要。因为如果初始化错得太远，后面的非线性优化往往并不会帮你，而只会把问题进一步带偏。

---

## 5. 真正的核心：把在线映射写成单条射线上的一维几何查询

如果这套东西只有标定，它还只是一个常规 3D 视觉模块。真正让我现在回头看仍然觉得漂亮的，是在线阶段的问题改写。

### 5.1 输入不是整幅图，而是一个 RGB 点

设 RGB 图像中一个关键点为

\[
p^{rgb} = (u,v,1)^\top.
\]

首先对其做近似逆畸变，得到理想像点，再左乘逆内参得到归一化视线方向：

\[
\hat x = K_{rgb}^{-1} \, p^{rgb}_{ideal}.
\]

于是从 RGB 光心出发可以得到一条三维射线

\[
\ell(s) = O_{rgb} + s\,\hat d,
\qquad s>0.
\]

代码里这一部分对应的是：

- `real2ideal(...)`
- `antiParam * rgbIdealPoint`
- `getStraightLine(...)`

### 5.2 只在工作深度区间上采样

与其在整张深度图甚至整个 3D 空间里找候选，不如直接利用任务先验，把搜索限制在

\[
s \in [z_{near}, z_{far}].
\]

于是在线采样问题变成

\[
X_s = \ell(s), \qquad s = z_{near}, z_{near}+\Delta s, \dots, z_{far}.
\]

也就是说，原来的二维 / 三维全局问题被压成了一个单变量搜索问题。

### 5.3 将候选点变换到 D 相机并投影

每一个候选点 \(X_s\) 在 RGB 相机坐标系下确定后，可变换到 D 相机坐标系：

\[
X_s^{(d)} = R X_s + t.
\]

再做归一化投影：

\[
(x_s^{(d)}, y_s^{(d)}) = \left(\frac{X_s^{(d)}}{Z_s^{(d)}}, \frac{Y_s^{(d)}}{Z_s^{(d)}}\right),
\]

并通过 D 相机内参和畸变得到深度图像素位置

\[
p_s^{(d)} = (u_s^{(d)}, v_s^{(d)}).
\]

代码里这一段主要对应：

- `linearTransform(...)`
- `getStraightLine(...)`（在 D 相机坐标系中构造相关视线）
- `getZPoint(...)`
- `ideal2real(...)`

### 5.4 用深度图只做一件事：验证这个候选是否成立

这一部分是我认为最关键的地方。

传统想法可能是：

> 先把 RGB 和 D 做完整映射，再建立对应关系。

而我的想法是：

> 候选 3D 点已经由 RGB 视线给出来了，深度图只需要判断这个候选是否与当前观测一致。

设从 D 光心到候选点的几何距离为

\[
d_{geom}(s) = \|X_s^{(d)} - O_d\|_2.
\]

从深度图像素 \(p_s^{(d)}\) 读取深度值，记为

\[
d_{cam}(s).
\]

则可以构造一个简单的一致性残差：

\[
r(s) = d_{geom}(s) - (a\,d_{cam}(s) + b).
\]

代码里对应的是：

```cpp
confidenceDegree = actual_dist - dCoeff[0] * camera_dist - dCoeff[1];
```

其中 \(a,b\) 对应于深度尺度修正参数。

一旦某个候选满足

\[
0 < r(s) < \tau,
\]

就认为这个候选在几何上与深度图观测足够一致，直接返回：

\[
X^* = X_{s^*}, \qquad p_d^* = p_{s^*}^{(d)}.
\]

这就是整套在线查询的本质。

---

## 6. 这其实不是“映射算法”，而是“假设验证算法”

现在回头看，我更愿意把它叫做：

> **面向 RGB 关键点的快速三维假设验证器**

而不是简单地叫“RGB-D 映射”。

因为它真正做的不是建立一张完整映射表，而是：

1. 由 RGB 点生成一条几何假设轨迹；
2. 由深度图对每个候选点做观测一致性验证；
3. 一旦找到满足条件的候选，立即输出三维坐标与深度图对应点。

从这个角度看，它更像：

\[
\text{RGB point} \Rightarrow \text{3D hypothesis family} \Rightarrow \text{depth validation}.
\]

这比“整图对齐”更贴近任务，也更节省计算。

---

## 7. 为什么它比厂商 API 快

现在可以比较明确地把原因拆开说。

### 7.1 搜索空间被降成 1D

如果官方 API 做的是整图级对应建立，那么其在线代价更接近：

\[
O(HW)
\]

或至少与整图大小相关。

而我的算法对单个关键点的代价更接近：

\[
O\left(\frac{z_{far}-z_{near}}{\Delta s}\right).
\]

这不是常数优化，而是问题维度本身降了。

### 7.2 每一步只做常数级运算

对每个候选样本，主要代价只有：

- 若干 3×3 线性变换；
- 一次投影；
- 一次深度图读数；
- 一次残差判断。

没有整图重采样，没有大规模插值，没有整图回拉。

### 7.3 深度区间先验直接压缩了候选数

若工作距离已知，则可直接设置

\[
[z_{near}, z_{far}]
\]

而不在无关深度上浪费代价。

### 7.4 具备早停

一旦第一个候选满足条件，算法立即终止。于是平均复杂度通常低于最坏复杂度。

所以这套算法快，并不是因为“C++ 写得更猛”，而是因为：

> **我没有在解和官方 API 同一个问题。**

---

## 8. 它的工程边界：为什么我说它是一个短期原型，而不是完整体系

这套东西虽然有价值，但我并不想把它说成一个已经完全打磨成熟的体系。它有明显的原型特征。

### 8.1 它优先追求快速可用，而不是全局最优

代码里一旦遇到第一个满足阈值的候选就 `break`。这说明它采用的是 **first-hit** 逻辑，而不是继续扫完整个区间去找全局最优解。

也就是说，它更像：

\[
\text{sufficiently good and fast}
\]

而不是

\[
\text{globally optimal and fully analyzed}.
\]

### 8.2 它依赖参数与工作带先验

算法效果依赖于：

- 标定参数是否足够准；
- `nearZ, farZ` 是否设得合适；
- 采样间隔 \(\Delta s\) 是否合适；
- 残差阈值 \(\tau\) 是否能区分真解与伪解。

这也解释了为什么它更像一个 **任务专用求解器**，而不是一个无条件通吃的通用 API。

### 8.3 代码层面也带有明显原型痕迹

比如在我保留下来的版本里，读取 16 位深度值的代码写法就存在可疑之处；从工程直觉看，当年的现场可运行版本大概率做过修正，但现在留下来的代码并不是最终整理过的展示版。

不过这恰恰说明了它的真实位置：

> **它首先是一个在很短时间内针对任务建立起来的底层原型，而不是一套为了发表或展示而精修过的“作品集工程”。**

---

## 9. 这件事后来对我真正重要的启发

现在回头看，这个项目真正重要的，不只是“做出了一个比 API 更快的 RGB-D 映射方案”。更重要的是，它再次验证了一个我后来越来越相信的判断：

> **很多视觉问题的关键，不在于有没有现成工具，而在于你是否抓住了问题真正的几何形态。**

如果问题本体是：

\[
\text{few-point sparse mapping + 3D recovery}
\]

那么沿着

\[
\text{full-frame dense remapping}
\]

这条路继续补丁式修修补补，往往永远不如把它重写成

\[
\text{ray-wise geometric consistency query}
\]

来得自然。

这也是为什么我后来越来越倾向于：

- 先找结构化几何中介；
- 再在这个中介里做低维、可控、可验证的求解；
- 而不是直接停留在像素层或接口层解决问题。

从今天回头看，这个很短时间内做出来、后来也没有继续展开的 RGB-D 原型，其实已经把这种方法论露出来了。

---

## 10. 结尾：如果不把它写下来，它就真的只剩零散代码了

这套 RGB-D 方案没有成为论文，没有成为完整项目，也没有被系统整理成正式技术说明。现在还能留下来的，只是一些设计文档、部分代码，以及我自己的回忆。

但我仍然觉得它值得写下来。

因为它代表的并不是某个小功能，而是一种我到今天仍然认可的底层视觉判断力：

- 不迷信厂商 API；
- 先把多传感器几何关系讲清楚；
- 先选对几何观测入口；
- 再把在线问题压缩成真正贴合任务的低维形式。

如果要把这篇文章压缩成一句话，我现在更愿意这样说：

> **这不是一个“RGB-D 标定模块”，而是一条从几何建模出发，把跨传感器对应问题改写成任务导向稀疏几何查询问题的底层 3D 视觉路线。**

---

## 附：如果要继续把它做成更完整的技术稿，还可以补什么

如果以后继续整理，我认为最值得补的有四块：

1. **完整符号统一**：把 RGB / D / IR 的坐标系、外参方向、深度定义统一成一个更规范的记号系统；
2. **残差与阈值分析**：讨论 \(r(s)\) 与采样间隔、深度噪声、姿态误差之间的关系；
3. **复杂度与成功率实验**：给出与官方 API 在少量关键点任务上的定量比较；
4. **RGB–IR 融合路线**：把当前的单点查询扩展成更强的双目/多视恢复框架。

也就是说，这个东西虽然当时只做了一周左右，后来没有继续，但它并不是一个无足轻重的小尝试。它更像是一个没被展开的底层线索：方向是对的，结构也是对的，只是当年没有继续把它推到完整形态。
