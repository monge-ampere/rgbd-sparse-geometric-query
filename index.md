---
layout: post
title: "从 RGB-D 标定到稀疏几何查询：一个未被正式写下来的 3D 视觉底层方案"
date: 2026-04-05 20:00:00 +0800
categories: [3d-vision, calibration, geometry]
---

# From RGB-D Calibration to Sparse Geometric Query: A Low-Level 3D Vision Scheme I Never Properly Wrote Down

> **English Abstract**  
> This article reconstructs a low-level RGB-D vision scheme I built around 2019–2020 for calibration, cross-sensor point mapping, and lightweight 3D recovery. The key insight was that the practical task was not dense RGB–depth alignment over the entire image, but sparse mapping of a small number of RGB feature points into the depth image and then into 3D. Instead of relying on the vendor API for full-frame remapping, the scheme reformulated the problem as a task-specific geometric query: given one RGB feature point, generate a 3D viewing ray, sample along a constrained depth interval, project each 3D hypothesis into the depth camera, and validate it by depth consistency. Although the prototype was built quickly and never developed into a complete research artifact, it remains a representative example of a lesson I have reused many times later: once the geometric level of the problem is rewritten correctly, the solution can become both simpler and faster.

这篇文章整理的是我在 2019–2020 年前后做过的一套 RGB-D 底层视觉方案。严格说，它不是一个已经被完整整理成论文或正式产品文档的成果；它更像一个**在很短时间内做出来、后来没有继续展开，但核心问题其实写对了**的原型。

如果只从功能描述上看，它不过是“RGB-D 标定 + 点映射 + 三维恢复”。但现在回头看，我觉得它真正有价值的地方不在功能罗列，而在于它体现了一种后来我反复使用的方法：

> **当现成 API 的问题定义与任务本体并不一致时，继续围绕 API 修补往往不是正路。更重要的是回到几何本身，把问题改写成更贴合任务的结构。**

这套方案就是一个很具体的例子。它不是继续沿着厂商提供的整幅 RGB–Depth 稠密映射接口往前堆，而是把问题重写成了一个**面向少量关键点的稀疏几何查询问题**。

---

## 1. 任务本体到底是什么

在 RGB-D 项目里，一个最自然的说法是：“需要 RGB 图和深度图对齐。”  
但这个说法虽然常见，却不一定准确。

对于很多工业场景，真正关心的往往不是“整幅 RGB 图和整幅 D 图都精确稠密对应”，而是下面这类更具体的问题：

- RGB 图上的某几个关键点，在深度图上到底对应哪里；
- 这些点的三维坐标能否稳定恢复；
- 后续能否与 IR、双目或其他几何线索融合，得到更稳的三维参数。

也就是说，任务本体其实更接近：

> **给定 RGB 图上的少量关键点，快速而稳定地找到其在深度图中的对应位置，并恢复其三维坐标。**

这个表述看似只是措辞变化，实际上已经决定了算法路线。

如果问题是“整幅图稠密映射”，那么自然会去找 full-frame remapping。  
如果问题是“少量点的快速恢复”，那么更自然的路线其实是**稀疏、几何化、按需求值**。

---

## 2. 为什么我没有直接采用厂商 API

当时 PICO Zense 已经提供了 RGB 与深度图之间的映射 API。设计文档里对它的描述很清楚：它的实现机理大致是把深度图经由相对位姿提升到三维，再投影到 RGB 图，然后再把 RGB 中的点拉回深度图完成互映射。

下面这张图来自当时保留下来的设计文档：

![API mapping result vs. our mapping result]({{ "/assets/images/rgbd-fig1-api-vs-ours.png" | relative_url }})

*图 1：RGB 映射到深度图的效果对比。左上为厂商 API 映射效果，左下为当时原型算法的映射效果，右上为 RGB 图，右下为深度图。深度缺失与边界不稳定会直接影响通用 API 的映射质量。*

如果目标真的是“整幅 RGB–Depth 稠密对齐”，这种 API 是自然的。  
但对我的任务来说，它至少有三个问题：

1. **它解决的是整幅图的通用稠密映射，不是少量关键点的稀疏查询。**
2. **它的质量高度依赖深度图本身的稳定性，而深度图在真实场景中经常存在边界跳变、孔洞和信息丢失。**
3. **为了得到几个点的对应关系，先计算整幅图映射场，属于明显的过度计算。**

因此，当时最关键的判断其实不是“API 好不好”，而是：

> **API 解决的不是我要解决的问题。**

这句话后来对我影响很大。很多系统级问题，并不是缺少工具，而是**工具的抽象层级与任务抽象层级并不相同**。

---

## 3. 几何模型：RGB、D、IR 不是三张图，而是一条几何链

这套方案的基础并不神秘，本质上还是一个标准的多传感器相机模型：

- RGB 相机有自己的内参与畸变；
- D 相机也有自己的内参与畸变；
- RGB 与 D 之间存在相对位姿；
- IR 与 D 在设备内部通常具有更紧的几何关联；
- 如果 IR 更适合标定，那么完全可以走 RGB–IR 这条更稳的链，再间接约束 D。

原始文档中的几何示意图如下：

![RGB-D camera geometry]({{ "/assets/images/rgbd-fig3-camera-geometry.png" | relative_url }})

*图 2：RGB-D 相机的几何模型示意。RGB、D 与 IR 不是孤立输出，而是共同处在一条跨传感器几何链中。*

### 3.1 针孔模型

对于 RGB 相机，采用标准针孔模型：

$$
\lambda
\begin{bmatrix}
 u \\
 v \\
 1
\end{bmatrix}
=
K
\begin{bmatrix}
R & t
\end{bmatrix}
\begin{bmatrix}
X \\
Y \\
Z \\
1
\end{bmatrix},
$$

其中内参矩阵为

$$
K=
\begin{bmatrix}
 f_x & s & c_x \\
 0 & f_y & c_y \\
 0 & 0 & 1
\end{bmatrix}.
$$

在代码里，这对应于 `rgbCamera.param` 与 `rgbCamera.antiParam` 的初始化，以及后续通过 `MultMatrix` 完成的反投影与投影操作。

### 3.2 径向畸变

原型代码里采用的是一个简化的径向畸变模型。若归一化像平面点为 $(x,y)$，则定义

$$
r^2=x^2+y^2,
$$

并令

$$
\delta(r)=k_1 r^2 + k_2 r^4 + k_3 r^6.
$$

于是理想到实际像点的映射可写成

$$
\begin{aligned}
 u &= u_0 + (u_0-c_x)\,\delta(r), \\
 v &= v_0 + (v_0-c_y)\,\delta(r),
\end{aligned}
$$

其中 $(u_0,v_0)$ 是不带畸变的像点。

代码中的 `ideal2real(...)` 与 `real2ideal(...)` 正是在做这层处理。它不是最复杂的反畸变实现，但足以说明：**整套方法是沿真实几何链条算的，而不是拿二维经验映射硬凑。**

### 3.3 RGB 到 D 的刚体关系

设 RGB 相机坐标系中的点为 $p_{rgb}$，D 相机坐标系中的对应点为 $p_d$，则有

$$
p_d = R_{d\leftarrow rgb} \, p_{rgb} + t_{d\leftarrow rgb}.
$$

这正对应 `relaMatrix` 和 `relaTrans`；其逆变换对应 `antiRelaMatrix` 和 `antiRelaTrans`。

所以一旦跨传感器几何关系被标定出来，所谓“RGB 点映到深度图”其实就不再是图像搬运，而是：

> **把 RGB 像点放回三维，再通过跨传感器坐标变换投到 D 相机。**

---

## 4. 标定链条：为什么除了 RGB–D，还要认真考虑 RGB–IR

当时方案里并不是只设计了一条路径，而是留了几种层次不同的做法。

### 4.1 RGB–D 联合标定

最直接的思路，是用棋盘格、平面模型和对应点，拟合：

- RGB 相机内参；
- D 相机内参；
- RGB 与 D 的相对位姿。

对应的初始化与细优化示意如下：

![Initialization model]({{ "/assets/images/rgbd-fig4-initialization-model.png" | relative_url }})

*图 3：RGB–D 联合标定的初始化示意。重点不是一开始追求极致精度，而是先建立一个足够可靠的几何起点。*

![Calibration board and plane reconstruction scene]({{ "/assets/images/rgbd-fig5-plane-calibration-board.png" | relative_url }})

*图 4：平面模型与标定场景示意。对于深度相机而言，平面通常比边界跳变强烈的对象更适合作为几何参照。*

当时我比较明确的判断是：**深度图并不天然适合直接当作高精度棋盘格角点检测的入口。**  
它会有孔洞、跳变和不稳定区域，所以如果直接在 Depth 图上强行做角点级初始化，往往很吃力。

因此在设计文档里，平面模型被放在了比较优先的位置。原因不复杂：

- 平面是最简单的流形参照；
- 深度图对平面通常比对边界跳变更稳定；
- 平面也更适合后续做几何残差和全局一致性约束。

### 4.2 RGB–IR 双目标定

现在回头看，我觉得当时方案里更成熟的一笔，其实是没有执着于“必须直接搞定 RGB–D”。

因为 IR 图像在很多设备上，比 Depth 图更自然地呈现棋盘格角点。  
于是如果 IR 与 D 在几何上天然绑定，那么就完全可以转而走 RGB–IR 双目标定，再间接约束 D。

相关图示如下：

![IR chessboard and Zense chessboard]({{ "/assets/images/rgbd-fig6-ir-chessboard-zense.png" | relative_url }})

*图 5：IR 图像中的棋盘格示意。对于这类设备，IR 常常比深度图更适合作为标定入口。*

![IR image vs RGB image of calibration board]({{ "/assets/images/rgbd-fig7-ir-vs-rgb-board.jpg" | relative_url }})

*图 6：IR 图与 RGB 图中的标定板示意。把 RGB–IR 视作更稳的双目标定路径，很多时候比直接在深度图上强行找角点更自然。*

这件事本身也很能说明问题：

> **在底层视觉系统里，好的方案不一定是“最直观”的链条，而是“最稳定的几何中介”。**

---

## 5. 核心转折：把“映射问题”改写成“几何查询问题”

如果这套方案只有标定，它还只是一个标准 3D 视觉模块。它真正让我现在回头看仍然觉得漂亮的，是在线映射部分。

### 5.1 问题的重新表述

厂商 API 想做的是：

> **建立整幅 RGB 与 D 之间的通用稠密对应关系。**

而我真正想做的是：

> **给定 RGB 上一个关键点，快速找到它在 D 图上的对应位置以及三维坐标。**

于是在线阶段的任务不再是“生成一张映射场”，而是：

1. 从一个 RGB 像点出发，生成一条 3D 视线；
2. 在工作深度范围内沿这条视线产生候选三维点；
3. 把每个候选点投到 D 相机；
4. 检查深度图是否支持这个几何假设；
5. 一旦支持，就返回对应位置与三维坐标。

这已经不是 remapping，而更像一个 **task-specific sparse geometric query**。

### 5.2 反投影：从像点到射线

给定 RGB 像点

$$
p_{rgb} = (u,v,1)^\top,
$$

先做反畸变，再乘逆内参，得到归一化像平面方向：

$$
\tilde p_{rgb}=K_{rgb}^{-1} p_{rgb}^{(undist)}.
$$

于是 RGB 相机中的视线可写成

$$
\ell(s)=o_{rgb}+\alpha(s)\,n_{rgb},
$$

其中 $o_{rgb}$ 为 RGB 光心，$n_{rgb}$ 为视线方向。

在代码里，这对应：

- `real2ideal(rgbIdealfPoint, rgbPoint, &rgbCamera)`
- `antiParam * rgbIdealfPoint -> rgbPhotoPoint`
- `getStraightLine(ray.n, ray.point, origin, rgbPhotoPoint)`

`getStraightLine(...)` 与 `getZPoint(...)` 这两个函数虽然简单，却非常关键：它们说明在线搜索不是在 2D 图像里乱找，而是**在单条三维视线的深度区间内做一维搜索**。

### 5.3 候选点投到 D 相机

对每个候选点 $X(s)$，通过刚体变换送到 D 相机坐标系：

$$
X_d(s)=R_{d\leftarrow rgb} X(s)+t_{d\leftarrow rgb}.
$$

再通过 D 相机投影模型得到其在 D 图像中的候选像素：

$$
p_d(s) = \Pi_d(X_d(s)).
$$

在代码里，这对应：

- `linearTransform(temp, sample_point, relaMatrix, relaTrans)`
- `getStraightLine(depth_ray.n, depth_ray.point, relaSample, relaDOpticCenter)`
- `getZPoint(dIdealPoint, depth_ray, 1)`
- `ideal2real(dRealPoint, dIdealPoint, &dCamera)`

从现代代码审美看，这条链可以写得更紧凑；但从几何含义上，它非常明确：

> **每个 RGB 候选点都被重新投成 D 相机中的一个像素假设。**

### 5.4 深度一致性验证

设候选点到 D 光心的预测距离为

$$
d_{pred}(s)=\|X_d(s)-o_d\|,
$$

深度图在对应像素处读出的实测深度为

$$
d_{obs}(s).
$$

原型代码里使用了一个非常直接的几何一致性残差：

$$
\varepsilon(s)=d_{pred}(s) - \big(a\,d_{obs}(s)+b\big),
$$

其中 $a,b$ 对应代码里的 `dCoeff[0], dCoeff[1]`。  
若

$$
0<\varepsilon(s)<\tau,
$$

则认为这个候选点与深度观测一致，直接返回。

代码中的核心逻辑其实就落在下面这几步：

```cpp
for (double s = zRange[0]; s < zRange[1]; s += sampleInterval)
{
    getZPoint(sample_point, ray, s);
    actual_dist = EuclidDist(relaDOpticCenter, sample_point, 3);
    linearTransform(temp, sample_point, relaMatrix, relaTrans);
    // ... project to depth camera ...
    camera_dist = (double)data;
    confidenceDegree = actual_dist - a * camera_dist - b;
    if (confidenceDegree < threshold && confidenceDegree > 0)
    {
        // accept and return
        break;
    }
}
```

因此，它真正做的不是“先重建整张 3D 场景，再去找点”，而是：

> **给定一个 RGB 点，在一条视线上连续提出 3D 假设，并让深度图来验证哪个假设成立。**

这是一种很典型的 **hypothesis generation + sensor validation** 结构。

---

## 6. 为什么它会比整图 API 快很多

现在回头看，这个速度优势其实不神秘，关键有四条。

### 6.1 稠密问题变成了稀疏问题

厂商 API 解决的是 full-frame dense mapping；  
这套原型解决的是 one-point sparse query。

两者复杂度层级本来就不同。

### 6.2 搜索空间被压缩成一维

不是在整张深度图上找，不是在整个 3D 空间里暴力找，而是只在一条视线的深度区间内采样。  
若采样步长为 $\Delta z$，则在线搜索复杂度近似为

$$
O\!\left(\frac{z_{far}-z_{near}}{\Delta z}\right).
$$

这比 full-frame remapping 的代价轻得多。

### 6.3 每个样本只做常数级计算

每一步只包含：

- 若干 3×3 线性变换；
- 一次投影；
- 一次深度读取；
- 一次残差比较。

没有整幅重采样、没有稠密映射缓存、没有整图回拉。

### 6.4 有早停

只要遇到第一个几何一致的候选点就直接返回，因此平均代价往往低于最坏代价。

所以这套方法快，不是因为我在 C++ 层面用了什么特别神奇的技巧，而是因为：

> **问题被改写成了一个更低维、更受约束、与任务真正一致的几何求值过程。**

---

## 7. 代码层面的几个注记

既然这是一篇技术博客，有些代码层面的边界也应该说清楚。

### 7.1 它是原型代码，不是整理后的展示代码

这套原型当时做得很快，后来也没有继续展开，因此代码明显带有工程原型痕迹，而不是为发表或展示精修过的版本。

例如：

- 有些资源释放并不完整；
- 某些参数和阈值是非常直接的工程设置；
- `first-hit` 的返回策略强调“快速可用”，不是全局最优；
- 某些基础函数从今天看可以写得更规整。

### 7.2 关于深度类型的一处疑点

从代码字面看，有一处值得特别留意：

```cpp
uchar data = dMat16.at<uint16_t>(dRealPoint[1], dRealPoint[0]);
```

如果 `dMat16` 真的是 `CV_16UC1` 的原始深度图，那么这里按字面会发生 16 位到 8 位的截断。  
因此这里更合理的写法应当是：

```cpp
uint16_t data = dMat16.at<uint16_t>(dRealPoint[1], dRealPoint[0]);
```

我现在已经无法完全确认这是贴代码时的笔误、某个阶段版本差异，还是当时在线分支里对深度做过另外的缩放处理。  
但把这一点写出来反而更好，因为它说明：**我现在整理它，不是为了把它包装成完美作品，而是把一条真正有价值的几何路线说清楚。**

---

## 8. 这件事后来给我留下的真正启发

现在回头看，这个项目对我真正重要的，不是“做出了一个比 API 更快的 RGB–D 点映射原型”，而是它再次证明了一件我后来越来越相信的事：

> **很多视觉问题的关键，不在于有没有现成工具，而在于是否抓住了问题真正的几何形态。**

如果问题本来是一个“关键点稀疏映射与三维恢复”问题，那么沿着整图稠密 API 修修补补，永远不如把它重新写成一条视线上的几何一致性查询来得自然。

这也是为什么后来我越来越倾向于从结构、几何和表示空间出发去组织问题，而不是停留在位图层和接口层面。  
从这个意义上说，这个原型虽然短、虽然没有继续展开，却已经很清楚地露出了我后来很多工作的同一条方法论线索：

- 先找一个更合适的几何中介；
- 把问题压到更低维、更可控的表示里；
- 再在那个结构里做求值、拟合或优化。

---

## 9. 结尾：如果不把这类工作写下来，它就真的会散掉

这套 RGB-D 标定与稀疏三维查询方案，当年没有写成论文，也没有被整理成一篇正式的技术文档。现在还能留下来的，主要只是内部设计说明、部分代码和我自己的回忆。

但我仍然觉得它值得写下来。  
因为它代表的不是某一个小功能，而是一种很典型的底层视觉判断力：

- 不迷信厂商 API；
- 先把跨传感器几何关系讲清楚；
- 先选对几何参照物；
- 再把在线问题压缩成真正贴合任务的形式。

如果要把这篇文章压缩成一句话，我更愿意这样说：

> **这不是一个“RGB-D 标定模块”的回顾，而是一条从几何建模出发，把跨传感器点对应问题改写成任务导向稀疏查询问题的底层视觉路线。**

对我来说，这类东西的价值并不在于它当年有没有继续做大，而在于：它很早就证明了一件事——**只要问题写对了，很多看似笨重的视觉流程，其实可以变得非常直接。**
