# 控制

## 滑模控制

### Lyapunov 稳定性分析

**平衡点**: 系统状态不再变化的点。
**稳定**: 平衡状态$x_e$受到扰动后仍然停留在$x_e$附近，则称$x_e$在李雅普诺夫意义下是稳定的（Lyapunov stable）。
**渐进稳定**: 平衡状态$x_e$受到扰动后，最终都会收敛到$x_e$，则称$x_e$在李雅普诺夫意义下是渐进稳定的（Asymptotically stable）。
**大范围内渐进稳定的**: 如果平衡状态$x_e$受到任何扰动后，最终都会收敛到$x_e$，我们就称$x_e$在李雅普诺夫意义下是大范围内渐进稳定的（Asymptotically stable in large）。

### Lyapunov 第二法

Lyapunov 第二法通过虚构一个包含状态向量$x$和时间$t$的能量函数来分析稳定性，记为$V(x,t)$或者$V(x)$。$V(x)$是一个标量函数，且考虑到能量总大于零，故$V(x)$为正定函数。能量衰减特性用$\dot{V}(x)$表示。当$V(x)$满足：

$$
V(x) = 0 \quad \text{if and only if} \quad  x = 0 \\
V(x) > 0 \quad \text{if and only if} \quad  x \neq 0 \\
\dot{V}(x) = \frac{\rm d}{{\rm d}t}V(x) = \sum^{n}_{i=1}\frac{\partial V}{\partial x_i}f_i(x) \leq 0 \quad \text{when} \quad x \neq 0\\
$$

则称系统在李雅普诺夫意义下是稳定的，特别的，若 $x \neq 0 $ 时，有 $\dot{V}(x) < 0$，则系统是渐进稳定的。

### 滑模控制

对于非线性系统$\dot{x} = f(x,u.t)$，其中 $\bf{x} \in \bf{R}^n$ ，$\bf{u} \in \bf{R}^m$， $\bf{t} \in \bf{R}$。在其状态空间中，有一超平面$s(x) = 0$, 两侧的点当接近超平面时都会趋于该区域运动，当在超平面上某一段区域全部点都为终止点时，该区域被称作滑动模态区，即滑模区。对于非线性系统$\dot{x} = f(x,u.t)$，滑模控制问题可被归结于两点：

1. 确定滑移面$s(x)$
2. 设计控制率 $u =
\begin{cases}
u^+(x) \qquad s>0 \\
u^-(x) \qquad s<0 \\
\end{cases}$

滑模控制需要满足的两个条件：

1. 保证滑膜运动存在
状态变量能够从任意初始状态趋近滑移面 $s = 0$ 是滑模运动的一个非常关键的部分。与此同时，系统运动点在滑移面 $s = 0$ 两侧小幅度快频率的来回运动。使得滑动模态区滑模存在的表达式为：$\lim_{s\rightarrow 0}s\dot{s} < 0$。

2. 滑模运动可达性
可达性条件可叙述为除了滑模面上的任意运动点全部可在限定时间内到达滑模面 。根据李雅普洛夫(Lyapunov)稳定性理论，可达性表达式为：$$
\begin{cases}
    V(x) = \frac{1}{2}s^2 \\
    \dot{V}(x) = s\dot{s} < 0\\
\end{cases}$$

### 滑模面设计

1. 线性滑移面
$$s = x_1 + \beta x_2 = 0 \qquad 0<\beta$$
线性滑模系统状态变量不断趋近于给定值，但是与给定值存在较大误差。状态变量靠近切换面后以指数函数速度收敛至 0。如若状态变量与平衡位置距离较大时，收敛速度比较快，反之当其与平衡位置距离较小时，收敛速度相对较慢。

2. Terminal滑模面
$$ s = x_2 + \beta x_1^{q/p} = 0 \qquad \beta>0, p > q > 0$$
式中，p 和 q 都为正奇数。系统状态变量趋近滑模面 $s = 0$ 时，能够通过限定的时间收敛为0。

3.非奇异Terminal滑模面
$$s = x_1 + \frac{1}{\beta}x_2^{q/p} \qquad z<p/q<2,0<\beta$$
式中，p 和 q 都为奇数且为正值。此滑模面解决了终端滑模奇异性的缺点，而且拥有良好的收敛性能，能够在有限时间内收敛至给定位置。但是在接近滑模面时的速度仍会较慢。

4.快速Terminal滑模面
$$s = x_2 +cx_1 + \beta x_1^{q/p} \qquad \qquad z<p/q<2,0<\beta,0<c$$
式中，p和q都为正奇数。如若状态变量与平衡位置距离较大时，收敛速度比较快，此时可视为线性滑动模态。如若其与平衡位置距离较小时，此时可视为终端滑模，因此具有终端滑模的特征。总而言之，该滑动模态式包含了上述滑模面的优点，从而使得整个系统具有全局快速收敛性。

5.积分滑模面
$$
s = k_px + k_i \int^t_0x \rm d t = 0 \qquad0<k_p,0<k_i
$$
滑模控制在趋近运动过程中时，线性系统对不确定因素不具备鲁棒性，因此引入积分器来抑制稳态误差。该方法增强了系统鲁棒性却以牺牲滑模系统的降阶功能为代价。

### 高阶滑模控制率

$$
\dot{u} =
\begin{cases}
    -u \qquad |u| > 1\\
    -V_msign(\sigma) \qquad \sigma \cdot \dot\sigma \leq 0, |u|\leq 1 \\
    -V_Msign(\sigma) \qquad \sigma \cdot \dot\sigma \> 0, |u|\leq 1 \\
\end{cases}
$$
其中$\sigma$为滑模量。

### 削弱滑模控制抖振

1. 单位矢量控制法
该方法选择非线性函数$v_{\sigma}(s)$替换控制量中的符号函数$sign(s)$。其表达式如下所示：
$$v_{\sigma}(s) = \frac{s}{|s|+\sigma}, \sigma > 0$$
该方法为高增益反馈，如若状态变量与滑模面$s= 0$距离比较大时，参数$\sigma$可以迅速的使状态变量到达滑模面。

2. 边界层法
边界层法在所有抑制抖振的策略中是相对比较容易实现的。该方法控制系统状态变量的运行轨迹为期望的某个区域，一般把这个区域作为滑模面的设定边界层即$\delta$，其既能取值为常数，又能进行自适应调节。选择饱和函数$sat(\frac{s}{\delta})$代替控制量中的符号函数$sign(s)$ ，$sat(\frac{s}{\delta})$的表达式如下所示：

$$
sat(\frac{s}{\delta}) =
\begin{cases}
    1 \qquad s> \delta\\
    \frac{1}{\delta} \qquad |s|<\delta\\
    -1 \qquad s< - \delta \\
\end{cases}
$$
如若状态变量不在边界层内部，其运行轨迹无异于普通滑模；如若状态变量运行至边界层内部，来回切换运动能够在边界层上实现，而不需要在滑模面上实现。此方法能够抑制系统固有抖振，达到了不连续控制连续化的目的，却以控制性能为代价

3. 趋近律法
通过改变趋近律参数值以抑制系统抖振。迄今为止，通常采用指数趋近律法，即 $\dot{s} = -\varepsilon sign(s) - \lambda \quad 0<\varepsilon, 0<\lambda $ ，参数$\varepsilon$
表示到达滑模面的趋近速度，如若状态变量距离滑模面相对较远时，通过减小$\varepsilon$和增大$\lambda$能够实现系统快速响应及削弱抖振。

## 反演控制

反步法的本质也是利用李雅普诺夫第二法对系统控制器进行状态反馈设计。通过对系统的每一个状态方程依次进行迭代设计，最终串联成一个控制方案。

考虑一个$n$阶单输入单输出系统：
$$
\begin{cases}
    x_1^{\prime} = x_2 + f_1(x_1) \\
    x_2^{\prime} = x_3 + f_2(x_1,x_2) \\
    \quad \cdots \\
    x_n^{\prime} = u + f_n(x_1,x_2,\cdots,x_n) \\
\end{cases}
$$
对每个$x_n^{\prime}$，可以将式中的$x_{n+1}$看作一个虚拟输入。设目标值为$x_d$，误差为$e_i$，$\alpha _i$为状态变量$x_{i+1}$的目标值（虚拟控制项），可以推导出误差向量组：
$$
\begin{cases}
    e_1 = x_1 - x_d\\
    e_2 = x_2 - \alpha_1 \\
    \cdots \\
    e_n = x_n = \alpha_{n-1}\\
\end{cases}
$$
由数学归纳法可知 
$$
u = \alpha_n = -e_{n-1} - e_n - f_n(x_1,x_2,\cdots,x_{n-1}) - \alpha_{n-1}^{\prime}
$$



## 自适应控制

### 自校正控制器（STC）

STC可以看作参数估计器+控制器，下图为STC的结构：
<div align="center"> 
<img src="C:/Users/18311/Desktop/毕设/遥控机器人/笔记/STC_Structure.png" height = 150 /> 
</div>
参数估计器可利用如下方法构成：递推最小二乘法（RLS）、快速仿射投影算法（FAP）、最小均方误差（MSE）、卡尔曼滤波（KF）、扩展卡尔曼滤波（EKF）。
可以构成STC的常见控制器：比例积分微分控制器（PID）、滑模控制（SMC）、模糊控制、神经网络、遗传算法、预测控制（PC）、二次型最优控制（LQR）、时间延迟控制（TDC）、基于不确定扰动估计器（UDE）的控制器。

### 模型参考自适应控制（MRAC）

MRAC可以看作是 参考模型+控制器+自适应率，下图为MRAC的结构：
<div align="center">
<img src="C:/Users/18311/Desktop/毕设/遥控机器人/笔记/MRAC_Structure.png" height = 200 />
</div>


#### 参考模型的设计
假设被控对象的状态方程和输出方程如下所示：
$$
\begin{cases}
    \dot{x}(t) = Ax(t) + Bu(t)\\
    y(t) = x(t)\\
    \tag{1}
\end{cases}
$$

参考模型的状态方程和输出方程可构造为如下所示：
$$
\begin{cases}
    \dot{x}_m(t) = A_mx_m(t) + B_mc(t)\\
    y_m(t) = x_m(t)\\
    \tag{2}
\end{cases}
$$

#### 控制器的设计

将公式 1 与 公式 2 相减：
$$
\dot{e}(t) = A_mx_m(t) + B_mc(t) - Ax(t) - Bu(t) \\
\Downarrow \text{化简} \tag{3} \\
\dot{e}(t) = A_me(t) + [A_mx(t) + B_mc(t) - Ax(t) - Bu(t)] \\
$$
如果误差$e$为零,系统需要满足矩阵$A_m$特征根大小为负且令下式控制器输出$u(t)$满足如下：
$$
u(t) = ac(t) + bx(t)
\tag{4}
$$

#### 自适应律的设计

由于控制器$u(t)$中的参数$a$、$b$未知，便有了自适应率的出现。自适应率的设计目的就是为了在线辨识出控制器中的这两个未知参数。

1. 梯度法

假设MRAC控制器中含有一个未知可调节的参数$\theta$，定义参考模型和被控对象的状态变量偏差如下：$e = x_m -x$。目标是通过调节参数$\theta$来令偏差$e$最小,这里引入一个损失函数$J = \frac{1}{2}e^2$.如果令$e \rightarrow 0$，则$J \rightarrow 0$。也就是需要求得$J$的最小值。因此沿着$J$
的负梯度方向变化参数$\theta$：
$$
\begin{cases}
    \Delta\theta = -\kappa \frac{\partial J}{\partial \theta} = -\kappa e\frac{\partial e}{\partial \theta}\\
    \dot{\theta} = -\gamma \frac{\partial J}{\partial \theta} = -\gamma e\frac{\partial e}{\partial \theta}\\
    \tag{5}
\end{cases}
$$
其中，$\Delta\theta$为两个步长的差值，$\kappa$为学习步长,$\gamma$为调整速率。
利用公式1,公式4重写到状态变量偏差$e$，并引入积分算子$p$：
$$
e(t) = x_m(t)- \frac{Ba}{p - A -Bb}c(t)
\tag{6}
$$

将公式5,公式6带入到公式4中，可得到：
$$
\begin{cases}
    \dot{a}(t) = \gamma e(t)\frac{B}{p - A - Bb}c(t) \\
    \dot{b}(t) = -\gamma e(t)\frac{B}{p - A - Bb}x(t) \\
    \tag{7}
\end{cases}
$$
$$\Downarrow \text{化简}\\$$
$$
\begin{cases}
    \dot{a}(t) = \gamma e(t)\frac{B}{p + A_m}c(t) \\
    \dot{b}(t) = -\gamma e(t)\frac{B}{p + A_m}x(t) \\
    \tag{8}
\end{cases}
$$

2. 稳定性理论分析法

参考模型和被控对象的状态变量偏差的变化率可以表示为如下所示：
$$
\dot{e} = \dot{x}_m(t) - \dot{x}(t) \\
\Downarrow \text{带入公式1、2、4}\\
\dot{e}(t) = A-me(t) + [(A_m - A -Bb)x(t) + (B_m - ba)c(t)]
\tag{9}
$$
当矩阵$A_m$特征根为负且下式成立时，误差将收敛为零：
$$
e_1 = A_m - A - Bb = 0 \\
e_2 = B_m - B_a = 0
$$
如果需要系统稳定，需要同时满足$e$、$e_1$、$e_2$都收敛为零,因此构建标量函数$V(e,e_1,e_2)$:
$$
\begin{align*}
    V &= \frac{1}{2}[e^2 + \frac{1}{B\gamma}(A_m - A - Bb)^2 + \frac{1}{B\gamma}(B_m - B_a)^2]\\
      &= \frac{1}{2}(e^2 + \frac{1}{B \gamma}e_1^2 + \frac{1}{B \gamma}e_2^2)\\
    \dot{V} &= e\frac{\rm d e}{\rm d t} + \frac{1}{B\gamma}e_1\frac{\rm d e_1}{\rm d t} + \frac{1}{B\gamma}e_2\frac{\rm d e_2}{\rm d t}\\
    &=A_me^2(t)+ \frac{e_1}{\gamma}[\gamma e(t)x(t) - \frac{db(t)}{dt}] + \frac{e_2}{\gamma}[\gamma e(t)c(t - \frac{da(t)}{dt})]
\end{align*}
$$
因为$A_me^2(t)<0$，为使$\dot{V}(e,e_1,e_2)<0$，则可得到下式：
$$
\frac{db(t)}{dt} = \gamma e(t)x(t) \\
\qquad\\
\frac{da(t)}{st} = \gamma e(t)c(t)\\
$$

## H $\infty$ 控制

H$\infty$控制是专门针对一类不确定性系统的控制方法，用来削弱外部扰动带来的影响，本质也是状态反馈，通过设计一个状态反馈矩阵K使闭环系统稳定的同时，让外部干扰对系统期望输出影响最小。
![Hinfinity](C:/Users/18311/Desktop/毕设/遥控机器人/笔记/Hinfinity.png "Hinfinity")
$$
\begin{cases}
    \dot{x} = Ax + (B_1 & B_2)\begin{pmatrix}
        w \\
        u \\
    \end{pmatrix}\\
    \begin{pmatrix}
        z \\
        y\\
    \end{pmatrix}
    = 
    \begin{pmatrix}
        C_1 \\
        C_2 \\
    \end{pmatrix} x + 
    \begin{pmatrix}
        D_{11} & D_{12} \\
        D_{21} & D_{22}\\
    \end{pmatrix}
    \begin{pmatrix}
        w \\
        u \\
    \end{pmatrix}\\
\end{cases}
$$

### 状态反馈H$\infty$控制

当系统的状态变量可以获得时，常采用状态反馈H$\infty$控制。
状态反馈控制分为标准状态反馈H$\infty$控制、次优状态反馈H$\infty$控制、最优状态反馈H$\infty$控制三种，本质是求解LMI不等式矩阵得到的控制矩阵K。下面是三种状态反馈H$\infty$控制的LMI不等式矩阵：

#### 标准状态反馈H$\infty$控制

$$
\left[
\begin{matrix}
    AX + B_2W + (AX + B_2W)^T & B_1 & (C_1X + D_{12}W)^T\\
    B_1^T & -I & D_{11}^T \\
    C_1X+D_{12}W & D_{11} & -I\\
\end{matrix}
\right] <0
$$

#### 次优状态反馈H$\infty$控制

$$
\left[
\begin{matrix}
    AX + B_2W + (AX + B_2W)^T & B_1 & (C_1X + D_{12}W)^T\\
    B_1^T & -I & D_{11}^T \\
    C_1X+D_{12}W & D_{11} & -\gamma^2I\\
\end{matrix}
\right] <0
$$

#### 最优状态反馈H$\infty$控制

$$
\left[
\begin{matrix}
    AX + B_2W + (AX + B_2W)^T & B_1 & (C_1X + D_{12}W)^T\\
    B_1^T & -I & D_{11}^T \\
    C_1X+D_{12}W & D_{11} & -\rho^2I\\
\end{matrix}
\right] <0
$$

### 输出反馈H$\infty$控制
  当系统的状态变量不可获得时，可采用输出反馈H$\infty$控制。此时输出反馈的LMI不等式矩阵中包含了未知的控制器参数，所以无法直接定义LMI矩阵，需要使用消元法和变量替代法消去：

$$
\left[
    \begin{matrix}
        N_0 & 0\\
        0 & I \\
    \end{matrix}
\right]
\left[
    \begin{matrix}
        A^TX + XA & XB_1 & C_1^T\\
        B_1X & -\gamma I & D_{11}^T \\
        C_1 & D_{11} & -\gamma I \\
    \end{matrix}
\right]
\left[
    \begin{matrix}
        N_0 & 0\\
        0 & I \\
    \end{matrix}
\right] < 0\\
$$
$$
\left[
    \begin{matrix}
        N_c & 0\\
        0 & I \\
    \end{matrix}
\right]
\left[
    \begin{matrix}
        AY + YA^T & YC_1^T & B_1\\
        C_1Y & -\gamma I & D_{11} \\
        B_1^T & D_{11}^T & -\gamma I \\
    \end{matrix}
\right]
\left[
    \begin{matrix}
        N_c & 0\\
        0 & I \\
    \end{matrix}
\right] < 0\\
$$
$$
\left[
    \begin{matrix}
        X & I\\
        I & Y\\
    \end{matrix}
\right] \geq 0\\
$$
