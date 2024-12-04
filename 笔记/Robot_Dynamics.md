# 机器人动力学

## 拉格朗日方程

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q_{k}}} - \frac{\partial \mathcal{L}}{\partial q_{k}} = \tau_{k}, \qquad k = 1, \cdots,n$$

拉格朗日算子$\mathcal{L} = \mathcal{K} + \mathcal{P}$, 其中$\mathcal{K}$是动能，$\mathcal{P}$是重力势能. $\tau_{k}$ 是与广义坐标$q_{k}$相关的广义力。

### 刚性连杆的动能与势能

#### 动能

刚性物体的动能是将整个物体收缩到质心而得到的**平移动能**和物体关于质心的**旋转动能**之和。单刚体的动能：
$$\mathcal{K} = \frac{1}{2}m v^\top v+\frac{1}{2}\omega^\top\mathcal{I}\omega$$
$m$为总质量，$v$和$\omega$分别为线速度和角速度，$\mathcal{I}$为惯性张量，一个$3\times 3$的对称矩阵。
**附体坐标系**(坐标系位于物体质心处)内的惯性张量可以由下式计算：
$$
I = 
\begin{bmatrix}
    I_{xx} & I_{xy} & I_{xz} \\
    I_{yx} & I_{yy} & I_{yz} \\
    I_{zx} & I_{zy} & I_{zz} \\
\end{bmatrix}$$
其中
$$
I_{xx} = \iiint(y^2+z^2)\rho(x,y,z)dxdydz \\
I_{yy} = \iiint(x^2+z^2)\rho(x,y,z)dxdydz \\
I_{zz} = \iiint(x^2+y^2)\rho(x,y,z)dxdydz \\
$$
$$
I_{xy} = I_{yx} = -\iiint xy\rho(x,y,z)dxdydz \\
I_{xz} = I_{zx} = -\iiint xz\rho(x,y,z)dxdydz \\
I_{yz} = I_{zy} = -\iiint yz\rho(x,y,z)dxdydz \\
$$

惯性张量中的对角元素$I_{xx}$,$I_{yy}$, $I_{zz}$分别为物体关于$x$，$y$以及$z$轴的主惯性矩；非对角元素被称为惯性叉积，若物体质量分布相较于附体坐标系对称，则惯性叉积为零。  
平行移轴定理描述了一个以刚体质心为原点的坐标系平移到另一个坐标系时惯性张量的变换关系。假设{C}为以2刚体质心为原点的坐标系，{A}为任意平移后的坐标系
$$
^{A}I_{xx} = ^{C}I_{xx} + m(x^{2}_{c}+y^{2}_{c}) \\
^{A}I_{xy} = ^{C}I_{xy} -mx_{c}y_{c}
$$
$x_{c}$,$y_{c}$,$z_{c}$表示刚体质心在坐标系{A}中的位置  

当考虑n连杆机械臂时任意连杆上任意一点的线速度和角速度都可以通过雅可比矩阵和关节向量的导数表示
$$
v_i = J_{v_i}(q)\dot{q} \\
\omega_i = J_{\omega_i}(q)\dot{q}
$$
当连杆$i$的质量为$m_i$，连杆$i$的惯性张量1为$I_i$，机械臂的总动能为
$$
\begin{align*}
    K &= \frac{1}{2}\dot{q}^\top[\sum^n_{i=1}\{m_iJ_{v_i}(q)^\top J_{v_i}(q)+J_{\omega_i}(q)^\top R_i(q)I_iR_i(q)^\top J_(\omega_i)(q) \}]\dot{q} \\
    &= \frac{1}{2}\dot{q}^\top D(q)\dot{q}
\end{align*}
$$

$D(q)$被称为**惯性矩阵**，是一个与位形相关的$n\times n$的对称且正定的矩阵

#### 势能

机器人的势能仅来自于重力，可以通过假设物体质量都集中在质心来计算第$i$个连杆的势能
$$
P_i = m_ig^\top r_i
$$
其中$g$是惯性坐标系中的重力向量，向量$r_i$是连杆$i$的质心坐标。该n连杆机器人的总势能为
$$
P = \sum^n_{i=1}P_i = \sum^n_{i=1}m_ig^\top r_i
$$

## 牛顿-欧拉法

### 牛顿方程
作用在质心上的力$F$引起的刚体加速度为
$$
F = m \dot{v}_c
$$

### 欧拉方程
作用在质心上的扭矩$N$引起刚体的转动为
$$
N= I\dot{\omega}+\omega \times I \omega
$$

### 动力学方程
对于$i+1$个关节的角速度公式：
$$
\omega_{i+1} =_{i}^{i+1}R\omega_{i} + \dot{\theta}_{i+1}\hat{Z}_{i+1}
$$
对于$i+1$个关节的角加速度公式：
$$
\dot{\omega}_{i+1} =_{i}^{i+1}R\dot{\omega_{i}} + _{i}^{i+1}R\omega_{i}\times\dot{\theta}_{i+1}\hat{Z}_{i+1} + \ddot{\theta}\hat{Z}_{i+1}
$$
对于$i+1$个关节关于其关节原点坐标系的线加速度公式：
$$
\dot{v}_{i+1} = ^{i+1}_{i}R[\dot{\omega}_i\times^{i}P_{i+1}+\omega _i\times(\omega _i\times^{i}P_{i+1}+\dot{v}_i)]
$$
对于$i+1$个关节关于其质心/附加坐标系的线加速度公式：
$$
\dot{v}_{i+1} = \dot{\omega}_i\times P_{C_i} + \omega_i\times(\omega _i \times P_{C_i}) + \dot{v}_i
$$
其中，上述公式中$_{i}^{i+1}R$为关节$i$到$i+1$的变换矩阵，$P$为附体坐标系的位置矩阵。  
在得到各关节质心上的线加速度和角加速度后，便可利用牛顿欧拉公式计算连杆质心上的惯性力和力矩，再通过得到的惯性力和力矩，计算对应的关节力矩。在这里，可以设  
$f_i$ 为连杆$i-1$作用在连杆$i$上的力  
$n_i$ 为连杆$i-1$作用在连杆$i$上的力矩  

将作用在连杆$i$上的力相加，得到力平衡方程：
$$
f_i = F_i + ^{i}_{i+1}RF_{i+1}
$$
将作用在质心上的力矩相加，并令其等于零，得到力矩平衡方程
$$
n_i = N_i + ^{i}_{i+1}Rn_{i+1} + P_{C_i}\times F_i + P_{i+1}\times  ^{i}_{i+1}Rf_{i+1}
$$
最后，可以由下式计算一个连杆施加于相邻两案的力矩再$\hat{Z}$方向的分量求得关节力矩
$$
\tau _i = n_i^{\top}\hat{Z}_i
$$

## 运动方程

$$
Q = M(q)\ddot{q}+C(q,\dot{q})\dot{q}+F(\dot{q})+G(q)+J(q)^{\top}f \\
$$
正向动力学
$$
\ddot{q} = M^{-1}(q)(Q-C(q,\dot{q})\dot{q} - F(\dot{q}) - G(q))
$$
其中$M$是关节空间惯量矩阵（惯性张量?），$C$是科氏力和向心力耦合矩阵，$F$为摩擦力，$G$是重力载荷，$Q$是与广义坐标$q$对应的广义驱动力向量，$J$是雅可比矩阵。
### 动态可操纵性
$$
\dot{v}^{\top}(JM^{-1}M^{-\top}J^{\top})^{-1}\dot{v} = 1
$$
对这个超椭球取左上角$3\times 3$子矩阵，可用来评判机械臂在三维的加速能力。

### 传动
#### 电机
动力模型：
$$
J_m\dot{\omega}+B\omega+\tau _c(\omega) = K_mK_au
$$
其中$u$为控制电压，$K_a$为放大器跨导，$K_m$为电机扭矩常数，$J_m$为电机总惯量，$B$为粘性摩擦系数，$\tau _c$为库仑摩擦力矩
从电机来看，关节$i$的总惯量为$^{m}J_i = J_{m_i} + \frac{1}{G^2_i}M_{ii}$

#### 摩擦
![示意](C:/Users/18311/Desktop/毕设/遥控机器人/笔记/摩擦.png "示意")
摩擦常采用以下模型进行模拟$Q_f = B\dot{q}+Q_c$,其中$B$为粘性摩擦系数，$Q_c$为库仑摩擦力。

### 控制
#### 前馈
力矩前馈控制器
$$
Q^* = Q + K_v(\dot{q}^* - \dot{q}) + K_p(q^* - q)
$$
$K_v$为速度增益，$K_p$为位置增益，$q^*$为期望机械臂状态