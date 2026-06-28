---
title: PID 控制理论 — 从数学推导到工程实践
author: finnyoun9
date: 2026-06-28
---

# PID 控制理论

> 对齐江科大 17 课教程，从数学公式到 C 代码实现，覆盖面试所有考点。

---

## 1. 基本公式

### 1.1 连续形式

PID 控制器的输出 $u(t)$ 由三项组成：

$$u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) d\tau + K_d \cdot \frac{de(t)}{dt}$$

其中：

| 符号 | 含义 |
|------|------|
| $e(t) = r(t) - y(t)$ | 误差 = 目标值 − 实际值 |
| $K_p$ | 比例增益（Proportional） |
| $K_i$ | 积分增益（Integral） |
| $K_d$ | 微分增益（Derivative） |
| $u(t)$ | 控制器输出（如 PWM 占空比） |

### 1.2 离散形式（实际程序用的）

嵌入式系统用定时中断周期性采样，离散化后：

$$u(k) = K_p \cdot e(k) + K_i \sum_{n=0}^{k} e(n) \cdot \Delta t + K_d \cdot \frac{e(k) - e(k-1)}{\Delta t}$$

对应 C 代码：

```c
error = target - actual;
integral += error * dt;
derivative = (error - last_error) / dt;
output = Kp * error + Ki * integral + Kd * derivative;
last_error = error;
```

---

## 2. 三种 PID 形式

### 2.1 位置式 PID（Positional）

输出直接是控制量的**绝对值**。

$$u(k) = K_p e(k) + K_i \sum e(k) + K_d [e(k) - e(k-1)]$$

| 优点 | 缺点 |
|------|------|
| 直观，直接对应物理量 | 积分饱和 (windup) |
| 适合执行器本身有"记忆"的系统 | 模式切换时输出跳变 |

**适用场景**：电机位置控制、温度控制、倒立摆角度环

### 2.2 增量式 PID（Incremental）

输出的是控制量的**变化量**。

$$\Delta u(k) = K_p [e(k)-e(k-1)] + K_i e(k) + K_d [e(k)-2e(k-1)+e(k-2)]$$

$$u(k) = u(k-1) + \Delta u(k)$$

| 优点 | 缺点 |
|------|------|
| 天然抗积分饱和 | 需要保存 3 个历史误差 |
| 模式切换平滑 | 对执行器要求高（需积分特性） |

**适用场景**：电机速度控制（电机本身是积分环节）、步进电机

### 2.3 微分先行 PID（Derivative-on-Measurement）

不对误差微分，只对测量值微分：

$$u(k) = K_p e(k) + K_i \sum e(k) - K_d [y(k) - y(k-1)]$$

| 优点 | 缺点 |
|------|------|
| 避免设定值突变导致"微分冲击" | 响应稍慢于标准形式 |

---

## 3. 三个参数的作用

### 3.1 比例项 P — 响应速度

$$u_P = K_p \cdot e(t)$$

| $K_p$ ↑ 效果 | $K_p$ ↓ 效果 |
|:---|:---|
| 响应更快 | 响应变慢 |
| 稳态误差减小（但不能消除） | 稳态误差增大 |
| 过大→震荡 | 过小→迟钝 |

**核心矛盾**：$K_p$ 永远无法消除稳态误差。

**为什么？**
设稳态时 $y = y_{ss}$，则 $e = r - y_{ss} \neq 0$。
此时 $u = K_p \cdot e$，如果 $u$ 刚好等于维持 $y_{ss}$ 所需的控制量，系统就平衡了——但 $e \neq 0$，存在稳态误差。

### 3.2 积分项 I — 消除稳态误差

$$u_I = K_i \int e(\tau) d\tau$$

| $K_i$ ↑ 效果 | $K_i$ ↓ 效果 |
|:---|:---|
| 更快消除稳态误差 | 消除慢 |
| 过大→超调、震荡 | 过小→有残余误差 |

**为什么能消除稳态误差？**
只要 $e > 0$，积分项就持续增长，直到 $e = 0$。这就是"不达目标不罢休"。

**副作用 — 积分饱和 (Integral Windup)**：
当执行器饱和（如 PWM 已经在 100%）时，误差仍然存在，积分项继续累加。
等到误差反号时，积分项已经大得离谱，导致巨大超调。

### 3.3 微分项 D — 预测与阻尼

$$u_D = K_d \cdot \frac{de(t)}{dt}$$

| $K_d$ ↑ 效果 | $K_d$ ↓ 效果 |
|:---|:---|
| 抑制超调 | 超调增大 |
| 提供"阻尼" | 震荡加剧 |
| 过大→放大噪声 | — |

**D 的本质**：根据误差变化趋势提前动作。

当 $e$ 正在快速减小时（$de/dt < 0$），D 项为负 → 提前"刹车"。
这就是 D 能抑制超调的原因——不等实际值超过目标就已经开始减速。

---

## 4. 抗积分饱和 (Anti-Windup)

### 4.1 积分限幅 (Clamping)

最直接的方法：限制积分项的范围。

```c
integral += error * dt;
if (integral > max_integral) integral = max_integral;
if (integral < min_integral) integral = min_integral;
```

### 4.2 积分分离 (Integral Separation)

大误差时不积分，小误差时才积分。

```c
if (abs(error) < threshold) {
    integral += error * dt;  // 小误差 → 积分
} else {
    integral = 0;             // 大误差 → 只用 PD
}
```

### 4.3 变速积分 (Variable-Speed Integration)

误差大时积分慢，误差小时积分快。

$$f(e) = \begin{cases} 1 & |e| \leq B \\ \frac{A - |e| + B}{A} & B < |e| \leq A+B \\ 0 & |e| > A+B \end{cases}$$

$$integral += f(e) \cdot e \cdot dt$$

### 4.4 条件积分 (Conditional Integration)

只有输出未饱和，或积分方向正确时才积分。

```c
if (out_min < last_out && last_out < out_max) {
    integral += error * dt;           // 输出在范围内 → 正常积分
} else if ((last_out <= out_min && error > 0) ||
           (last_out >= out_max && error < 0)) {
    integral += error * dt;           // 积分方向正确 → 可以积分
}
// 否则：不积分（积分方向会把输出推向更深饱和）
```

---

## 5. 调参方法

### 5.1 手动调参法

```
Step 1: Ki=0, Kd=0, 逐步增大 Kp 直到出现轻微震荡
Step 2: Kp 减半, 逐步增大 Ki 直到稳态误差消除
Step 3: 逐步增大 Kd 直到超调被抑制、响应满意
```

### 5.2 Ziegler-Nichols 法

1. Ki=0, Kd=0，增大 Kp 直到系统**持续等幅震荡**
2. 记录此时的 Kp = $K_u$（临界增益），震荡周期 = $T_u$
3. 查表：

| 控制器 | $K_p$ | $K_i$ | $K_d$ |
|--------|-------|-------|-------|
| P | $0.5K_u$ | — | — |
| PI | $0.45K_u$ | $0.54K_u/T_u$ | — |
| PID | $0.6K_u$ | $1.2K_u/T_u$ | $0.075K_u \cdot T_u$ |

### 5.3 调参口诀

```
参数整定找最佳，从小到大顺序查
先是比例后积分，最后再把微分加
曲线振荡很频繁，比例度盘要放大
曲线漂浮绕大湾，比例度盘往小扳
曲线偏离回复慢，积分时间往下降
曲线波动周期长，积分时间再加长
曲线振荡频率快，先把微分降下来
动差大来波动慢，微分时间应加长
```

---

## 6. 串级 PID (Cascaded PID)

### 6.1 为什么需要串级？

单环 PID 直接控制被控量，响应慢。串级 PID 用内环控制"快变量"，外环控制"慢变量"。

**例：平衡车直立控制**

```
外环（角度环）: 目标角度=0° → 输出=目标角速度
内环（角速度环）: 目标角速度=外环输出 → 输出=PWM
```

### 6.2 数学形式

外环：
$$u_{outer} = K_{p,outer} \cdot e_{angle} + K_{i,outer} \int e_{angle} + K_{d,outer} \cdot \frac{de_{angle}}{dt}$$

内环：
$$u_{inner} = K_{p,inner} \cdot (u_{outer} - y_{gyro}) + K_{i,inner} \int (u_{outer} - y_{gyro})$$

### 6.3 关键规则

> **内环频率 ≥ 5× 外环频率**

平衡车典型参数：
- 内环（角速度）：1kHz
- 外环（角度）：200Hz

---

## 7. 评价指标

| 指标 | 公式 | 含义 |
|------|------|------|
| 超调量 | $\sigma = \frac{y_{max} - r}{r} \times 100\%$ | 超过目标多少 |
| 调节时间 | $t_s$ (settling time) | 进入 ±2% 的时间 |
| 稳态误差 | $e_{ss} = r - y(\infty)$ | 最终偏差 |
| IAE | $\int |e(t)| dt$ | 绝对误差积分 |
| ISE | $\int e^2(t) dt$ | 平方误差积分（惩罚大误差） |
| ITAE | $\int t|e(t)| dt$ | 时间加权（惩罚后期误差） |

---

## 8. 代码实现对照

### STM32 C（位置式，带抗饱和）

```c
typedef struct {
    float Target, Actual, Out;
    float Kp, Ki, Kd;
    float Error0, Error1, ErrorInt;
    float OutMax, OutMin;
} PID_t;

void PID_Update(PID_t *p) {
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;       // e(k)
    
    if (p->Ki != 0)                           // 抗积分饱和
        p->ErrorInt += p->Error0;
    else
        p->ErrorInt = 0;
    
    p->Out = p->Kp * p->Error0               // P
           + p->Ki * p->ErrorInt             // I
           + p->Kd * (p->Error0 - p->Error1);// D
    
    // 输出限幅
    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;
}
```

### Python（三种 PID 对比）

```python
# 位置式
class PID:
    def update(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return Kp * error + Ki * self.integral + Kd * derivative

# 抗积分饱和（钳位法）
class PID_AntiWindup:
    def update(self, measurement, dt):
        error = self.setpoint - measurement
        p_term = Kp * error
        d_term = Kd * (error - self.last_error) / dt
        # 只有不饱和时才积分
        if out_min < self.last_out < out_max:
            self.integral += error * dt
        out = p_term + Ki * self.integral + d_term
        self.last_out = clip(out, out_min, out_max)

# 增量式
class PID_Incremental:
    def update(self, measurement, dt):
        error = self.setpoint - measurement
        delta = (Kp * (error - self.last_error) +
                 Ki * error * dt +
                 Kd * (error - 2*self.last_error + self.prev_error) / dt)
        self.last_out = clip(self.last_out + delta, out_min, out_max)
```

---

## 9. 面试高频问题

1. **什么是 PID 控制？P、I、D 各自的作用？**
2. **位置式 PID 和增量式 PID 的区别？什么场景用哪个？**
3. **什么是积分饱和 (Integral Windup)？如何解决？**（至少说 3 种方法）
4. **微分先行有什么用？**
5. **什么是不完全微分？**
6. **为什么要用串级 PID？内外环频率关系？**
7. **Ziegler-Nichols 调参法的步骤？**
8. **评价一个控制系统好坏的指标有哪些？**
9. **从数学上推导为什么纯 P 控制会有稳态误差？**
10. **在 STM32 上实现 PID，dt 怎么确定？如果 dt 不稳定怎么办？**

---

> 📚 继续学习：配合江科大 17 课视频，每课对应代码在 `from-mac/PID入门教程资料/程序源码/`
> 🔬 动手实验：`python3 from-pi/pid_tune.py <plant> <Kp> <Ki> <Kd>`
> 💻 真机验证：接上 STM32 PID 套件，烧录 Stage 01-16 代码
