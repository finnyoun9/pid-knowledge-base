# PID Control Knowledge Base

> 🔥 三台机器（Ubuntu Linux / 树莓派 Debian / macOS）的 PID 控制理论 + 实战代码全汇总
> 
> STM32F103C8T6 · 平衡车串级 PID · 无刷电机 FOC · Python 仿真 · 面试训练

---

## 📂 仓库结构

```
pid-knowledge-base/
├── from-linux/              ← Ubuntu 本机（符号链接）
│   ├── balance-car-source   → 平衡车程序源码（9阶段）
│   ├── pid-tutorial-source  → PID入门教程-程序源码
│   ├── motor-pid-migration-plan.md → 差速底盘 PID 迁移计划
│   └── embedded-notes       → 嵌入式学习笔记
│
├── from-mac/                ← macOS（完整拷贝）
│   └── PID入门教程资料/
│       ├── 程序源码/        → 17个递进式 PID 教程
│       ├── 基础程序/
│       ├── 原理图等文档/    → PDF 教程/用户手册
│       └── 工具软件/
│
├── from-pi/                 ← 树莓派（完整拷贝）
│   ├── pid_sim.py           → Python PID 仿真工具（3种PID+4种植物）
│   ├── pid_tune.py          → CLI 调参工具
│   ├── stm32-pid-motor/     → STM32F103 裸机寄存器 PID 电机控制
│   ├── pid-control-skill/   → Hermes Agent PID 技能
│   └── pid_*.png            → 7张 PID 可视化图表
│
└── README.md                ← 本文件
```

---

## 🎯 PID 知识全景

### 一、PID 基础算法

| 算法 | 实现位置 | 语言 |
|------|---------|------|
| 位置式 PID | `from-linux/balance-car-source/*/User/PID.c` | C |
| 位置式 PID | `from-mac/PID入门教程资料/程序源码/02/` | C |
| 位置式 PID | `from-pi/stm32-pid-motor/main.c` | C (裸机寄存器) |
| 位置式 PID | `from-pi/pid_sim.py` → `class PID` | Python |
| 增量式 PI | `from-linux/motor-pid-migration-plan.md` (plan) | C |
| 增量式 PID | `from-mac/PID入门教程资料/程序源码/03,05/` | C |
| 增量式 PID | `from-pi/pid_sim.py` → `class PID_Incremental` | Python |

### 二、抗积分饱和

| 方法 | 实现位置 |
|------|---------|
| Ki≠0 才积分 | `from-linux/balance-car-source/*/User/PID.c` |
| 积分限幅 (clamping) | `from-mac/PID入门教程资料/程序源码/06/` |
| 积分分离 | `from-mac/PID入门教程资料/程序源码/07/` |
| 变速积分 | `from-mac/PID入门教程资料/程序源码/08/` |
| 钳位法 anti-windup | `from-pi/pid_sim.py` → `class PID_AntiWindup` |

### 三、微分处理

| 方法 | 实现位置 |
|------|---------|
| 微分先行 | `from-mac/PID入门教程资料/程序源码/09/` |
| 不完全微分 | `from-mac/PID入门教程资料/程序源码/10/` |

### 四、实用技巧

| 技巧 | 实现位置 |
|------|---------|
| 输出偏移 | `from-mac/PID入门教程资料/程序源码/11/` |
| 输入死区 | `from-mac/PID入门教程资料/程序源码/11/` |

### 五、高级控制

| 主题 | 实现位置 |
|------|---------|
| 双环 PID（速度+位置） | `from-mac/PID入门教程资料/程序源码/12,13/` |
| 倒立摆控制 | `from-mac/PID入门教程资料/程序源码/15,16/` |
| 平衡车串级 PID | `from-linux/balance-car-source/03-09/` |
| 差速底盘 PI | `from-linux/motor-pid-migration-plan.md` |
| FOC (Clarke/Park/PI/SVPWM) | `from-pi/stm32-pid-motor/sim_foc.py` |
| 电机速度环 PID | `from-pi/stm32-pid-motor/main.c` |

### 六、调参方法

| 方法 | 工具 |
|------|------|
| 手动调参 (Kp→Ki→Kd) | `from-pi/pid_sim.py` mode 1 (Walkthrough) |
| Ziegler-Nichols | `from-pi/pid_sim.py` mode 3 |
| CLI 调参 | `from-pi/pid_tune.py` |
| 面试模拟 | `from-pi/pid_sim.py` mode 4 |

---

## 🚀 快速开始

### Python 仿真（树莓派工具）

```bash
# 安装依赖
sudo apt install python3-numpy python3-matplotlib

# 生成所有图表
python3 from-pi/pid_sim.py

# 交互调参
python3 from-pi/pid_tune.py list              # 查看可用植物
python3 from-pi/pid_tune.py 0 5 5 0           # 调电机 Kp=5 Ki=5 Kd=0
```

### STM32 PID 电机控制（树莓派固件）

```bash
cd from-pi/stm32-pid-motor
make                    # 编译
make flash              # 烧录 (st-flash)
```

### 平衡车编译（Linux 适配版）

```bash
# 需要先修复 Makefile 工具链路径
cd from-linux/balance-car-source/01-基础驱动代码
# arm-none-eabi-gcc 已在 PATH，直接用
make
```

### Hermes 技能

```bash
# 安装 PID 技能
cp -r from-pi/pid-control-skill ~/.hermes/skills/embedded/pid-control
```

---

## 📊 PID 覆盖矩阵

| 知识点 | Linux | Pi | Mac |
|-----------|:-----:|:--:|:---:|
| 位置式 PID | ✅ | ✅ | ✅ |
| 增量式 PID | ✅ | ✅ | ✅ |
| 积分限幅 | ✅ | ✅ | ✅ |
| 积分分离 | ❌ | ❌ | ✅ |
| 变速积分 | ❌ | ❌ | ✅ |
| 微分先行 | ✅ | ❌ | ✅ |
| 不完全微分 | ❌ | ❌ | ✅ |
| 抗积分饱和 | ✅ | ✅ | ❌ |
| Ziegler-Nichols | ❌ | ✅ | ❌ |
| 双环/串级 PID | ✅ | ❌ | ✅ |
| 倒立摆 PID | ✅ | ❌ | ✅ |
| FOC 仿真 | ❌ | ✅ | ❌ |
| Python 仿真 | ❌ | ✅ | ❌ |
| 面试训练工具 | ❌ | ✅ | ❌ |
| 裸机寄存器版 | ❌ | ✅ | ❌ |

---

## 🔗 相关仓库

| 仓库 | 内容 |
|------|------|
| [robot-vacuum-lab](https://github.com/finnyoun9/robot-vacuum-lab) | 平衡车串级 PID |
| [stm32-from-scratch](https://github.com/finnyoun9/stm32-from-scratch) | STM32F103 裸机编程 |
| [embedded-notes](https://github.com/finnyoun9/embedded-notes) | 嵌入式学习笔记 |
| [esp32-playground](https://github.com/finnyoun9/esp32-playground) | ESP32 实验 |

---

## 🖥️ 三台机器

| 机器 | IP | OS | 角色 |
|------|----|----|------|
| 本机 | 192.168.0.101 | Ubuntu 24.04 | 主力开发 + VSCode |
| 树莓派 | 192.168.0.113 | Debian 12 | PID 仿真 + STM32 固件 |
| MacBook | 192.168.0.114 | macOS 26.5 | PID 教学资源 |
