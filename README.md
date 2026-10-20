<div align="center">

# ⚡ BMS 状态机模型

**基于模型设计（MBD）的电池管理系统应用层状态机**

Simulink / Stateflow 实现 · 行为对齐 C 参考代码 · 16 场景自动回归

[![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-orange?logo=mathworks&logoColor=white)](https://www.mathworks.com)
[![Simulink](https://img.shields.io/badge/Simulink-Stateflow-blue)](https://www.mathworks.com/products/simulink.html)
[![Tests](https://img.shields.io/badge/tests-16%20passed-brightgreen)](#-测试)
[![License](https://img.shields.io/badge/license-Internal-lightgrey)](#-许可)

</div>

---

## 📖 简介

BMS（电池管理系统）应用层状态机的 Simulink 模型，**行为逐拍对齐** `BMS_V1.0` 的 C 实现
（`state_machine.c` 的 `ts_sm_update()`）。包含 7 个状态、8 路故障去抖与锁存、预充控制、
充电/放电流程，可直接用于仿真验证与代码生成。

> 仅应用层：不含 SPI / CAN / GPIO 驱动，也不含 flash 存储。

![状态机](docs/bms_state_machine.png)

---

## ✨ 特性

- 🔁 **7 状态机**：INACTIVE / PRECHARGE / PRE_HOLD / ACTIVE / CHARGE / DISCHARGE / ERROR
- 🛡️ **8 路故障保护**：单体过压/欠压/过温、AFE 通信、分流器超时/过流、铜排过温、压差
- ⏱️ **去抖 + 锁存**：500/1000 ms 去抖，确认后锁存，只有 CLEAR 能清
- ⚡ **预充逻辑**：车端电压 > 95% 包压（且 > 150 mV）→ 合主正 → 200 ms 保持 → 高压就绪
- 🧪 **自动回归**：16 个场景脚本化注入，状态序列自动比对
- 📐 **与 C 对齐**：所有时间参数、边界判据与参考实现一致

---

## 🏗️ 架构

```mermaid
flowchart LR
    F[8 路故障信号] --> P[protection_debounce]
    P -->|mon_flags / mon_kind| S[ts_sm_update<br/>Stateflow]
    T[power_up / charge_mode] --> H[ts_sm_handle_bms_in]
    H -->|target_state| S
    S -->|sm_latched| M[ts_sm_set_error_source]
    P -->|mon_flags / mon_kind| M
    M --> E[error_flags / error_kind]
    S --> O[AIR_* / HV_ACTIVE<br/>NERROR / current_state]
```

| 模块 | 作用 | 对应 C |
|---|---|---|
| `protection_debounce` | 8 路去抖 + 锁存 + 位图 | `error_monitor_update()` |
| `ts_sm_update` | 状态机（7 状态） | `ts_sm_update()` |
| `ts_sm_handle_bms_in` | 计算目标状态 | `ts_sm_handle_bms_in()` |
| `ts_sm_set_error_source` | 合并故障源 | `ts_sm_set_error_source()` |

---

## 🔄 状态定义

| 值 | 状态 | 含义 | AIR 主正/主负/预充 | HV_ACTIVE | NERROR |
|:--:|---|---|:--:|:--:|:--:|
| `0` | INACTIVE | 空闲 | 断 / 断 / 断 | 0 | 1 |
| `2` | PRECHARGE | 预充 | 断 / **合** / **合** | 0 | — |
| `5` | PRE_HOLD | 预充保持 200 ms | **合** / **合** / **合** | 0 | — |
| `1` | ACTIVE | 高压就绪 | **合** / **合** / 断 | 1 | — |
| `6` | CHARGE | 充电 | **合** / **合** / 断 | 1 | — |
| `3` | DISCHARGE | 放电等待 5 s | 断 / 断 / 断 | 0 | — |
| `4` | ERROR | 故障 | 断 / 断 / 断 | 0 | 0 |

**主要转移**

```text
INACTIVE  ──(目标 ACTIVE/CHARGE 且 SDC 闭合)──► PRECHARGE
PRECHARGE ──(veh > 150mV 且 veh > 0.95×bat)──► PRE_HOLD
PRECHARGE ──(5 s 未到 95%)──────────────────► ERROR
PRE_HOLD  ──(200 ms 且 95%)─────────────────► ACTIVE / CHARGE
PRE_HOLD  ──(200 ms 后电压掉)───────────────► ERROR
ACTIVE/CHARGE/PRE_HOLD ──(目标撤销 / SDC 断开)──► DISCHARGE
CHARGE    ──(电流 < -300 mA)────────────────► ERROR
DISCHARGE ──(5 s)───────────────────────────► INACTIVE
ERROR     ──(故障清 0 稳定 1 s)─────────────► DISCHARGE
任意状态  ──(mon_flags ≠ 0)─────────────────► ERROR
```

---

## 🔌 接口

### 输入

| 信号 | 类型 | 单位 | 说明 |
|---|---|:--:|---|
| `now_ms` | uint32 | ms | 毫秒时间戳（`Clock × 1000`） |
| `power_up` | 0/1 | — | 启动请求 |
| `charge_mode` | 0/1 | — | 充电机请求 |
| `SDC_CLOSE` | uint8 | — | 安全回路闭合 |
| `clear_errors` | uint8 | — | 清故障（单次脉冲） |
| `voltage_bat` | int32 | mV | 包电压 |
| `voltage_veh` | int32 | mV | 车端电压 |
| `i_shunt` | int32 | mA | 包电流（充电为正） |
| 8 路故障 | 0/1 | — | `cell_ov` `cell_uv` `cell_ot` `afe_comm` `shunt_tmo` `shunt_oc` `busbar_ot` `v_mismatch` |

### 输出

| 信号 | 类型 | 说明 |
|---|---|---|
| `current_state` | uint8 | 0~6 |
| `AIR_POS` / `AIR_NEG` / `AIR_PRE` | uint8 | 继电器命令 |
| `HV_ACTIVE` | uint8 | 高压就绪 |
| `NERROR` | uint8 | 故障灯（低有效） |
| `error_flags` | uint16 | 故障源：1 单体 / 2 分流器 / 4 状态机 |
| `error_kind` | uint16 | 故障种类位图 |

---

## ⏲️ 关键参数

| 参数 | 值 | 说明 |
|---|:--:|---|
| 预充超时 | 5000 ms | 5 s 未到 95% → ERROR |
| 预充判据 | `veh > 150 mV` 且 `veh > 0.95 × bat` | |
| 预充保持 | 200 ms | PRE_HOLD |
| 放电时间 | 5000 ms | DISCHARGE |
| 故障恢复 | 1000 ms | 故障清 0 后稳定时间 |
| 目标保持 | 500 ms | power_up 撤销后 ACTIVE 保持 |
| 去抖 | 1000 / 500 ms | 单体 / 分流器 |

---

## 🚀 快速开始

```matlab
% 需要 MATLAB R2025b + Simulink + Stateflow
cd BMS_MBD
addpath('lib','scripts','models');
bms_mbd_types('Exported');        % 定义枚举/总线

open_system('models/bms.slx')
```

**仿真设置**：Fixed-step，步长 **0.15 s**（与 C 的 150 ms 节拍一致）。

---

## 🧪 测试

```matlab
hil_suite(1:16)     % 16 个场景自动回归
```

| # | 场景 | 结果 |
|:--:|---|:--:|
| 1 | 主流程 + 故障锁存 + CLEAR | ✅ |
| 2 | 预充超时 → ERROR（4/512） | ✅ |
| 3 | 预充中撤销 | ✅ |
| 4 | 预充中 SDC 断开 | ✅ |
| 5 | PRE_HOLD 中撤销 | ✅ |
| 6 | PRE_HOLD 电压掉 → ERROR | ✅ |
| 7 | 充电流程 → CHARGE | ✅ |
| 8 | 充电电流反向 → ERROR | ✅ |
| 9 | 电流 −300 mA 不触发 | ✅ |
| 10 | ACTIVE 中故障 → ERROR | ✅ |
| 11 | 去抖 500 ms 不触发 | ✅ |
| 12 | 双重故障 flags = 3 | ✅ |
| 13 | 上电即故障 | ✅ |
| 14 | ACTIVE 下按 CLEAR 无影响 | ✅ |
| 15 | 恢复后再次故障 | ✅ |
| 16 | power_up 短抖保持 ACTIVE | ✅ |

> `models/bms_replay.slx` 是数据回灌模型，`replay_replay_sil()` 可与 C 黄金数据逐拍对比。

---

## 📁 目录结构

```text
BMS_MBD/
├── models/          Simulink 模型
│   ├── bms.slx          状态机主模型
│   └── bms_replay.slx   数据回灌模型
├── lib/             公共函数与类型定义
│   ├── bms_mbd_types.m
│   ├── sm_state_machine.m
│   └── monitor_layer.m
├── scripts/         构建与测试脚本
│   ├── hil_suite.m      16 场景自动测试
│   └── hil_test_bms.m   单场景手工测试
├── docs/            文档
│   ├── bms_state_machine.md
│   ├── bms_state_machine.png
│   └── sm_defs.h
└── host/            主机侧 C 参考工程
```

---

## ⚠️ 已知限制

- 仅应用层：无 SPI / CAN / GPIO 驱动、无 flash 存储
- 硬件级故障（继电器粘连、绝缘、互锁、AFE 断线）不在模型范围
- AIR 闭合间隔 10 ms 未建模（步长 0.15 s）
- 串口控制 / 手动 AIR 测试模式未实现

---

## 📚 参考

- 行为对齐 `BMS_V1.0/APP/Core/Src/state_machine.c`
- 状态图与条件详见 [`docs/bms_state_machine.md`](docs/bms_state_machine.md)

---

<div align="center">

**如果这个项目对你有帮助，欢迎 ⭐ Star**

</div>
