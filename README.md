# bms_replay.slx — BMS 1.0 数据回灌模型

`models/bms_replay.slx`：**输入一组数据，输出 BMS 的判断结果**。没有通信层
（CAN 帧、遥测帧、串口、心跳全都没有），也没有驱动层（LTC6804 SPI、PEC 字节、
GPIO 引脚、flash 记录都没有）。模型里只有两个逻辑块，模块之间用 Goto/From 传参。

```
   数据输入 ──▶ 保护与SOC ──▶ 状态机 ──▶ 结果输出
```

| 块 | 对应 C | 实现形式 | 代码文件 |
|---|---|---|---|
| `保护与SOC` | `slave_monitor.c` + `shunt_monitor.c` + `soc_estimation.c` | MATLAB Function（代码） | `lib/monitor_layer.m` |
| `状态机` | `state_machine.c` 的 `ts_sm_update()` | MATLAB Function（代码，**不再用 Stateflow**） | `lib/sm_state_machine.m` |

两个块的代码都存了同源文件，块里是同一份；改完文件后重新
`build_replay_model('force',1)` 即可同步。状态机代码用 `persistent` 保存状态，按
8 步顺序执行：清故障 → 合并保护层故障 → 故障锁存 → 定目标状态 → 驱动模式超时
→ 状态处理（7 个分支）→ AIR 继电器 → 输出。

配置固定为 BMS V1.0 的 `default_config`（12 串、4.2 V/3.0 V、50 ℃、500/1000 ms
去抖等），不再占用输入端口。

## 输入（19 个，全是数据）

| 端口 | 类型 | 含义 |
|---|---|---|
| `now_ms` | uint32 | 采样时刻（ms），去抖和超时都基于它 |
| `sdc_closed` | boolean | SDC 回路闭合 |
| `power_up` | boolean | 启动请求 |
| `vcu_drive_rx` | boolean | 本拍收到整车驱动使能报文 |
| `vcu_drive_bit2` | boolean | 该报文要求进入驱动模式 |
| `i_shunt_mA` | int32 | 包电流，充电为正 |
| `v_bat_mV` | int32 | 包电压 |
| `v_veh_mV` | int32 | 车端电压（预充判据） |
| `busbar_dC` | int32 | 铜排温度 ×10 |
| `shunt_age_ms` | uint32 | 距上一帧分流器数据的时长 |
| `current_counter` | single | 分流器安时累计（mAs） |
| `cell_mV[144]` | uint16 | 单体电压（12 串 × 12 通道，IC 优先） |
| `temp_C[72]` | int16 | 单体温度（12 串 × 6 通道，IC 优先） |
| `afe_comm_bad` | boolean | 本次 AFE 读取 CRC 失败 |
| `charge_mode` | uint8 | 充电机请求充电（接收路径已做 1 s 超时） |
| `clear_errors` | boolean | 单次清故障请求 |
| `charge_finish` | uint8 | 充电完成请求 |
| `soc_init_valid` | boolean | 是否用 `soc_init_centi` 作为初始 SOC |
| `soc_init_centi` | uint32 | 初始 SOC，百分比 ×100 |

`cell_mV` / `temp_C` 是**已解码的测量值**：C 里 `cell_codes/10` 和 `temp_cal()` 的
结果。偶数串的第 12 通道不参与（实际 138 个单体），与 `slaves_check()` 一致。

## 输出（21 个）

| 端口 | 含义 |
|---|---|
| `cur_state` / `target_state` | 状态（0 INACTIVE, 1 ACTIVE, 2 PRECHARGE, 3 DISCHARGE, 4 ERROR, 5 PRE_HOLD, 6 CHARGE） |
| `error_flags` / `error_kind` | 故障源位图 / 故障种类位图 |
| `air_pos` / `air_neg` / `air_pre` | 主正 / 主负 / 预充继电器命令 |
| `hv_active` | 高压就绪输出 |
| `bms_nerror` | 故障指示灯（低有效） |
| `drive_mode` | 当前驱动模式（内部也回馈给监视层） |
| `slave_error` / `slave_kind` | AFE 侧故障与种类 |
| `shunt_error` / `shunt_kind` | 分流器侧故障与种类 |
| `hv_mismatch` | 电压不一致 |
| `bat_volt` | 138 个单体电压之和（mV） |
| `min_cell_mV` / `max_cell_mV` | 单体电压极值 |
| `min_temp_C` / `max_temp_C` | 单体温度极值 |
| `soc_centi` | SOC，百分比 ×100 |

## 怎么做数据回灌

把录制的数据按列组织成 timeseries 灌进根端口即可，`scripts/replay_replay_sil.m`
就是完整例子：读 CSV → 每个端口一条 timeseries → `Simulink.SimulationInput`
→ `sim` → 按信号名从 `yout` 取结果。

```matlab
load_system('models/bms_replay.slx');
t  = (0:N-1).' * 0.15;
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(uint32(t_ms), t, 'Name', 'now_ms'), 'now_ms');
% ... 其余端口同理 ...
so = sim(Simulink.SimulationInput('bms_replay').setExternalInput(ds));
cur = so.yout.getElement('cur_state').Values.Data;
```

## 验证：与 BMS_V1.0 的 C 代码逐拍对齐

```bash
bash host/build_app.sh          # 用真实 C 源码跑出 golden（不改一行 C）
matlab -batch "addpath lib scripts models; replay_replay_sil()"
```

`host/tests/test_app_ref.c` 把 `state_machine.c` / `slave_monitor.c` /
`shunt_monitor.c` / `soc_estimation.c` 和 `main.c` 的循环体一起编译，只替换 SPI、
CAN、UART、flash 这些载体；回灌脚本把同一份数据解码成 `cell_mV` / `temp_C` /
`afe_comm_bad` 灌进模型，逐拍对比 19 个输出。

当前结果：**261/261 拍、19 个信号全部一致**。场景覆盖：

```
怠速 → 预充 → PRE_HOLD → ACTIVE → 分流器过流 → 清故障 → 放电
     → SOC 充/放 → 心跳丢失 → 预充超时
     → 充电（CCS 报文）→ CHARGE → 充电中电流反向 → 清故障
     → 单体过压 → 清故障 → 单体欠压 → 清故障
     → 单体过温 → 清故障 → 电压不一致 → 清故障 → AFE 通信故障
```

## 这套场景查出来的四个逻辑问题（都已修）

1. **CHARGE 状态根本到不了**：原模型把 `charge_mode` 写成常量 0，状态机的
   `TS_CHARGE` 分支是死代码。现在 `charge_mode` 是输入（充电机请求解码后的结果）。
2. **预充超时清不掉**：`clear_errors` 原来只进监视层，状态机自己的
   `TS_ERROR_PRECHARGE_TIMEOUT`（flag 4 / kind 512）没有入口可清，故障会永久锁死。
   现在状态机也有 `clear_errors` 输入，每拍入口先清一次，顺序与 C 的
   `ts_sm_serial_clear_errors()` 一致。
3. **CHARGE→ERROR 的 NERROR 早了一拍**：C 在充电中电流反向时只返回 `TS_ERROR`，
   当拍不动 NERROR，下一拍 ERROR 处理函数才拉低；图里原来是无条件拉低。
   现在 ERROR 入口只在 `error_flags ~= 0` 时拉低。
4. **监视层看到的 drive_mode 晚一拍**：C 里 `drive_mode` 由接收中断写入，监视层
   先看到它、`ts_sm_update()` 之后才做 2 s 超时；现在用
   `Switch(收到报文 ? 报文值 : 上一拍) + Memory` 复现，上升沿不再晚一拍。

## 状态机：代码实现（不用 Stateflow）

`状态机` 块是一个 MATLAB Function，源码在 `lib/sm_state_machine.m`，块里是
同一份代码。函数签名：

```matlab
function [cur_state, target_state, error_flags, error_kind, air_pos, air_neg, ...
          air_pre, hv_active, bms_nerror, drive_mode] = state_machine( ...
          now_ms, sdc_closed, power_up, drive_can_rx, drive_can_bit2, ...
          v_veh_mV, v_bat_mV, i_shunt_mA, charge_mode, slave_error, ...
          slave_kind, shunt_error, shunt_kind, clear_errors)
```

- 状态用 `persistent` 保存（cur/tgt/err/kind、预充/放电/恢复计时、AIR 状态、
  HV_ACTIVE、NERROR、drive_mode），等价 C 里的文件静态变量；
- 每拍按 8 步执行：清故障 → 合并保护层故障 → 故障锁存 → 定目标状态 →
  驱动模式超时 → 7 个状态分支 → AIR 继电器 → 输出；
- 所有 `*_ms` 都是毫秒，`now_ms` 是本拍时刻；
- AIR 闭合间隔（10 ms）在代码里不用模拟：C 里那段 `HAL_Delay` 只影响引脚
  翻转的时刻，不改变本拍的输出，回灌模型里没有时钟可推。

端口类型在块里显式钉住（`now_ms` uint32、`v_veh_mV` int32、`charge_mode` uint8、
`slave_kind` uint32、输出 `cur_state` uint8、`error_flags` uint32、其余 boolean），
避免 From/Goto 通路上的类型推断不一致。

## 块内中文注释（参数说明）

两个逻辑块的代码里**注释已经全部是中文**，只保留 `%#codegen` 这个编译指示
（它必须原样写），打开块就能看到。

**保护与SOC**（`lib/monitor_layer.m`）：

- 文件头列出全部 14 个输入、11 个输出的含义与单位，例如
  `now_ms` 是本拍时刻（去抖/超时基准）、`busbar_dC` 是 0.1 ℃、`cell_mV` 是
  12 串 × 12 通道的 mV、`soc_init_centi` 是百分比 ×100；
- 固定配置常量逐个注释：单体过压 4200 mV、欠压 3000 mV、过温 50 ℃、
  去抖 1000 ms（单体）/ 500 ms（分流器）、放电过流 −200 A、充电过流 +15 A、
  铜排过温 100.0 ℃、分流器数据超时 500 ms；
- SOC 段注明：上电第一拍装载初始 SOC、`delta_mas` 是相邻两拍安时差、
  容量基准 50400000 mAs（14 Ah）、充电完成置 100%、`soc_centi` 是百分比 ×100。

**状态机**（`lib/sm_state_machine.m`，块里同一份代码）：

- 文件头列出 14 个输入、10 个输出的含义与单位；
- 固定配置逐个注释：AIR 间隔 10 ms、放电 5 s、错误恢复 1 s、预充超时 5 s、
  PRE_HOLD 200 ms、ACTIVE 保持 500 ms、驱动超时 2 s、预充判据下限 150 mV；
- 8 个执行步骤各自有中文说明（清故障、合并故障源、故障锁存、定目标、驱动超时、
  7 个状态分支、AIR 命令、输出）；
- 每个状态的注释写明动作、跳转条件和用到的参数，例如 PRECHARGE：
  “判据 v_veh > 0.95*v_bat 且 > 150 mV；超时 5 s 进 ERROR”。

## 检查结论（改成代码状态机之后）

- 整机回灌：**261/261 拍、19 个输出全部一致**，函数实现与 C 的
  `ts_sm_update()` 行为完全一致；
- 模型可编译、无未连接端口，连线和 Goto/From 都是正常实线；
- 状态机块保留在原位置，保护与SOC 块未改动。

## 不包含

- 通信层：CAN 帧 / 遥测帧 / 串口 / 心跳；串口命令仲裁
  （`SET_TARGET_STATE`、`SET_MANUAL_AIR`）也不在模型里，`clear_errors` 是保留的
  服务输入。
- 驱动层：LTC6804 SPI 时序、PEC 字节、GPIO 引脚、flash 读写。
- `persist`（SOC 写 flash）不作为输出；`blocked_ms`、`status4`、`bms_error`
  属于被删掉的层或 C 里注释掉的代码，不建模。

## 重新生成

```matlab
build_replay_model('force', 1)   % 或 run scripts/run_build_replay.m
```
