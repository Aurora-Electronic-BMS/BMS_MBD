/*
 * sm_defs.h — BMS 状态机用的状态 / 故障 / AIR 宏定义
 * 数值与 BMS_V1.0 的 state_machine.h 完全一致（CAN 和遥测按原始值上报，
 * 自己实现时不要改这些数）。
 */
#ifndef BMS_SM_DEFS_H
#define BMS_SM_DEFS_H

#include <stdint.h>
#include <stdbool.h>

/* =====================================================================
 * 1) 状态
 * ===================================================================== */
typedef enum {
    TS_INACTIVE  = 0,   /* 空闲：三个继电器断开，NERROR 高（正常） */
    TS_ACTIVE    = 1,   /* 高压就绪（行车）：主正+主负闭合，HV_ACTIVE=1 */
    TS_PRECHARGE = 2,   /* 预充：主负+预充闭合，等车端电压到 95% */
    TS_DISCHARGE = 3,   /* 放电：三个继电器断开，等 5 s 回 INACTIVE */
    TS_ERROR     = 4,   /* 故障：三个继电器断开，NERROR 低 */
    TS_PRE_HOLD  = 5,   /* 预充保持：三个全闭，保持 200 ms */
    TS_CHARGE    = 6,   /* 充电：主正+主负闭合，HV_ACTIVE=1 */
} TSState;

/* 不喜欢 enum 的话，用这一组等价：#define TS_INACTIVE 0U ... */
#define TS_STATE_COUNT   7U
#define TS_STATE_VALID(s)  ((unsigned)(s) < TS_STATE_COUNT)

/* 只有这三个能作为目标状态（对应 ts_sm_serial_set_target 的检查） */
#define TS_TARGET_ALLOWED(s) \
    ((s) == TS_INACTIVE || (s) == TS_ACTIVE || (s) == TS_CHARGE)

/* 调试打印用；不要放进中断里 */
static inline const char *ts_state_name(TSState s)
{
    switch (s) {
    case TS_INACTIVE:  return "INACTIVE";
    case TS_ACTIVE:    return "ACTIVE";
    case TS_PRECHARGE: return "PRECHARGE";
    case TS_DISCHARGE: return "DISCHARGE";
    case TS_ERROR:     return "ERROR";
    case TS_PRE_HOLD:  return "PRE_HOLD";
    case TS_CHARGE:    return "CHARGE";
    default:           return "?";
    }
}

/* =====================================================================
 * 2) 故障源（error_flags 的位，多源可同时置位，锁存）
 * ===================================================================== */
#define TS_ERROR_SOURCE_SLAVE         (1U << 0)   /* 单体/从机侧，由保护层置位 */
#define TS_ERROR_SOURCE_SHUNT         (1U << 1)   /* 分流器侧，由保护层置位 */
#define TS_ERROR_SOURCE_STATE_MACHINE (1U << 2)   /* 状态机自己：预充超时 */

/* =====================================================================
 * 3) 故障种类（error_kind 的位，可同时置多位）
 *    约定：bit0 = NONE，有故障时清 bit0；每组故障只清自己那一组掩码
 * ===================================================================== */
typedef enum {
    TS_ERROR_NONE                 = 1U << 0,
    TS_ERROR_SLAVE_OVERVOLT       = 1U << 1,
    TS_ERROR_SLAVE_UNDERVOLT      = 1U << 2,
    TS_ERROR_SLAVE_OVERTEMP       = 1U << 3,
    TS_ERROR_SHUNT_TIMEOUT        = 1U << 4,
    TS_ERROR_SHUNT_OVERCURRENT    = 1U << 5,
    TS_ERROR_SHUNT_OVERTEMP       = 1U << 6,
    TS_ERROR_VOLTAGE_MISMATCH     = 1U << 7,
    TS_ERROR_INTERNAL_COMM_ERROR  = 1U << 8,
    TS_ERROR_PRECHARGE_TIMEOUT    = 1U << 9,
} TSErrorKind;

/* 每个故障源只管自己这组种类位，清位时按组来（对应 C 的 source_kind_mask） */
#define TS_KIND_MASK_SLAVE \
    (TS_ERROR_SLAVE_OVERVOLT | TS_ERROR_SLAVE_UNDERVOLT | \
     TS_ERROR_SLAVE_OVERTEMP | TS_ERROR_INTERNAL_COMM_ERROR)          /* 2|4|8|256 = 270 */

#define TS_KIND_MASK_SHUNT \
    (TS_ERROR_SHUNT_TIMEOUT | TS_ERROR_SHUNT_OVERCURRENT | \
     TS_ERROR_SHUNT_OVERTEMP | TS_ERROR_VOLTAGE_MISMATCH)             /* 16|32|64|128 = 240 */

#define TS_KIND_MASK_STATE_MACHINE \
    (TS_ERROR_PRECHARGE_TIMEOUT)                                      /* 512 */

/* 置/清一个故障源：
 *   flags    故障源位图
 *   kind     故障种类位图
 *   flag     本故障源对应的位（TS_ERROR_SOURCE_xxx）
 *   mask     本源只管哪些种类位（TS_KIND_MASK_xxx）
 *   incoming 本次监视到的种类（0 表示本源没故障）
 */
static inline void ts_set_error_source(uint32_t *flags, uint16_t *kind,
                                       uint32_t flag, uint16_t mask,
                                       uint16_t incoming)
{
    *kind &= (uint16_t)~mask;
    if ((incoming & mask) != 0U) {
        *flags |= flag;
        *kind  |= (uint16_t)(incoming & mask);
    } else {
        *flags &= ~flag;
    }
    *kind = (*flags == 0U) ? (uint16_t)TS_ERROR_NONE
                           : (uint16_t)(*kind & (uint16_t)~TS_ERROR_NONE);
}

/* =====================================================================
 * 4) AIR 继电器
 * ===================================================================== */
typedef enum {
    AIR_POS = 0,    /* 主正 */
    AIR_NEG,        /* 主负 */
    AIR_PRE,        /* 预充 */
} AIR;

#define AIR_BIT_POS (1U << 0)
#define AIR_BIT_NEG (1U << 1)
#define AIR_BIT_PRE (1U << 2)
#define AIR_BIT_ALL (AIR_BIT_POS | AIR_BIT_NEG | AIR_BIT_PRE)

/* 每个状态对应的继电器命令（1 = 闭合），与 ts_sm_set_AIR_positions 一致 */
#define AIR_CMD_INACTIVE   (0U)
#define AIR_CMD_PRECHARGE  (AIR_BIT_NEG | AIR_BIT_PRE)
#define AIR_CMD_PRE_HOLD   (AIR_BIT_POS | AIR_BIT_NEG | AIR_BIT_PRE)
#define AIR_CMD_ACTIVE     (AIR_BIT_POS | AIR_BIT_NEG)
#define AIR_CMD_CHARGE     (AIR_BIT_POS | AIR_BIT_NEG)
#define AIR_CMD_DISCHARGE  (0U)
#define AIR_CMD_ERROR      (0U)

/* =====================================================================
 * 5) 时间参数（BMS_V1.0 default_config 的值；自己有配置表就删掉这一段）
 * ===================================================================== */
#define BMS_AIR_CLOSE_WAIT_MS       10U      /* 两次 AIR 闭合的最小间隔 */
#define BMS_PRECHARGE_95_WAIT_MS    5000U    /* 预充到 95% 的超时 */
#define BMS_PRE_HOLD_MS             200U     /* 到 95% 后保持多久 */
#define BMS_DISCHARGE_MS            5000U    /* 放电保持时间 */
#define BMS_NO_ERROR_MS             1000U    /* 故障消失后需稳定的时间 */
#define BMS_MAX_STATE_HOLD_MS       500U     /* POWER_UP 撤销后 ACTIVE 再保持 */
#define BMS_DRIVE_TIMEOUT_MS        2000U    /* 整车报文超时 */
#define MIN_VEHICLE_SIDE_VOLTAGE    150      /* 预充判据下限 mV，硬件固定 */

/* 预充完成判据：车端电压 > 150 mV 且 > 包压的 95%（整数比较，不用浮点） */
#define PRECHARGE_DONE(veh_mv, bat_mv) \
    (((int32_t)(veh_mv) > MIN_VEHICLE_SIDE_VOLTAGE) && \
     ((int32_t)(veh_mv) * 100 > (int32_t)(bat_mv) * 95))

/* 充电电流反向（进 ERROR）的阈值，mA */
#define CHARGE_CURRENT_REVERSE_MA   (-300)

#endif /* BMS_SM_DEFS_H */
