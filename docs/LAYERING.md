# 25_R2_leg 分层架构说明（bsp / osal / ser / app）

> 本次重构的目标：把"看起来分了目录、实际依赖成环"的工程改成**依赖单向、算法可单测**的四层结构。
> 备份快照：`25_R2_leg_backup_20260927_192725`（对应重构前 HEAD `b2656d5`）。

---

## 1. 四层职责

| 层 | 目录 | 职责 | 允许依赖 | 严禁 |
|---|---|---|---|---|
| **app** | `app/` | 业务决策、任务节拍、对外接口、调试量发布 | ser、osal、common | HAL 外设调用、控制算法 |
| **ser** | `ser/{algo,dev,leg}` | `algo`=纯算法；`dev`=设备协议；`leg`=运动编排 | bsp 头、common、ser 内部 | HAL 类型、app、RTOS |
| **osal** | `osal/` | RTOS 与时间源抽象（任务、延时、tick、dt） | cmsis_os2、bsp、common | 业务语义、ser、app |
| **bsp** | `bsp/` | 外设原语（FDCAN 收发、过滤器、DWT）、上行注册 | HAL/CMSIS、common | FreeRTOS、任何上层对象 |
| （公共） | `common/inc/` | 四层共享的"词汇表"：类型、限位、统一返回码 | 仅 C 标准库 | 任何层的实现头 |

依赖方向严格单向，不跳层、不反向：

```
app ──▶ ser ──▶ osal ──▶ bsp ──▶ HAL/CMSIS
                 └──────────────▶ bsp
        （ser/dev ──▶ bsp 只取"总线编号"，不接触 HAL 类型）
        （ser/algo 只依赖 common，无任何芯片/RTOS 依赖 → 可 PC 单测）
```

两条铁律：

1. **下行是调用，上行是注册**。中断/反馈这条反向路径靠"注册回调 + 句柄注入"实现，
   下层绝不允许 `extern` 上层变量。
2. **头文件不越层**：`bsp/` 不出现 FreeRTOS 与上层头；`ser/algo` 不出现 HAL/时间源；
   `app/` 不出现 HAL 与 `cmsis_os2.h`。

---

## 2. 目录结构

```
common/inc/
  leg_types.h          公共类型 + 机械限位 + 限速宏 + 统一返回码 + 限位判定(inline)
  struct_typedef.h     历史遗留通用 typedef（保留，暂无引用）
bsp/
  bsp_can.{c,h}        FDCAN 过滤器/启动/中断取帧/收发原语 + 上行注册表
  bsp_dwt.{c,h}        DWT 时间戳（由原 dwt/ 平移）
osal/
  osal_time.{c,h}      OSAL_Time_Init/Dt/ResetDt/GetTick/GetMs/DelayMs
  osal_task.{c,h}      OSAL_Task_Create/Destroy/DelayUntil（固定周期节拍）
ser/
  algo/kinematics.{c,h}    FK / IK / 雅可比 / 关节↔电机零点换算
  algo/joint_traj.{c,h}    五次多项式轨迹（只依赖 leg_types.h）
  algo/motion_alg.{c,h}    关节限速器 + 重力补偿（句柄化，dt 由调用方传入）
  dev/ak_motor.{c,h}       AK 电机 MIT 协议（去 HAL 化）
  dev/el05_motor.{c,h}     EL05 电机私有协议（去 HAL 化）
  dev/can_router.{c,h}     帧路由：标准帧→AK，扩展帧→EL05（注册表 + 注入）
  leg/leg_motion.{c,h}     腿部运动服务：反馈→目标管理→轨迹→限速→重力→下发
app/
  leg_task.{c,h}       强定义 leg_task()；设备实例与节拍编排
  leg_config.h         接线/节拍/调试开关等业务参数
```

---

## 3. 旧 → 新 文件映射

| 旧位置 | 新位置 | 说明 |
|---|---|---|
| `Task/leg_task.c` | `app/leg_task.c` + `ser/leg/leg_motion.c` | 359 行拆成"节拍编排"与"运动服务" |
| `Task/leg_task.h` | `app/leg_task.h` | 去掉 `cmsis_os2.h`（不再暴露 RTOS 类型） |
| `Mycan/Mycan.c` | `bsp/bsp_can.c` + `ser/dev/can_router.c` | 硬件原语与协议路由分离 |
| `Mycan/Mycan.h` | `bsp/bsp_can.h` + `ser/dev/can_router.h` | 同上 |
| `AK_Driver/ak_motor.{c,h}` | `ser/dev/ak_motor.{c,h}` | 协议属服务层；`void* bus` → `bsp_can_bus_t` |
| `AK_Driver/EL05_motor.{c,h}` | `ser/dev/el05_motor.{c,h}` | 同上 |
| `bsp/kinematics.{c,h}` | `ser/algo/kinematics.{c,h}` | 运动学不是 bsp；剥离限速与重力补偿 |
| `bsp/joint_traj.{c,h}` | `ser/algo/joint_traj.{c,h}` | 改为只依赖 `leg_types.h` |
| （原在 kinematics.c 内） | `ser/algo/motion_alg.{c,h}` | 限速器 + 重力补偿独立成句柄化模块 |
| `dwt/bsp_dwt.{c,h}` | `bsp/bsp_dwt.{c,h}` | 纯平移（内容未改） |
| `Mycan/struct_typedef.h` | `common/inc/struct_typedef.h` | 纯平移 |
| `Mycan/Mycan.c` 的 `g_rx_frames` | `BSP_CAN_GetRxFrameCount()` / `CAN_Router_GetFrameCount()` | 全局变量改为接口 |

---

## 4. 关键改动清单

1. **切断环状依赖（最重要）**
   旧 `Mycan.c` 属 bsp 层，却 `extern AK_Motor motors[]`（定义在 app 层），形成 `bsp → app` 反向依赖。
   现改为：`BSP_CAN_RegisterRx()` 注册回调 + `CAN_Router_BindAk/BindEl05()` 依赖注入，
   路由器与 bsp 都不再认识任何全局实例。

2. **算法层去时间源**
   旧 `kinematics.c` 内部调 `DWT_GetDeltaT()` 给重力补偿做变化率限制，导致纯数学模块无法脱离芯片。
   现 `GravityComp_Update(..., dt, ...)` 由调用方传入 `dt`，`ser/algo` 的 include 只剩 `leg_types.h`。

3. **驱动层去 HAL**
   `AK_Motor.bus` / `EL05_Motor.bus` 由 `void*`（强转 `FDCAN_HandleTypeDef*`）改为 `bsp_can_bus_t` 总线编号，
   `ak_motor.c` / `el05_motor.c` 不再包含 `stm32h7xx_hal.h`。

4. **隐式单例 → 显式句柄**
   旧 `g_q_cmd`、`g_q1_prev`、`last_tau_*`、`g_traj`、`g_target_pending` 等文件级 static 全部收进
   `JointRateLimiter` / `GravityComp` / `JointTraj` / `LegMotion`，模块内无可变全局量，第二条腿可直接复用。

5. **统一返回码**
   旧 `AK_OK=0`（0 成功）与 `JTRAJ_OK=1`（1 成功、0 错误）语义相反，跨层混用必错。
   现全工程统一 `SER_OK=0`、负数=错误；轨迹状态（`JTRAJ_IDLE/RUNNING/DONE`）与返回码彻底分离。

6. **轨迹与运动学解耦**
   限位判定（`LegJointInLimit` / `LegJointsClamp`）下沉到 `leg_types.h` 的唯一 `static inline` 实现，
   `joint_traj` 不再反向依赖 `kinematics`。

7. **时间与节拍进 osal**
   `osDelay` / `osDelayUntil` / `osKernelGetTickCount` / `DWT_*` 统一到 `OSAL_Time_*` / `OSAL_Task_*`，
   `app` 头文件不再暴露 RTOS 类型。

8. **控制行为保持不变**（刻意逐条对齐旧实现）
   - 主循环仍是 `AK 帧 → +1ms → EL05 腕帧 → +1ms`，AK 侧实际周期 2ms；
   - `dt` 仍由 DWT 实测，`<=0` 退回 2ms、`>50ms` 钳位；
   - 目标去重/运行中锁存、IK 失败不更新 `target_planned`（下拍重试）、故障保持不刷新腕部快照、
     腕部位置 `π−(q1+q2)`、重力补偿首拍不限速 —— 全部与旧版一致；
   - 调试全局量（`b/c/d`、`q_cmd_dbg`、`v_cmd_dbg`、`dbg_*`、`motors[]`、`motor_angles`、`FK_pos`、`target_pos`）
     **变量名保持不变**，MDK Watch 表达式无需修改。

---

## 5. 验证记录

工具链：`arm-none-eabi-gcc`（STM32CubeCLT 1.18.0）。

1. **语法/类型编译**：12 个源文件 `-fsyntax-only -Wall`，**0 error / 0 warning**。
2. **目标文件编译 + 符号审计**：12/12 目标文件构建成功；
   21 个外部未定义符号全部有归属（6×HAL_FDCAN、`hfdcan1`、6×CMSIS-RTOS2、libm/newlib），
   **无"声明未定义"符号**。
3. **越权依赖检查**：按层规则扫描全部 include，**0 处违规**。
4. **反向依赖探针**：`bsp` / `osal` / `ser` 中不存在对 `motors[]` / `el05_motors` / `g_leg` 的 `extern`。
5. **MDK 工程**：`25_R2_leg.uvprojx` 重新解析通过（13 个分组），include 路径已更新。

> 注：工程的 ARMCC5（`uAC6=0`）与 FreeRTOS 的 `portable/RVDS/ARM_CM4F` 端口无法用 GCC 完整链接，
> 因此验证到"编译 + 符号就绪"层级；最终需在 Keil 中重新构建确认。

---

## 6. 编译约定（跨编译器，务必遵守）

**每个 `.c` 必须直接包含自己用到的标准头，不允许依赖"传递包含"。**

原因：GCC 常常通过别的头文件悄悄把 `NULL` / `size_t` / math 函数带进来，
而 ARMCC5 会直接报 `error: #20: identifier "xxx" is undefined`。

实测踩过的两处（已修）：

| 文件 | 缺什么 | 补法 |
|---|---|---|
| `app/leg_task.c` | `NULL` | `#include <stddef.h>` |
| `ser/algo/kinematics.c` | `cosf / sinf / atan2f / sqrtf / fabsf / fmaxf` | `#include <math.h>`（原先靠 `kinematics.h` 里的 `"math.h"` 传递；已改为头文件只做声明、由 `.c` 各自直接包含） |

对应的静态检查方法（本次重构的验证脚本就用这个）：

对每个 `.c` 扫描标识符，核对它是否**直接**包含了规范来源头 ——

| 标识符 | 规范来源 |
|---|---|
| `NULL` / `size_t` / `ptrdiff_t` | `stddef.h` |
| `bool` / `true` / `false` | `stdbool.h` |
| `cosf` / `sinf` / `atan2f` / `sqrtf` / `fabsf` / `fmaxf` … | `math.h` |
| `memcpy` / `memset` / `strlen` | `string.h` |
| `os*` / `osStatus_t` / `osOK` | `cmsis_os2.h` |
| `HAL_FDCAN_*` / `hfdcan1` / `FDCAN_*` | `fdcan.h` |
| `DWT_Init` / `DWT_GetDeltaT` / `SysTime` | `bsp_dwt.h` |

> 另一条硬约束：工程用 ARMCC5（`uAC6=0`），本机验证只能用 `arm-none-eabi-gcc`。
> GCC 比 ARMCC 宽松，所以"GCC 通过"不等于"Keil 通过" —— 尤其是上面这类
> 传递包含问题，必须靠静态审计补齐。

## 7. 已知遗留（不影响本次目标，供后续决定）

- `bsp/bsp_dwt.h` 仍 `#include "main.h"`，会使包含它的 `osal_time.c` 间接引入 HAL 头。
  彻底清理需把 `bsp_dwt` 的 HAL 依赖剥离（改用 `SystemCoreClock` 等）。
- `ser/algo/kinematics.c` 的连杆参数 `g_link` 与 IK 连续性状态 `g_q1_prev/g_q2_prev` 仍是模块级，
  即"单腿单实例"；多腿需把它们并入句柄。
- `ser/dev/can_router.c` 的绑定表是单例（`s_ak` / `s_el05`），多总线/多腿需按 `bus` 建表。
- `common/inc/struct_typedef.h` 已无引用，保留仅为不丢历史文件。
- `MDK-ARM/25_R2_leg.uvoptx`（IDE 视图/断点状态）未同步，Keil 打开后会自动重建。
