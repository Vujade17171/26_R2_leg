/*****************************************************************************
 * rc_receiver.c —— 遥控接收机服务实现（ser/dev）
 *
 * 数据流：
 *   USART1 总线空闲
 *     -> USART1_IRQHandler -> BSP_UART_IRQHandler()（bsp 只读 NDTR 算出本帧长度）
 *       -> 本文件的 RC_OnFrame()（static，bsp 通过注册回调调到）
 *         -> sbus_to_rc() 解包
 *           -> 写进 app 通过 RC_RegisterHandle() 注入的 RC_Data
 *
 * 协议（移植自 common-drivers / 遥控器驱动 / Myremote.c 的 sbus_to_rc）：
 *   32 字节一帧：
 *     [0]=0x20  [1]=0x40                     帧头
 *     [2..29]  8 个 11 位通道（小端跨字节）
 *     [30][31] 前 30 字节累加和取反（小端 16 位）
 *
 * 本文件是本模块唯一能看到 bsp / HAL 的地方 —— rc_receiver.h 里没有它们。
 *****************************************************************************/
#include "rc_receiver.h"

#include "bsp_uart.h"   /* 只有本 .c 看得见 bsp */

/* ==================== 协议常量（私有，app 看不到） ==================== */
#define RC_FRAME_LEN   32u   /* 一帧字节数（参考 RC_FRAME_LENGTH）*/

#define RC_CH_MID      1500  /* 摇杆通道中值 */

/* ==================== 本模块持有的资源 ==================== */
/* 环形接收缓冲：归本模块所有，app 不需要（也不应该）关心。
 * 大小 = 2 倍帧长 —— 这样一帧在缓冲里始终连续、不会跨越回绕边界。 */
#define RC_BUF_SIZE    (2u * RC_FRAME_LEN)
static uint8_t  s_rc_buf[RC_BUF_SIZE];

/* app 注入的句柄（对应 CAN_Router 里的 s_vesc） */
static RC_Data *s_rc;

/* --------------------------------------------------------------------------
 * 单帧解包（移植自参考实现的 static int sbus_to_rc()）
 * 返回 SER_OK / SER_ERR_LIMIT（帧头或校验不过）
 * -------------------------------------------------------------------------- */
static int32_t sbus_to_rc(const uint8_t *buf, uint16_t len, RC_Data *out)
{
    uint16_t calc = 0u;
    uint16_t ori;
    uint8_t  i;
    uint16_t v;

    if ((buf == 0) || (out == 0)) { return SER_ERR_PARAM; }
    if (len != RC_FRAME_LEN)      { return SER_ERR_LIMIT; }   /* 长度不对：丢帧 */

    /* ---- 帧头 ---- */
    if (buf[0] != 0x20u) { return SER_ERR_LIMIT; }
    if (buf[1] != 0x40u) { return SER_ERR_LIMIT; }

    /* ---- 校验：前 30 字节累加和取反 == 后两字节（小端） ---- */
    for (i = 0u; i < 30u; i++) { calc = (uint16_t)(calc + (uint16_t)buf[i]); }
    calc ^= 0xFFFFu;
    ori   = (uint16_t)((uint16_t)buf[30] + ((uint16_t)buf[31] << 8));
    if (calc != ori) { return SER_ERR_LIMIT; }

    /* ---- 三个摇杆通道（减中值 1500） ---- */
    v = (uint16_t)((uint16_t)buf[0 * 2 + 2] + (((uint16_t)buf[0 * 2 + 3] & 0x0Fu) << 8));
    out->ch[0] = (int16_t)((int32_t)v - RC_CH_MID);

    v = (uint16_t)((uint16_t)buf[1 * 2 + 2] + (((uint16_t)buf[1 * 2 + 3] & 0x0Fu) << 8));
    out->ch[1] = (int16_t)((int32_t)v - RC_CH_MID);

    v = (uint16_t)((uint16_t)buf[3 * 2 + 2] + (((uint16_t)buf[3 * 2 + 3] & 0x0Fu) << 8));
    out->ch[2] = (int16_t)((int32_t)v - RC_CH_MID);

    /* ---- 拨轮原始值 ---- */
    v = (uint16_t)((uint16_t)buf[2 * 2 + 2] + (((uint16_t)buf[2 * 2 + 3] & 0x0Fu) << 8));
    out->num = (int16_t)v;

    /* ---- 4 个拨杆开关 ----
     * 照参考实现的判断条件：值不等于 1000/1500/2000 时【保留上一次的值】。
     * 若希望"非预期值一律当 0"，给每个分支补 else { out->s[x] = 0; } 即可。 */
    v = (uint16_t)((uint16_t)buf[4 * 2 + 2] + (((uint16_t)buf[4 * 2 + 3] & 0x0Fu) << 8));
    if      (v == 2000u) { out->s[0] = 1; }
    else if (v == 1000u) { out->s[0] = 0; }

    v = (uint16_t)((uint16_t)buf[5 * 2 + 2] + (((uint16_t)buf[5 * 2 + 3] & 0x0Fu) << 8));
    if      (v == 2000u) { out->s[1] = 2; }
    else if (v == 1000u) { out->s[1] = 0; }
    else if (v == 1500u) { out->s[1] = 1; }
    else                 { out->s[1] = 0; }

    v = (uint16_t)((uint16_t)buf[6 * 2 + 2] + (((uint16_t)buf[6 * 2 + 3] & 0x0Fu) << 8));
    if      (v == 2000u) { out->s[2] = 1; }
    else if (v == 1000u) { out->s[2] = 0; }

    v = (uint16_t)((uint16_t)buf[7 * 2 + 2] + (((uint16_t)buf[7 * 2 + 3] & 0x0Fu) << 8));
    if      (v == 2000u) { out->s[3] = 1; }
    else if (v == 1000u) { out->s[3] = 0; }

    out->frames++;
    return SER_OK;
}

/* --------------------------------------------------------------------------
 * bsp_uart 的接收回调 —— 【static】：app 看不到，也不需要看到
 * 中断上下文：只做解包，不打印 / 不延时 / 不调 OS API
 * -------------------------------------------------------------------------- */
static void RC_OnFrame(bsp_uart_bus_t bus, const uint8_t *data, uint16_t len)
{
    (void)bus;                          /* 当前只挂了一条串口 */

    if (s_rc == 0) { return; }          /* 还没注册句柄：丢弃 */

    (void)sbus_to_rc(data, len, s_rc);  /* 不合格的帧已返回错误码，直接忽略 */
}

/* ==========================================================================
 * 面向 app 的 3 个函数
 * ========================================================================== */

/* ② 注册句柄（依赖注入，对应 CAN_Router_BindAk） */
int32_t RC_RegisterHandle(RC_Data *rc)
{
    if (rc == 0) { return SER_ERR_PARAM; }

    /* 清零，避免上电瞬间读到脏数据 */
    rc->ch[0] = 0; rc->ch[1] = 0; rc->ch[2] = 0;
    rc->num   = 0;
    rc->s[0]  = 0; rc->s[1] = 0; rc->s[2] = 0; rc->s[3] = 0;
    rc->frames = 0u;

    s_rc = rc;
    return SER_OK;
}

/* ③ 初始化：启动接收
 *    注意第 4 个参数是【一帧字节数】（= 单块缓冲区大小），不是缓冲区大小上限。 */
int32_t RC_Init(void)
{
    int32_t rc;

    if (s_rc == 0) { return SER_ERR_STATE; }   /* 必须先注册句柄 */

    /* 顺序很重要：先注册回调，再启动接收 —— 反了会丢最开始几帧 */
    rc = BSP_UART_RegisterRx(BSP_UART_BUS_1, RC_OnFrame);
    if (rc != SER_OK) { return rc; }

    return BSP_UART_Init(BSP_UART_BUS_1, s_rc_buf, RC_BUF_SIZE);
}

/* ④ 获取遥控器数据 */
const RC_Data *RC_Get(void)
{
    return s_rc;    /* 未注册时为 0 */
}
