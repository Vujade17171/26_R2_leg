/*****************************************************************************
 * can_router.c —— CAN 帧路由实现
 *
 * 数据流（上行）：bsp 中断取帧 -> 本文件按帧格式分流 -> 各驱动解析到句柄
 * 依赖方向：can_router -> bsp_can（注册）、can_router -> ak/el05 驱动
 * 本文件不引用任何 app 层符号。
 *****************************************************************************/
#include "can_router.h"

/* 绑定的句柄数组（由 app 初始化时注入，本模块不定义实例） */
static AK_Motor *s_ak;
static uint8_t s_ak_num;
static EL05_Motor *s_el05;
static uint8_t s_el05_num;
static VESC_Motor *s_vesc;
static uint8_t s_vesc_num;
static bsp_can_bus_t s_bus;

int32_t CAN_Router_Init(bsp_can_bus_t bus)
{
    s_bus = bus;
    return BSP_CAN_RegisterRx(bus, CAN_Router_OnFrame);
}

int32_t VESC_CAN_Router_Init(bsp_can_bus_t bus)
{
    s_bus = bus;
    return BSP_CAN_RegisterRx(bus, VESC_CAN_Router_OnFrame); // VESC回调接口
}

int32_t CAN_Router_BindAk(AK_Motor *motors, uint8_t count)
{
    if ((motors == 0) || (count == 0u))
    {
        return SER_ERR_PARAM;
    }
    s_ak = motors;
    s_ak_num = count;
    return SER_OK;
}

int32_t CAN_Router_BindEl05(EL05_Motor *motors, uint8_t count)
{
    if ((motors == 0) || (count == 0u))
    {
        return SER_ERR_PARAM;
    }
    s_el05 = motors;
    s_el05_num = count;
    return SER_OK;
}

int32_t CAN_Router_VESC(VESC_Motor *motors, uint8_t count)
{
    if ((motors == 0) || (count == 0u))
    {
        return SER_ERR_PARAM;
    }
    s_vesc = motors;
    s_vesc_num = count;
    return SER_OK;
}

uint32_t CAN_Router_GetFrameCount(void)
{
    return BSP_CAN_GetRxFrameCount(s_bus);
}

/* 中断上下文：不做打印 / 延时 / OS 调用 */
void CAN_Router_OnFrame(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                        const uint8_t *data, uint8_t len)
{
    (void)bus; /* 当前只有一条总线；将来多总线时按 bus 选择设备表 */

    if ((data == 0) || (len < 8u))
    {
        return;
    }

    if (id_type == (uint32_t)BSP_CAN_ID_STD)
    {
        if (s_ak != 0)
        {
            (void)AK_Motor_RxDispatch(s_ak, s_ak_num, id, data, len);
        }
    }
    else
    {
        if (s_el05 != 0)
        {
            (void)EL05_RxDispatch(s_el05, s_el05_num, id, data, len);
        }
    }
}

void VESC_CAN_Router_OnFrame(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                             const uint8_t *data, uint8_t len)
{
    (void)bus;
    (void)id_type;

    if ((data == 0) || (len < 8u))
    {
        return;
    }

    if (s_vesc != 0)
    {
        (void)VESC_Motor_RxDispatch(s_vesc, s_vesc_num, id, data, len);
    }
}