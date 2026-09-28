/*****************************************************************************
 * osal_task.c —— 任务与节拍抽象实现
 *
 * 依赖方向：osal -> cmsis_os2（RTOS 原语）
 *****************************************************************************/
#include "osal_task.h"

#include "cmsis_os2.h"

osal_task_t OSAL_Task_Create(const char *name, osal_task_entry_t entry, void *arg,
                             uint32_t stack_bytes, uint32_t prio)
{
    osThreadAttr_t attr;

    if ((name == 0) || (entry == 0) || (stack_bytes == 0u)) { return 0; }

    attr.name       = name;
    attr.attr_bits  = 0u;
    attr.cb_mem     = 0;
    attr.cb_size    = 0u;
    attr.stack_mem  = 0;
    attr.stack_size = stack_bytes;
    attr.priority   = (osPriority_t)prio;
    attr.tz_module  = 0u;
    attr.reserved   = 0u;

    return (osal_task_t)osThreadNew(entry, arg, &attr);
}

int32_t OSAL_Task_Destroy(osal_task_t task)
{
    if (task == 0) { return SER_ERR_PARAM; }
    return (osThreadTerminate((osThreadId_t)task) == osOK) ? SER_OK : SER_ERR_STATE;
}

int32_t OSAL_Task_DelayUntil(uint32_t *tick, uint32_t inc)
{
    if ((tick == 0) || (inc == 0u)) { return SER_ERR_PARAM; }
    *tick += inc;
    return (osDelayUntil(*tick) == osOK) ? SER_OK : SER_ERR_STATE;
}