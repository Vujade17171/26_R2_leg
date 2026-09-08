/**
  ******************************************************************************
  * @file    motor_app.c
  * @brief   Application layer: drive 2 MIT motors (AK80-9 + AK45-10)
  ******************************************************************************
  * Description:
  *   - FDCAN1 on PD0/PD1, both MIT motors share the same bus.
  *   - Each motor is a separate row in the MIT config table (mit_motor_add).
  *   - Watch in Keil: mit_motor_state[0] (ID1) and mit_motor_state[1] (ID2).
  ******************************************************************************
  */
#include "motor_app.h"
#include "main.h"
#include "fdcan_drv.h"
#include "mit_motor.h"

extern FDCAN_HandleTypeDef hfdcan1;

#define CTRL_PERIOD  10        /* control period ms */
#define TEST_VEL     1.0f      /* slow constant speed, rad/s */
#define TEST_KD      2.0f      /* velocity damping */

static uint32_t s_last_tick = 0;
static uint8_t  s_inited = 0;

volatile uint32_t g_rx_fifo_level = 0;
uint32_t          g_mit_tx_cnt = 0;

void motor_app_init(void)
{
    mit_motor_cfg_t cfg;

    /* 1. init universal FDCAN bus (filters + start) */
    fdcan_drv_init(&hfdcan1);

    /* 2. init unified MIT driver (register rx callback) */
    mit_motor_init(&hfdcan1);

    /* 3. add motor configs */

    /* AK80-9 (ID=1) - da bi */
    cfg.id      = 1;
    cfg.p_min   = -12.5f;   cfg.p_max   = 12.5f;
    cfg.v_min   = -50.0f;   cfg.v_max   = 50.0f;
    cfg.t_min   = -18.0f;   cfg.t_max   = 18.0f;
    cfg.kp_min  = 0.0f;     cfg.kp_max  = 500.0f;
    cfg.kd_min  = 0.0f;     cfg.kd_max  = 5.0f;
    cfg.sign    = +1;
    mit_motor_add(&cfg);

    /* AK45-10 (ID=2) - xiao bi */
    cfg.id      = 2;
    cfg.p_min   = -12.6f;   cfg.p_max   = 12.6f;
    cfg.v_min   = -8.0f;    cfg.v_max   = 8.0f;   /* 参考已测试工程 */
    cfg.t_min   = -7.0f;    cfg.t_max   = 7.0f;
    cfg.kp_min  = 0.0f;     cfg.kp_max  = 500.0f;
    cfg.kd_min  = 0.0f;     cfg.kd_max  = 5.0f;
    cfg.sign    = -1;                            /* 参考工程反方向 */
    mit_motor_add(&cfg);

    /* 4. enable both motors to MIT run state */
    mit_motor_enable(&hfdcan1, 1);
    mit_motor_enable(&hfdcan1, 2);

    s_inited = 1;
}

void motor_app_run(void)
{
    uint32_t now = HAL_GetTick();

    if (s_inited == 0) { return; }

    if ((now - s_last_tick) >= CTRL_PERIOD)
    {
        s_last_tick = now;

        /* slow constant-speed (velocity mode, Kp=0) for both motors */
        mit_motor_set_control(&hfdcan1, 1, 0.0f, TEST_VEL, 0.0f, TEST_KD, 0.0f);
        mit_motor_set_control(&hfdcan1, 2, 0.0f, TEST_VEL, 0.0f, TEST_KD, 0.0f);

        g_mit_tx_cnt++;
    }

    /* RX diagnostic: FIFO0 fill level */
    g_rx_fifo_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
}