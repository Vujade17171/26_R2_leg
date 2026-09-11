/**
  ******************************************************************************
  * @file    motor_app.c
  * @brief   Application layer: test modes and robot-arm entry
  ******************************************************************************
  * Modes:
  *   0 = normal arm control
  *   1 = EL05-only test (extended frame, master id 0xFF)
  *   2 = all three motors: feedback-only safe test (zero torque / zero gains)
  ******************************************************************************
  */
#include "motor_app.h"
#include "main.h"
#include "arm_control.h"
#include "fdcan_drv.h"
#include "mit_motor.h"
#include "robstride.h"

#define MOTOR_APP_TEST_MODE  2

extern FDCAN_HandleTypeDef hfdcan1;

/* App-layer receive diagnostic: does not control any motor. */
motor_app_can_dbg_t motor_app_can_dbg = {0};

static void motor_app_rx_diag_cb(FDCAN_HandleTypeDef *hfdcan,
                                 FDCAN_RxHeaderTypeDef *rx_header,
                                 uint8_t *rx_data)
{
    uint8_t i;
    uint32_t com;

    (void)hfdcan;

    motor_app_can_dbg.rx_total++;
    motor_app_can_dbg.last_id_type = (uint8_t)rx_header->IdType;
    motor_app_can_dbg.last_id      = rx_header->Identifier;

    if (rx_header->IdType != FDCAN_EXTENDED_ID)
    {
        motor_app_can_dbg.rx_std++;
        return;
    }

    motor_app_can_dbg.rx_ext++;
    motor_app_can_dbg.last_ext_id  = rx_header->Identifier;
    motor_app_can_dbg.last_ext_fid = (uint8_t)((rx_header->Identifier & 0xFF00U) >> 8);
    motor_app_can_dbg.last_ext_dlc = (uint8_t)rx_header->DataLength;
    com = (rx_header->Identifier & 0x3F000000U) >> 24;
    motor_app_can_dbg.last_ext_com = (uint8_t)com;

    for (i = 0U; i < 8U; i++)
    {
        motor_app_can_dbg.last_ext_data[i] = rx_data[i];
    }

    if (com == 2U)
    {
        motor_app_can_dbg.rx_com2++;
        motor_app_can_dbg.last_com2_id = rx_header->Identifier;
        motor_app_can_dbg.com2_fid     = motor_app_can_dbg.last_ext_fid;
        for (i = 0U; i < 8U; i++)
        {
            motor_app_can_dbg.com2_data[i] = rx_data[i];
        }
    }
}

#if (MOTOR_APP_TEST_MODE == 1)
/* ---------- EL05-only test ---------- */
static uint8_t s_el05_test_ready = 0U;

static void el05_only_init(void)
{
    robstride_cfg_t cfg = {0};

    if (fdcan_drv_init(&hfdcan1) != 0U) { return; }
    if (robstride_init(&hfdcan1) != 0U) { return; }

    cfg.id        = 3U;
    cfg.p_min     = -12.57f; cfg.p_max = 12.57f;
    cfg.v_min     = -50.0f;  cfg.v_max = 50.0f;
    cfg.t_min     = -6.0f;   cfg.t_max = 6.0f;
    cfg.kp_min    = 0.0f;    cfg.kp_max = 500.0f;
    cfg.kd_min    = 0.0f;    cfg.kd_max = 5.0f;
    cfg.sign      = +1;
    cfg.master_id = 0xFFU;

    if (robstride_add(&cfg) < 0) { return; }

    s_el05_test_ready = 1U;
    HAL_Delay(800U);
    robstride_enable(&hfdcan1, 3U);
}

static void el05_only_run(uint32_t now_ms)
{
    static uint32_t last_enable_ms  = 0U;
    static uint32_t last_control_ms = 0U;

    if (s_el05_test_ready == 0U) { return; }

    if ((uint32_t)(now_ms - last_enable_ms) >= 200U)
    {
        robstride_enable(&hfdcan1, 3U);
        last_enable_ms = now_ms;
    }

    if ((uint32_t)(now_ms - last_control_ms) >= 10U)
    {
        robstride_set_control(&hfdcan1, 3U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        last_control_ms = now_ms;
    }
}
#elif (MOTOR_APP_TEST_MODE == 2)
/* ---------- three-motor feedback-only safe test ---------- */
static uint8_t s_all_test_ready = 0U;

static void all_motor_safe_test_init(void)
{
    mit_motor_cfg_t mcfg = {0};
    robstride_cfg_t rcfg = {0};

    if (fdcan_drv_init(&hfdcan1) != 0U) { return; }
    if (mit_motor_init(&hfdcan1) != 0U) { return; }
    if (robstride_init(&hfdcan1) != 0U) { return; }

    /* AK80-9 shoulder, ID=1, standard frame */
    mcfg.id    = 1U;
    mcfg.p_min = -12.5f; mcfg.p_max = 12.5f;
    mcfg.v_min = -50.0f; mcfg.v_max = 50.0f;
    mcfg.t_min = -18.0f; mcfg.t_max = 18.0f;
    mcfg.kp_min = 0.0f;  mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;  mcfg.kd_max = 5.0f;
    mcfg.sign = +1;
    if (mit_motor_add(&mcfg) < 0) { return; }

    /* AK45-10 elbow, ID=2, standard frame */
    mcfg.id    = 2U;
    mcfg.p_min = -12.6f; mcfg.p_max = 12.6f;
    mcfg.v_min = -8.0f;  mcfg.v_max = 8.0f;
    mcfg.t_min = -7.0f;  mcfg.t_max = 7.0f;
    mcfg.kp_min = 0.0f;  mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;  mcfg.kd_max = 5.0f;
    mcfg.sign = +1;
    if (mit_motor_add(&mcfg) < 0) { return; }

    /* Lingzu EL05 wrist, ID=3, extended frame */
    rcfg.id        = 3U;
    rcfg.p_min     = -12.57f; rcfg.p_max = 12.57f;
    rcfg.v_min     = -50.0f;  rcfg.v_max = 50.0f;
    rcfg.t_min     = -6.0f;   rcfg.t_max = 6.0f;
    rcfg.kp_min    = 0.0f;    rcfg.kp_max = 500.0f;
    rcfg.kd_min    = 0.0f;    rcfg.kd_max = 5.0f;
    rcfg.sign      = +1;
    rcfg.master_id = 0xFFU;
    if (robstride_add(&rcfg) < 0) { return; }

    s_all_test_ready = 1U;
    HAL_Delay(800U);

    /* Give EL05 the first enable slot, then enable the two AK motors. */
    robstride_enable(&hfdcan1, 3U);
    mit_motor_enable(&hfdcan1, 1U);
    mit_motor_enable(&hfdcan1, 2U);
}

static void all_motor_safe_test_run(uint32_t now_ms)
{
    static uint32_t last_el05_enable_ms = 0U;
    static uint32_t last_control_ms = 0U;
    uint32_t free_level;

    if (s_all_test_ready == 0U) { return; }

    /* EL05 is stable in mode 1 with a periodic enable.  Keep the same
       behavior here, but only for EL05 so we do not refill the Tx FIFO
       with six frames on the same tick. */
    if ((uint32_t)(now_ms - last_el05_enable_ms) >= 200U)
    {
        free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        if (free_level != 0U)
        {
            robstride_enable(&hfdcan1, 3U);
            motor_app_can_dbg.tx_el05_enable_cnt++;
        }
        else
        {
            motor_app_can_dbg.tx_el05_skipped++;
        }
        last_el05_enable_ms = now_ms;
    }

    if ((uint32_t)(now_ms - last_control_ms) >= 10U)
    {
        /* Send EL05 first.  In mode 2 it used to be the last of three
           control frames, so a full 4-entry Tx FIFO could drop exactly
           the EL05 frame. */
        free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        if (free_level != 0U)
        {
            robstride_set_control(&hfdcan1, 3U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            motor_app_can_dbg.tx_el05_control_cnt++;
        }
        else
        {
            motor_app_can_dbg.tx_el05_skipped++;
        }

        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0U)
        {
            mit_motor_set_control(&hfdcan1, 1U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            motor_app_can_dbg.tx_ak1_control_cnt++;
        }

        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0U)
        {
            mit_motor_set_control(&hfdcan1, 2U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            motor_app_can_dbg.tx_ak2_control_cnt++;
        }

        last_control_ms = now_ms;
    }

    if ((mit_motor_state[0].online != 0U) &&
        ((uint32_t)(now_ms - mit_motor_state[0].last_rx_ms) > 200U))
    {
        mit_motor_state[0].online = 0U;
    }
    if ((mit_motor_state[1].online != 0U) &&
        ((uint32_t)(now_ms - mit_motor_state[1].last_rx_ms) > 200U))
    {
        mit_motor_state[1].online = 0U;
    }
    if ((robstride_state[0].online != 0U) &&
        ((uint32_t)(now_ms - robstride_state[0].last_rx_ms) > 200U))
    {
        robstride_state[0].online = 0U;
    }
}
#endif

void motor_app_init(void)
{
    /* Register diagnostics before starting the bus. */
    fdcan_drv_reg_rx_cb(&hfdcan1, motor_app_rx_diag_cb);

#if (MOTOR_APP_TEST_MODE == 1)
    el05_only_init();
#elif (MOTOR_APP_TEST_MODE == 2)
    all_motor_safe_test_init();
#else
    /* Let the EL05 finish power-up before its first enable frame. */
    HAL_Delay(800U);
    arm_init(&hfdcan1);
#endif
}

void motor_app_run(void)
{
    static uint32_t last_run_ms = 0U;
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_run_ms) < 10U)
    {
        return;
    }

    last_run_ms = now_ms;

#if (MOTOR_APP_TEST_MODE == 1)
    el05_only_run(now_ms);
#elif (MOTOR_APP_TEST_MODE == 2)
    all_motor_safe_test_run(now_ms);
#else
    arm_run();
#endif
}