#include "Moto_Task.h"
float position,speed=1,kp,kd=1,torque;
void Moto_Diver(void *argument)
{
  /* USER CODE BEGIN Moto_Diver */
	can_filter_init();
	AK_Motion_Init(&g_ak80,&hfdcan1, AK80_ID);
	AK_Motion_Enter(&g_ak80);
  /* Infinite loop */
  for(;;)
  {
		AK_Motion_Control(&g_ak80,position,speed,kp,kd,torque);
    osDelay(1);
  }
  /* USER CODE END Moto_Diver */
}
