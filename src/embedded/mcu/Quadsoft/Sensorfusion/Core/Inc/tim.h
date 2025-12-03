/* Minimal tim.h stub to satisfy MEMS code when CubeMX-generated tim.h is missing.
 * Provides declaration for TIM handle and MX_TIM2_Init used by BSP_IP mappings.
 * Replace with full CubeMX-generated tim.h when available.
 */

#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Declaration of the timer handle used by the MEMS BSP mapping (htim2)
   and the MX_TIM2_Init prototype. If your application uses a different
   timer, update these accordingly or use the CubeMX-generated tim.h. */
extern TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
