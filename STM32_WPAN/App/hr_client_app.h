/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hr_client_app.h
  * @author  MCD Application Team
  * @brief   Header for hr_client_app.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HR_APPLICATION_H
#define HR_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  HR_CONN_HANDLE_EVT,
  HR_DISCON_HANDLE_EVT,
} HRC_APP_Opcode_Notification_evt_t;

typedef struct
{
  HRC_APP_Opcode_Notification_evt_t           HR_Evt_Opcode;
  uint16_t                                    ConnectionHandle;

} HRC_APP_ConnHandle_Not_evt_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void HRC_APP_Init( void );
void HRC_APP_Notification( HRC_APP_ConnHandle_Not_evt_t *pNotification );
uint8_t HeartRate_Client_APP_Get_State( void );
/* USER CODE BEGIN EFP */
void HRC_APP_SW1_Button_Action(void);
void HRC_APP_SW2_Button_Action(void);
void HRC_APP_SW3_Button_Action(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*HR_APPLICATION_H */
