/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hr_client_app.c
  * @author  MCD Application Team
  * @brief   Heart Rate Client Application
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

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "app_common.h"

#include "dbg_trace.h"

#include "ble.h"
#include "hr_client_app.h"

#include "stm32_seq.h"
#include "app_ble.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

typedef enum
{
  HEART_RATE_MEASUREMENT_INFO_RECEIVED_EVT,
} HeartRate_Client_Opcode_Notification_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t     Length;
} HeartRate_Client_Data_t;

typedef struct
{
  HeartRate_Client_Opcode_Notification_evt_t  HeartRate_Client_Evt_Opcode;
  HeartRate_Client_Data_t DataTransfered;
} HeartRate_Client_App_Notification_evt_t;

typedef struct
{
  /**
   * state of the HeartRate Client
   * state machine
   */
  APP_BLE_ConnStatus_t state;

  /**
   * connection handle
   */
  uint16_t connHandle;

  /**
   * handle of the HeartRate service
   */
  uint16_t HeartRateServiceHandle;

  /**
   * end handle of the HeartRate service
   */
  uint16_t HeartRateServiceEndHandle;

  /**
   * handle of the Heart Rate Measurement characteristic
   */
  uint16_t HeartRateMeasurementCharHdle;

  /**
   * handle of the client configuration
   * descriptor of Tx characteristic
   */
  uint16_t HeartRateMeasurementDescHandle;

  /**
   * handle of the Body Sensor Location characteristic
   */
  uint16_t BodySensorLocationCharHdle;

  /**
   * handle of the Heart Rate Control Point characteristic
   */
  uint16_t HeartRateControlPointCharHdle;

}HeartRate_ClientContext_t;

/* USER CODE BEGIN PTD */
typedef struct{
  uint8_t                          Write;
  uint16_t                         Notify;
} HeartRate_ButtonCharValue_t;

typedef struct
{
  uint8_t                          Notification_Status; /* used to check if HeartRate Server is enabled to Notify */
  HeartRate_ButtonCharValue_t      ButtonStatus;
  uint16_t                         ConnectionHandle;
  uint8_t                          Disconnect_mgr_timer_Id;
} HeartRate_Client_App_Context_t;

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISCONNECT_TIMEOUT            (0.1*1000*1000/CFG_TS_TICK_VAL) /**< 100ms */
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static HeartRate_ClientContext_t aHeartRateClientContext[BLE_CFG_CLT_MAX_NBR_CB];

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE BEGIN PV */
static HeartRate_Client_App_Context_t HeartRate_Client_App_Context;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void Gatt_Notification(HeartRate_Client_App_Notification_evt_t *pNotification);
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event);
/* USER CODE BEGIN PFP */
static tBleStatus Write_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload);
static tBleStatus Read_Char(uint16_t UUID, uint8_t Service_Instance);
static tBleStatus Toggle_Notify_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload);
static void Button1_Trigger_Received( void );
static void Button2_Trigger_Received( void );
static void Button3_Trigger_Received( void );
static void Update_Service( void );
static void HRC_APP_Disconnect(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void HRC_APP_Init(void)
{
  uint8_t index =0;
/* USER CODE BEGIN HRC_APP_Init_1 */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SEARCH_SERVICE_ID, UTIL_SEQ_RFU, Update_Service );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Button1_Trigger_Received );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SW2_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Button2_Trigger_Received );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SW3_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Button3_Trigger_Received );
  
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(HeartRate_Client_App_Context.Disconnect_mgr_timer_Id), hw_ts_SingleShot, HRC_APP_Disconnect);

  /**
   * Initialize LedButton Service
   */
  HeartRate_Client_App_Context.Notification_Status = 0;
  HeartRate_Client_App_Context.ConnectionHandle = 0x00;

  HeartRate_Client_App_Context.ButtonStatus.Write = 0x00;
  HeartRate_Client_App_Context.ButtonStatus.Notify = 0x0000;
/* USER CODE END HRC_APP_Init_1 */
  for(index = 0; index < BLE_CFG_CLT_MAX_NBR_CB; index++)
  {
    aHeartRateClientContext[index].state = APP_BLE_IDLE;
  }

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterCltHandler(Event_Handler);

#if(CFG_DEBUG_APP_TRACE != 0)
  APP_DBG_MSG("-- HEARTRATE CLIENT INITIALIZED \n");
#endif

/* USER CODE BEGIN HRC_APP_Init_2 */

/* USER CODE END HRC_APP_Init_2 */
  return;
}

void HRC_APP_Notification(HRC_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN HRC_APP_Notification_1 */

/* USER CODE END HRC_APP_Notification_1 */
  switch(pNotification->HR_Evt_Opcode)
  {
/* USER CODE BEGIN HR_Evt_Opcode */

/* USER CODE END HR_Evt_Opcode */

  case HR_CONN_HANDLE_EVT :
/* USER CODE BEGIN HR_CONN_HANDLE_EVT */
    HeartRate_Client_App_Context.ConnectionHandle = pNotification->ConnectionHandle;
/* USER CODE END HR_CONN_HANDLE_EVT */
      break;

    case HR_DISCON_HANDLE_EVT :
/* USER CODE BEGIN HR_DISCON_HANDLE_EVT */
      {
      uint8_t index = 0;
      HeartRate_Client_App_Context.ConnectionHandle = 0x00;
      while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aHeartRateClientContext[index].state != APP_BLE_IDLE))
      {
        aHeartRateClientContext[index].state = APP_BLE_IDLE;
      }
      BSP_LED_Off(LED_BLUE); 
      }
/* USER CODE END HR_DISCON_HANDLE_EVT */
      break;

    default:
/* USER CODE BEGIN HR_Evt_Opcode_Default */

/* USER CODE END HR_Evt_Opcode_Default */
      break;
  }
/* USER CODE BEGIN HRC_APP_Notification_2 */

/* USER CODE END HRC_APP_Notification_2 */
  return;
}
/* USER CODE BEGIN FD */
void HRC_APP_SW1_Button_Action(void)
{
  UTIL_SEQ_SetTask(1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
}

void HRC_APP_SW2_Button_Action(void)
{
  UTIL_SEQ_SetTask(1<<CFG_TASK_SW2_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
}

void HRC_APP_SW3_Button_Action(void)
{
  UTIL_SEQ_SetTask(1<<CFG_TASK_SW3_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
}

static void HRC_APP_Disconnect(void)
{
  UTIL_SEQ_SetTask(1u << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;

  HeartRate_Client_App_Notification_evt_t Notification;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
    {
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch(blecore_evt->ecode)
      {

        case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
        {
          aci_att_read_by_group_type_resp_event_rp0 *pr = (void*)blecore_evt->data;
          uint8_t numServ, i, idx;
          uint16_t uuid, handle;

          uint8_t index;
          handle = pr->Connection_Handle;
          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aHeartRateClientContext[index].state != APP_BLE_IDLE))
          {
            APP_BLE_ConnStatus_t status;

            status = APP_BLE_Get_Client_Connection_Status(aHeartRateClientContext[index].connHandle);

            if((aHeartRateClientContext[index].state == APP_BLE_CONNECTED_CLIENT)&&
                    (status == APP_BLE_IDLE))
            {
              /* Handle deconnected */

              aHeartRateClientContext[index].state = APP_BLE_IDLE;
              aHeartRateClientContext[index].connHandle = 0xFFFF;
              break;
            }
            index++;
          }

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {
            aHeartRateClientContext[index].connHandle = handle;

            numServ = (pr->Data_Length) / pr->Attribute_Data_Length;

            /* the event data will be
             * 2bytes start handle
             * 2bytes end handle
             * 2 or 16 bytes data
             * we are interested only if the UUID is 16 bit.
             * So check if the data length is 6
             */
#if (UUID_128BIT_FORMAT==1)
          if (pr->Attribute_Data_Length == 20)
          {
            idx = 16;
#else
          if (pr->Attribute_Data_Length == 6)
          {
            idx = 4;
#endif
              for (i=0; i<numServ; i++)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
                if(uuid == HEART_RATE_SERVICE_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : HEARTRATE_SERVICE_UUID FOUND - connection handle 0x%x \n", aHeartRateClientContext[index].connHandle);
#endif
#if (UUID_128BIT_FORMAT==1)
                aHeartRateClientContext[index].HeartRateServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-16]);
                aHeartRateClientContext[index].HeartRateServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-14]);
#else
                aHeartRateClientContext[index].HeartRateServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-4]);
                aHeartRateClientContext[index].HeartRateServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-2]);
#endif
                  aHeartRateClientContext[index].state = APP_BLE_DISCOVER_CHARACS ;
                }
                idx += 6;
              }
            }
          }
        }
        break;

        case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
        {

          aci_att_read_by_type_resp_event_rp0 *pr = (void*)blecore_evt->data;
          uint8_t idx;
          uint16_t uuid, handle;

          /* the event data will be
           * 2 bytes start handle
           * 1 byte char properties
           * 2 bytes handle
           * 2 or 16 bytes data
           */

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aHeartRateClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            /* we are interested in only 16 bit UUIDs */
#if (UUID_128BIT_FORMAT==1)
            idx = 17;
            if (pr->Handle_Value_Pair_Length == 21)
#else
              idx = 5;
            if (pr->Handle_Value_Pair_Length == 7)
#endif
            {
              pr->Data_Length -= 1;
              while(pr->Data_Length > 0)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                /* store the characteristic handle not the attribute handle */
#if (UUID_128BIT_FORMAT==1)
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-14]);
#else
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-2]);
#endif
                if(uuid == HEART_RATE_MEASURMENT_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : HEART_RATE_MEASURMENT_UUID FOUND - connection handle 0x%x\n", aHeartRateClientContext[index].connHandle);
#endif
                  aHeartRateClientContext[index].state = APP_BLE_DISCOVER_WRITE_DESC;
                  aHeartRateClientContext[index].HeartRateMeasurementCharHdle = handle;
                }

                else if(uuid == SENSOR_LOCATION_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : SENSOR_LOCATION_UUID FOUND  - connection handle 0x%x\n", aHeartRateClientContext[index].connHandle);
#endif
                  aHeartRateClientContext[index].state = APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC;
                  aHeartRateClientContext[index].BodySensorLocationCharHdle = handle;
                }
                
                else if(uuid == CONTROL_POINT_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : CONTROL_POINT_UUID FOUND  - connection handle 0x%x\n", aHeartRateClientContext[index].connHandle);
#endif
                  aHeartRateClientContext[index].state = APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC;
                  aHeartRateClientContext[index].HeartRateControlPointCharHdle = handle;
                }
#if (UUID_128BIT_FORMAT==1)
                pr->Data_Length -= 21;
                idx += 21;
#else
                pr->Data_Length -= 7;
                idx += 7;
#endif
              }
            }
          }
        }
        break;

        case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
        {
          aci_att_find_info_resp_event_rp0 *pr = (void*)blecore_evt->data;

          uint8_t numDesc, idx, i;
          uint16_t uuid, handle;

          /*
           * event data will be of the format
           * 2 bytes handle
           * 2 bytes UUID
           */

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aHeartRateClientContext[index].connHandle != pr->Connection_Handle))

            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            numDesc = (pr->Event_Data_Length) / 4;
            /* we are interested only in 16 bit UUIDs */
            idx = 0;
            if (pr->Format == UUID_TYPE_16)
            {
              for (i=0; i<numDesc; i++)
              {
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx]);
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx+2]);

                if(uuid == CLIENT_CHARACTERISTIC_CONFIG_DESCRIPTOR_UUID)
                {
#if(CFG_DEBUG_APP_TRACE != 0)
                  APP_DBG_MSG("-- GATT : CLIENT_CHARACTERISTIC_CONFIG_DESCRIPTOR_UUID - connection handle 0x%x\n", aHeartRateClientContext[index].connHandle);
#endif
                  if( aHeartRateClientContext[index].state == APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC)
                  {
                    aHeartRateClientContext[index].HeartRateMeasurementDescHandle = handle;
                    aHeartRateClientContext[index].state = APP_BLE_ENABLE_NOTIFICATION_DESC;
                  }
                }
                idx += 4;
              }
            }
          }
        }
        break; /*ACI_ATT_FIND_INFO_RESP_VSEVT_CODE*/

        case ACI_GATT_NOTIFICATION_VSEVT_CODE:
        {
          aci_gatt_notification_event_rp0 *pr = (void*)blecore_evt->data;
          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aHeartRateClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            if (pr->Attribute_Handle == aHeartRateClientContext[index].HeartRateMeasurementCharHdle)
            {

              Notification.HeartRate_Client_Evt_Opcode = HEART_RATE_MEASUREMENT_INFO_RECEIVED_EVT;
              Notification.DataTransfered.Length = pr->Attribute_Value_Length;
              Notification.DataTransfered.pPayload = &pr->Attribute_Value[0];

              Gatt_Notification(&Notification);

              /* INFORM APPLICATION BUTTON IS PUSHED BY END DEVICE */

            }
          }
        }
        break;/* end ACI_GATT_NOTIFICATION_VSEVT_CODE */

        case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
        {
          aci_gatt_proc_complete_event_rp0 *pr = (void*)blecore_evt->data;
#if(CFG_DEBUG_APP_TRACE != 0)
          APP_DBG_MSG("-- GATT : ACI_GATT_PROC_COMPLETE_VSEVT_CODE - Connection Handle: %04X, Error Code: %02X\n", pr->Connection_Handle, pr->Error_Code);
          APP_DBG_MSG("\n");
#endif

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aHeartRateClientContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_NBR_CB)
          {

            UTIL_SEQ_SetTask( 1<<CFG_TASK_SEARCH_SERVICE_ID, CFG_SCH_PRIO_0);

          }
        }
        break; /*ACI_GATT_PROC_COMPLETE_VSEVT_CODE*/
        
        case ACI_GATT_PROC_TIMEOUT_VSEVT_CODE:
        {
          aci_gatt_proc_timeout_event_rp0 *p_evt_rsp = (void*)blecore_evt->data;
          APP_DBG_MSG("-- GATT : ACI_GATT_PROC_TIMEOUT_VSEVT_CODE - Connection Handle: %04X\n", p_evt_rsp->Connection_Handle);
          HW_TS_Start(HeartRate_Client_App_Context.Disconnect_mgr_timer_Id, (uint32_t) DISCONNECT_TIMEOUT);
        }
        break;/* ACI_GATT_PROC_TIMEOUT_VSEVT_CODE */
        
        case ACI_ATT_READ_RESP_VSEVT_CODE:
        {
          aci_att_read_resp_event_rp0 *pr = (void*)blecore_evt->data;
          
          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aHeartRateClientContext[index].connHandle != pr->Connection_Handle))
            index++;
          
#if(CFG_DEBUG_APP_TRACE != 0)
          APP_DBG_MSG("-- ATT : ACI_ATT_READ_RESP_VSEVT_CODE - connection handle 0x%x - ", aHeartRateClientContext[index].connHandle);
          APP_DBG_MSG("Length: %02d, Data: ", pr->Event_Data_Length);
          for (int i = 0; i < pr->Event_Data_Length; i++)
          {
            APP_DBG_MSG("%02X ", pr->Attribute_Value[i]);
          }
          APP_DBG_MSG(" \n\n\r");
#endif
          
        }
        break; /*ACI_ATT_READ_RESP_VSEVT_CODE*/
        
        default:
          break;
      }
    }

    break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

    default:
      break;
  }

  return(return_value);
}/* end BLE_CTRL_Event_Acknowledged_Status_t */

void Gatt_Notification(HeartRate_Client_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN Gatt_Notification_1*/

/* USER CODE END Gatt_Notification_1 */
  switch(pNotification->HeartRate_Client_Evt_Opcode)
  {
/* USER CODE BEGIN HeartRate_Client_Evt_Opcode */

/* USER CODE END HeartRate_Client_Evt_Opcode */

    case HEART_RATE_MEASUREMENT_INFO_RECEIVED_EVT:
/* USER CODE BEGIN HEART_RATE_MEASUREMENT_INFO_RECEIVED_EVT */
    {
      APP_DBG_MSG("-- HEART RATE APPLICATION CLIENT : NOTIFICATION RECEIVED - ");
      for (int i = 0; i < pNotification->DataTransfered.Length; i++)
      {
        APP_DBG_MSG("%02X ", pNotification->DataTransfered.pPayload[i]);
      }
      APP_DBG_MSG("\n\r");
    }
/* USER CODE END HEART_RATE_MEASUREMENT_INFO_RECEIVED_EVT */
      break;

    default:
/* USER CODE BEGIN HeartRate_Client_Evt_Opcode_Default */

/* USER CODE END HeartRate_Client_Evt_Opcode_Default */
      break;
  }
/* USER CODE BEGIN Gatt_Notification_2*/

/* USER CODE END Gatt_Notification_2 */
  return;
}

uint8_t HeartRate_Client_APP_Get_State( void ) {
  return aHeartRateClientContext[0].state;
}
/* USER CODE BEGIN LF */
tBleStatus Write_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t index;

  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aHeartRateClientContext[index].state != APP_BLE_IDLE))
  {
    switch(UUID)
    {
      case CONTROL_POINT_UUID:
        ret = aci_gatt_write_char_value(aHeartRateClientContext[index].connHandle,
                                         aHeartRateClientContext[index].HeartRateControlPointCharHdle,
                                         1, /* charValueLen */
                                         (uint8_t *)  pPayload);
        break;
      default:
        break;
    }
    index++;
  }

  return ret;
}/* end Write_Char() */

tBleStatus Read_Char(uint16_t UUID, uint8_t Service_Instance)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t index;

  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aHeartRateClientContext[index].state != APP_BLE_IDLE))
  {
    switch(UUID)
    {
      case SENSOR_LOCATION_UUID:        
        ret = aci_gatt_read_char_value(aHeartRateClientContext[index].connHandle,
                                      aHeartRateClientContext[index].BodySensorLocationCharHdle);
        break;
      default:
        break;
    }
    index++;
  }

  return ret;
}/* end Read_Char() */

tBleStatus Toggle_Notify_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t index;

  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aHeartRateClientContext[index].state != APP_BLE_IDLE))
  {
    switch(UUID)
    {
      case HEART_RATE_MEASURMENT_UUID:        
        ret = aci_gatt_write_char_desc(aHeartRateClientContext[index].connHandle,
                                      aHeartRateClientContext[index].HeartRateMeasurementDescHandle,
                                      2, /* descValueLen */
                                      (uint8_t *) pPayload);
        break;
      default:
        break;
    }
    index++;
  }

  return ret;
}/* end Toggle_Notify_Char() */

void Button1_Trigger_Received(void)
{
  HeartRate_Client_App_Context.ButtonStatus.Write = 0x01;
  
  APP_DBG_MSG("-- HEART RATE APPLICATION CLIENT : BUTTON PUSHED - WRITE HEART RATE CONTROL POINT - %02X \n ", HeartRate_Client_App_Context.ButtonStatus.Write);
  APP_DBG_MSG(" \n\r");

  Write_Char( CONTROL_POINT_UUID, 0, (uint8_t *)&HeartRate_Client_App_Context.ButtonStatus);

  return;
}

void Button2_Trigger_Received(void)
{
  APP_DBG_MSG("-- HEART RATE APPLICATION CLIENT : BUTTON PUSHED - READ BODY SENSOR LOCATION \n");

  Read_Char( SENSOR_LOCATION_UUID, 0);

  return;
}

void Button3_Trigger_Received(void)
{
  if(HeartRate_Client_App_Context.ButtonStatus.Notify == 0x0000)
  {
    HeartRate_Client_App_Context.ButtonStatus.Notify = 0x0001;
    APP_DBG_MSG("-- HEART RATE APPLICATION CLIENT : BUTTON PUSHED - ENABLE HEART RATE MEASUREMENT NOTIFICATION \n ");
  }
  else 
  {
    HeartRate_Client_App_Context.ButtonStatus.Notify = 0x0000;
    APP_DBG_MSG("-- HEART RATE APPLICATION CLIENT : BUTTON PUSHED - DISABLE HEART RATE MEASUREMENT NOTIFICATION \n ");
  }
  APP_DBG_MSG(" \n\r");

  Toggle_Notify_Char( HEART_RATE_MEASURMENT_UUID, 0, (uint8_t *)&HeartRate_Client_App_Context.ButtonStatus.Notify);

  return;
}

void Update_Service()
{
  uint16_t enable = 0x0001;
  uint16_t disable = 0x0000;
  uint8_t index;

  index = 0;
  while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
          (aHeartRateClientContext[index].state != APP_BLE_IDLE))
  {
    switch(aHeartRateClientContext[index].state)
    {
      case APP_BLE_DISCOVER_SERVICES:
        APP_DBG_MSG("HEARTRATE_DISCOVER_SERVICES\n");
        break;
      case APP_BLE_DISCOVER_CHARACS:
        APP_DBG_MSG("* GATT : Discover HeartRate Characteristics\n");
        aci_gatt_disc_all_char_of_service(aHeartRateClientContext[index].connHandle,
                                          aHeartRateClientContext[index].HeartRateServiceHandle,
                                          aHeartRateClientContext[index].HeartRateServiceEndHandle);
        break;
      case APP_BLE_DISCOVER_WRITE_DESC: /* Not Used - No descriptor */
        APP_DBG_MSG("* GATT : Discover Descriptor of Heart Rate Control Point Characteristic\n");
        aci_gatt_disc_all_char_desc(aHeartRateClientContext[index].connHandle,
                                    aHeartRateClientContext[index].HeartRateControlPointCharHdle,
                                    aHeartRateClientContext[index].HeartRateControlPointCharHdle+2);
        break;
      case APP_BLE_DISCOVER_NOTIFICATION_CHAR_DESC:
        APP_DBG_MSG("* GATT : Discover Descriptor of Heart Rate Measurement Characteristic\n");
        aci_gatt_disc_all_char_desc(aHeartRateClientContext[index].connHandle,
                                    aHeartRateClientContext[index].HeartRateMeasurementCharHdle,
                                    aHeartRateClientContext[index].HeartRateMeasurementCharHdle+2);
        break;
      case APP_BLE_ENABLE_NOTIFICATION_DESC:
        APP_DBG_MSG("* GATT : Enable Heart Rate Measurement Notification\n");
        aci_gatt_write_char_desc(aHeartRateClientContext[index].connHandle,
                                 aHeartRateClientContext[index].HeartRateMeasurementDescHandle,
                                 2,
                                 (uint8_t *)&enable);
        HeartRate_Client_App_Context.ButtonStatus.Notify = 0x0001;
        aHeartRateClientContext[index].state = APP_BLE_CONNECTED_CLIENT;
        BSP_LED_Off(LED_RED);
        break;
      case APP_BLE_DISABLE_NOTIFICATION_DESC :
        APP_DBG_MSG("* GATT : Disable Heart Rate Measurement Notification\n");
        aci_gatt_write_char_desc(aHeartRateClientContext[index].connHandle,
                                 aHeartRateClientContext[index].HeartRateMeasurementDescHandle,
                                 2,
                                 (uint8_t *)&disable);
        aHeartRateClientContext[index].state = APP_BLE_CONNECTED_CLIENT;
        break;
      default:
        break;
    }
    index++;
  }
  return;
}
/* USER CODE END LF */
