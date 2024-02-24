#ifndef __DW3K_H__
#define __DW3K_H__

#include "dw3k_conn.h"
#include "dw3k_disp.h"

#define DW3K_SET_ANCHOR 1
#define DW3K_SET_TAG    2

#define DW3K_CONN_TASK_STACK_SIZE     25600
#define DW3K_CONN_TASK_PRIORITY       1
#define DW3K_CONN_TASK_CORE           0
#define DW3K_CONN_TASK_DELAY          20
#define DW3K_CONN_TASK_REQ_QUEUE_LEN  8
#define DW3K_CONN_TASK_REQ_WAIT_MS    10

#define DW3K_CONN_TASK_STATE_IDLE             0
#define DW3K_CONN_TASK_STATE_ON_WIFI_CONNECT  1

#define DW3K_CONN_TASK_REQ_NONE 0

#define DW3K_DISP_TASK_STACK_SIZE     12800
#define DW3K_DISP_TASK_PRIORITY       1
#define DW3K_DISP_TASK_CORE           1
#define DW3K_DISP_TASK_DELAY          40
#define DW3K_DISP_TASK_REQ_QUEUE_LEN  8
#define DW3K_DISP_TASK_REQ_WAIT_MS    10

#define DW3K_DISP_TASK_STATE_IDLE                 0
#define DW3K_DISP_TASK_STATE_SPLASH               1
#define DW3K_DISP_TASK_STATE_ON_WIFI_CONNECT      2
#define DW3K_DISP_TASK_STATE_WIFI_CONNECT_SUCCESS 3
#define DW3K_DISP_TASK_STATE_DISTANCE             4

#define DW3K_DISP_TASK_REQ_NONE                 0
#define DW3K_DISP_TASK_REQ_ON_WIFI_CONNECT      1
#define DW3K_DISP_TASK_REQ_WIFI_CONNECT_SUCCESS 2

// FreeRTOS task handler
extern TaskHandle_t dw3k_conn_task_handler,
                    dw3k_disp_task_handler;

// FreeRTOS queue handler
extern QueueHandle_t  dw3k_conn_req_queue,
                      dw3k_disp_req_queue;

// Queue messages
extern String dw3k_conn_req_msgs[2];

// Connection task
void dw3k_conn_task(void* pvParameters);

// Display task
void dw3k_disp_task(void* pvParameters);

// Function to start all the tasks
void dw3k_begin(uint8_t *uwb_mode);

#endif