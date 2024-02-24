#include "../include/dw3k.h"



TaskHandle_t  dw3k_conn_task_handler,
              dw3k_disp_task_handler;

QueueHandle_t dw3k_conn_req_queue,
              dw3k_disp_req_queue;

String dw3k_conn_req_msgs[2];



void dw3k_conn_task(void* pvParameters) {

  randomSeed(micros());

  uint8_t dw3k_mode     = *((uint8_t*)pvParameters),
          task_state    = (dw3k_mode == DW3K_SET_ANCHOR) ? DW3K_CONN_TASK_STATE_ON_WIFI_CONNECT : DW3K_CONN_TASK_STATE_IDLE,
          task_in_req   = DW3K_CONN_TASK_REQ_NONE,
          task_out_req  = 0;

  String  ssid_uid = String(esp_random() % 256);

  DW3KConn  dw3k_conn = DW3KConn(dw3k_mode);

  while(true) {

    xQueueReceive(dw3k_conn_req_queue, &task_in_req, pdMS_TO_TICKS(DW3K_CONN_TASK_REQ_WAIT_MS));
    switch(task_in_req) {

      case DW3K_CONN_TASK_REQ_NONE:
        break;
    }
    task_in_req = DW3K_CONN_TASK_REQ_NONE;



    switch(task_state) {

      case DW3K_CONN_TASK_STATE_IDLE:
        break;


      
      case DW3K_CONN_TASK_STATE_ON_WIFI_CONNECT:

        dw3k_conn_req_msgs[0] = String(DW3K_CONN_WIFI_SSID) + ssid_uid;
        task_out_req          = DW3K_DISP_TASK_REQ_ON_WIFI_CONNECT;
        xQueueSend(dw3k_disp_req_queue, &task_out_req, portMAX_DELAY);

        if(!dw3k_conn.runWiFiMan(ssid_uid)) {
          break;
        }

        dw3k_conn_req_msgs[1] = WiFi.SSID();
        task_out_req          = DW3K_DISP_TASK_REQ_WIFI_CONNECT_SUCCESS;
        xQueueSend(dw3k_disp_req_queue, &task_out_req, portMAX_DELAY);

        task_state = DW3K_CONN_TASK_STATE_IDLE;
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(DW3K_CONN_TASK_DELAY));
  }
}



void dw3k_disp_task(void* pvParameters) {
  
  uint8_t dw3k_mode     = *((uint8_t*)pvParameters),
          task_state    = DW3K_DISP_TASK_STATE_SPLASH,
          task_in_req   = DW3K_DISP_TASK_REQ_NONE,
          task_out_req  = 0;

  DW3KDisp  dw3k_disp = DW3KDisp(dw3k_mode);

  while(true) {

    xQueueReceive(dw3k_disp_req_queue, &task_in_req, pdMS_TO_TICKS(DW3K_DISP_TASK_REQ_WAIT_MS));
    switch(task_in_req) {

      case DW3K_DISP_TASK_REQ_NONE:
        break;



      case DW3K_DISP_TASK_REQ_ON_WIFI_CONNECT:
        task_state = DW3K_DISP_TASK_STATE_ON_WIFI_CONNECT;
        break;



      case DW3K_DISP_TASK_REQ_WIFI_CONNECT_SUCCESS:
        task_state = DW3K_DISP_TASK_STATE_WIFI_CONNECT_SUCCESS;
        break;
    }
    task_in_req = DW3K_DISP_TASK_REQ_NONE;



    switch(task_state) {

      case DW3K_DISP_TASK_STATE_IDLE:
        break;



      case DW3K_DISP_TASK_STATE_SPLASH:
        
        dw3k_disp.changeView(DW3K_DISP_VIEW_SPLASH);
        dw3k_disp.refreshView();

        vTaskDelay(pdMS_TO_TICKS(2000));
        task_state = DW3K_DISP_TASK_STATE_IDLE;
        break;


      
      case DW3K_DISP_TASK_STATE_ON_WIFI_CONNECT:

        dw3k_disp.changeView(DW3K_DISP_VIEW_ON_WIFI_CONNECT, dw3k_conn_req_msgs[0].c_str());
        dw3k_disp.refreshView();

        task_state = DW3K_DISP_TASK_STATE_IDLE;   
        break;


      
      case DW3K_DISP_TASK_STATE_WIFI_CONNECT_SUCCESS:

        dw3k_disp.changeView(DW3K_DISP_VIEW_WIFI_CONNECT_SUCCESS, dw3k_conn_req_msgs[1].c_str());
        dw3k_disp.refreshView();

        vTaskDelay(pdMS_TO_TICKS(2000));
        task_state = DW3K_DISP_TASK_STATE_DISTANCE;   
        break;

      

      case DW3K_DISP_TASK_STATE_DISTANCE:            
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(DW3K_DISP_TASK_DELAY));
  }
}



void dw3k_begin(uint8_t *uwb_mode) {

  dw3k_conn_req_queue = xQueueCreate(DW3K_CONN_TASK_REQ_QUEUE_LEN, sizeof(uint8_t));
  dw3k_disp_req_queue = xQueueCreate(DW3K_DISP_TASK_REQ_QUEUE_LEN, sizeof(uint8_t));

  xTaskCreatePinnedToCore(
    dw3k_conn_task,
    "dw3k_conn_task",
    DW3K_CONN_TASK_STACK_SIZE,
    uwb_mode,
    DW3K_CONN_TASK_PRIORITY,
    &dw3k_conn_task_handler,
    DW3K_CONN_TASK_CORE
  );

  xTaskCreatePinnedToCore(
    dw3k_disp_task,
    "dw3k_disp_task",
    DW3K_DISP_TASK_STACK_SIZE,
    uwb_mode,
    DW3K_DISP_TASK_PRIORITY,
    &dw3k_disp_task_handler,
    DW3K_DISP_TASK_CORE
  );
}