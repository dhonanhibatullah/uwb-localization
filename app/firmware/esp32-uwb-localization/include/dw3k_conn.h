#ifndef __DW3K_CONN_H__
#define __DW3K_CONN_H__

#include <WiFi.h>
#include <WiFiManager.h>
#include "dw3000/dw3000.h"

#define DW3K_CONN_AS_ANCHOR 1
#define DW3K_CONN_AS_TAG    2

#define DW3K_CONN_WIFI_SSID "ESP32_UWB_"
#define DW3K_CONN_WIFI_PASS "12345678"
#define DW3K_CONN_TIMEOUT   5

class DW3KConn {

  private:
    uint8_t uwb_mode;

  public:
    // Constructor, this method also initiates WiFiManager
    // The WiFiManager runs on ESP32 server
    DW3KConn(uint8_t uwb_mode);
    
    // Destructor
    ~DW3KConn();

    // Starts WifiManager
    bool runWiFiMan(String uid);
};

#endif