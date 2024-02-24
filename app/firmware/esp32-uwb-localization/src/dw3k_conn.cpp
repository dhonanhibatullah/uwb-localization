#include "../include/dw3k_conn.h"



DW3KConn::DW3KConn(uint8_t uwb_mode) {

  this->uwb_mode = uwb_mode;

  if(uwb_mode == DW3K_CONN_AS_ANCHOR) {
    
    WiFi.mode(WIFI_STA);
  }
}



DW3KConn::~DW3KConn() {

}



bool DW3KConn::runWiFiMan(String uid) {
  
  WiFiManager wm;
  wm.setConnectTimeout(DW3K_CONN_TIMEOUT);
  bool wm_res = wm.autoConnect((String(DW3K_CONN_WIFI_SSID) + uid).c_str(), DW3K_CONN_WIFI_PASS);

  if(!wm_res) {
    wm.disconnect();
    wm.stopWebPortal();
  }

  return wm_res;
}