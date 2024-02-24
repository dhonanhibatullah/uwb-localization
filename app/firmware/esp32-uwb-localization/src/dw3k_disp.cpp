#include "../include/dw3k_disp.h"



DW3KDisp::DW3KDisp(uint8_t uwb_mode) {

  this->uwb_mode = uwb_mode;
  this->display.begin(SSD1306_SWITCHCAPVCC, DW3K_DISP_ADDR);
  this->display.setTextColor(SSD1306_WHITE);
}



DW3KDisp::~DW3KDisp() {

}



void DW3KDisp::changeView(uint8_t view_enum, ...) {

  va_list args;
  String  str_msg;

  this->display.clearDisplay();

  switch(view_enum) {

    case DW3K_DISP_VIEW_SPLASH:

      this->display.setTextSize(2);
      this->display.setCursor(30, 8);
      this->display.print(F("DW3000"));

      if(this->uwb_mode == DW3K_DISP_AS_ANCHOR) {
        this->display.setCursor(30, 28);
        this->display.print(F("ANCHOR"));
      }
      else {
        this->display.setCursor(46, 28);
        this->display.print(F("TAG"));
      }

      this->display.setTextSize(1);
      this->display.setCursor(17, 48);
      this->display.print(F("UWB Localization"));
      break;



    case DW3K_DISP_VIEW_ON_WIFI_CONNECT:

      va_start(args, 1);
      str_msg = String(va_arg(args, const char*));

      this->display.setTextSize(1);
      this->display.setCursor(0, 0);
      this->display.print(F("--[ WIFI CONNECT  ]--"));

      this->display.setCursor(5, 20);
      this->display.print(F("Configure from >>"));

      this->display.setCursor(5, 40);
      this->display.print(F("SSID: "));
      this->display.print(str_msg.c_str());

      this->display.setCursor(5, 52);
      this->display.print(F("IP  : 192.168.4.1"));
      break;



    case DW3K_DISP_VIEW_WIFI_CONNECT_SUCCESS:

      va_start(args, 1);
      str_msg = String(va_arg(args, const char*));

      this->display.setTextSize(1);
      this->display.setCursor(0, 0);
      this->display.print(F("--[ WIFI CONNECT  ]--"));

      this->display.setCursor(5, 20);
      this->display.print(F("Connection success!"));

      this->display.setCursor(5, 40);
      this->display.print(F("SSID: "));
      this->display.print(str_msg.c_str());
      break;



    case DW3K_DISP_VIEW_DISTANCE:
      break;
  }
}



void DW3KDisp::refreshView() {

  this->display.display();
}