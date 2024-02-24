#ifndef __DW3K_DISP_H__
#define __DW3K_DISP_H__

#include <stdarg.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DW3K_DISP_AS_ANCHOR 1
#define DW3K_DISP_AS_TAG    2

#define DW3K_DISP_WIDTH   128
#define DW3K_DISP_HEIGHT  64
#define DW3K_DISP_RESET   -1
#define DW3K_DISP_ADDR    0x3C

#define DW3K_DISP_VIEW_SPLASH               0
#define DW3K_DISP_VIEW_ON_WIFI_CONNECT      1
#define DW3K_DISP_VIEW_WIFI_CONNECT_SUCCESS 2
#define DW3K_DISP_VIEW_DISTANCE             3

class DW3KDisp {

  private:
    uint8_t           uwb_mode;
    Adafruit_SSD1306  display = Adafruit_SSD1306(DW3K_DISP_WIDTH, DW3K_DISP_HEIGHT, &Wire, DW3K_DISP_RESET);

  public:
    // Constructor, this method also initiate OLED SSD1306
    DW3KDisp(uint8_t uwb_mode);

    // Destructor
    ~DW3KDisp();

    // Initiate the new view on the buffer
    void changeView(uint8_t view_enum, ...);
    
    // Refresh and display the view on the buffer
    void refreshView();
};

#endif