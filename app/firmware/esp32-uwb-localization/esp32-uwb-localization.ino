/*--------[ SELECT THE UWB MODULE ]--------*/
// #define __DW1000__
#define __DW3000__


/*--------[ SELECT THE MODE ]--------*/
#define __ANCHOR__
// #define __TAG__




// #ifdef __DW1000__

//   #include "dw1k.h"

//   void setup() {

//     #ifdef __ANCHOR__
//       dw1000_begin();
//     #endif

//     #ifdef __TAG__
//       dw1000_begin();
//     #endif
//   }

// #endif


#ifdef __DW3000__

  #include "include/dw3k.h"

  #ifdef __ANCHOR__
    uint8_t uwb_mode = DW3K_SET_ANCHOR;
  #endif

  #ifdef __TAG__
    uint8_t uwb_mode = DW3K_SET_TAG:
  #endif

  void setup() {

    Serial.begin(115200);
    dw3k_begin(&uwb_mode);
  }

#endif


void loop() {}