#ifndef _IT8951_H_
#define _IT8951_H_
#include <stdint.h>

// You need panelW x panelH bytes buffer for full frame
typedef struct IT8951DevInfo {
  uint16_t panelW;
  uint16_t panelH;
  uint16_t imgBufAddrL;
  uint16_t imgBufAddrH;
  uint8_t fwVersion[16]; 	//16 Bytes String
  uint8_t lutVersion[16]; 	//16 Bytes String
} IT8951DevInfo;


int it8951_init();  // success = 0
IT8951DevInfo it8951_get_device_info();
int it8951_is_busy();
void it8951_transfer_image(int x, int y, int w, int h, uint8_t *buf);
void it8951_display_area(int x, int y, int w, int h, int mode/*2 is normal?*/);
#endif
