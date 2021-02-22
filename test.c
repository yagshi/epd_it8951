#include <stdio.h>
#include <stdlib.h>
#include "it8951.h"

int main() {
  uint8_t *buf;
  IT8951DevInfo dev;
  
  it8951_init();
  dev = it8951_get_device_info();
  if(!(buf = malloc(dev.panelW * dev.panelH))) {
    perror("malloc failed.");
    return -1;
  }
  for (int i = 0; i < dev.panelW * dev.panelH; i++) {
    buf[i] = (i / dev.panelW) >> 2;
  }
  while(it8951_is_busy());
  it8951_transfer_image(0, 0, dev.panelW, dev.panelH, buf);
  while(it8951_is_busy());
  it8951_display_area(0, 0, dev.panelW, dev.panelH, 2);
  return 0;
}
