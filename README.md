# epd_it8951

# tl;dr

This is a simple C language library for WaveShare's IT-8951 controlled EPD.


# how to use

Basically, just follow the 3 steps below.

 1. `it8951_init()` to initialize device
 2. `it8951_transfer_image()` to send image data to EPD memory
 3. `it8951_display_area()` to update EPD screen

```C
#include <stdlib.h>
#include "it8951.h"

int main() {
  uint8_t *buf;
  IT8951DevInfo dev;
  
  it8951_init();
  dev = it8951_get_device_info();
  buf = malloc(dev.panelW * dev.panelH);
  for (int i = 0; i < dev.panelW * dev.panelH; i++) {
    buf[i] = (i / dev.panelW) >> 2;
  }
  while(it8951_is_busy());
  it8951_transfer_image(0, 0, dev.panelW, dev.panelH, buf);
  while(it8951_is_busy());
  it8951_display_area(0, 0, dev.panelW, dev.panelH, 2);
  return 0;
}
```
