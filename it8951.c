#include "it8951_p.h"
#include "it8951.h"
#include <stdio.h>

//Global varivale
IT8951DevInfo gstI80DevInfo;
uint32_t gulImgBufAddr; //IT8951 Image buffer address


//-----------------------------------------------------------
//Host controller function 1---Wait for host data Bus Ready
//-----------------------------------------------------------
void lcd_wait_for_ready() {
  while (bcm2835_gpio_lev(HRDY) == 0);
}

//-----------------------------------------------------------
//Host controller function 2---Write command code to host data Bus
//-----------------------------------------------------------
void lcd_write_cmd_code(uint16_t usCmdCode) {
  //Set Preamble for Write Command
  uint16_t wPreamble = 0x6000; 
  
  lcd_wait_for_ready();
  
  bcm2835_gpio_write(CS, LOW);
  
  bcm2835_spi_transfer(wPreamble >> 8);
  bcm2835_spi_transfer(wPreamble);
  
  lcd_wait_for_ready();
  
  bcm2835_spi_transfer(usCmdCode >> 8);
  bcm2835_spi_transfer(usCmdCode);
  
  bcm2835_gpio_write(CS, HIGH); 
}

//-----------------------------------------------------------
//  Read Burst N words Data
//-----------------------------------------------------------
void lcd_read_n_data(uint16_t* pwBuf, uint32_t ulSizeWordCnt) {
  uint32_t i;
  uint16_t wPreamble = 0x1000;

  lcd_wait_for_ready();
	
  bcm2835_gpio_write(CS, LOW);

  bcm2835_spi_transfer(wPreamble >> 8);
  bcm2835_spi_transfer(wPreamble);
	
  lcd_wait_for_ready();
	
  pwBuf[0] = bcm2835_spi_transfer(0x00);//dummy
  pwBuf[0] = bcm2835_spi_transfer(0x00);//dummy
	
  lcd_wait_for_ready();
	
  for (i = 0; i < ulSizeWordCnt; i++) {
    pwBuf[i] = bcm2835_spi_transfer(0x00) << 8;
    pwBuf[i] |= bcm2835_spi_transfer(0x00);
  }

  bcm2835_gpio_write(CS,HIGH); 
}

//-----------------------------------------------------------
//Host controller function 3---Write Data to host data Bus
//-----------------------------------------------------------
void lcd_write_data(uint16_t usData) {
  //Set Preamble for Write Data
  uint16_t wPreamble = 0x0000;

  lcd_wait_for_ready();

  bcm2835_gpio_write(CS, LOW);

  bcm2835_spi_transfer(wPreamble >> 8);
  bcm2835_spi_transfer(wPreamble);
	
  lcd_wait_for_ready();
			
  bcm2835_spi_transfer(usData >> 8);
  bcm2835_spi_transfer(usData);
	
  bcm2835_gpio_write(CS, HIGH); 
}

//-----------------------------------------------------------
//Host controller function 4---Read Data from host data Bus
//-----------------------------------------------------------
uint16_t lcd_read_data() {
  uint16_t wRData; 
  uint16_t wPreamble = 0x1000;

  lcd_wait_for_ready();

  bcm2835_gpio_write(CS, LOW);
		
  bcm2835_spi_transfer(wPreamble >> 8);
  bcm2835_spi_transfer(wPreamble);

  lcd_wait_for_ready();
        
  wRData = bcm2835_spi_transfer(0x00);//dummy
  wRData = bcm2835_spi_transfer(0x00);//dummy
	
  lcd_wait_for_ready();
        
  wRData = bcm2835_spi_transfer(0x00) << 8;
  wRData |= bcm2835_spi_transfer(0x00);
		
  bcm2835_gpio_write(CS, HIGH); 
		
  return wRData;
}

//-----------------------------------------------------------
//Host controller function 2---Write command code to host data Bus
//-----------------------------------------------------------
void lcd_wWrite_cmd_code(uint16_t usCmdCode) {
  //Set Preamble for Write Command
  uint16_t wPreamble = 0x6000; 
	
  lcd_wait_for_ready();

  bcm2835_gpio_write(CS,LOW);
	
  bcm2835_spi_transfer(wPreamble >> 8);
  bcm2835_spi_transfer(wPreamble);
	
  lcd_wait_for_ready();
	
  bcm2835_spi_transfer(usCmdCode >> 8);
  bcm2835_spi_transfer(usCmdCode);
	
  bcm2835_gpio_write(CS, HIGH); 
}

//-----------------------------------------------------------
//Host controller function 5---Write command to host data Bus with aruments
//-----------------------------------------------------------
void lcd_send_cmd_arg(uint16_t usCmdCode,
                      uint16_t* pArg,
                      uint16_t usNumArg) {
  uint16_t i;
  //Send Cmd code
  lcd_write_cmd_code(usCmdCode);
  //Send Data
  for(i = 0; i < usNumArg; i++) {
    lcd_write_data(pArg[i]);
  }
}


//-----------------------------------------------------------
//Initial function 2---Set Image buffer base address
//-----------------------------------------------------------
void it8951_set_img_buf_base_addr(uint32_t ulImgBufAddr) {
  uint16_t usWordH = (uint16_t)((ulImgBufAddr >> 16) & 0x0000FFFF);
  uint16_t usWordL = (uint16_t)( ulImgBufAddr & 0x0000FFFF);
  //Write LISAR Reg
  it8951_write_reg(LISAR + 2, usWordH);
  it8951_write_reg(LISAR, usWordL);
}



//-----------------------------------------------------------
//Host Cmd 4---REG_RD
//-----------------------------------------------------------
uint16_t it8951_read_reg(uint16_t usRegAddr) {
  uint16_t usData;
  //Send Cmd and Register Address
  lcd_write_cmd_code(IT8951_TCON_REG_RD);
  lcd_write_data(usRegAddr);
  //Read data from Host Data bus
  usData = lcd_read_data();
  return usData;
}

//-----------------------------------------------------------
//Host Cmd 5---REG_WR
//-----------------------------------------------------------
void it8951_write_reg(uint16_t usRegAddr, uint16_t usValue) {
  //Send Cmd , Register Address and Write Value
  lcd_write_cmd_code(IT8951_TCON_REG_WR);
  lcd_write_data(usRegAddr);
  lcd_write_data(usValue);
}

//-----------------------------------------------------------
//Host Cmd 12---LD_IMG_END
//-----------------------------------------------------------
void it8951_load_img_end(void) {
  lcd_write_cmd_code(IT8951_TCON_LD_IMG_END);
}


uint16_t it8951_get_vcom(void) {
  uint16_t vcom;
  lcd_write_cmd_code(USDEF_I80_CMD_VCOM);
  lcd_write_data(0);
  //Read data from Host Data bus
  vcom = lcd_read_data();
  return vcom;
}

void it8951_set_vcom(uint16_t vcom) {
  lcd_write_cmd_code(USDEF_I80_CMD_VCOM);
  lcd_write_data(1);
  //Read data from Host Data bus
  lcd_write_data(vcom);
}


int it8951_init() {
  if (!bcm2835_init()) {
    perror("bcm2835_init() failed.");
    return -1;
  }
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);
  bcm2835_gpio_fsel(CS, BCM2835_GPIO_FSEL_OUTP);  
  bcm2835_gpio_fsel(HRDY, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(RESET, BCM2835_GPIO_FSEL_OUTP);
	
  bcm2835_gpio_write(CS, HIGH);
  //printf("****** IT8951 ******\n");
  bcm2835_gpio_write(RESET, LOW);
  bcm2835_delay(100);
  bcm2835_gpio_write(RESET, HIGH);
  //Get Device Info
  gstI80DevInfo = it8951_get_device_info();
  gulImgBufAddr = gstI80DevInfo.imgBufAddrL | (gstI80DevInfo.imgBufAddrH << 16);
 	
  //Set to Enable I80 Packed mode
  it8951_write_reg(I80CPCR, 0x0001);
  if (VCOM != it8951_get_vcom()) {
    it8951_set_vcom(VCOM);
    //printf("VCOM = -%.02fV\n", (float)it8951_get_vcom() / 1000);
  }
  return 0;
}

//-----------------------------------------------------------
//Host Cmd 11---LD_IMG_AREA
//-----------------------------------------------------------
void it8951_load_img_area_start(IT8951LdImgInfo* pstLdImgInfo,
                                IT8951AreaImgInfo* pstAreaImgInfo) {
  uint16_t usArg[5];
  //Setting Argument for Load image start
  usArg[0] = (pstLdImgInfo->usEndianType << 8 )
    |(pstLdImgInfo->usPixelFormat << 4)
    |(pstLdImgInfo->usRotate);
  usArg[1] = pstAreaImgInfo->usX;
  usArg[2] = pstAreaImgInfo->usY;
  usArg[3] = pstAreaImgInfo->usWidth;
  usArg[4] = pstAreaImgInfo->usHeight;
  //Send Cmd and Args
  lcd_send_cmd_arg(IT8951_TCON_LD_IMG_AREA, usArg, 5);
}

//-----------------------------------------------------------
//Display function 1---Wait for LUT Engine Finish
//                     Polling Display Engine Ready by LUTNo
//-----------------------------------------------------------
int it8951_is_busy() {
  return it8951_read_reg(LUTAFSR);
}


IT8951DevInfo it8951_get_device_info() {
  IT8951DevInfo dev_info;
  
  //Send I80 CMD
  lcd_write_cmd_code(USDEF_I80_CMD_GET_DEV_INFO);
  
  //Burst Read Request for SPI interface only
  lcd_read_n_data((uint16_t*)&dev_info, sizeof(IT8951DevInfo) / 2);
  
  //Show Device information of IT8951
  //printf("Panel(W,H) = (%d, %d)\r\n",
  //dev_info.panelW, dev_info.panelH );
  //printf("Image Buffer Address = %X\r\n",
  //dev_info.imgBufAddrL | (dev_info.imgBufAddrH << 16));
  //Show Firmware and LUT Version
  //printf("FW Version = %s\r\n", (char*)dev_info.fwVersion);
  //printf("LUT Version = %s\r\n", (char*)dev_info.lutVersion);
  return dev_info;
}

//-----------------------------------------------------------
//Display function 2---Load Image Area process
//-----------------------------------------------------------
void it8951_host_area_packed_pixel_write(IT8951LdImgInfo* pstLdImgInfo,
                                         IT8951AreaImgInfo* pstAreaImgInfo) {
  uint32_t i,j;
  //Source buffer address of Host
  uint16_t* pusFrameBuf = (uint16_t*)pstLdImgInfo->ulStartFBAddr;

  //Set Image buffer(IT8951) Base address
  it8951_set_img_buf_base_addr(pstLdImgInfo->ulImgBufBaseAddr);
  //Send Load Image start Cmd
  it8951_load_img_area_start(pstLdImgInfo , pstAreaImgInfo);
  //Host Write Data
  for(j = 0; j < pstAreaImgInfo->usHeight; j++) {
    for(i = 0; i < pstAreaImgInfo->usWidth / 2; i++) {
      //Write a Word(2-Bytes) for each time
      lcd_write_data(*pusFrameBuf);
      pusFrameBuf++;
    }
  }
  it8951_load_img_end();
}

//-----------------------------------------------------------
//Display functions 3---Application for Display panel Area
//-----------------------------------------------------------
void it8951_display_area(int x, int y, int w, int h, int mode) {
  //Send I80 Display Command (User defined command of IT8951)
  lcd_write_cmd_code(USDEF_I80_CMD_DPY_AREA); //0x0034
  //Write arguments
  lcd_write_data(x);
  lcd_write_data(y);
  lcd_write_data(w);
  lcd_write_data(h);
  lcd_write_data(mode);
}


void it8951_transfer_image(int x, int y, int w, int h, uint8_t *buf) {
  IT8951LdImgInfo img_info;
  IT8951AreaImgInfo area_info;
  img_info.ulStartFBAddr = (uint32_t)buf;
  img_info.usEndianType = IT8951_LDIMG_L_ENDIAN;
  img_info.usPixelFormat    = IT8951_8BPP;
  img_info.usRotate         = IT8951_ROTATE_0;
  img_info.ulImgBufBaseAddr = gulImgBufAddr;
  area_info.usX = x;
  area_info.usY = y;
  area_info.usWidth = w;
  area_info.usHeight = h;
  it8951_host_area_packed_pixel_write(&img_info, &area_info);
}
