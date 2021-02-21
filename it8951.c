#include "it8951.h"
#include <stdio.h>

//Global varivale
IT8951DevInfo gstI80DevInfo;
uint8_t* gpFrameBuf; //Host Source Frame buffer
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



void it8951_get_system_info(void* pBuf) {
  uint16_t* pusWord = (uint16_t*)pBuf;
  IT8951DevInfo* pstDevInfo;
  
  //Send I80 CMD
  lcd_write_cmd_code(USDEF_I80_CMD_GET_DEV_INFO);
  
  //Burst Read Request for SPI interface only
  lcd_read_n_data(pusWord, sizeof(IT8951DevInfo) / 2); //Polling HRDY for each words(2-bytes) if possible
  
  //Show Device information of IT8951
  pstDevInfo = (IT8951DevInfo*)pBuf;
  printf("Panel(W,H) = (%d, %d)\r\n",
         pstDevInfo->usPanelW, pstDevInfo->usPanelH );
  printf("Image Buffer Address = %X\r\n",
         pstDevInfo->usImgBufAddrL | (pstDevInfo->usImgBufAddrH << 16));
  //Show Firmware and LUT Version
  printf("FW Version = %s\r\n", (uint8_t*)pstDevInfo->usFWVersion);
  printf("LUT Version = %s\r\n", (uint8_t*)pstDevInfo->usLUTVersion);
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


uint8_t it8951_init() {
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
  printf("****** IT8951 ******\n");
  bcm2835_gpio_write(RESET, LOW);
  bcm2835_delay(100);
  bcm2835_gpio_write(RESET, HIGH);
  //Get Device Info
  it8951_get_system_info(&gstI80DevInfo);
  gpFrameBuf = malloc(gstI80DevInfo.usPanelW * gstI80DevInfo.usPanelH);
  if (!gpFrameBuf) {
    perror("malloc error!\n");
    return 1;
  }
  gulImgBufAddr = gstI80DevInfo.usImgBufAddrL | (gstI80DevInfo.usImgBufAddrH << 16);
 	
  //Set to Enable I80 Packed mode
  it8951_write_reg(I80CPCR, 0x0001);
  if (VCOM != it8951_get_vcom()) {
    it8951_set_vcom(VCOM);
    printf("VCOM = -%.02fV\n", (float)it8951_get_vcom() / 1000);
  }
  return 0;
}


