/**
  ******************************************************************************
  * @file    st7735.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    24-November-2014
  * @brief   This file contains all the functions prototypes for the st7735.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ST7735_H
#define __ST7735_H

#include <stdint.h>
#include "hardware_init.h"
#include "stm32l1xx_hal_spi.h"
#ifdef __cplusplus
 extern "C" {
#endif


 /** @addtogroup BSP
   * @{
   */

 /** @addtogroup Components
   * @{
   */

 /** @addtogroup LCD
   * @{
   */

 /** @defgroup LCD_Exported_Types
   * @{
   */
 /**
   * @brief  LCD driver structure definition
   */
 typedef struct
 {
   void     (*Init)(void);
   uint16_t (*ReadID)(void);
   void     (*DisplayOn)(void);
   void     (*DisplayOff)(void);
   void     (*SetCursor)(uint16_t, uint16_t);
   void     (*WritePixel)(uint16_t, uint16_t, uint16_t);
   uint16_t (*ReadPixel)(uint16_t, uint16_t);

    /* Optimized operation */
   void     (*SetDisplayWindow)(uint16_t, uint16_t, uint16_t, uint16_t);
   void     (*DrawHLine)(uint16_t, uint16_t, uint16_t, uint16_t);
   void     (*DrawVLine)(uint16_t, uint16_t, uint16_t, uint16_t);

   uint16_t (*GetLcdPixelWidth)(void);
   uint16_t (*GetLcdPixelHeight)(void);
   void     (*DrawBitmap)(uint16_t, uint16_t, uint8_t*);
   void     (*DrawRGBImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t*);
 }LCD_DrvTypeDef;

 /**
   * @}
   */

 /**
   * @}
   */

 /**
   * @}
   */

 /**
   * @}



/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup ST7735
  * @{
  */

/** @defgroup ST7735_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup ST7735_Exported_Constants
  * @{
  */

/**
  * @brief  ST7735 Size
  */
#define  ST7735_LCD_PIXEL_WIDTH    ((uint16_t)128)
#define  ST7735_LCD_PIXEL_HEIGHT   ((uint16_t)160)

/**
  * @brief  ST7735 Registers
  */
#define  LCD_REG_0               0x00 /* No Operation: NOP */
#define  LCD_REG_1               0x01 /* Software reset: SWRESET */
#define  LCD_REG_4               0x04 /* Read Display ID: RDDID */
#define  LCD_REG_9               0x09 /* Read Display Statu: RDDST */
#define  LCD_REG_10              0x0A /* Read Display Power: RDDPM */
#define  LCD_REG_11              0x0B /* Read Display: RDDMADCTL */
#define  LCD_REG_12              0x0C /* Read Display Pixel: RDDCOLMOD */
#define  LCD_REG_13              0x0D /* Read Display Image: RDDIM */
#define  LCD_REG_14              0x0E /* Read Display Signal: RDDSM */
#define  LCD_REG_16              0x10 /* Sleep in & booster off: SLPIN */
#define  LCD_REG_17              0x11 /* Sleep out & booster on: SLPOUT */
#define  LCD_REG_18              0x12 /* Partial mode on: PTLON */
#define  LCD_REG_19              0x13 /* Partial off (Normal): NORON */
#define  LCD_REG_32              0x20 /* Display inversion off: INVOFF */
#define  LCD_REG_33              0x21 /* Display inversion on: INVON */
#define  LCD_REG_38              0x26 /* Gamma curve select: GAMSET */
#define  LCD_REG_40              0x28 /* Display off: DISPOFF */
#define  LCD_REG_41              0x29 /* Display on: DISPON */
#define  LCD_REG_42              0x2A /* Column address set: CASET */
#define  LCD_REG_43              0x2B /* Row address set: RASET */
#define  LCD_REG_44              0x2C /* Memory write: RAMWR */
#define  LCD_REG_45              0x2D /* LUT for 4k,65k,262k color: RGBSET */
#define  LCD_REG_46              0x2E /* Memory read: RAMRD*/
#define  LCD_REG_48              0x30 /* Partial start/end address set: PTLAR */
#define  LCD_REG_52              0x34 /* Tearing effect line off: TEOFF */
#define  LCD_REG_53              0x35 /* Tearing effect mode set & on: TEON */
#define  LCD_REG_54              0x36 /* Memory data access control: MADCTL */
#define  LCD_REG_56              0x38 /* Idle mode off: IDMOFF */
#define  LCD_REG_57              0x39 /* Idle mode on: IDMON */
#define  LCD_REG_58              0x3A /* Interface pixel format: COLMOD */
#define  LCD_REG_177             0xB1 /* In normal mode (Full colors): FRMCTR1 */
#define  LCD_REG_178             0xB2 /* In Idle mode (8-colors): FRMCTR2 */
#define  LCD_REG_179             0xB3 /* In partial mode + Full colors: FRMCTR3 */
#define  LCD_REG_180             0xB4 /* Display inversion control: INVCTR */
#define  LCD_REG_192             0xC0 /* Power control setting: PWCTR1 */
#define  LCD_REG_193             0xC1 /* Power control setting: PWCTR2 */
#define  LCD_REG_194             0xC2 /* In normal mode (Full colors): PWCTR3 */
#define  LCD_REG_195             0xC3 /* In Idle mode (8-colors): PWCTR4 */
#define  LCD_REG_196             0xC4 /* In partial mode + Full colors: PWCTR5 */
#define  LCD_REG_197             0xC5 /* VCOM control 1: VMCTR1 */
#define  LCD_REG_199             0xC7 /* Set VCOM offset control: VMOFCTR */
#define  LCD_REG_209             0xD1 /* Set LCM version code: WRID2 */
#define  LCD_REG_210             0xD2 /* Customer Project code: WRID3 */
#define  LCD_REG_217             0xD9 /* NVM control status: NVCTR1 */
#define  LCD_REG_218             0xDA /* Read ID1: RDID1 */
#define  LCD_REG_219             0xDB /* Read ID2: RDID2 */
#define  LCD_REG_220             0xDC /* Read ID3: RDID3 */
#define  LCD_REG_222             0xDE /* NVM Read Command: NVCTR2 */
#define  LCD_REG_223             0xDF /* NVM Write Command: NVCTR3 */
#define  LCD_REG_224             0xE0 /* Set Gamma adjustment (+ polarity): GAMCTRP1 */
#define  LCD_REG_225             0xE1 /* Set Gamma adjustment (- polarity): GAMCTRN1 */

/**
  * @brief  LCD Lines depending on the chosen fonts.
  */
#define LCD_LINE_0               LINE(0)
#define LCD_LINE_1               LINE(1)
#define LCD_LINE_2               LINE(2)
#define LCD_LINE_3               LINE(3)
#define LCD_LINE_4               LINE(4)
#define LCD_LINE_5               LINE(5)
#define LCD_LINE_6               LINE(6)
#define LCD_LINE_7               LINE(7)
#define LCD_LINE_8               LINE(8)
#define LCD_LINE_9               LINE(9)
#define LCD_LINE_10              LINE(10)
#define LCD_LINE_11              LINE(11)
#define LCD_LINE_12              LINE(12)
#define LCD_LINE_13              LINE(13)
#define LCD_LINE_14              LINE(14)
#define LCD_LINE_15              LINE(15)
#define LCD_LINE_16              LINE(16)
#define LCD_LINE_17              LINE(17)
#define LCD_LINE_18              LINE(18)
#define LCD_LINE_19              LINE(19)
 // some flags for initR() :(
 #define INITR_GREENTAB 0x0
 #define INITR_REDTAB   0x1
 #define INITR_BLACKTAB   0x2

 #define INITR_18GREENTAB    INITR_GREENTAB
 #define INITR_18REDTAB      INITR_REDTAB
 #define INITR_18BLACKTAB    INITR_BLACKTAB
 #define INITR_144GREENTAB   0x1

 #define ST7735_TFTWIDTH  128
 // for 1.44" display
 #define ST7735_TFTHEIGHT_144 128
 // for 1.8" display
 #define ST7735_TFTHEIGHT_18  160

 #define ST7735_NOP     0x00
 #define ST7735_SWRESET 0x01
 #define ST7735_RDDID   0x04
 #define ST7735_RDDST   0x09

 #define ST7735_SLPIN   0x10
 #define ST7735_SLPOUT  0x11
 #define ST7735_PTLON   0x12
 #define ST7735_NORON   0x13

 #define ST7735_INVOFF  0x20
 #define ST7735_INVON   0x21
 #define ST7735_DISPOFF 0x28
 #define ST7735_DISPON  0x29
 #define ST7735_CASET   0x2A
 #define ST7735_RASET   0x2B
 #define ST7735_RAMWR   0x2C
 #define ST7735_RAMRD   0x2E

 #define ST7735_PTLAR   0x30
 #define ST7735_COLMOD  0x3A
 #define ST7735_MADCTL  0x36

 #define ST7735_FRMCTR1 0xB1
 #define ST7735_FRMCTR2 0xB2
 #define ST7735_FRMCTR3 0xB3
 #define ST7735_INVCTR  0xB4
 #define ST7735_DISSET5 0xB6

 #define ST7735_PWCTR1  0xC0
 #define ST7735_PWCTR2  0xC1
 #define ST7735_PWCTR3  0xC2
 #define ST7735_PWCTR4  0xC3
 #define ST7735_PWCTR5  0xC4
 #define ST7735_VMCTR1  0xC5

 #define ST7735_RDID1   0xDA
 #define ST7735_RDID2   0xDB
 #define ST7735_RDID3   0xDC
 #define ST7735_RDID4   0xDD

 #define ST7735_PWCTR6  0xFC

 #define ST7735_GMCTRP1 0xE0
 #define ST7735_GMCTRN1 0xE1

 // Color definitions
 #define	ST7735_BLACK   0x0000
 #define	ST7735_BLUE    0x001F
 #define	ST7735_RED     0xF800
 #define	ST7735_GREEN   0x07E0
 #define ST7735_CYAN    0x07FF
 #define ST7735_MAGENTA 0xF81F
 #define ST7735_YELLOW  0xFFE0
 #define ST7735_WHITE 0xFFFF


/**
  * @}
  */

/** @defgroup ADAFRUIT_SPI_LCD_Exported_Functions
  * @{
  */
void     st7735_Init(void);
uint16_t st7735_ReadID(void);

void     st7735_DisplayOn(void);
void     st7735_DisplayOff(void);
void     st7735_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     st7735_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
void     st7735_WriteReg(uint8_t LCDReg, uint8_t LCDRegValue);
uint8_t  st7735_ReadReg(uint8_t LCDReg);

void     st7735_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     st7735_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     st7735_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);

uint16_t st7735_GetLcdPixelWidth(void);
uint16_t st7735_GetLcdPixelHeight(void);
void     st7735_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);

/* LCD driver structure */
extern LCD_DrvTypeDef   st7735_drv;

/* LCD IO functions */
void     LCD_IO_Init(void);
void     LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void     LCD_IO_WriteReg(uint8_t Reg);
void     LCD_Delay(uint32_t delay);
/**
  * @}
  */

#define DELAY 0x80
static const uint8_t
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd2green144[] = {              // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
100 }; //     100 ms delay


#ifdef __cplusplus
}
#endif

#endif /* __ST7735_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
