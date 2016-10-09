/**
  ******************************************************************************
  * @file    st7735.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    24-November-2014
  * @brief   This file includes the driver for ST7735 LCD mounted on the Adafruit
  *          1.8" TFT LCD shield (reference ID 802).
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

/* Includes ------------------------------------------------------------------*/
#include "st7735.h"
extern SPI_HandleTypeDef SpiHandle;

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup ST7735
  * @brief      This file provides a set of functions needed to drive the
  *             ST7735 LCD.
  * @{
  */

/** @defgroup ST7735_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup ST7735_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup ST7735_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup ST7735_Private_Variables
  * @{
  */


LCD_DrvTypeDef   st7735_drv =
{
  st7735_Init,
  0,
  st7735_DisplayOn,
  st7735_DisplayOff,
  st7735_SetCursor,
  st7735_WritePixel,
  0,
  st7735_SetDisplayWindow,
  st7735_DrawHLine,
  st7735_DrawVLine,
  st7735_GetLcdPixelWidth,
  st7735_GetLcdPixelHeight,
  st7735_DrawBitmap,
};

static uint16_t ArrayRGB[320] = {0};

/**
* @}
*/

/** @defgroup ST7735_Private_FunctionPrototypes
  * @{
  */

/**
* @}
*/

/** @defgroup ST7735_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the ST7735 LCD Component.
  * @param  None
  * @retval None
  */
void st7735_Init(void)
{
  uint8_t data = 0;

  /* Initialize ST7735 low level bus layer -----------------------------------*/
  /* Out of sleep mode, 0 args, no delay */
  st7735_WriteReg(LCD_REG_17, 0x00);
  /* Frame rate ctrl - normal mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D)*/
  LCD_IO_WriteReg(LCD_REG_177);
  data = 0x01;
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x2C;
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x2D;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Frame rate control - idle mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D) */
  st7735_WriteReg(LCD_REG_178, 0x01);
  st7735_WriteReg(LCD_REG_178, 0x2C);
  st7735_WriteReg(LCD_REG_178, 0x2D);
  /* Frame rate ctrl - partial mode, 6 args: Dot inversion mode, Line inversion mode */
  st7735_WriteReg(LCD_REG_179, 0x01);
  st7735_WriteReg(LCD_REG_179, 0x2C);
  st7735_WriteReg(LCD_REG_179, 0x2D);
  st7735_WriteReg(LCD_REG_179, 0x01);
  st7735_WriteReg(LCD_REG_179, 0x2C);
  st7735_WriteReg(LCD_REG_179, 0x2D);
  /* Display inversion ctrl, 1 arg, no delay: No inversion */
  st7735_WriteReg(LCD_REG_180, 0x07);
  /* Power control, 3 args, no delay: -4.6V , AUTO mode */
  st7735_WriteReg(LCD_REG_192, 0xA2);
  st7735_WriteReg(LCD_REG_192, 0x02);
  st7735_WriteReg(LCD_REG_192, 0x84);
  /* Power control, 1 arg, no delay: VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD */
  st7735_WriteReg(LCD_REG_193, 0xC5);
  /* Power control, 2 args, no delay: Opamp current small, Boost frequency */
  st7735_WriteReg(LCD_REG_194, 0x0A);
  st7735_WriteReg(LCD_REG_194, 0x00);
  /* Power control, 2 args, no delay: BCLK/2, Opamp current small & Medium low */
  st7735_WriteReg(LCD_REG_195, 0x8A);
  st7735_WriteReg(LCD_REG_195, 0x2A);
  /* Power control, 2 args, no delay */
  st7735_WriteReg(LCD_REG_196, 0x8A);
  st7735_WriteReg(LCD_REG_196, 0xEE);
  /* Power control, 1 arg, no delay */
  st7735_WriteReg(LCD_REG_197, 0x0E);
  /* Don't invert display, no args, no delay */
  LCD_IO_WriteReg(LCD_REG_32);
  /* Set color mode, 1 arg, no delay: 16-bit color */
  st7735_WriteReg(LCD_REG_58, 0x05);
  /* Column addr set, 4 args, no delay: XSTART = 0, XEND = 127 */
  LCD_IO_WriteReg(LCD_REG_42);
  data = 0x00;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x7F;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Row addr set, 4 args, no delay: YSTART = 0, YEND = 127 */
  LCD_IO_WriteReg(LCD_REG_43);
  data = 0x00;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x7F;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Magical unicorn dust, 16 args, no delay */
  st7735_WriteReg(LCD_REG_224, 0x02);
  st7735_WriteReg(LCD_REG_224, 0x1c);
  st7735_WriteReg(LCD_REG_224, 0x07);
  st7735_WriteReg(LCD_REG_224, 0x12);
  st7735_WriteReg(LCD_REG_224, 0x37);
  st7735_WriteReg(LCD_REG_224, 0x32);
  st7735_WriteReg(LCD_REG_224, 0x29);
  st7735_WriteReg(LCD_REG_224, 0x2d);
  st7735_WriteReg(LCD_REG_224, 0x29);
  st7735_WriteReg(LCD_REG_224, 0x25);
  st7735_WriteReg(LCD_REG_224, 0x2B);
  st7735_WriteReg(LCD_REG_224, 0x39);
  st7735_WriteReg(LCD_REG_224, 0x00);
  st7735_WriteReg(LCD_REG_224, 0x01);
  st7735_WriteReg(LCD_REG_224, 0x03);
  st7735_WriteReg(LCD_REG_224, 0x10);
  /* Sparkles and rainbows, 16 args, no delay */
  st7735_WriteReg(LCD_REG_225, 0x03);
  st7735_WriteReg(LCD_REG_225, 0x1d);
  st7735_WriteReg(LCD_REG_225, 0x07);
  st7735_WriteReg(LCD_REG_225, 0x06);
  st7735_WriteReg(LCD_REG_225, 0x2E);
  st7735_WriteReg(LCD_REG_225, 0x2C);
  st7735_WriteReg(LCD_REG_225, 0x29);
  st7735_WriteReg(LCD_REG_225, 0x2D);
  st7735_WriteReg(LCD_REG_225, 0x2E);
  st7735_WriteReg(LCD_REG_225, 0x2E);
  st7735_WriteReg(LCD_REG_225, 0x37);
  st7735_WriteReg(LCD_REG_225, 0x3F);
  st7735_WriteReg(LCD_REG_225, 0x00);
  st7735_WriteReg(LCD_REG_225, 0x00);
  st7735_WriteReg(LCD_REG_225, 0x02);
  st7735_WriteReg(LCD_REG_225, 0x10);
  /* Normal display on, no args, no delay */
  st7735_WriteReg(LCD_REG_19, 0x00);
  /* Main screen turn on, no delay */
  st7735_WriteReg(LCD_REG_41, 0x00);
  /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
  st7735_WriteReg(LCD_REG_54, 0xC0);
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void st7735_DisplayOn(void)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_19);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_41);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_54);
  data = 0xC0;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void st7735_DisplayOff(void)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_19);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_40);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_54);
  data = 0xC0;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Sets Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void st7735_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_42);
  data = (Xpos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteReg(LCD_REG_43);
  data = (Ypos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteReg(LCD_REG_44);
}

/**
  * @brief  Writes pixel.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
void st7735_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  uint8_t data = 0;
  if((Xpos >= ST7735_LCD_PIXEL_WIDTH) || (Ypos >= ST7735_LCD_PIXEL_HEIGHT))
  {
    return;
  }

  /* Set Cursor */
  st7735_SetCursor(Xpos, Ypos);

  data = RGBCode >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = RGBCode;
  LCD_IO_WriteMultipleData(&data, 1);
}


/**
  * @brief  Writes to the selected LCD register.
  * @param  LCDReg: Address of the selected register.
  * @param  LCDRegValue: value to write to the selected register.
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t Reg){

         __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
         __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
         HAL_SPI_Transmit(&SpiHandle,(uint8_t)Reg, 1,500);
         //__LOW(LCD_CMD);
         __HIGH(LCD_ChipSelect);
         __HIGH(LCD_CMD);


}
void st7735_WriteReg(uint8_t LCDReg, uint8_t LCDRegValue)
{
    //LCD_IO_WriteReg((uint8_t)Data);
     __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
     __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
     HAL_SPI_Transmit(&SpiHandle,(uint8_t)LCDReg, 1,500);
     //__LOW(LCD_CMD);
     LCD_IO_WriteMultipleData(&LCDRegValue, 1);
     __HIGH(LCD_ChipSelect);
     __HIGH(LCD_CMD);
}

void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size) {
    __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
    __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
    HAL_SPI_Transmit(&SpiHandle,(uint8_t*)pData, Size,500);
    __HIGH(LCD_ChipSelect);
    __HIGH(LCD_CMD);
}
void LCD_IO_WriteData(uint8_t *pData, uint32_t Size) {
    __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
    __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
    HAL_SPI_Transmit(&SpiHandle,(uint8_t*)pData, Size,500);
    __HIGH(LCD_ChipSelect);

}

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void commandList(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = (*addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    LCD_IO_WriteMultipleData((*addr++),1); //   Read, issue command
    numArgs  = (*addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
	LCD_IO_WriteData((*addr++),1);  //     Read, issue argument
    }

    if(ms) {
      ms = (*addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      HAL_Delay(ms);
    }
  }
}
// Initialization code common to both 'B' and 'R' type displays
void commonInit(const uint8_t *cmdList) {

  if(cmdList) commandList(cmdList);
}
// Initialization for ST7735R screens (green or red tabs)
void initR() {
    commonInit(Rcmd1);
    commandList(Rcmd2green144);
    commandList(Rcmd3);
    st7735_DisplayOn();
    st7735_DisplayOff();
}
/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void st7735_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint8_t data = 0;
  /* Column addr set, 4 args, no delay: XSTART = Xpos, XEND = (Xpos + Width - 1) */
  LCD_IO_WriteReg(LCD_REG_42);
  data = (Xpos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos + Width - 1) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos + Width - 1) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Row addr set, 4 args, no delay: YSTART = Ypos, YEND = (Ypos + Height - 1) */
  LCD_IO_WriteReg(LCD_REG_43);
  data = (Ypos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos + Height - 1) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos + Height - 1) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Draws horizontal line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.
  * @retval None
  */
void st7735_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;

  if(Xpos + Length > ST7735_LCD_PIXEL_WIDTH) return;

  /* Set Cursor */
  st7735_SetCursor(Xpos, Ypos);

  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }
  LCD_IO_WriteMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);
}

/**
  * @brief  Draws vertical line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.
  * @retval None
  */
void st7735_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;

  if(Ypos + Length > ST7735_LCD_PIXEL_HEIGHT) return;
  for(counter = 0; counter < Length; counter++)
  {
    st7735_WritePixel(Xpos, Ypos + counter, RGBCode);
  }
}

/**
  * @brief  Gets the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t st7735_GetLcdPixelWidth(void)
{
  return ST7735_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Gets the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t st7735_GetLcdPixelHeight(void)
{
  return ST7735_LCD_PIXEL_HEIGHT;
}

/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void st7735_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, size = 0;

  /* Read bitmap size */
  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  size = (size - index)/2;
  pbmp += index;

  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 0, MX = 1, MV = 0, ML = 0 */
  st7735_WriteReg(LCD_REG_54, 0x40);

  /* Set Cursor */
  st7735_SetCursor(Xpos, Ypos);

  LCD_IO_WriteMultipleData((uint8_t*)pbmp, size*2);

  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
  st7735_WriteReg(LCD_REG_54, 0xC0);
}
void TFTInit()
{

	TFTWriteCmd(ST7735_SWRESET); // software reset
	Tick(50);
	TFTWriteCmd(ST7735_SLPOUT);  // out of sleep mode
	Tick(500);

	TFTWriteCmd(ST7735_COLMOD);  // set color mode
	TFTWriteData(0x05);          // 16-bit color
	Tick(10);

	TFTWriteCmd(ST7735_FRMCTR1); // frame rate control
	TFTWriteData(0x00);          // fastest refresh
	TFTWriteData(0x06);          // 6 lines front porch
	TFTWriteData(0x03);          // 3 lines backporch
	Tick(10);

	TFTWriteCmd(ST7735_MADCTL);  // memory access control (directions)
	TFTWriteData(0xC8);          // row address/col address, bottom to top refresh

	TFTWriteCmd(ST7735_DISSET5); // display settings #5
	TFTWriteData(0x15);          // 1 clock cycle nonoverlap, 2 cycle gate rise, 3 cycle oscil. equalize
	TFTWriteData(0x02);          // fix on VTL

	TFTWriteCmd(ST7735_INVCTR);  // display inversion control
	TFTWriteData(0x0);           // line inversion

	TFTWriteCmd(ST7735_PWCTR1);  // power control
	TFTWriteData(0x02);          // GVDD = 4.7V
	TFTWriteData(0x03);          // 1.0uA
	Tick(10);
	TFTWriteCmd(ST7735_PWCTR2);  // power control
	TFTWriteData(0x05);          // VGH = 14.7V, VGL = -7.35V
	TFTWriteCmd(ST7735_PWCTR3);  // power control
	TFTWriteData(0x01);          // Opamp current small
	TFTWriteData(0x02);          // Boost frequency


	TFTWriteCmd(ST7735_VMCTR1);  // power control
	TFTWriteData(0x3C);          // VCOMH = 4V
	TFTWriteData(0x38);          // VCOML = -1.1V
	Tick(10);

	TFTWriteCmd(ST7735_PWCTR6);  // power control
	TFTWriteData(0x11);
	TFTWriteData(0x15);

	TFTWriteCmd(ST7735_GMCTRP1);
	TFTWriteData(0x09);
	TFTWriteData(0x16);
	TFTWriteData(0x09);
	TFTWriteData(0x20);
	TFTWriteData(0x21);
	TFTWriteData(0x1B);
	TFTWriteData(0x13);
	TFTWriteData(0x19);
	TFTWriteData(0x17);
	TFTWriteData(0x15);
	TFTWriteData(0x1E);
	TFTWriteData(0x2B);
	TFTWriteData(0x04);
	TFTWriteData(0x05);
	TFTWriteData(0x02);
	TFTWriteData(0x0E);
	TFTWriteCmd(ST7735_GMCTRN1);
	TFTWriteData(0x0B);
	TFTWriteData(0x14);
	TFTWriteData(0x08);
	TFTWriteData(0x1E);
	TFTWriteData(0x22);
	TFTWriteData(0x1D);
	TFTWriteData(0x18);
	TFTWriteData(0x1E);
	TFTWriteData(0x1B);
	TFTWriteData(0x1A);
	TFTWriteData(0x24);
	TFTWriteData(0x2B);
	TFTWriteData(0x06);
	TFTWriteData(0x06);
	TFTWriteData(0x02);
	TFTWriteData(0x0F);
	Tick(10);

	TFTWriteCmd(ST7735_CASET);	/* Set window area X*/
	TFTWriteData(0x00);
	TFTWriteData(0x02);
	TFTWriteData(0x00);
	TFTWriteData(0x81);
	Tick(10);
	TFTWriteCmd(ST7735_RASET);	/* Set window area Y*/
	TFTWriteData(0x00);
	TFTWriteData(0x02);
	TFTWriteData(0x00);
	TFTWriteData(0x81F);
	Tick(10);

	TFTWriteCmd(ST7735_NORON);   // normal display on
	Tick(10);

	TFTWriteCmd(ST7735_DISPON);
	Tick(500);

	TFTWriteCmd(0x00);

	//TFTBackLight(1);

}
void TFTInit2()
{
	__HIGH(LCD_Reset);

	TFTWriteCmd(ST7735_SWRESET); // software reset
	Tick(250);
	TFTWriteCmd(ST7735_SLPOUT);  // out of sleep mode
	Tick(250);
	TFTWriteCmd(0x3A); //Interface pixel format
	//TFTWriteData(0xC6); //18 bit color/ 18bit/pixel
	TFTWriteData(0x55); //16bit
	TFTWriteCmd(0x26); //GAMMA Curve
	TFTWriteData(0x01); //CURVE 1

	TFTWriteCmd(0x13); //normal display mode ON

	TFTWriteCmd(0xB1); //Frame Rate 61,7Hz for 128*160 px
	TFTWriteData(0x0E);  //DIVA 14 decimal
	TFTWriteData(0x14);    //VPA 20 decimal

	TFTWriteCmd(0xB4);   //display invert frame
	TFTWriteData(0x07);

	TFTWriteCmd(0xC0);   //PWR CTRL 1
	TFTWriteData(0x0A);   //4.3VC
	TFTWriteData(0x03);   //2.6VRH

	TFTWriteCmd(0xC1);   //PWR CTRL 2
	TFTWriteData(0x02);   //VCL VGH	VGL 2 2xVCI -1xVCI1 2.5xAVDD -2.5xAVDD


	TFTWriteCmd(0xC5);   //VCOM Ctrl 1
	TFTWriteData(0x50);   //VCOML 4.50
	TFTWriteData(0x56);   //VCOML -0.350

	TFTWriteCmd(0xC7);   //VCOM Offset Ctrl
	TFTWriteData(0x00);

	TFTWriteCmd(0x2A);//Set Column Address
	TFTWriteData(0x00);
	TFTWriteData(0x7F); //127

	TFTWriteCmd(0x2B);//Set Page Address
	TFTWriteData(0x00);
	TFTWriteData(0x9F); //159

	TFTWriteCmd(ST7735_DISPON);  // set color mode
	Tick(50);

}
void Tick(uint16_t i)
{
	HAL_Delay(i);
}
void TFTWriteCmd(uint8_t command)
{
	uint8_t i = 8;

	 __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
	 __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
	 HAL_SPI_Transmit(&SpiHandle,&command, 1,500);
	__HIGH(LCD_ChipSelect);
	__HIGH(LCD_CMD);

	return;
}

void TFTWriteData(uint8_t data)
{
	uint8_t i = 8;

	__LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
	__HIGH(LCD_CMD); //LCD_CMD pin = HIGH = Send Data
	HAL_SPI_Transmit(&SpiHandle,&data, 1,500);
	__HIGH(LCD_ChipSelect);


	return;
}
void TFTSetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	TFTWriteCmd(ST7735_CASET);   /* col address command */
	TFTWriteData(0x00);
	TFTWriteData(x0);          /* X Start */
	TFTWriteData(0x00);
	TFTWriteData(x1);          /* X end */

	TFTWriteCmd(ST7735_RASET);   /* row address command */
	TFTWriteData(0x00);
	TFTWriteData(y0);          /* Y Start */
	TFTWriteData(0x00);
	TFTWriteData(y1);          /* Y end */
}

void TFTPixel(uint16_t x, uint16_t y, uint16_t color)
{
	TFTSetWindow(x, y, x+1 , y+1);
	TFTWriteCmd(ST7735_RAMWR);		/* RAM Access */

	TFTWriteData(color >> 8);
	TFTWriteData(color);
}
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
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

