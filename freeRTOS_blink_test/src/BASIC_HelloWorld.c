/*********************************************************************
*                SEGGER MICROCONTROLLER SYSTEME GmbH                 *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2004  SEGGER Microcontroller Systeme GmbH        *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

***** emWin - Graphical user interface for embedded applications *****
emWin is protected by international copyright laws.   Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with a license and should not be re-
distributed in any way. We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : BASIC_HelloWorld.c
Purpose     : Simple demo drawing "Hello world"
----------------------------------------------------------------------
*/

#include "st7735.h"

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       MainTask
*/
void MainTask(void) {
    int x,y,z=0;
    TFTSetWindow(0, 0, 128, 128);

    /*for(z = 0; z < 65535; z=z+100) {

    	for(y = 0; y < 128; y++) {
    		for(x = 0; x < 128; x++) {
    			TFTPixel(x, y, z);
    		}
    	}

    }*/
  while(1){

  };
}

/*************************** End of file ****************************/
