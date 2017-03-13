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

#include "GUI.h"
#include "BUTTON.h"
#include "heatingSys.h"
#include "temperatureSens.h"

static BUTTON_Handle hButton;
;
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
    GUI_SetFont(&GUI_Font8x16);
    GUI_SetBkColor(GUI_BLUE);
    static int i=0;
    GUI_DispStringAt("i: ",20,20); GUI_DispDecAt( i++, 110,20,4);
    GUI_DispStringAt("Temp: ",20,40); GUI_DispDecAt((int)actualTemperature(),110,40,4);
    GUI_DispStringAt("SOLL Temp: ",20,60); GUI_DispDecAt(SOLLtemperature,110,60,4);
}

/*************************** End of file ****************************/
