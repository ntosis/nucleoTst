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
    GUI_Clear();
    GUI_SetPenSize(10);
    GUI_SetColor(GUI_BLACK);
    GUI_DrawLine(50, 0, 150, 150);
    GUI_DrawLine(50, 0, 150, 150);
    GUI_SetBkColor(GUI_WHITE);
    GUI_SetColor(GUI_BLUE);
    GUI_SetTextMode(GUI_TM_NORMAL);
    GUI_DispStringHCenterAt("GUI_TM_NORMAL" , 50, 0);
    GUI_SetTextMode(GUI_TM_REV);
    GUI_DispStringHCenterAt("GUI_TM_REV" , 50, 26);
    GUI_SetTextMode(GUI_TM_TRANS);
    GUI_DispStringHCenterAt("GUI_TM_TRANS" , 50, 42);
    GUI_SetTextMode(GUI_TM_XOR);
    GUI_DispStringHCenterAt("GUI_TM_XOR" , 50, 58);
    GUI_SetTextMode(GUI_TM_TRANS | GUI_TM_REV);
    GUI_DispStringHCenterAt("GUI_TM_TRANS | GUI_TM_REV", 50, 74);
  while(1);
}

/*************************** End of file ****************************/
