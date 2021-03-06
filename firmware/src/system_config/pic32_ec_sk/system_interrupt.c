/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void __ISR(_UART_1_VECTOR, ipl1AUTO) _IntHandlerDrvUsartInstance0(void)
{    
    if(!DRV_USART0_ReceiverBufferIsEmpty())
    {
        if(DRV_USART0_ReadByte() == 10)
        {
            appData.rfid_Input_Start = true;
        }
        
        if(appData.rfid_Input_Start == true && appData.rfid_data_Received == false)
        {
            
            appData.length = DRV_USART0_ReceiverBufferSizeGet();
           
            for(appData.i; appData.i < 12;)
                {

                if(!DRV_USART0_ReceiverBufferIsEmpty())
                    {
                        appData.rfid[appData.i] = DRV_USART0_ReadByte();
                        if(appData.rfid[appData.i] == 13)
                        {
                            appData.rfid_data_Received = true;
                        }
                        appData.i = appData.i + 1;
                    }
                }
            /*if(!DRV_USART0_ReceiverBufferIsEmpty())
                {
                        appData.rfid[appData.i] = DRV_USART0_ReadByte();
                        appData.i++;
                }*/
            //appData.data_Received = true;
        }
         
    }
    
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
    DRV_USART_TasksReceive(sysObj.drvUsart0);
    DRV_USART_TasksError(sysObj.drvUsart0);
}
 
 
 

void __ISR(_UART_2_VECTOR, ipl1AUTO) _IntHandlerDrvUsartInstance1(void)
{
    
    if(!DRV_USART1_ReceiverBufferIsEmpty())
    {
        appData.GSM_Interrupt = true;
        appData.GSM_RX[appData.x] = DRV_USART1_ReadByte();     
       // DRV_USART1_WriteByte(appData.GSM_RX[appData.x]); //echo what was sent back
       
        appData.x++;
    }
    
    DRV_USART_TasksTransmit(sysObj.drvUsart1);
    DRV_USART_TasksReceive(sysObj.drvUsart1);
    DRV_USART_TasksError(sysObj.drvUsart1);
}
 
 
 

 

 

 

 
  
/*******************************************************************************
 End of File
*/

