/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <proc/p32mx270f256b.h>

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.rfid_data_Received = false;
    appData.i = 0 ;
    appData.x = 0;
    appData.rfid[11] = '\0';
    //appData.rfid = "123456789A\r";
    //appData._AT = "test";
    appData.timeout = 0;
    //appData.rfid[8] = '0x0D';
    
    appData.rfid_data_Received = false;
    appData.rfid_Input_Start = false;
    appData.URCS_ON = false;
    appData.GPRS_CON_ON = false;
    appData.APN_ON = false;
    appData.HTTP_ON = false;
    appData.conID_0 = false;
    appData.hcMethod_0 = false;
    appData.Internet_Address = false;
    appData.SISO_0 = false;
    appData.Web_data_read = false;
    appData.GSM_data_done = false;
    
    appData.state = APP_STATE_INIT;
    
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            TRISAbits.TRISA1 = 0x00;
            //TRISBbits.TRISB8 = 0x01;
            TRISBbits.TRISB11 = 0x00;
            
            appData.currentState = appData.state;
 
            memset(appData.rfid,11,NULL);
            memset(appData.GSM_RX,300,NULL);
            bool appInitialized = true;
            if (appInitialized)
            {
                appData.state = APP_STATE_IDLE;
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        case APP_STATE_RX:
        {
            appData.currentState = appData.state;
            
            if(!DRV_USART1_ReceiverBufferIsEmpty())
            {
                appData.rx_byte = DRV_USART1_ReadByte();
                appData.tx_byte = appData.rx_byte + 1;
                appData.state = APP_STATE_TX;
            }
            break;
        }
        
        case APP_STATE_TX:
        {
            appData.currentState = appData.state;
            
            if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART1_TransferStatus()))
            {
                DRV_USART1_WriteByte(appData.tx_byte);
                appData.state = APP_STATE_RX;
            }
            break;
        }
        
        case WAIT_OK:
        {
            
            volatile char *rx = strstr(appData.GSM_RX,"OK"); 
            volatile char *rx2 = strstr(appData.GSM_RX,appData.rfid);
            appData.timeout++;
            
            if(appData.timeout == 500000)
            {
                appData.timeout = 0;
                appData.x = 0;
                appData.state = appData.currentState;
                break;
            }
            
            if(rx)
            {
                if(appData.currentState == GSM_TEST)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_URCS_ON; //1
                    break;
                }
                
                if(appData.currentState == GSM_URCS_ON)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_GPRS_ON; //2
                    break;
                }
                
                if(appData.currentState == GSM_GPRS_ON)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_APN_SET; //3
                    break;
                }
                
                if(appData.currentState == GSM_APN_SET)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_SERV_SET; //4
                    break;
                }
                
                if(appData.currentState == GSM_SERV_SET)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_CONID_SET; //5
                    break;
                }
                
                if(appData.currentState == GSM_CONID_SET)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_HCMETHOD_SET; //6
                    break;
                }
                
                if(appData.currentState == GSM_HCMETHOD_SET)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_ADDRESS_SET; //7
                    break;
                }
                
                if(appData.currentState == GSM_ADDRESS_SET)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_SISO_SET; //8
                    break;
                }
                
                if(appData.currentState == GSM_SISO_SET)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    memset(appData.GSM_RX, NULL,300);
                    appData.state = GSM_READ; //9
                    break;
                }
                
                if(appData.currentState == GSM_READ)
                    {
                        appData.timeout = 0;
                        appData.x = 0;
                        appData.GSM_Interrupt = false;
                        memset(appData.GSM_RX, NULL,300);
                        appData.state = GSM_CLOSE; //10
                        break;
                    }
                
                if(appData.currentState == GSM_CLOSE)
                {
                    appData.timeout = 0;
                    appData.x = 0;
                    appData.GSM_Interrupt = false;
                    appData.rfid_data_Received = false;
                    memset(appData.GSM_RX, NULL,300);
                    //appData.rfid_data_Received = false;
                    appData.state = APP_STATE_IDLE; //10
                    break;
                }
            }
                
            
            break;
        }
        
        case GSM_TEST:
        {
            appData.currentState = appData.state;
            appData._AT = "at\r\n";
            _gsm_Send("at\r\n\0");    
            _delayMs(1500);
            appData.state = WAIT_OK;
            break;
        }
            
        case GSM_URCS_ON:
        {
            appData.currentState = appData.state;
            //char *AT_2 = {'a','t','^','s','c','f','g','=','t','c','p','/','w','i','t','h','u','c','r','s',',',0x22,'o','n',0x22,'\0'};
            char* _AT = "at^scfg=tcp/withurcs,\"on\"\r\n";
            _gsm_Send("at^scfg=tcp/withurcs,\"on\"\r\n\0");  
            _delayMs(1500);
            appData.state = WAIT_OK;
            break;
        }
            
        case GSM_GPRS_ON:
        {
            appData.currentState = appData.state;
            //char AT_3[] = {'a','t','^','s','c','i','s','=','0',',','c','o','n','T','y','p','e',',',0x22,'G','P','R','S','0',0x22,'\0'};
            char* _AT = "at^sics=0,conType,\"GPRS0\"\r\n" ;
            _gsm_Send("at^sics=0,conType,\"GPRS0\"\r\n\0");  
            _delayMs(1500);
            appData.state = WAIT_OK;
            break;
        }
        
    
            
        case GSM_APN_SET:
        {
            appData.currentState = appData.state;
            //char AT_4[] = {'a','t','^','s','c','i','s','=','0',',','a','p','n',',',0x22,'i','n','t','e','r','n','e','t',0x22,'\0'};
            char* _AT = "at^sics=0,apn,\"internet\"\r\n";        
            _gsm_Send("at^sics=0,apn,\"internet\"\r\n\0");   
            _delayMs(1500);
            appData.state = WAIT_OK;
            break;
        }
        

            
        case GSM_SERV_SET:
        {
            appData.currentState = appData.state;
            //char AT_5[] = {'a','t','^','s','i','s','s','=','0',',','s','r','v','T','y','p','e',',',0x22,'H','t','t','p',0x22,'\0'};
            char* _AT = "at^siss=0,srvType,\"Http\"\r\n";
            _gsm_Send("at^siss=0,srvType,\"Http\"\r\n\0");
            _delayMs(1500);
            appData.state = WAIT_OK;
            break;
        }
           
       
            
        case GSM_CONID_SET:
        {
            appData.currentState = appData.state;
            //char AT_6[] = {'a','t','^','s','i','s','s','=','0',',','c','o','n','I','d',',','0','\0'};
            char* _AT = "at^siss=0,conId,0\r\n";        
            _gsm_Send("at^siss=0,conId,0\r\n\0");
            _delayMs(1500);
            appData.state = WAIT_OK;
            break;
        }
            
        case GSM_HCMETHOD_SET:
        {
            appData.currentState = appData.state;
            //char AT_7[] = {'a','t','^','s','i','s','s','=','0',',','h','c','M','e','t','h','o','d',',','0','\0'};
            char* _AT = "at^siss=0,hcMethod,0\r\n";
            _gsm_Send("at^siss=0,hcMethod,0\r\n\0");
            _delayMs(1500);
            appData.state = WAIT_OK;
            break;
          
        }
            
        case GSM_ADDRESS_SET:
        {
            unsigned int y = 0;
            appData.currentState = appData.state;
            char _AT[200];
            memset(_AT,0,200);
            char* _address = "\"http://www.wilhelmbotha.co.za/sendID.php?id=";
            strcpy(_AT,"at^siss=0,address,");
            strcat(_AT,_address);
            
            strcat(_AT,appData.rfid);
            //strcat(_AT,"\"");
            _AT[73] = 0x22;
            strcat(_AT,"\r\n\0");
            _delayMs(1500);
            _gsm_Send(_AT);
            //_gsm_Send("at^siss=0,address,\"http://www.wilhelmbotha.co.za\"\r\n\0");
            _delayMs(500);
            appData.state = WAIT_OK;
            break;
        }
            
        case GSM_SISO_SET:
        {
            appData.currentState = appData.state;
            //char AT_9[] = {'a','t','^','s','i','s','o','=','0','\0'};
            char* _AT = "at^siso=0\r\n";
            _gsm_Send("at^siso=0\r\n\0");  
            _delayMs(1500);
            appData.GSM_SETUP_DONE = true;
            appData.state = WAIT_OK;
            break;
        }
            
        case GSM_READ:
        {
            appData.currentState = appData.state;
            //char AT_10[] = {'a','t','^','s','i','s','r','=','0',',','1','5','0','0','\0'};;
            char* _AT = "at^sisr=0,1500\r\n";
            _delayMs(500000);
            _gsm_Send("at^sisr=0,1500\r\n\0");
            _delayMs(500000);
            appData.state = WAIT_OK;
            break;
        }
            
        case GSM_CLOSE:
        {
            appData.currentState = appData.state;
            //char AT_11[] = {'a','t','^','s','i','s','c','=','0','\0'};
            char* _AT = "at^sisc=0\r\n";
            _gsm_Send("at^sisc=0\r\n\0");
            _delayMs(1500);
            //appData.rifd_data_Received = false;
            PORTBbits.RB11 = 0x00;
            appData.state = WAIT_OK;
            break;
            
        }
        
        case APP_STATE_IDLE:
        {
            appData.currentState = appData.state;
            //PORTBbits.RB11 = 0x01;
            PORTAbits.RA1 = 0x01;
            if(_strContains(appData.rfid, "\r") && appData.rfid_data_Received == true && appData.GSM_SETUP_DONE == true)
            {
                appData.i = 0;
                PORTBbits.RB11 = 0x01;
                //appData.rfid_data_Received = false;
                appData.rfid_Input_Start = false;
                appData.rfid[10] = '\0';
                appData.state = GSM_SERV_SET;
            }
            else if(_strContains(appData.rfid, "\r") && appData.rfid_data_Received == true)
            {
                appData.i = 0;
                PORTBbits.RB11 = 0x01;
                //appData.rfid_data_Received = false;
                appData.rfid_Input_Start = false;
                appData.rfid[10] = '\0';
                //memset(appData.rfid,11, NULL);
                appData.state = GSM_TEST;
            }
             
            break;
        }
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

char* _strAdd(char* _dest, const char*_toAdd)
{
    size_t i,j;
    for (i = 0; _dest[i] != '\0'; i++)
        ;
    for (j = 0; _toAdd[j] != '\0'; j++)
        _dest[i+j] = _toAdd[j];
    _dest[i+j] = '\0';
    return _dest;
}


void _delayMs(unsigned long delay)
{
    uint32_t start = _CP0_GET_COUNT();
    uint32_t total = ((delay*(SYS_FREQ)/80000));
    
    while((_CP0_GET_COUNT() - start)<total);
    /*delay = (delay*1000000);

    while(delay)
    {
        //nop();
        delay--;
    }*/
}

void _gsm_Send(const char *msg)
{
    unsigned int y = 0;
    
    for(y = 0; y <= strlen(msg);y++)
    {
        //if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART1_TransferStatus()))
        //{
            _delayMs(3000);
            DRV_USART1_WriteByte(msg[y]);
            _delayMs(3000);
        //}
    }
    
}

char* _strContains(const char *_myString, const char* _contains)
{
    if((*_myString != 0x00) && (*_contains != 0x00))
    {
        unsigned int y,x = 0;
        unsigned int _containLength = _strLength(_contains);
        char* _s1Start = (char*)_myString;
        for(y=0;y<_containLength;y++)
        {
            if(_myString[y] == _contains[0])
            {
                for(x=0;x<_containLength;x++)
                {
                    if(!_myString[y+x] == _contains[x])
                    {
                        return NULL;
                    }
                    else
                    {
                        return _s1Start;
                    }
                }
            }
        }
    }
}

int _strLength(const char* _string)
{
    unsigned int y = 0;
    if(*_string != 0x00)
    {
        while(1)
        {
            if(_string[y] == '\0')
            {
                return y;
            }
            y++;
        }
    }
}

/*******************************************************************************
 End of File
 */
