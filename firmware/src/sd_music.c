/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sd_music.c

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

#include "sd_music.h"
#include "driver/driver_common.h"

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

SD_MUSIC_DATA sd_musicData;

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
    void SD_MUSIC_Initialize ( void )

  Remarks:
    See prototype in sd_music.h.
 */

void SD_MUSIC_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    sd_musicData.state = SD_MUSIC_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    sd_musicData.tmrHandle = DRV_HANDLE_INVALID;

}


/******************************************************************************
  Function:
    void SD_MUSIC_Tasks ( void )

  Remarks:
    See prototype in sd_music.h.
 */

void SD_MUSIC_Tasks ( void )
{
    uint32_t myFreq;
    uint32_t clkFreq;
    uint32_t divider;
    
    /* Check the application's current state. */
    switch ( sd_musicData.state )
    {
        /* Application's initial state. */
        case SD_MUSIC_STATE_INIT:
        {
            bool appInitialized = true;
            
            // Timer Open
            if (sd_musicData.tmrHandle == DRV_HANDLE_INVALID)
            {
                sd_musicData.tmrHandle = DRV_TMR_Open( DRV_TMR_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );
                while( DRV_TMR_CLIENT_STATUS_READY != DRV_TMR_ClientStatus(sd_musicData.tmrHandle));
                
                appInitialized &= ( DRV_HANDLE_INVALID != sd_musicData.tmrHandle );
            }
        
            if (appInitialized)
            {
                // Periodic 1ms
                myFreq = 1000;
                clkFreq = DRV_TMR_CounterFrequencyGet(sd_musicData.tmrHandle);
                divider = clkFreq / myFreq;
                DRV_TMR_AlarmRegister ( sd_musicData.tmrHandle, divider, true, 0, NULL );
                
                // Timer Start
                DRV_TMR_Start( sd_musicData.tmrHandle );
                
                // PWM duty 60(OFF):40(ON)
                // palue_width = 390(TMR2 PR2) * 40 / 100
                DRV_OC0_Start();
                DRV_OC0_PulseWidthSet(0x009C);
                sd_musicData.state = SD_MUSIC_STATE_SERVICE_TASKS;
            }
            break;
        }

        case SD_MUSIC_STATE_SERVICE_TASKS:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
