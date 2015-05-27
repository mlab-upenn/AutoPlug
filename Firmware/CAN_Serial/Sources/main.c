/* TORCS - CAN gateway
 *
 * Receives car params from TORCS and sends them on the CAN bus
 * Receives driver inputs from the CAN bus and sends them to TORCS
 */

#include <hidef.h>      /* common defines and macros */
#include <MC9S12C128.h> /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c128"
#include <string.h>
#include "common.h"
#include "CAN.h"
#include "SCI.h"
#include "types.h"
#include "PLL.h"

CarParams carParams;
volatile TorcsInput torcsInput;
volatile UINT8 carInputsUpdated = 0;
volatile UINT8 carParamsUpdated = 0;

UINT8 serialRxBuffer[sizeof(CarParams)];

void init(void);

#pragma CODE_SEG __NEAR_SEG NON_BANKED

interrupt 20 void SCIRx_vect(void)
{
    UINT8 status, dummy;
    static UINT8 serialRxState = 0;
    UINT8 serialData;
    static UINT8 serialDataLength = 0;
    static UINT8 serialRxChksum = 0;
    static UINT8 *rxPtr;

    status = SCISR1;

    if(SCISR1_RDRF == 0)  /* No data */
        return;

    /* Check for Errors (Framing, Noise, Parity) */
    if( (status & 0x07) != 0 )
    {
        dummy = SCIDRL;
        return;
    }

    /* Good Data */
    serialData = SCIDRL; /* load SCI register to data */
    SCIDataFlag = 1;

    switch(serialRxState)
    {
        case 0:
            if(serialData == 0xAA)
            {
                serialRxChksum = 0xAA;
                serialRxState = 1;
            }
            break;

        case 1:
            if(serialData == 0xCC && serialRxState == 1)
            {
                serialDataLength = sizeof(CarParams);
                serialRxChksum ^= 0xCC;
                rxPtr = serialRxBuffer;
                serialRxState = 2;
            }
            else
            {
                serialRxState = 0;
            }
            break;

        case 2:
            if(serialDataLength > 0)
            {
                *rxPtr = serialData;
                serialRxChksum ^= serialData;
                rxPtr++;
                serialDataLength--;
            }
            else
            {
                if(serialData == serialRxChksum)
                {
                    if(!carParamsUpdated) // Only update when old value has been used
                    {
                        memcpy(&carParams, serialRxBuffer, sizeof(CarParams));
                        carParamsUpdated = 1;
                    }
                }
                serialRxState = 0;
            }
            break;
    }
}

interrupt 38 void CANRx_vect(void)
{
    UINT16 identifier;
    UINT8 length;

    identifier = (CANRXIDR0 << 3) + (CANRXIDR1 >> 5);

    length = CANRXDLR & 0x0F;

    if(identifier == CAN_INPUT_MSG_ID)
    {
        CarInputs carInputs;

        if(length > sizeof(CarInputs))
            length = sizeof(CarInputs);

        memcpy(&carInputs, &CANRXDSR0, length);
        torcsInput.steer = carInputs.steer;
        carInputsUpdated = 1;
    }
    else if(identifier == CAN_BRAKE_MSG_ID)
    {
        BrakeMsg brakeMsg;

        if(length > sizeof(BrakeMsg))
            length = sizeof(BrakeMsg);

        memcpy(&brakeMsg, &CANRXDSR0, length);
        torcsInput.brakeFL = brakeMsg.brakeFL;
        torcsInput.brakeFR = brakeMsg.brakeFR;
        torcsInput.brakeRL = brakeMsg.brakeRL;
        torcsInput.brakeRR = brakeMsg.brakeRR;
        carInputsUpdated = 1;
    }
    else if(identifier == CAN_ACCEL_MSG_ID)
    {
        AccelMsg accelMsg;
        if(length > sizeof(AccelMsg))
            length = sizeof(AccelMsg);

        memcpy(&accelMsg, &CANRXDSR0, length);
        torcsInput.accel = accelMsg.accel;
        torcsInput.gear = accelMsg.gear;
        torcsInput.clutch = accelMsg.clutch;
        carInputsUpdated = 1;
    }
    CANRFLG_RXF = 1; // Reset the flag
}

#pragma CODE_SEG DEFAULT


void main(void)
{
    UINT16 i;

    init();

    EnableInterrupts;

    for(;;)
    {
        if(carParamsUpdated)
        {
            CANTx(CAN_PARAM_MSG_ID, &carParams, sizeof(CarParams));
            carParamsUpdated = 0;
        }

        if(carInputsUpdated)
        {
            TorcsInput temp;
            memcpy(&temp, &torcsInput, sizeof(TorcsInput));
            SCITxPkt(&temp, sizeof(TorcsInput));
            carInputsUpdated = 0;
        }
    }
}

void init()
{
    setclk24();    // set Eclock to 24 MHZ
    SCIInit();
    CANInit();
}
