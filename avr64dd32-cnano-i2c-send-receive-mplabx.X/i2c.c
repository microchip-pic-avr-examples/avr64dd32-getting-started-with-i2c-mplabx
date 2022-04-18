
/*
    \file   i2c.c

    \brief  TWI I2C Driver

    (c) 2022 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third-party
    license terms applicable to your use of third-party software (including open source software) that
    may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/
#include "clkctrl.h"
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "i2c.h"

#define I2C_BAUD(F_SCL, T_RISE)   ((((((float)(F_CPU) / (float)(F_SCL))) - 10 - ((float)(F_CPU) * (T_RISE) / 1000000))) / 2)
#define I2C_SEND_TX_ADDR(data)    TWI0.MADDR=(data)
#define I2C_SEND_TX_DATA(data)    TWI0.MDATA=(data)
#define I2C_GET_RX_DATA()         (TWI0.MDATA)
#define I2C_SEND_NACK()           TWI0.MCTRLB|=TWI_ACKACT_bm
#define I2C_SEND_ACK()            TWI0.MCTRLB&=~TWI_ACKACT_bm
#define I2C_SEND_STOP()           TWI0.MCTRLB|=TWI_MCMD_STOP_gc
#define I2C_CLEAR_INT_FLAGS()     TWI0.MSTATUS|=(TWI_RIF_bm|TWI_WIF_bm)
#define I2C_RESET_BUS()           do{TWI0.MCTRLA&=~TWI_ENABLE_bm;TWI0.MCTRLA|=TWI_ENABLE_bm;TWI0.MSTATUS|=TWI_BUSSTATE_IDLE_gc;}while(0)
#define I2C_IS_NACK()             (TWI0.MSTATUS&TWI_RXACK_bm)?true:false
#define I2C_IS_BUS_ERROR()        (TWI0.MSTATUS&TWI_BUSERR_bm)?true:false  
#define I2C_IS_DATA()             (TWI0.MDATA)?true:false
#define I2C_IS_ADDR()             (TWI0.MADDR)?true:false
#define I2C_IS_ARB_LOST()         (TWI0.MSTATUS&TWI_ARBLOST_bm)?true:false
#define I2C_IS_BUSY()             (internal_status.busy || !(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))?true:false

typedef enum
{
    I2C_STATE_IDLE = 0,
    I2C_STATE_SEND_RD_ADDR,
    I2C_STATE_SEND_WR_ADDR,
    I2C_STATE_TX,
    I2C_STATE_RX,
    I2C_STATE_NACK,
    I2C_STATE_ERROR,
    I2C_STATE_STOP,
    I2C_STATE_RESET
} i2c_states_t;

typedef struct
{
    bool busy;
    uint8_t address;
    uint8_t *writePtr;
    size_t writeLength;
    uint8_t *readPtr;
    size_t readLength;
    bool switchToRead;
    i2c_error_t errorState; 
    i2c_states_t state;
} i2c_internal_t;

typedef i2c_states_t (*i2c_handler_t)(void);

static i2c_states_t I2C_EventIdle(void);
static i2c_states_t I2C_EventSendReadAddr(void);
static i2c_states_t I2C_EventSendWriteAddr(void);
static i2c_states_t I2C_EventTransmit(void);
static i2c_states_t I2C_EventReceive(void);
static i2c_states_t I2C_EventNACK(void);
static i2c_states_t I2C_EventError(void);
static i2c_states_t I2C_EventStop(void);
static i2c_states_t I2C_EventReset(void);

static volatile i2c_internal_t internal_status;

static const i2c_handler_t events_table[] = {
    I2C_EventIdle,
    I2C_EventSendReadAddr,
    I2C_EventSendWriteAddr,
    I2C_EventTransmit,
    I2C_EventReceive,
    I2C_EventNACK,
    I2C_EventError,
    I2C_EventStop,
    I2C_EventReset
};

static void I2C_Close(void)
{
    internal_status.busy = false;
    internal_status.address = 0xFF;
    internal_status.writePtr = NULL;
    internal_status.readPtr = NULL;
    internal_status.state = I2C_STATE_IDLE;
    I2C_CLEAR_INT_FLAGS();
    TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
}

static i2c_states_t I2C_EventIdle(void)
{
    internal_status.busy = false;
    TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
    return I2C_STATE_RESET;
}

static i2c_states_t I2C_EventSendReadAddr(void)
{
    I2C_SEND_TX_ADDR((uint8_t) (internal_status.address << 1 | 1));
    return I2C_STATE_RX;
}

static i2c_states_t I2C_EventSendWriteAddr(void)
{
    I2C_SEND_TX_ADDR((uint8_t) (internal_status.address << 1));
    return I2C_STATE_TX;
}

static i2c_states_t I2C_EventTransmit(void)
{
    i2c_states_t retEventState = I2C_STATE_TX;

        if (internal_status.writeLength)
        {
            internal_status.writeLength--;
            I2C_SEND_TX_DATA(*internal_status.writePtr++);
            retEventState = I2C_STATE_TX;
        }
        else
        {
            if (internal_status.switchToRead)
            {
                internal_status.switchToRead = false;
                retEventState = I2C_STATE_SEND_RD_ADDR;
            }
            else
            {
                retEventState = I2C_EventStop();
            }
        }
    return retEventState;
}

static i2c_states_t I2C_EventReceive(void)
{
    i2c_states_t retEventState = I2C_STATE_RX;

    if (internal_status.readLength == 1)
    {
        /* Next byte will be last to be received, setup NACK */
        I2C_SEND_NACK();
    }
    else
    {
        /* More bytes to receive, setup ACK */
        I2C_SEND_ACK();
    }

    if (--internal_status.readLength)
    {
        *internal_status.readPtr++ = I2C_GET_RX_DATA();
        /* Execute Acknowledge Action followed by a byte read operation */
        TWI0.MCTRLB |= TWI_MCMD_RECVTRANS_gc;
        retEventState = I2C_STATE_RX;
    }
    else
    {
        *internal_status.readPtr++ = I2C_GET_RX_DATA();
        I2C_SEND_NACK();
        retEventState = I2C_EventStop();
    }
    return retEventState;
}

static i2c_states_t I2C_EventNACK(void)
{
    i2c_states_t retEventState = I2C_STATE_NACK;
    retEventState = I2C_EventStop();
    return retEventState;
}

static i2c_states_t I2C_EventError(void)
{
    /* Clear bus collision status flag */
    i2c_states_t retEventState = I2C_STATE_ERROR;
    I2C_CLEAR_INT_FLAGS();
    retEventState = I2C_EventReset();
    return retEventState;
}

static i2c_states_t I2C_EventReset(void)
{
    I2C_RESET_BUS();
    internal_status.busy = false;
    return I2C_STATE_IDLE;
}

static i2c_states_t I2C_EventStop(void)
{
    I2C_SEND_STOP();
    I2C_Close();
    return I2C_STATE_IDLE;
}

static void I2C_ErrorHandler(void)
{
    if (I2C_IS_BUS_ERROR())
    {
        internal_status.state = I2C_STATE_ERROR;
        internal_status.errorState = I2C_ERROR_BUS_COLLISION;
        TWI0.MSTATUS |= TWI_BUSERR_bm;
    }
    else if (I2C_IS_ADDR() && I2C_IS_NACK())
    {
        internal_status.state = I2C_STATE_NACK;
        internal_status.errorState = I2C_ERROR_ADDR_NACK;
        TWI0.MSTATUS |= TWI_RXACK_bm;
    }
    else if (I2C_IS_DATA() && I2C_IS_NACK())
    {
        internal_status.state = I2C_STATE_NACK;
        internal_status.errorState = I2C_ERROR_DATA_NACK;
        TWI0.MSTATUS |= TWI_RXACK_bm;
    }
    else if(I2C_IS_ARB_LOST())
    {
        internal_status.state = I2C_STATE_ERROR;
        internal_status.errorState = I2C_ERROR_BUS_COLLISION;
        TWI0.MSTATUS |= TWI_ARBLOST_bm;
    }
    internal_status.state = events_table[internal_status.state]();
}

static void I2C_Loop(void)
{
    if (I2C_IS_BUSY())
    {
        if(((TWI0.MSTATUS & TWI_RXACK_bm) && (TWI0.MSTATUS & TWI_WIF_bm) && (!(TWI0.MSTATUS & TWI_BUSERR_bm)) && (!(TWI0.MSTATUS & TWI_ARBLOST_bm))) || (TWI0.MSTATUS & TWI_BUSERR_bm) || (TWI0.MSTATUS & TWI_ARBLOST_bm))
        {
            I2C_ErrorHandler();
        }
        if ((TWI0.MSTATUS & TWI_RIF_bm) || (TWI0.MSTATUS & TWI_WIF_bm))
        {
            if ((TWI0.MSTATUS & TWI_RXACK_bm) || (TWI0.MSTATUS & TWI_BUSERR_bm) || (TWI0.MSTATUS & TWI_ARBLOST_bm))
            {
                I2C_ErrorHandler();
            }
            else
            {
                internal_status.state = events_table[internal_status.state]();
            }
        }
    }
}

void    I2C_Init(void)
{
    /* Enable Pull-ups */
    PORTA.PIN2CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN3CTRL |= PORT_PULLUPEN_bm;

    /* FMPEN OFF; INPUTLVL I2C; SDAHOLD OFF; SDASETUP 4CYC */
    TWI0.CTRLA = 0x0;
    
    /* Debug Run */
    TWI0.DBGCTRL = 0x0;
    
    /* Host Baud Rate Control */
    TWI0.MBAUD = (uint8_t)I2C_BAUD(I2C_SPEED, 0.1);
    
    /* ENABLE enabled; QCEN disabled; RIEN false; SMEN disabled; TIMEOUT DISABLED; WIEN false */
    TWI0.MCTRLA = TWI_ENABLE_bm;
    
    /* ARBLOST disabled; BUSERR disabled; BUSSTATE UNKNOWN; CLKHOLD disabled; RIF disabled; RXACK disabled; WIF disabled */
    TWI0.MSTATUS = 0x0;
    
    /* Host Address */
    TWI0.MADDR = 0x0;
    
    /* ACKACT ACK; FLUSH disabled; MCMD NOACT */
    TWI0.MCTRLB = 0x0;
    
    /* Host Data */
    TWI0.MDATA = 0x0;

    TWI0.MCTRLB  |= TWI_FLUSH_bm; 
    TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
    
    /* reset internal status structure */
    memset((void*)&internal_status, 0, sizeof(internal_status));
}

i2c_error_t I2C_Write(uint8_t address, uint8_t *pData, size_t dataLength)
{
    i2c_error_t retErrorState;
    if (!I2C_IS_BUSY())
    {
        internal_status.busy = true;
        internal_status.address = address;
        internal_status.switchToRead = false;
        internal_status.writePtr = pData;
        internal_status.writeLength = dataLength;
        internal_status.readPtr = NULL;
        internal_status.readLength = 0;
        internal_status.errorState = I2C_ERROR_NONE;
        internal_status.state = I2C_EventSendWriteAddr();
        do
        {
            I2C_Loop();
        } while(I2C_IS_BUSY());
        retErrorState = internal_status.errorState;
        internal_status.errorState = I2C_ERROR_NONE;
        return retErrorState;
    }
    else
        return I2C_ERROR_BUSY;
}

i2c_error_t I2C_Read(uint8_t address, uint8_t *pData, size_t dataLength)
{
    i2c_error_t retErrorState;
    if (!I2C_IS_BUSY())
    {
        internal_status.busy = true;
        internal_status.address = address;
        internal_status.switchToRead = false;
        internal_status.readPtr = pData;
        internal_status.readLength = dataLength;
        internal_status.writePtr = NULL;
        internal_status.writeLength = 0;
        internal_status.errorState = I2C_ERROR_NONE;
        internal_status.state = I2C_EventSendReadAddr();
        do
        {
            I2C_Loop();
        }while(I2C_IS_BUSY());
        retErrorState = internal_status.errorState;
        internal_status.errorState = I2C_ERROR_NONE;
        return retErrorState;
    }
    else
        return I2C_ERROR_BUSY;
}

