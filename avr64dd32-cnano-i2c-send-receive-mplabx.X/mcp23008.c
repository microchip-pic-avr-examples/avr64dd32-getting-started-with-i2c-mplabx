/*
    \file   mcp23008.c

    \brief  8-bit I/O Expander I2C Client Driver

© [2022] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include "mcp23008.h"
#include "i2c.h"

/* Sends a command to the IO Expander I2C client device 
 * - returns true if 2 bytes have been sent 
 * - return false if client did not ACK/an unexpected number of bytes have been 
 * sent
 * The arguments are the address of the I2C client device register and the data 
 * to be written to that register
 */
bool MCP23008_Write(uint8_t address, uint8_t data)
{
    i2c_error_t i2c_ret;
    uint8_t command[2] = {address, data};
    
    i2c_ret = I2C_Write(MCP23008_CLIENT_ADDR, command, 2);

    if(i2c_ret == I2C_ERROR_NONE)
    {
        return true;
    }
    
    return false;
}
