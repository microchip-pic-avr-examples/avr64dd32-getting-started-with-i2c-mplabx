/*
    \file   mcp3221.c

    \brief  12-bit ADC I2C Client Driver

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

#include "mcp3221.h"
#include "i2c.h"
#include <stdio.h>

/* Reads the ADC conversion result from the 12-bit ADC I2C client device
 * - returns true if 2 bytes have been received
 * - returns false if client did not ACK/an unexpected number of bytes have been 
 * received
 * Only one single word (uint16_t) variable must be allocated
 */
bool MCP3221_Read (uint16_t *pData)
{
    i2c_error_t i2c_ret;
    uint8_t *pDataTemp = (uint8_t *)pData;
    
    if(pData == NULL)
    {
        return false;
    }
    
    i2c_ret = I2C_Read(MCP3221_CLIENT_ADDR, pDataTemp, 2);
      
    if(i2c_ret == I2C_ERROR_NONE)
    {
        /* Change received bytes order */
        pDataTemp[0] = pDataTemp[0] + pDataTemp[1];
        pDataTemp[1] = pDataTemp[0] - pDataTemp[1];
        pDataTemp[0] = pDataTemp[0] - pDataTemp[1];
        return true;
    }
    
    return false;
}
