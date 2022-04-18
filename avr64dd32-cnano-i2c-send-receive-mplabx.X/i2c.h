/*
    \file   i2c.h

    \brief  TWI I2C Driver - header file

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

#ifndef I2C_H_INCLUDED
#define	I2C_H_INCLUDED

#include <avr/io.h>
#include <stdint.h>


#define I2C_SPEED   100000      /* Hz */

typedef enum
{
    I2C_ERROR_NONE,             /* No Error */
    I2C_ERROR_ADDR_NACK,        /* Address Not Acknowledged */
    I2C_ERROR_DATA_NACK,        /* Data Not Acknowledged */
    I2C_ERROR_BUS_COLLISION,    /* Bus Collision Error */
    I2C_ERROR_BUSY              /* I2C Host is busy */
} i2c_error_t;

void        I2C_Init(void);
i2c_error_t I2C_Write(uint8_t address, uint8_t *pData, size_t dataLength);
i2c_error_t I2C_Read(uint8_t address, uint8_t *pData, size_t dataLength);

#endif	/* I2C_H_INCLUDED */
