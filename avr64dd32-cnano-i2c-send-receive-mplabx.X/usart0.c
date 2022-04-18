/*
    \file   usart0.c

    \brief  USART0 Driver

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


#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include "clkctrl.h"
#include "usart0.h"

/* Normal Mode, Baud register value */
#define USART_BAUD_RATE(BAUD_RATE) (uint16_t)(((float)(F_CPU) * 64 / (16 * (float)(BAUD_RATE))) + 0.5)


int USART0_printCHAR(char character, FILE *stream)
{
    (void)stream;
    while (!(USART0.STATUS & USART_DREIF_bm))
            ;
    USART0.TXDATAL = character;
    return 0;
}

FILE USART0_stream = FDEV_SETUP_STREAM(USART0_printCHAR, NULL, _FDEV_SETUP_WRITE);

void USART0_Init(void)
{
    /* Set the TX pin as output */
    PORTD.DIRSET = PIN4_bm;

    /* Select USART0 pin PD4 */
    PORTMUX.USARTROUTEA = PORTMUX_USART0_ALT3_gc;
    
    /* Configure Baud Rate */
    USART0.BAUD = USART_BAUD_RATE(BAUD_RATE);
	
    /* Enable TX */
    USART0.CTRLB =  USART_TXEN_bm;

    stdout = &USART0_stream;
}
