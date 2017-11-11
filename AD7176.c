/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 812
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7176.h"
#if defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#endif

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - Idle state for clock is a low level; active
 *                                  state is a high level;
 *	                      0x1 - Idle state for clock is a high level; active
 *                                  state is a low level.
 * @param clockEdg - SPI clock edge (0 or 1).
 *                   Example: 0x0 - Serial output data changes on transition
 *                                  from idle clock state to active clock state;
 *                            0x1 - Serial output data changes on transition
 *                                  from active clock state to idle clock state.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char SPI_Init()
{
    // Add your code here.
	TRISGbits.TRISG6 = 0;
	TRISGbits.TRISG7 = 1;
	TRISGbits.TRISG8 = 0;
	
    TRISGbits.TRISG15 = 0;//CS1
    TRISCbits.TRISC1 = 0;//CS2
    TRISDbits.TRISD5 = 0;//CS3
    TRISDbits.TRISD4 = 0;//CS4
	
    IFS2bits.SPI2IF = 0; // Clear the Interrupt Flag
	IEC2bits.SPI2IE = 0; // Disable the Interrupt
    SPI2STATbits.SPIROV=0;
	// SPI1CON1 Register Settings
	SPI2CON1bits.PPRE = 4; // 4:1 primary prescale
	SPI2CON1bits.SPRE = 7; // 1:1 secondary prescale
	SPI2CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
	SPI2CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
	SPI2CON1bits.MODE16 = 0; // Communication is byte-wide (8 bits)
	SPI2CON1bits.SMP = 1; // 
	SPI2CON1bits.CKE = 0;// 
	SPI2CON1bits.CKP = 1; // Idle state for clock is a low level; 
	// active state is a high level
	SPI2CON1bits.MSTEN = 1; // Master mode Enabled
	SPI2STATbits.SPIEN = 1; // Enable SPI module
	//SPI2BUF = 0x0000; // Write data to be trantesmitted 
	// Interrupt Controller Settings
	
	nSS1 = 1;
	nSS2 = 1;
	nSS3 = 1;
	nSS4 = 1;



}

unsigned char  read_write_byte(unsigned char data)
{
	unsigned char temp=0;
	
	asm("nop");
	while(SPI2STATbits.SPITBF){asm("nop");}
	SPI2BUF = data;
	asm("nop");
	
	while(!SPI2STATbits.SPIRBF){asm("nop");}
	temp= SPI2BUF;

	SPI2STATbits.SPIROV = 0; //clear overflow
	return temp;
}

/*****空为reserved 0*******/
void AD7176set(void)
{
	int i;

	/*****
	 AD 模式寄存器
	bit 15     0 disable REF_EN
	bit 13     0 sing_cyc
	bit 10 :8  111  1ms
	bit 6  :4  000 continual
	bit 3  :2  10  xtal2 
	
	0x01	
	************/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x01);
		read_write_byte(0x01);   
		read_write_byte(0x08);
		cs_high(i);
		DELAY(100); 
	}
	
	/*************
	借口模式寄存器
	bit  12  0  alt-sync
	bit  11  0  iostrength
	bit  8   1  dout-reset
	bit  7   0  contread
	bit  6   1  data-stat
	bit  5   0  reg-check
	bit  3:2  00  crc-en
	bit  0   0  wl16  
		
	0x02
	**********/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x02);
		read_write_byte(0x01);
		read_write_byte(0x40);
		cs_high(i);
		DELAY(100); 
	}
	
	/*************
	寄存器检查
	bit 23 0  
		
	0x03
	**********/
		
	/*************
	数据
	bit 23 0  
		
	0x04
	**********/
		
	/*************
	gpio
	bit 15 0  
	
	0x06
	**********/
		
	/*************
	ID
	bit 15 0  	
	
	0x07
	**********/
	
	/*************
	通道映射 寄存器0 
	bit 15      0    ch-en
	bit 13 12   000   setup-sel0
	bit 9  5    00000  ain0
	bit 4  0    10110  ref- 
	
	0x10
	**********/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x10);
		read_write_byte(0x80);
		read_write_byte(0x16);
		cs_high(i);
		DELAY(100); 
	}
	
	/*************
	通道映射 寄存器1
	bit 15      0    ch-en
	bit 13 12   000   setup-sel0
	bit 9  5    00001  ain1
	bit 4  0    10110  ref- 
	
	0x11
	**********/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x11);
		read_write_byte(0x80);
		read_write_byte(0x36);
		cs_high(i);
		DELAY(100); 
	}
	
	/*************
	通道映射 寄存器2
	bit 15      0    ch-en
	bit 13 12   000   setup-sel0
	bit 9  5    00000  ain2
	bit 4  0    10110  ref- 
	
	0x12
	**********/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x12);
		read_write_byte(0x80);
		read_write_byte(0x56);
		cs_high(i);
		DELAY(100); 
	}
	
	/*************
	通道映射 寄存器3
	bit 15      0    ch-en
	bit 13 12   000   setup-sel0
	bit 9  5    00011  ain3
	bit 4  0    10110  ref- 
	
	0x13
	**********/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x13);
		read_write_byte(0x80);
		read_write_byte(0x76);
		cs_high(i);
		DELAY(100); 
	}
	
	/***********
	设置配置寄存器 0
	bit 12   1  双极性输出
	bit 5:4  00  外部电压源 
	
	0x20
	*******/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x20);
		read_write_byte(0x10);
		read_write_byte(0x00);
		cs_high(i);
		DELAY(100); 
	}
	
	/***********
	滤波器设置 0
	bit 15  0
	bit 11 1
	bit 10 8    110
	bit 6 5   00
	bit 4 0  10011 输出速率 10
	
	0x28
	*******/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x28);
		read_write_byte(0x0e);
		read_write_byte(0x13);
		cs_high(i);
		DELAY(100); 
	}

	/***********
	增益设置 0
	
	0x38
	*******/
	for(i = 0;i < 4;i ++)
	{
		cs_low(i);
        DELAY(100); 
		read_write_byte(0x38);
		read_write_byte(0x55);
		read_write_byte(0x55);
		read_write_byte(0x55);
		cs_high(i);
		DELAY(100); 
	}

}

unsigned char check(int i)
{
	unsigned char flag;
	
	cs_low(i);
	DELAY(50);
	read_write_byte(0x40);
	flag=read_write_byte(0x00);
	cs_high(i);
	DELAY(50);
	
	return flag;
} 

void cs_low(int num)
{
	switch(num)
	{
		case 0:
		{
			nSS1 = 0;
			break;
		}
		case 1:
		{
			nSS2 = 0;
			break;
		}
		case 2:
		{
			nSS3 = 0;
			break;
		}
		case 3:
		{
			nSS4 = 0;
			break;
		}
		default: break;
	}
}

void cs_high(int num)
{
	switch(num)
	{
		case 0:
		{
			nSS1 = 1;
			break;
		}
		case 1:
		{
			nSS2 = 1;
			break;
		}
		case 2:
		{
			nSS3 = 1;
			break;
		}
		case 3:
		{
			nSS4 = 1;
			break;
		}
		default: break;
	}
}

void DELAY(unsigned int t) //t=1 15us
{
	unsigned int i,j;
  	for(i=0;i<50;i++)
  	{
	  	for(j=0;j<t;j++)
   		{
	   		asm("nop");	
    	}
   } 	
} 