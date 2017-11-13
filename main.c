#if defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#endif
#include "timer.h"
#include "adc.h"
#include "sci.h"
#include "AD7176.h" 
#include "stdio.h"
#include "collect.h"

_FBS(0xCF);
_FSS(0xCF);
_FGS(0x07);
_FOSCSEL(0xA2);  //Select HS without PLL
_FOSC(0x46);  // HS oscilator, OSC2 is clock output
_FWDT(0xDF);
_FPOR(0xE7);
//_FICD(0xC3);
#define CLRWDT {__asm__ volatile ("CLRWDT");}

#define COMM LATCbits.LATC14
#define WORK LATCbits.LATC13
#define STAT LATDbits.LATD0
#define FAIL LATDbits.LATD11
#define Nrest LATFbits.LATF6

typedef union
{ 
   struct 
   {
        unsigned long AD_value;
   };
   struct   
   {
        unsigned char AD_1;
        unsigned char AD_2;
        unsigned char AD_3;
        unsigned char AD_4;
   };
   struct   
   {
        unsigned int AD_11;
        unsigned int AD_12;
   };
} AD_type;

AD_type AD_buffer[4][4];
AD_type AD_value[4][4];

unsigned char flag_data=0,flag1=0;
unsigned char work_enable=0;
unsigned char capture_enable=0;
unsigned char MAIN_ID=17; 

unsigned char send_data[200]={1,2,3,4,5,6,7,8,9,10};
char send_ascii[250];
char flag_ascii_or_bin = 'b';

volatile unsigned int Tick_1S=0;

unsigned char uart2_enable = 0;//����ʹ��λ
unsigned char uart1_enable = 0;//����ʹ��λ

void UART2_Send(unsigned char str[], int len)
{
	int i;
	for(i=0;i<len;i++)
	{
		U2TXREG = str[i];
		while((U2STA&0x0100)==0x0000){asm("nop");}
	}
}

void UART1_Send(unsigned char str[], int len)
{
	int i;
	for(i=0;i<len;i++)
	{
		U1TXREG = str[i];
		while((U1STA&0x0100)==0x0000){asm("nop");}
	}
}

unsigned int Tick_sys = 0;
void __attribute__((interrupt,no_auto_psv)) _T6Interrupt(void)  // 1ms interrupt
{
	IFS2bits.T6IF = 0;
	Tick_sys++;	

	if(Tick_sys >= 1000)
	{
		Tick_sys = 0;
		FAIL = ~FAIL;
		WORK = FAIL;
	    Tick_1S ++;
	}

    if(Tick_sys % 123 == 1)
    {
		capture_enable = 1;
	}

	if(Tick_1S >= 1920) //32min
	{
		Tick_1S = 0;
		Nrest=1;
			IEC1bits.U2RXIE = 0; // Enable UART2 RX interrupt
            IEC1bits.U2TXIE = 0;
			IEC0bits.U1RXIE = 0; //  Enable UART1 RX interrupt
			IEC0bits.U1TXIE = 0;
			IEC2bits.T6IE = 0;
			IEC2bits.C1IE=0;
			C1INTEbits.RBIE=0;
			while(1); //�������ù�������
	}

	else if(Tick_1S == 1918)
	{
		Nrest=0;
	}

	else if(Tick_1S == 1860)
	{
		InitSCI();
		work_enable = 1;
		uart1_enable = 1;
		uart2_enable = 1;
	}
 		
}

/******
�����ж�
**********/

void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;
	unsigned char data[17];
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	data[0] = U1RXREG;
			 
	while(data[i]!='E')
	{
		i++;
		while((0==(U1STA&0x0001))&&(UART_Timeout<50000))
		{UART_Timeout++;}
		if(UART_Timeout>=50000)
		{
			data[i]='E'; // ������ճ�ʱ���˳��ж�
		}
		else
		{
			data[i] = U1RXREG;
		}
		UART_Timeout = 0;
	}
	
	if( (i==16)&&(data[2]==0X03)&&(data[3]==0X04)&&(data[0]=='S') )
	{	
		    flag_ascii_or_bin = 'b';
			work_enable = 1;//����ָ���е�ID�����һ��ʱ���ŻῪ���òɼ��伤ƵʰƵ
			uart1_enable =1;		
	}//if(i==16)
	return;	
}//





/******
�ⲿ�����ж�


**********/
int start_judge = 0;

void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
	unsigned int UART_Timeout = 0;
	unsigned char i = 0;
	unsigned char dat;
	unsigned char receive_buf[60];
	
	IFS1bits.U2RXIF = 0;
	dat = U2RXREG;
	
	if((start_judge == 0) && (dat == 'S'))
    {
		start_judge = 1;
		receive_buf[0] = dat;
    }
	else if((start_judge == 0) && (dat == 'C'))
    {
		start_judge = 3;
    }
    else if(start_judge == 1)
	{
		while(receive_buf[i]!='E')
		{
			if(i == 0)
			{
				i = 1;
				receive_buf[i] = dat;
			}	
			i++;
			while((0==(U2STA&0x0001))&&(UART_Timeout<50000))
			{UART_Timeout++;}
			if(UART_Timeout>=50000)
			{
				receive_buf[i]='E'; // ������ճ�ʱ���˳��ж�
			}
			else
			{
				receive_buf[i] = U2RXREG;
			}
			UART_Timeout = 0;
		}	
		start_judge = 0; 
		if((i==16)&&(receive_buf[2]==0X01)&&(receive_buf[3]==0X02)) 
		{	
			work_enable = 1;//����ָ���е�ID�����һ��ʱ���ŻῪ���òɼ��伤ƵʰƵ
			uart2_enable =1;
			flag_ascii_or_bin = 'b';	
		}
	}
	
	else if((start_judge == 0) && (dat == '5'))
    {
		start_judge = 2;
		receive_buf[0] = dat;
    }
    else if(start_judge == 2)
	{
		while((receive_buf[i] != '5') || (i == 0))
		{
			if(i == 0)
			{
				i = 1;
				receive_buf[i] = dat;
			}
			i++;
			while((0==(U2STA&0x0001))&&(UART_Timeout<50000))
			{UART_Timeout++;}
			if(UART_Timeout>=50000)
			{
				receive_buf[i]='5'; // ������ճ�ʱ���˳��ж�
			}
			else
			{
				receive_buf[i] = U2RXREG;
			}
			UART_Timeout = 0;
		}	
		start_judge = 0; 
		if((i==49)&&(receive_buf[7]=='1')&&(receive_buf[10]=='2')) 
		{	
			work_enable = 1;//����ָ���е�ID�����һ��ʱ���ŻῪ���òɼ��伤ƵʰƵ
			uart2_enable =1;
			flag_ascii_or_bin = 'a';	
		}
	}
	else if(start_judge == 3)
	{
		start_judge = 0;
		if(dat == 'Q')
		{
			IEC1bits.U2RXIE = 0; // Enable UART2 RX interrupt
            IEC1bits.U2TXIE = 0;
			IEC0bits.U1RXIE = 0; //  Enable UART1 RX interrupt
			IEC0bits.U1TXIE = 0;
			IEC2bits.T6IE = 0;
			IEC2bits.C1IE=0;
			C1INTEbits.RBIE=0;
			while(1); //�������ù�������
		}
	}	
	return;	
}

int main()
{
	int i;
	int k,j;
	int n,m,p;
	int s=0;
	int temp1;
    unsigned int humi_val_i,temp_val_i;
	unsigned char error,checksum;
    unsigned char read_temp;
    unsigned char adc_temp[3];
    unsigned char test[7] = {0x55,0x55,0x55,'t','e','s','t'};
	unsigned int wait;
	unsigned int flag_break = 0;

    CLRWDT
	OSCCON = 0x2200;

    SPI_Init();
    DELAY(5000);
    AD7176set();
	InitTimer6();  //// Timer6 �ṩ0.1s�ж϶�ʱ
    StartTimer6();
	InitSCI();
    s_connectionreset();
     
    TRISCbits.TRISC13 = 0;
    TRISDbits.TRISD0 = 0;
    TRISFbits.TRISF6 = 0; 
	TRISCbits.TRISC14 = 0; 
	TRISDbits.TRISD11 = 0;

    AD1PCFGLbits.PCFG7 = 1;
    AD2PCFGLbits.PCFG7 = 1;

    Nrest=1;
    WORK = 1;
	FAIL = 1;
    STAT = 0;
	COMM = 0;

	CLRWDT
 
    //UART2_Send(test,7);
	
	while(1)
	{
		CLRWDT
		
		if((U1STA & 0x000E) != 0x0000)
		{
			read_temp = U1RXREG;
			U1STAbits.OERR = 0;
        }
		
		if((U2STA & 0x000E) != 0x0000)
		{
			read_temp = U2RXREG;
			U2STAbits.OERR = 0;
        }
		
		if(capture_enable)
		{
			for(p = 0;p < 4;p ++)
			{
				for(i = 0;i < 4;i ++)
				{
					cs_low(i);
			        DELAY(100); 
					read_write_byte(0x10 + p);
					read_write_byte(0x80);
					read_write_byte(0x20 * p + 0x16);
					cs_high(i);
					DELAY(100); 
				}
				DELAY(1000);

				for(i = 0;i < 4;i ++)
				{
					cs_low(i);
	                DELAY(100);

					read_write_byte(0x01);
					read_write_byte(0x01);   
					read_write_byte(0x18);

					DELAY(9000);

					read_write_byte(0x44);

					adc_temp[2]=read_write_byte(0x00); 
					adc_temp[1]=read_write_byte(0x00);
					adc_temp[0]=read_write_byte(0x00);
                    flag_data=read_write_byte(0x00);

					if(!(flag_data&0x80)) //������ת���ɹ�
					{
						flag_data = flag_data&0x03;////ͨ��0 ����ͨ���� 
						//if(adc_temp[0] != 0 && adc_temp[1] != 0 && adc_temp[2] != 0)
						{
							AD_buffer[i][flag_data].AD_3=adc_temp[2]; 
							AD_buffer[i][flag_data].AD_2=adc_temp[1];
							AD_buffer[i][flag_data].AD_1=adc_temp[0];
							AD_buffer[i][flag_data].AD_4=0;
						}	
					}
     	
					cs_high(i);
					DELAY(100); 

					CLRWDT
				}

				for(i = 0;i < 4;i ++)
				{
					cs_low(i);
			        DELAY(100); 
					read_write_byte(0x10 + p);
					read_write_byte(0x00);
					read_write_byte(0x20 * p + 0x16);
					cs_high(i);
					DELAY(100); 
				}
				DELAY(1000);

			}
			capture_enable = 0;
		}

    	if(work_enable)
		{
			error=0;
     		error+=s_measure((unsigned char*) &humi_val_i,&checksum,HUMI); 
     		error+=s_measure((unsigned char*) &temp_val_i,&checksum,TEMP); 

			if(error!=0)
        		s_connectionreset(); 
			else
	    		error=0;

			/*for(i = 0;i < 4;i ++)
			{
				test[i] = check(i);
			}
			UART1_Send(test,4);*/
			
			send_data[0]='S';
			send_data[1]=1;
			send_data[2]=2;	
	
			send_data[3] = 3;
			send_data[4] =4;
			send_data[5] = 5;
			send_data[6] = 6;
			send_data[7] = MAIN_ID;
	        s=8;

			for(k = 0;k < 4;k ++)
			{
				send_data[s] = k;s++;
				for(m = 0;m < 4;m ++)
				{   
					send_data[s] = AD_buffer[k][m].AD_3;s++;
					send_data[s] = AD_buffer[k][m].AD_2;s++;
					send_data[s] = AD_buffer[k][m].AD_1;s++; 
				}
			}
			
			send_data[s] = 0x00;s++;
			send_data[s] = 0x00;s++;

			send_data[s] = humi_val_i>>8;s++;
			send_data[s] = humi_val_i;s++;
			send_data[s] = temp_val_i>>8;s++;
     		send_data[s] = temp_val_i;s++;

			send_data[s]=  'E';s++;

			if(uart2_enable ==1)
			{
				UART2_Send(send_data,s);
			}
			uart2_enable = 0;

        	UART1_Send(send_data,s);	
            
			STAT = ~STAT;
			COMM = ~COMM;
			work_enable = 0;
			Tick_1S = 0;
        }      
    }    
	return 0;
}

