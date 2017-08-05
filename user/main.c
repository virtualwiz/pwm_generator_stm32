#include "stm32f10x.h"
#include "misc.h"
#include "stdarg.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"


#define EVENT_FP 0
#define EVENT_FN 1
#define EVENT_DP 2
#define EVENT_DN 3

uint8_t TxBuffer1[] = "USART Interrupt Example";  
uint8_t RxBuffer1[],rec_f,tx_flag;
__IO uint8_t TxCounter1 = 0x00;
__IO uint8_t RxCounter1 = 0x00; 
uint32_t Rec_Len;

int PWM2_Freq = 10;    //Hz
int PWM2_DutyPrecentage = 50;
bool SettingChanged_Flag = 0;

void RCC_Configuration(void);
void Delay(__IO uint32_t nCount); 

char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */


void IO_Init()
{
	RCC_Configuration();   
	RCC ->APB2ENR = 0x00004044;  //GPIOA,E,USART1
	GPIOA->CRH = 0x444444b4;
}

void Timer_Init(u16 arr,u16 psc)
{    
	RCC->APB2ENR|= 1<<2;			//GPIOA时钟使能
	RCC->APB1ENR |=1<<1;      //TIM3时钟使能     
	GPIOA->CRL&=0X0FFFFFFF;//PA7输出
	GPIOA->CRL|=0XB0000000;//复用功能输出 PWM模式 
		
	
	TIM3->ARR=arr;  //设定计数器自动重装值 
	TIM3->PSC=psc;  //预分频器
	
	TIM3->CCMR1|=7<<12;  //CH2 PWM2模式（高电平为占空比）  
	TIM3->CCMR1|=1<<11; //CH2预装载使能    
	TIM3->CCER|=1<<4;   //OC2 输出使能    
	TIM3->CR1=0x8000;   //ARPE使能 
	TIM3->CR1|=0x01;    //开定时器3    
}       

void Timer_Config(bool mode,int freq,int duty)
{
	switch(mode){
		case 0:
			TIM3->CNT = 0;
			TIM3->ARR = 100000 / freq;
			TIM3->CCR2 = (1000 / freq) * duty;
		break;
		case 1:
			TIM3->CNT = 0;
			TIM3->ARR = 10000 / freq;
			TIM3->CCR2 = (100 / freq) * duty;
		break;
	}
}
void OnChip_SerialPort_Init(USART_TypeDef* USARTx)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 115200;						//速率115200bps
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
  USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

  /* Configure USART1 */
  USART_Init(USARTx, &USART_InitStructure);							//配置串口参数函数
   
  /* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                    //使能接收中断
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);						//使能发送缓冲空中断   

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);	
}

void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 
	const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //判断是否到达字符串结束符
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);	   

					Data++;
					break;
				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //字符串
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //十进制
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}

void KeyEvent_Handler(u8 input)
{
	switch(input)
	{
		case EVENT_FP:PWM2_Freq += 10;SettingChanged_Flag = 1;
			Delay(700000);break;
		case EVENT_FN:PWM2_Freq -= 10;SettingChanged_Flag = 1;
			Delay(700000);break;
		case EVENT_DP:PWM2_DutyPrecentage += 1;SettingChanged_Flag = 1;
			Delay(400000);break;
		case EVENT_DN:PWM2_DutyPrecentage -= 1;SettingChanged_Flag = 1;
			Delay(400000);break;
		default:break;
	}	
	if(PWM2_Freq>=1000)PWM2_Freq=1000;
	if(PWM2_Freq<=0){
		PWM2_Freq=1;
		Timer_Init(1000,7199);
	}
	if(PWM2_Freq == 11){
		PWM2_Freq=10;
		Timer_Init(1000,719);
	}
	if(PWM2_DutyPrecentage>=100)PWM2_DutyPrecentage=100;
	if(PWM2_DutyPrecentage<=0)PWM2_DutyPrecentage=0;
}

void Hardware_GPU_ShowStatus_Internal(int frequency,int duty)
{
	char frequency_String[5];
	char duty_String[5];
	itoa(frequency,frequency_String,10);
	itoa(duty,duty_String,10);
	USART_OUT(USART1,"输出频率: ");
	USART_OUT(USART1,frequency_String);
	USART_OUT(USART1," 赫兹 , 占空比: 百分之");
	USART_OUT(USART1,duty_String);
	USART_OUT(USART1,"           \n");
}

int main(void)
{
	u16 KeyInputBuffer;
	IO_Init();
	OnChip_SerialPort_Init(USART1);
	Timer_Init(1000,719);
	Timer_Config(0,PWM2_Freq,PWM2_DutyPrecentage);
	Delay(2000000);
	USART_OUT(USART1," 武汉工程大学智能车团队 PWM信号发生器 (2017/01/06)\r\n\r\n"); 
  Hardware_GPU_ShowStatus_Internal(PWM2_Freq,PWM2_DutyPrecentage);
	while(1){
		KeyInputBuffer = GPIOE->IDR;
		if(SettingChanged_Flag)
		{
			if(PWM2_Freq == 1)
				Timer_Config(1,PWM2_Freq,PWM2_DutyPrecentage);
			else
				Timer_Config(0,PWM2_Freq,PWM2_DutyPrecentage);
			SettingChanged_Flag = 0;
			Hardware_GPU_ShowStatus_Internal(PWM2_Freq,PWM2_DutyPrecentage);
		}
		switch(KeyInputBuffer)
		{
			case 0x0000FFFB:KeyEvent_Handler(EVENT_FP);break;
			case 0x0000FFF7:KeyEvent_Handler(EVENT_FN);break;
			case 0x0000FFDF:KeyEvent_Handler(EVENT_DP);break;
			case 0x0000FFEF:KeyEvent_Handler(EVENT_DN);break;
		}
	}
}


void RCC_Configuration(void)
{   
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}


void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */



void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif



/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
