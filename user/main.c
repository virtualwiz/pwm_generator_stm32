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
	RCC->APB2ENR|= 1<<2;			//GPIOAʱ��ʹ��
	RCC->APB1ENR |=1<<1;      //TIM3ʱ��ʹ��     
	GPIOA->CRL&=0X0FFFFFFF;//PA7���
	GPIOA->CRL|=0XB0000000;//���ù������ PWMģʽ 
		
	
	TIM3->ARR=arr;  //�趨�������Զ���װֵ 
	TIM3->PSC=psc;  //Ԥ��Ƶ��
	
	TIM3->CCMR1|=7<<12;  //CH2 PWM2ģʽ���ߵ�ƽΪռ�ձȣ�  
	TIM3->CCMR1|=1<<11; //CH2Ԥװ��ʹ��    
	TIM3->CCER|=1<<4;   //OC2 ���ʹ��    
	TIM3->CR1=0x8000;   //ARPEʹ�� 
	TIM3->CR1|=0x01;    //����ʱ��3    
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
	
	USART_InitStructure.USART_BaudRate = 115200;						//����115200bps
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ8λ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			//ֹͣλ1λ
  USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

  /* Configure USART1 */
  USART_Init(USARTx, &USART_InitStructure);							//���ô��ڲ�������
   
  /* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                    //ʹ�ܽ����ж�
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);						//ʹ�ܷ��ͻ�����ж�   

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);	
}

void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 
	const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //�ж��Ƿ񵽴��ַ���������
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);	   

					Data++;
					break;
				case 'n':							          //���з�
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
				case 's':										  //�ַ���
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //ʮ����
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
	USART_OUT(USART1,"���Ƶ��: ");
	USART_OUT(USART1,frequency_String);
	USART_OUT(USART1," ���� , ռ�ձ�: �ٷ�֮");
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
	USART_OUT(USART1," �人���̴�ѧ���ܳ��Ŷ� PWM�źŷ����� (2017/01/06)\r\n\r\n"); 
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
