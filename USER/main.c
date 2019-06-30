#include "stm32f10x.h"
#include "stdio.h"	
#include "sys.h"
#define ch1 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define ch2 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define ch3 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)
#define ch4 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)
#define ch5 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)
#define ch6 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)
#define ch7 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)
void uart_init(u32);
void all_init(void); 
void sendB(unsigned char);
void TIM3_Int_Init(u16 arr,u16 psc);
//created by hanyang huang
//created time: 2019/06/30 21:18
typedef enum {
    false=0,
    true
} bool;
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
unsigned char ch_num[7];	//�������ͨ����⵽�ߵ�ƽ����
unsigned char ch_mk[7];	// ����ͨ����������Ϣ����ΪҪ���͵�����
bool flag_ready = false;	//����׼���ñ�־
unsigned char step = 0;	// 0-��ǰ���ڲɼ�����ͨ���źš�
			//1-����ͨ���ź��Ѿ��ɼ���ɣ����ڵȴ��µ����ڵ�����



 int main(void)
 {	
	 bool first_frame = true;	//��һ֡��־
	all_init();
	
	TIM3_Int_Init(1339,0);//72Mhz�ļ���Ƶ�ʣ�������1339Ϊ20us  
	
	while(1)
    {
	if(flag_ready)
	{
		if(first_frame)
	    {
		first_frame = false;
		flag_ready = false;
	    }
	    else
	    {
		unsigned char i;
		//�����������Ϣת��������ֵ��������
		sendB(0x01);	//һ֡������ʼ��־
		for(i=0;i<7;i++)
		{
		    ch_mk[i] = ch_num[i] * 2;
		    sendB(ch_mk[i]);
		}
		flag_ready = false;
	    }
	}
	 
	 
	 
 }}
void uart_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}
void all_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	uart_init(9600);	 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);//ʹ��PORTA,PORTEʱ��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_4;//GPIOB0-6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //set as floating input
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
}
void sendB(unsigned char dat)
{
    printf("%d/n",dat);
    while(!USART_GetITStatus(USART1,USART_IT_TC));//Sending Interrupt flag
    USART_ClearFlag(USART1, USART_FLAG_TC);
}
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
}
//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{		static unsigned char over_num = 0;	//����������⵽����ͨ�����ǵ͵�ƽ�Ĵ���
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
		}

		   switch(step)
    {
    case 0:	//�ɼ��źţ�������һ��˵���Ѿ�����������һ·Ϊ�ߵ�ƽ
	//����ͨ������������ͨ���ĸߵ�ƽ�����γ��ֵģ�����ͬʱ����
	if(ch1)		{ch_num[0]++; over_num = 0;}
	else if(ch2)	{ch_num[1]++; over_num = 0;}
	else if(ch3)	{ch_num[2]++; over_num = 0;}
	else if(ch4)	{ch_num[3]++; over_num = 0;}
	else if(ch5)	{ch_num[4]++; over_num = 0;}
	else if(ch6)	{ch_num[5]++; over_num = 0;}
	else if(ch7)	{ch_num[6]++; over_num = 0;}
	else	//����ͨ������0���ܿ���������ͨ���źŽ���
	{
	    over_num ++;
	    if(over_num == 100)	//�����������ͨ�����ǵ͵�ƽ��ȷ�ϱ������źŽ���
				/*�����������Ϊ��ʶ�������źŽ�����һ�������ڽ���ǰ���нϳ�ʱ��ȫ��ͨ����Ϊ�͵�ƽ����WFR07����
				  �еĽ��ջ��ڸ���ͨ���ź������϶Ҳ����ֽϳ�ʱ�������ͨ���͵�ƽ���,��ʱ��Ҫ�Ӵ�����֣�
				  �Ѽ�϶֮��ĵ͵�ƽ�����ڽ���ǰ�ĵ͵�ƽ���ֿ�������WFR07���ջ���Ҫ��2ms���֡�*/
	    {
		unsigned char i;
		for(i=0; i<7; i++)	//������ͨ���Ƿ���С��500us�ĸߵ�ƽ����
		    if(ch_num[i] < 25)
			break;
		if(i == 7)	//����ͨ���źŶ���Ч
		    flag_ready = true;	//֪ͨ�����̷������ݣ��˱���������������
		step = 1;	//����ȴ��׶�
	    }
	}
	break;
    case 1:	//�ɼ���ɣ��ȴ��¸����ڵ���
	if(ch1 | ch2 | ch3 | ch4 | ch5 | ch6 | ch7)	//�Ѿ���⵽��ͨ��Ϊ�ߵ�ƽ���µ��źŵ���
	{
	    unsigned char i;
	    step = 0;
	    over_num = 0;
	    for(i=0;i<7;i++)
		ch_num[i]=0;
	    if(ch1) ch_num[0]++;
	    else if(ch2) ch_num[1]++;
	    else if(ch3) ch_num[2]++;
	    else if(ch4) ch_num[3]++;
	    else if(ch5) ch_num[4]++;
	    else if(ch6) ch_num[5]++;
	    else if(ch7) ch_num[6]++;
	}
	break;
    }
}
