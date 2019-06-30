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
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
unsigned char ch_num[7];	//保存各个通道检测到高电平次数
unsigned char ch_mk[7];	// 三个通道的脉宽信息，此为要发送的数据
bool flag_ready = false;	//数据准备好标志
unsigned char step = 0;	// 0-当前正在采集各个通道信号。
			//1-所有通道信号已经采集完成，正在等待新的周期到来。



 int main(void)
 {	
	 bool first_frame = true;	//第一帧标志
	all_init();
	
	TIM3_Int_Init(1339,0);//72Mhz的计数频率，计数到1339为20us  
	
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
		//把脉宽个数信息转换成脉宽值，并发送
		sendB(0x01);	//一帧数据起始标志
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
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}
void all_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	uart_init(9600);	 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);//使能PORTA,PORTE时钟
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_4;//GPIOB0-6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //set as floating input
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{		static unsigned char over_num = 0;	//保存连续检测到所有通道都是低电平的次数
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
		}

		   switch(step)
    {
    case 0:	//采集信号，进入这一步说明已经出现至少有一路为高电平
	//各个通道计数，各个通道的高电平是依次出现的，不会同时出现
	if(ch1)		{ch_num[0]++; over_num = 0;}
	else if(ch2)	{ch_num[1]++; over_num = 0;}
	else if(ch3)	{ch_num[2]++; over_num = 0;}
	else if(ch4)	{ch_num[3]++; over_num = 0;}
	else if(ch5)	{ch_num[4]++; over_num = 0;}
	else if(ch6)	{ch_num[5]++; over_num = 0;}
	else if(ch7)	{ch_num[6]++; over_num = 0;}
	else	//三个通道都是0，很可能是所有通道信号结束
	{
	    over_num ++;
	    if(over_num == 100)	//连续多次所有通道都是低电平，确认本周期信号结束
				/*这里的数字是为了识别本周期信号结束，一般在周期结束前会有较长时间全部通道都为低电平（如WFR07）。
				  有的接收机在各个通道信号输出间隙也会出现较长时间的所有通道低电平情况,此时需要加大此数字，
				  把间隙之间的低电平和周期结束前的低电平区分开，对于WFR07接收机需要用2ms区分。*/
	    {
		unsigned char i;
		for(i=0; i<7; i++)	//检查各个通道是否有小于500us的高电平脉宽
		    if(ch_num[i] < 25)
			break;
		if(i == 7)	//所有通道信号都有效
		    flag_ready = true;	//通知主进程发送数据，此变量由主进程清零
		step = 1;	//进入等待阶段
	    }
	}
	break;
    case 1:	//采集完成，等待下个周期到来
	if(ch1 | ch2 | ch3 | ch4 | ch5 | ch6 | ch7)	//已经检测到有通道为高电平，新的信号到来
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
