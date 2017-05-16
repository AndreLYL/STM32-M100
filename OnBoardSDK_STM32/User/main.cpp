/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  An exmaple program of DJI-onboard-SDK portable for stm32
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 *******************************************************************************
 *                                                                             *
 *          --------               --------                 --------           *
 *         |        |   USART2    |        |    USART3     |        |          *
 *         |   PC   | <---------> | stm32  |  <----------> |  M100  |          *
 *         |        | (USB-TTL)   |        |               |        |          *
 *         |        |             |        |               |        |          *
 *          --------               --------                 --------           *
 *                                                                             *
 *                                                                             *
 *******************************************************************************
 * */

#include "main.h"


#undef USE_ENCRYPT
/*-----------------------DJI_LIB VARIABLE-----------------------------*/
using namespace DJI::onboardSDK;

HardDriver* driver = new STM32F4;
CoreAPI defaultAPI = CoreAPI(driver);
CoreAPI *coreApi = &defaultAPI;

Flight flight = Flight(coreApi);
FlightData flightData;


VirtualRC virtualrc = VirtualRC(coreApi);
VirtualRCData myVRCdata =
{ 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
    1024, 1024 };

RadioData a={0,0,0,0,0,0};
GimbalData b;
		
extern TerminalCommand myTerminal;
extern LocalNavigationStatus droneState;
extern uint8_t myFreq[16];
		

#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2
//#define PWM3  TIM9->CCR1
#define Changmen  TIM14->CCR1	
void PWM_Configuration(void);
void TIM14_PWM_Init(u32 arr,u32 psc);
		
		

/*
 * @brief Helper function to assemble two bytes into a float number
 */
static float32_t hex2Float(uint8_t HighByte, uint8_t LowByte)
{
  float32_t high = (float32_t) (HighByte & 0x7f);
  float32_t low  = (float32_t) LowByte;
  if (HighByte & 0x80)//MSB is 1 means a negative number
  {
    return -(high*256.0f + low)/100.0f;
  }
  else
  {
    return (high*256.0f + low)/100.0f;
  }
}




int main()
{
  BSPinit();
  delay_nms(30);
	PWM_Configuration();
	PWM1=1000;
	TIM14_PWM_Init(2000-1,840-1);
	Changmen=1900;

//  printf("This is the example App to test DJI onboard SDK on STM32F4Discovery Board! \r\n");
//  printf("Refer to \r\n");
//  printf("https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/STM32/README.html \r\n");
//  printf("for supported commands!\r\n");
//  printf("Board Initialization Done!\r\n");
//  delay_nms(1000);

  uint32_t runOnce = 1;
  uint32_t next500MilTick;
  while (1)
  {
    // One time automatic activation
    if (runOnce)
    {
      runOnce = 0;
      coreApi->setBroadcastFreq(myFreq);
      delay_nms(50);

      // The Local navigation example will run in broadcast call back function,
      // immediate after GPS position is updated
      coreApi->setBroadcastCallback(myRecvCallback,(DJI::UserData)(&droneState));

      //! Since OSDK 3.2.1, the new versioning system does not require you to set version.
      //! It automatically sets activation version through a call to getDroneVersion.
      coreApi->getDroneVersion();
      delay_nms(1000);

      User_Activate();      
      delay_nms(50);

      next500MilTick = driver->getTimeStamp() + 500;
			
			
	
    }

//    if (driver->getTimeStamp() >= next500MilTick)
//    {
//      next500MilTick = driver->getTimeStamp() + 500;

//      // Handle user commands from mobile device
//      mobileCommandHandler(coreApi, &flight);

//      // Handle user commands from serial (USART2)
//      myTerminal.terminalCommandHandler(coreApi, &flight);
//    }
		
		
		

		
		

		
		
		a=virtualrc.getRCData();
		b=coreApi->getBroadcastData().gimbal;

//		Changmen=1000;
//		PWM2=1000;
//		PWM3=1000;
		
		if((a.mode==0)&&(a.gear==(-4545)))    //A档和拨码在下面  舱门开
		{
				Changmen=1000;	
		}
		else
			;
		if((a.mode==0)&&(a.gear==(-10000)))     //舱门关
		{
				Changmen=1900;
		}
		else 
			;
		if((a.mode==(-8000))&&(a.gear==(-4545)))			//P档和拨码在下面 摩擦轮转
		{
				PWM1=1300;
		}		
		else
		{
				PWM1=1000;
		}
		if((a.mode==(8000))&&(a.gear==(-4545)))			//P档和拨码在下面 摩擦轮转
		{
				PWM2=1500;
			
//				coreApi->setControl(0x01);							//请求控制权		
//				flight.task(Flight::TASK_TAKEOFF);			//一键起飞
//				flightData.flag = 0x88;									
//				flightData.x = hex2Float(0x00, 0x64);
//				flightData.y = hex2Float(0x00, 0x00);
//				flightData.z = hex2Float(0x00, 0x00);
//				flightData.yaw = hex2Float(0x00, 0x05);
//				flight.setFlight(&flightData);
//				TIM_Cmd(TIM2, ENABLE);			
//				coreApi->setControl(0x00);							//请求控制权	
//////		while(1);
//				delay_nms(1000);
//				delay_nms(1000);
//				delay_nms(1000);
//				flight.task(Flight::TASK_LANDING);

					
		}		
		else
		{
//				coreApi->setControl(0x00);			//释放控制权						
				PWM2=900;
		}
//		QuaternionData q;
//		q=flight.getQuaternion();

//		coreApi->sendPoll();
		
//		delay_nms(1000);
//		delay_nms(1000);
//		delay_nms(1000);

	
  }



}





//舵机PWM控制
void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   //PCLK1=42MHz,TIM5 clk =84MHz
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);   //PCLK1=42MHz,TIM2 clk =84MHz

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);	
    /* TIM5 */
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);
		
    /* TIM9 */
	tim.TIM_Prescaler = 1680-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2000;   //1ms,1KHz
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9,&tim);
		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM5,&oc);
    TIM_OC2Init(TIM5,&oc);
		oc.TIM_Pulse = 0;
		TIM_OC1Init(TIM9,&oc);
    
    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_ARRPreloadConfig(TIM9,ENABLE);
		
    TIM_Cmd(TIM5,ENABLE);
		TIM_Cmd(TIM9,ENABLE);
}


void TIM14_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); //GPIOF9复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure);              //初始化PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM14,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM14, ENABLE);  //使能TIM14
 										  
}  


