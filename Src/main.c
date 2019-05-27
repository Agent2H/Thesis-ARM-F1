/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile bool flagTimeout=0;
uint16_t TIM_Period = 399; //This values may need to be recalculated = (timer_tick_freq/PWM_freq)-1, with timer_tick_freq = timer_default_freq / (pre_scaler+1)
uint32_t Pulse_LeftMotor=0, Pulse_RightMotor=0; 	//Variable to count Encoder Left n Right
uint32_t NotReset_PulseLeft =0, NotReset_PulseRight=0;
float Speed_LeftMotor=0, Speed_RightMotor=0; 	//Variable from 0-100 to control duty cycle
float Ke=3.5,Kde=0.5,Kout=50;													//Parameter
float CurrentX=0, CurrentY=0, CurrentPhi=0, PrevPhi=0;//Current position of the vehicle
float PrevError=0, CurrErr=0;
float dt=0.1f; 																		//Control Period
float KhoangCach2Banh = 0.6f; //Khoang cach giua 2 banh xe
uint8_t tim2Indicator=0; //Check if timer2 works
float distance;
uint8_t receive_data_1[8];
uint8_t send_data[8];
float firstFloat = 1.27f, secondFloat = 0.73f;
uint8_t Tx[10]; //UART Transmit buffer
float SendX,SendY;

float EuclidThresh=0.3f;

//Variable for going to different Destination
float DestXList[]={	10.0f,	10.0f	,0.0f	,0.0f,	10.0f	,10.0f	,0.0f};
float DestYList[]={	0.0f	,	4.0f	,4.0f	,8.0f,	8.0f	,12.0f	,12.0f};
int countDestList=0;
float DestX,DestY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
//Ham truyen SPI
void Send_2Float(){
	unsigned char *chptr;
	
	chptr=(unsigned char *)&firstFloat;
	send_data[0]=(*chptr++); 	//LSB
	send_data[1]=(*chptr++);
	send_data[2]=(*chptr++);
	send_data[3]=(*chptr);		//MSB
	
	chptr=(unsigned char *)&secondFloat;
	send_data[4]=(*chptr++);	//LSB
	send_data[5]=(*chptr++);
	send_data[6]=(*chptr++);
	send_data[7]=(*chptr);		//MSB
	
	HAL_SPI_Transmit_IT(&hspi1,&send_data[0],8);
}
//Ham truyen UART
void SendPos(){
	/*Send total 10Bytes include 2 end bytes 0x0D & 0x0A
	Send CurrentX,CurrentY in bytes
	*/
	unsigned char *chptr;
	SendX = (float)CurrentX;
	chptr = (unsigned char *)&SendX;

	Tx[0]=(*chptr++);
	Tx[1]=(*chptr++);
	Tx[2]=(*chptr++);
	Tx[3]=(*chptr);

	SendY =(float) CurrentY;
	chptr = (unsigned char *)&SendY;

	Tx[4]=(*chptr++);
	Tx[5]=(*chptr++);
	Tx[6]=(*chptr++);
	Tx[7]=(*chptr);

	Tx[8]=0x0D;
	Tx[9]=0x0A;

	HAL_UART_Transmit(&huart2,Tx,10,8000);
}
//Ham tinh toan
float EuclidDistance(float x1,float y1,float x2, float y2){
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
//Ham xuat % dong co
void SetPWM_withDutyCycle_withStop(TIM_HandleTypeDef *htim, uint32_t Channel, int dutyCycle, int STOP_CODE){

	dutyCycle=(dutyCycle > 100 ) ? 100 : dutyCycle;
	dutyCycle=(dutyCycle < 0)? 0: dutyCycle;

	/*This function allow to Write PWM in duty cycle with timer and channel parameters*/
	int32_t pulse_length = TIM_Period*dutyCycle/100;
	__HAL_TIM_SET_COMPARE(htim, Channel, pulse_length*(1-STOP_CODE));

};

void SetPWM_withDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, int dutyCycle){

	dutyCycle=(dutyCycle > 100 ) ? 100 : dutyCycle;
	dutyCycle=(dutyCycle < 0)? 0: dutyCycle;

	/*This function allow to Write PWM in duty cycle with timer and channel parameters*/
	int32_t pulse_length = TIM_Period*dutyCycle/100;
	__HAL_TIM_SET_COMPARE(htim, Channel, pulse_length);
};

//Fuzzy functions
double mftrap(double x,double L,double C1,double C2,double R){
	double y;
	if (x<L)
		y=0;
	else if (x<C1)
		y=(x-L)/(C1-L);
	else if (x<C2)
		y=1;
	else if (x<R)
		y=(R-x)/(R-C2);
	else
		y=0;
	return y;}
int maxArray(double a[],int n) {
  int i, max;
					
  max = a[0];
   
 
  for (i = 1; i <n; i++) {
    if (a[i] > max) { 
       max = a[i];
       }
  
  }
 
  return max;
}
double max(double num1,double num2){
	return (num1 > num2 ) ? num1 : num2;
}
double Defuzzication_R(double e, double de){

	//Return e and de in range [-1 1]
	e=(e>1)?1:e;
	e=(e<-1)?-1:e;
	de=(de>1)?1:de;
	de=(de<-1)?-1:de;

	double e_NE,e_ZE,e_PO,de_NE,de_ZE,de_PO;

	e_NE=mftrap(e,-1.1,-1,-0.1,0);
	e_ZE=mftrap(e,-0.1,0,0,0.1);
	
	e_PO=mftrap(e,0,0.1,1,1.1);

	de_NE=mftrap(de,-1.1,-1,-1,0);
	de_ZE=mftrap(de,-1,0,0,1);
	de_PO=mftrap(de,0,1,1,1.1);

	double	dR_NB=-1;
	double	dR_NS=-0.4;
	double	dR_ZE=0;
	double	dR_PS=0.8;
	double	dR_PB=1;

	double	beta1=e_NE*de_NE;   //R=NB
	double	beta2=e_NE*de_ZE;   //R=NS
	double	beta3=e_NE*de_PO;   //R=ZE
	double	beta4=e_ZE*de_NE;   //R=NS
	double	beta5=e_ZE*de_ZE;   //R=ZE ->PS
	double	beta6=e_ZE*de_PO;   //R=PS
	double	beta7=e_PO*de_NE;   //R=ZE
	double	beta8=e_PO*de_ZE;   //R=PS
	double	beta9=e_PO*de_PO;   //R=PB
	//Elements of each array
  double NSArray[]={beta2,beta4};
	double ZEArray[]={beta3,beta7};
	double PSArray[]={beta6,beta8,beta5};
	
	double beta_NB=beta1;
	double beta_NS=max(beta2,beta4);
	double beta_ZE=max(beta3,beta7);
	double beta_PS=max(beta6,max(beta8,beta5));
	double beta_PB=beta9;


	double sumBeta=beta_NB+beta_NS+beta_ZE+beta_PS+beta_PB;
	double dR=(dR_NB*beta_NB+dR_NS*beta_NS+dR_ZE*beta_ZE+dR_PS*beta_PS+dR_PB*beta_PB)/sumBeta;

	return dR;
}
double Defuzzication_L(double e, double de){
		e=(e>1)?1:e;
	e=(e<-1)?-1:e;
	de=(de>1)?1:de;
	de=(de<-1)?-1:de;

	double e_NE,e_ZE,e_PO,de_NE,de_ZE,de_PO;

	e_NE=mftrap(e,-1.1,-1,-0.1,0);
	e_ZE=mftrap(e,-0.1,0,0,0.1);
	
	e_PO=mftrap(e,0,0.1,1,1.1);

	de_NE=mftrap(de,-1.1,-1,-1,0);
	de_ZE=mftrap(de,-1,0,0,1);
	de_PO=mftrap(de,0,1,1,1.1);

	double	dR_NB=-1;
	double	dR_NS=-0.4;
	double	dR_ZE=0;
	double	dR_PS=0.8;
	double	dR_PB=1;

	double	beta1=e_NE*de_NE;   //R=PB
	double	beta2=e_NE*de_ZE;   //R=PS
	double	beta3=e_NE*de_PO;   //R=ZE
	double	beta4=e_ZE*de_NE;   //R=PS
	double	beta5=e_ZE*de_ZE;   //R=ZE =>PS
	double	beta6=e_ZE*de_PO;   //R=NS
	double	beta7=e_PO*de_NE;   //R=ZE
	double	beta8=e_PO*de_ZE;   //R=NS
	double	beta9=e_PO*de_PO;   //R=NB
	//Elements of each array
  double NSArray[]={beta6,beta8};
	double ZEArray[]={beta3,beta7};
	double PSArray[]={beta2,beta4,beta5};
	
	double beta_NB=beta9;
	double beta_NS=max(beta6,beta8);
	double beta_ZE=max(beta3,beta7);
	double beta_PS=max(beta2,max(beta5,beta4));
	double beta_PB=beta1;


	double sumBeta=beta_NB+beta_NS+beta_ZE+beta_PS+beta_PB;
	double dL=(dR_NB*beta_NB+dR_NS*beta_NS+dR_ZE*beta_ZE+dR_PS*beta_PS+dR_PB*beta_PB)/sumBeta;

	return dL;
}
//Control Function
double GetNewGocLai(float KhoangNhinTruoc,float xDest,float yDest,float xCurr, float yCurr,float phiCurr){
	float l = KhoangNhinTruoc;
	float xg=0,yg=0; //forhead point

	//Case 1: xDest != xCurr, we calculate (xg,yg), which is the forehead point
	if(xDest!=xCurr){
		float m =(yDest-yCurr)/(xDest-xCurr);
		float c =(yCurr*xDest-yDest*xCurr)/(xDest-xCurr);

		float A= 1+m*m;
		float B= -2*xCurr+2*m*c-2*m*yCurr;
		float C= xCurr*xCurr+(c-yCurr)*(c-yCurr)-l*l;

		if(xDest < xCurr){
			xg = (-B-sqrt(B*B-4*A*C))/(2*A);
			yg = m*xg +c;
		}else{
			xg = (-B+sqrt(B*B-4*A*C))/(2*A);
			yg = m*xg +c;
		}
	}

	//Case 2: xDest = xCurr, we calculate (xg,yg), which is the forehead point
	if(xDest == xCurr){
		if(yDest < yCurr){
			xg = xDest;
			yg = yCurr-l;
		}else if(yDest > yCurr){
			xg = xDest;
			yg = yCurr+l;
		}
	}

	//Now we have (xg,yg), scale it into the vehicle coordinate
	float xG = xg - xCurr;
	float yG = yg - yCurr;

	float x = xG*sin(phiCurr)-yG*cos(phiCurr);
	float y = xG*cos(phiCurr)+yG*sin(phiCurr);

	//Now we have (x,y), finally let's calculate GocLai
	float alpha = atan(x/y);
	return phiCurr-alpha;
}
double GetError(float GocLai,float CurrentPhi){
	return GocLai-CurrentPhi;
}
double GetDerivativeError(float oldErr,float newErr){
	return (newErr-oldErr)/dt;
}





void Fuzzy_ControlMotor(float e, float de){
	float eFuzzy = Ke*e;
	float deFuzzy =Kde*de;

	double aRight = Defuzzication_R(Ke*eFuzzy,Kde*deFuzzy);
	double aLeft 	= Defuzzication_L(Ke*eFuzzy,Kde*deFuzzy);

	Speed_RightMotor+=Kout*aRight*dt;
	Speed_LeftMotor+=Kout*aLeft*dt;

	Speed_LeftMotor=(Speed_LeftMotor>100)?100:Speed_LeftMotor;
	Speed_LeftMotor=(Speed_LeftMotor<0)?0:Speed_LeftMotor;

	Speed_RightMotor=(Speed_RightMotor>100)?100:Speed_RightMotor;
	Speed_RightMotor=(Speed_RightMotor<0)?0:Speed_RightMotor;

	distance=EuclidDistance(CurrentX,CurrentY,DestX,DestY);

	//STOP CODE RUN-Decide wehther to move or not
	int STOPCODE=(distance>EuclidThresh)?0:1;
	SetPWM_withDutyCycle_withStop(&htim4,TIM_CHANNEL_2,(int)Speed_LeftMotor,STOPCODE); //TIM_CHANNEL_2 = PB7 = Left Motor
	SetPWM_withDutyCycle_withStop(&htim4,TIM_CHANNEL_3,(int)Speed_RightMotor,STOPCODE); //TIM_CHANNEL_3 = PB8 = Right Motor

	//WITHOUT STOP CODE
//	SetPWM_withDutyCycle(&htim4,TIM_CHANNEL_2,(int)Speed_LeftMotor); //TIM_CHANNEL_2 = PB7 = Left Motor
//	SetPWM_withDutyCycle(&htim4,TIM_CHANNEL_3,(int)Speed_RightMotor); //TIM_CHANNEL_3 = PB8 = Right Motor
}
float Encoder2Distance(int pulse){
	// 1 vong dong co = 350 xung
	// ban kinh truc dong co R = 7 mm
	return (float)pulse*2*3.14f*7/(350*1000);
}
void CalculateNewPosition(float RightDistance, float LeftDistance){

	CurrentX+=(float)((RightDistance+LeftDistance)*cos(PrevPhi)/2);
	CurrentY+=(float)((RightDistance+LeftDistance)*sin(PrevPhi)/2);
	CurrentPhi+=(RightDistance-LeftDistance)/KhoangCach2Banh;

	//Reset Encoder
	Pulse_LeftMotor =0;
	Pulse_RightMotor=0;
}
void UpdateParams(float Error,float Phi){
	PrevError=Error;
	PrevPhi=Phi;
}



//Turn Around Control Functions
void SpinRight(){
	/*Function for right spin*/
	SetPWM_withDutyCycle(&htim4,TIM_CHANNEL_2,50);//TIM_CHANNEL_2 = PB7 = Left Motor
	SetPWM_withDutyCycle(&htim4,TIM_CHANNEL_3,20);//TIM_CHANNEL_3 = PB8 = Right Motor
}
void SpinLeft(){
	/*Function for left spin*/
	SetPWM_withDutyCycle(&htim4,TIM_CHANNEL_2,20);//TIM_CHANNEL_2 = PB7 = Left Motor
	SetPWM_withDutyCycle(&htim4,TIM_CHANNEL_3,50);//TIM_CHANNEL_3 = PB8 = Right Motor
}

void Fuzzy_TurnAround(float xDest,float yDest,float xCurr, float yCurr,float phiCurr,float AcceptAngle){
	/*Fuzzy control and turn around when angle became too big*/
	float xG = xDest - xCurr;
	float yG = yDest - yCurr;

	float x = xG*sin(phiCurr)-yG*cos(phiCurr);
	float y = xG*cos(phiCurr)+yG*sin(phiCurr);
	
	//Decide to spin or use Fuzzy Control
	if ((x>0)&&((y-tan(3.14/2-AcceptAngle))<0))
		SpinRight();
	else if ((x<0)&&((y+tan(3.14/2-AcceptAngle))<0))
		SpinLeft();
	else {
		//GetNewGocLai
		/*
		Params from DestX,DestY, CurrentX, Current Y, CurrentPhi
		return GocLai
		*/
		float gocLai=GetNewGocLai(0.6f, DestX,DestY,CurrentX,CurrentY,CurrentPhi);
		
		//Get current Error
		/*
		Params from GocLai and CurrentPhi
		Return Error
		*/
	  CurrErr = GetError(gocLai,CurrentPhi);
		
		//Get current Deriative Error
		/*
		Params from the new Error, and the PrevErr
		Return Derivative Error
		*/
		float dCurrErr = GetDerivativeError(PrevError,CurrErr);
		
		//Control Motors Fuzzy
		/*
		Params from the new Error and the new Derivative Error
		Control Motor Left n Right
		Reset Encoder Counters
		After this, Encoder will continued to be counted for the next dt(s) (in this case, dt = 0.1)
		*/
		Fuzzy_ControlMotor(CurrErr,dCurrErr);  
	}	
}

void PP_TurnAround(float KhoangNhinTruoc,float xDest,float yDest,float xCurr, float yCurr,float phiCurr,int vSumPP,float AcceptAngle){
	float l = KhoangNhinTruoc;
	float L=0.6; //Distance between 2 wheels
	float xg=0,yg=0; //forhead point

	//Case 1: xDest != xCurr, we calculate (xg,yg), which is the forehead point
	if(xDest!=xCurr){
		float m =(yDest-yCurr)/(xDest-xCurr);
		float c =(yCurr*xDest-yDest*xCurr)/(xDest-xCurr);

		float A= 1+m*m;
		float B= -2*xCurr+2*m*c-2*m*yCurr;
		float C= xCurr*xCurr+(c-yCurr)*(c-yCurr)-l*l;

		if(xDest < xCurr){
			xg = (-B-sqrt(B*B-4*A*C))/(2*A);
			yg = m*xg +c;
		}else{
			xg = (-B+sqrt(B*B-4*A*C))/(2*A);
			yg = m*xg +c;
		}
	}

	//Case 2: xDest = xCurr, we calculate (xg,yg), which is the forehead point
	if(xDest == xCurr){
		if(yDest < yCurr){
			xg = xDest;
			yg = yCurr-l;
		}else if(yDest > yCurr){
			xg = xDest;
			yg = yCurr+l;
		}
	}

	//Now we have (xg,yg), scale it into the vehicle coordinate
	float xG = xg - xCurr;
	float yG = yg - yCurr;

	float x = xG*sin(phiCurr)-yG*cos(phiCurr);
	float y = xG*cos(phiCurr)+yG*sin(phiCurr);
	
	//Decide to spin or use the PP Control
	if ((x>0)&&((y-tan(3.14/2-AcceptAngle))<0))
		SpinRight();
	else if ((x<0)&&((y+tan(3.14/2-AcceptAngle))<0))
		SpinLeft();
	else {/*Pure Pursuit Control*/
		/*From Theory, calculate vRight/vLeft ratio k*/
		float k=(l*l-L*x)/(l*l+L*x); 
		
		/*Calculate 2 Wheel speed*/
		Speed_LeftMotor=vSumPP/(k+1);
		Speed_RightMotor=vSumPP-Speed_LeftMotor;
		
		/*Decide to move or not*/
		distance=EuclidDistance(CurrentX,CurrentY,DestX,DestY);
		
		int STOPCODE=(distance>EuclidThresh)?0:1;
		SetPWM_withDutyCycle_withStop(&htim4,TIM_CHANNEL_2,(int)Speed_LeftMotor,STOPCODE); //TIM_CHANNEL_2 = PB7 = Left Motor
		SetPWM_withDutyCycle_withStop(&htim4,TIM_CHANNEL_3,(int)Speed_RightMotor,STOPCODE); //TIM_CHANNEL_3 = PB8 = Right Motor		
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	
		//Start PWM
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	
	//Set destination
	DestX=DestXList[0];
	DestY=DestYList[0];
	//Start Timer 2
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_SPI_Receive_DMA(&hspi1,&receive_data_1[0],8);

	//Set GPIOC 13,14 to Vdd to Enable for the Right Driver,GPIOA 10,11 to Vdd to Enable for the Left Driver,GPIOB 9 and GPIOA 1 to GND
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
  
	//set first destination

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

//Timer 2 call back
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==htim2.Instance){
		//Some Indicator
		tim2Indicator ++;
    
		//Encoder2Distance
		/*
		Get
		*/
		float distanceR = Encoder2Distance(Pulse_RightMotor);
		float distanceL = Encoder2Distance(Pulse_LeftMotor);

		//Calculate new Position
		/*
		Params from Left and Right Distance, cope with CurrentX, CurrentY, PrevPhi
		Return CurrenX, CurrentY, CurrentPhi
		*/
		CalculateNewPosition(distanceR,distanceL);
		SendPos();

	//Update Paramss
		/*
		Params from Error and Phi
		update PrevErr, PrevPhi
		*/
		UpdateParams(CurrErr,CurrentPhi);


	//Tinh lai DestX, DestY
		/*
		Params from Current dest and current position
		*/
		distance=EuclidDistance(CurrentX,CurrentY,DestX,DestY);
		
		if (distance<EuclidThresh){
			countDestList++;
			DestX=(countDestList<=6)?DestXList[countDestList]:CurrentX;
			DestY=(countDestList<=6)?DestYList[countDestList]:CurrentY;			
		}			
		
		//GetNewDestImprove(DestX,DestY,CurrentX,CurrentY);
	
	//Control Motors - with turn around ability, accept angle = pi/4 rad = 0.785 rad = 45 degree
  	Fuzzy_TurnAround(DestX,DestY,CurrentX,CurrentY,CurrentPhi,0.785);
		
	 //PP_TurnAround(0.5f,DestX,DestY,CurrentX,CurrentY,CurrentPhi,150,0.52);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
