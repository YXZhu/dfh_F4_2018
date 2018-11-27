#include "hmc5983.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "flash.h"
#include  <math.h> 

#define WrAdr 0x3C
#define ReAdr 0x3D
#define ModeAdr 0x02
#define CRA 0x00

uint8_t Modedata[] = {ModeAdr,0x00};
uint8_t CRAdata[] = {CRA,0xFC};
uint8_t Adr1[] = {0x03};
uint8_t ReDa[6],SWhmc5983 = 0;
extern int16_t x,y,z,offsetX,offsetY,offsetZ,hmcwritedata[4],hmcreaddata[4];
extern double angle;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern int32_t setSPA,setSPB;
void hmc5983_init(void)
{  
	HAL_I2C_Master_Transmit(&hi2c1,WrAdr,CRAdata,2,1);
	osDelay(1);
	HAL_I2C_Master_Transmit(&hi2c1,WrAdr,Modedata,2,1);
	osDelay(1);
}

void hmc5983_read(void)
{   
	if(SWhmc5983 == 0)
	{
		//osDelay(200);
		
		hmc5983_init();
		SWhmc5983 = 1;
	}
	else
	{
	    if(HAL_I2C_Master_Transmit(&hi2c1,WrAdr,Adr1,1,1) != HAL_OK)
		
		{
			SWhmc5983 = 0;
			HAL_GPIO_TogglePin(I2CERR_GPIO_Port,I2CERR_Pin);
	  	   HAL_I2C_DeInit(&hi2c1);        //释放IO口为GPIO，复位句柄状态标志 
			HAL_I2C_Init(&hi2c1);
         //HAL_I2C_Init(&hi2c1);          //这句重新初始化I2C控制器
		}
		else
		{
		 osDelay(1);
	    HAL_I2C_Master_Receive(&hi2c1,ReAdr,ReDa,6,1);
		 osDelay(1);
	    x = (ReDa[0] << 8 | ReDa[1])-offsetX; //Combine MSB and LSB of X Data output register
       z = (ReDa[2] << 8 | ReDa[3])-offsetZ; //Combine MSB and LSB of Z Data output register
       y = (ReDa[4] << 8 | ReDa[5])-offsetY; //Combine MSB and LSB of Y Data output register
		 //if(ReDa[1]==0&ReDa[2]==0&ReDa[5]==0&ReDa[4]==0) SWhmc5983 = 0;
	    angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180;
	    osDelay(4);
		}
   }
}

void hmc5983task(void const * argument)
{
	//uint8_t JL1[] = {"zcdsfsf\n\r"};
	for(;;)
	{
		hmc5983_read();
		osDelay(1);
		//HAL_UART_Transmit(&huart2,JL1,9,0xffff);
	}
}

void hmc5983jzinit(void)
{
    //int16_t hmcwritedata[4],hmcreaddata[4];
	int xMax, xMin, yMax, yMin, zMax, zMin; 
	//flashread1(hmcreaddata,4);
		flashinit();
		HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
		setSPA = 3000;
		setSPB = 3000;
		for(unsigned int i =0;i<2000;i++)
		{
			hmc5983_read();
			if(x>xMax) xMax = x*0.2 + xMax*0.8;  			 
			if(x<xMin) xMin = x*0.2 + xMin*0.8; 					  
			if(y>yMax) yMax = y*0.2 + yMax*0.8;  
         if(y<yMin) yMin = y*0.2 + yMin*0.8;        
         if(z>zMax) zMax = z*0.2 + zMax*0.8; 
         if(z<zMin) zMin = z*0.2 + zMin*0.8;
			
		}
		HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
      offsetX = (xMax + xMin)/2;  
      offsetY = (yMax + yMin)/2;  
      offsetZ = (zMax +zMin)/2;
		hmcwritedata[0] = 0x3c;
		hmcwritedata[1] = offsetX;
		hmcwritedata[2] = offsetY;
		hmcwritedata[3] = offsetZ;
		flashwrite1(hmcwritedata,4);
//	else
//	{
//		offsetX = hmcreaddata[1];
//		offsetY = hmcreaddata[2];
//		offsetZ = hmcreaddata[3];
//	}
}		
		
