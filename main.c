#include "stm32l476xx.h"

#include "I2C.h"
#include "LCD.h"
#include "LED.h"
#include "PWM.h"
#include "SysClock.h"
#include "string.h"
#include "UART.h"
#include "EXTI.h"

#include <string.h>
#include <stdio.h>

// Initializes USARTx
// USART2: UART Communication with Termite
// USART1: Bluetooth Communication with Phone
void Init_USARTx(int x) {
	if(x == 1) {
		UART1_Init();
		UART1_GPIO_Init();
		USART_Init(USART1);
	} else if(x == 2) {
		UART2_Init();
		UART2_GPIO_Init();
		USART_Init(USART2);
	} else {
		// Do nothing...
	}
}

void setGreenVal(double value) {
	TIM1->CCR1 = (value/3) * TIM1->ARR;
}

void setBlueVal(double value) {
	TIM8->CCR1 = (value/3) * TIM8->ARR;
}

void setRedVal(double value) {
	TIM2->CCR1 = (value/3) * TIM2->ARR;
}

int main(void) {
	System_Clock_Init(); // System Clock = 80 MHz
	
	LCD_Initialization();
	LCD_Clear();
	LED_Init();
	PWM_Init1();
	PWM_Init2();
	PWM_Init3();
	EXTI_Init();
	
	// Initialize I2C
	I2C_GPIO_Init();
	I2C_Initialization();
	
	int i;
	char message[6];
	uint8_t SlaveAddress;
	uint8_t tempSensorAddress = 0x48;
	uint8_t lightSensorAddress = 0x29;
	uint8_t Data_ReceiveTemp;
	uint8_t Data_ReceiveC;
	uint8_t Data_Receive0;
	uint8_t Data_Receive1;
	uint8_t Data_SendTemp = 0x00;
	uint8_t CommandBit = 0xA0;
	uint8_t EnablePON = 0x01;
	uint8_t EnableAEN = 0x02;
	uint8_t EnableAIEN = 0x10;
	uint8_t EnableSAI = 0x40;
	uint8_t EnableNPIEN = 0x80;
	uint8_t EnableRegister = 0x00;
	uint8_t Data_SendC0L = 0x14;
	uint8_t Data_SendC0H = 0x15;
	uint8_t Data_SendC1L = 0x16;
	uint8_t Data_SendC1H = 0x17;
	uint8_t Data_SendID = 0x12;
	
	Init_USARTx(2);
	
	char rxByte;

	double clear[3] = {0, 0, 0};

	setRedVal(clear[0]);
	setGreenVal(clear[1]);
	setBlueVal(clear[2]);
		
	
	
	
	
	while(1) {
		
		// Determine Slave Address
		//
		// Note the "<< 1" must be present because bit 0 is treated as a don't care in 7-bit addressing mode
		SlaveAddress = tempSensorAddress << 1UL; // STUB - Fill in correct address 
		
		// [TODO] - Get Temperature
		// 
		// First, send a command to the sensor for reading the temperature
		// Next, get the measurement
		
		// [TODO] - Print Temperature to LCD
		
		// First, send a command to the sensor for reading the temperature
		I2C_SendData(I2C1, SlaveAddress, &Data_SendTemp, 1); // send one 0 byte
		
		// Next, get the measurement
		I2C_ReceiveData(I2C1, SlaveAddress, &Data_ReceiveTemp, 1); // read 1 byte
		
		uint8_t tempMeasurement = Data_ReceiveTemp;
		

		if(tempMeasurement & 0x80) {
			// 2S COMPLIMENT CONVERT
			tempMeasurement = ~tempMeasurement;
			tempMeasurement += 0x01;
			tempMeasurement *= -1;
		}
		
		sprintf(message, "%6d", tempMeasurement);

		// Print Temperature to LCD
		LCD_DisplayString(message);

		//printf("Temperature: %6d \n",tempMeasurement);
		

		SlaveAddress = lightSensorAddress << 1UL;
		uint8_t ds = CommandBit|EnableRegister;
		uint8_t data = EnableAIEN | EnablePON | EnableAEN | EnableNPIEN;
		
		uint8_t da[2];
		da[0] = ds;
		da[1] = data;
		//da = da | data;
		// First, send a command to the sensor for reading the temperature
		
		I2C_SendData(I2C1, SlaveAddress, da, 2); // send one 0 byte
		I2C_ReceiveData(I2C1, SlaveAddress, &Data_ReceiveC, 1); // read 1 byte

		uint8_t command = Data_ReceiveC;
		
		
		
		uint8_t c0l = CommandBit|Data_SendC0L;
		uint8_t c0h = CommandBit|Data_SendC0H;
		uint8_t c1l = CommandBit|Data_SendC1L;
		uint8_t c1h = CommandBit|Data_SendC1H;
		uint8_t address = CommandBit|Data_SendID;
		// Next, get the measurement
		I2C_SendData(I2C1, SlaveAddress, &c0l, 1); // send one 0 byte
		I2C_ReceiveData(I2C1, SlaveAddress, &Data_Receive0, 1); // read 1 byte

		
		uint8_t chan0 = Data_Receive0;
		
		// Next, get the measurement
		I2C_SendData(I2C1, SlaveAddress, &c1l, 1); // send one 0 byte
		I2C_ReceiveData(I2C1, SlaveAddress, &Data_Receive1, 1); // read 1 byte
		
		uint8_t chan1 = Data_Receive1;
		
		
		float atime, again;
		float cpl, lux1, lux2, lux;
		atime = 100.0F;
		again = 1.0F;

		float TSL2591_LUX_DF = 408.0F;
		cpl = (atime * again) / TSL2591_LUX_DF;
		lux = (((float)chan0 - (float)chan1)) * (1.0F - ((float)chan1 / (float)chan0)) / cpl;
		
		double brightness = lux/600.0;
		
		printf("Light: %6f \n",brightness);
		
		double mult;
		
		if (brightness > 1) {
			mult = 0;
		} else if ((brightness > 0) && (brightness < 1)){
			mult = 1-brightness;
		} else {
			mult = 1;
		}
	
	
		
		double warmest[3] = {0.5, 0.075, 0.075};
		double white[3] = {0.4, 0.15, 0.15};
		double coolest[3] = {0.2, 0.88627451, 1};
		double diffWarmtoWhite[3] = {0.1, 0.075, 0.075};
		double diffWhitetoCool[3] = {0.2, 0.73627451, 0.85};
		
		double maxTemp = 30;
		double midTemp = 20;
		double minTemp = 10;
		
		double linearRed;
		double linearGreen;
		double linearBlue;
		
		double linearWarmtoWhite = maxTemp - midTemp;
		double linearWhitetoCool = midTemp - minTemp;
		
		if((tempMeasurement - midTemp) > 0) {
			double fracWarmtoWhite = (tempMeasurement - midTemp) / linearWarmtoWhite;
		
			linearRed = (fracWarmtoWhite * diffWarmtoWhite[0]) + white[0];
			linearGreen = white[1] - (fracWarmtoWhite * diffWarmtoWhite[1]);
			linearBlue = white[2]- (fracWarmtoWhite * diffWarmtoWhite[2]);
		} else {
			double fracWhitetoCool = (tempMeasurement - minTemp) / linearWhitetoCool;
		
			linearRed = (fracWhitetoCool * diffWhitetoCool[0]) + coolest[0];
			linearGreen = coolest[1] - (fracWhitetoCool * diffWhitetoCool[1]);
			linearBlue = coolest[2]- (fracWhitetoCool * diffWhitetoCool[2]);
		}
		
		
		linearRed *= mult;
		linearBlue *= mult;
		linearGreen *= mult;
		
		
		if (tempMeasurement < minTemp) {
			setRedVal(coolest[0]);
			setGreenVal(coolest[1]);
			setBlueVal(coolest[2]);
		} else if (tempMeasurement > maxTemp) {
			setRedVal(warmest[0]);
			setGreenVal(warmest[1]);
			setBlueVal(warmest[2]);
		} else {
			setRedVal(linearRed);
			setGreenVal(linearGreen);
			setBlueVal(linearBlue);
		}
		
		
		// Some delay
		for(i = 0; i < 50000; ++i); 
	}
}
