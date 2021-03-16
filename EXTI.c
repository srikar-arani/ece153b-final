#include "EXTI.h"

#include "LED.h"
#include "UART.h"
#include "PWM.h"

#include <string.h>
#include <stdio.h>

void EXTI_Init(void) {
  // Configure EXTI for joystick UP and CENTER
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODE1;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD1_1;
	
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
	
	EXTI->FTSR1 |= EXTI_FTSR1_FT1;
	
	EXTI->IMR1 |= EXTI_IMR1_IM1;
	
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI1_IRQn,0);
}


double warmest[3] = {0.5, 0.075, 0.075};
double middle[3] = {0,0,1};
double coolest[3] = {0.2, 0.88627451, 1};
double clear[3] = {0, 0, 0};
double hold[3] = {0,0,0};
char rxByte;

// Interrupt handlers

void EXTI1_IRQHandler(void) {
	EXTI->PR1 |= EXTI_PR1_PIF3;
	int i = 1;
	while (i) {
		char chr;
		double rgbRed;
		double rgbGreen;
		double rgbBlue;
		double rgbAlpha;
		printf("Enter '(M)anual' or '(A)utomatic' \n");
		scanf("%c",&chr);
		if (chr == 'A') {
			i = 0;
		} else if (chr == 'M') {
			writeValue(clear);
			printf("Enter RGB Red: ");
			scanf("%lf",&rgbRed);
			hold[0] = rgbRed/255.0;
			writeValue(hold);
			printf("Enter RGB Green: ");
			scanf("%lf",&rgbGreen);
			hold[1] = rgbGreen/255.0;
			writeValue(hold);
			printf("Enter RGB Blue: ");
			scanf("%lf",&rgbBlue);
			hold[2] = rgbBlue/255.0;
			writeValue(hold);
			
			printf("Enter Alpha(Brightness) Value: ");
			scanf("%lf",&rgbAlpha);
			double mult = rgbAlpha/255.0;
			hold[0] = hold[0]*mult;
			hold[1] = hold[1]*mult;
			hold[2] = hold[2]*mult;
			writeValue(hold);
		} else {
			printf("Unrecognized Command \n");
		}
	}
}