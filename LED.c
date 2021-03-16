#include "LED.h"

void LED_Init(void) {
	// [TODO]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	// Initialize Red LED
	// [TODO]
	GPIOB->MODER &= ~(3UL<<4);
	GPIOB->MODER |= 1UL<<4;
	
	GPIOB->OTYPER &= ~(1UL<<2);
	
	// Initialize Green LED
	// [TODO]
	GPIOE->MODER &= ~(3UL<<16);
	GPIOE->MODER |= 1UL<<16;
	
	GPIOE->OTYPER &= ~(1UL<<8);
}

void Red_LED_Off(void) {
	// [TODO]
	GPIOB->ODR &= 0UL<<2;
}

void Red_LED_On(void) {
	// [TODO]
	GPIOB->ODR |= 1UL<<2;
}

void Red_LED_Toggle(void){
	// [TODO]
	GPIOB->ODR ^= 1UL<<2;
}

void Green_LED_Off(void) {
	// [TODO]
	GPIOE->ODR &= 0UL<<8;
}

void Green_LED_On(void) {
	// [TODO]
	GPIOE->ODR |= 1UL<<8;
}

void Green_LED_Toggle(void) {
	// [TODO]
	GPIOE->ODR ^= 1UL<<8;
}
