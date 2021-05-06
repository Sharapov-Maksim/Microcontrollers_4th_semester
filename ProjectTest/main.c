#include <stm32f0xx.h>

#define DELAY 300000

static void runningLight(void){
	// PC6 - Red
	// PC7 - Blue
	// PC8 - Yellow
	// PC9 - Green
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // dali taktirovanie dlya pereferiynogo modulya
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	
	while (1) {
		GPIOC->ODR |= GPIO_ODR_6;
		for (int i = 0; i < DELAY; i++){}
		GPIOC->ODR &= ~GPIO_ODR_6;
		GPIOC->ODR |= GPIO_ODR_9;
		for (int i = 0; i < DELAY; i++){}
		GPIOC->ODR &= ~GPIO_ODR_9;
		
		GPIOC->ODR |= GPIO_ODR_7;
		for (int i = 0; i < DELAY; i++){}
		GPIOC->ODR &= ~GPIO_ODR_7;
		
		GPIOC->ODR |= GPIO_ODR_8;
		for (int i = 0; i < DELAY; i++){}
		GPIOC->ODR &= ~GPIO_ODR_8;
		
	}
	
}

static void Init(void){
	// PC6 - PC9 - LEDs
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // dali taktirovanie dlya pereferiynogo modulya
	
	GPIOC->MODER |= GPIO_MODER_MODER6_0;  //PC6, 0-mladshiy bit iz dvuh
	
	//GPIOC->PUPDR |= GPIO_PUPDR_PUPDR6_0; // pull up  pull down  register  mladshiy-podtyazka up   starshiy - down
	//GPIOC->OTYPER |= GPIO_OTYPER_OT_6;
	//GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1; // izmenit speed
	//uint32_t a = GPIOC->IDR;
	
	
	
	GPIOC->ODR |= GPIO_ODR_6;     // 3 comands
	GPIOC->BSRR = GPIO_BSRR_BS_6; // 1 comand
	
	
	// BRR
	
	
	// R |= M;  // set bit
	// R &= ~M; // reset bit
	// R = R | M; // read modificate write  - 3 takta
	
	// BSRR = M   - bit set/reset register   v starshih bitah    //
	// BRR = M   - bit reset register  // R & ~M
}


static void buttonBlueLight(void){
	// PA0 - USER Button
	// PC7 - Blue LED
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	// GPIOA->MODER by default 00
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	while (1){
		if (GPIOA->IDR & GPIO_IDR_0)
			GPIOC->ODR |= GPIO_ODR_7;    // Blue Light
			for (int i = 0; i < DELAY; i++){}
			GPIOC->ODR &= ~GPIO_ODR_7;
	}
}

static int checkButton (void){
	if (GPIOA->IDR & GPIO_IDR_0)
			return 1;
	//GPIOC->ODR |= GPIO_ODR_7;    // Blue Light
	else 
			return 0;
			//GPIOC->ODR &= ~GPIO_ODR_7;   // Off blue light
	
}


static void trafficLight(void){
	// PC6 - Red
	// PC7 - Blue
	// PC8 - Yellow
	// PC9 - Green
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	int state = 0;
	while (1) {
		int buttonState = checkButton();
		
			switch (state){
				case 0: // Red light
					GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BR_8;
					//delay *= 3;
					break;
				case 3 * DELAY: // Yellow light
					GPIOC->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BR_6;
					break;
				case 4 * DELAY: // Green light
					GPIOC->BSRR = GPIO_BSRR_BS_9 | GPIO_BSRR_BR_8;
					//delay *= 4;
					break;
				case 8 * DELAY: // Yellow light
					GPIOC->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BR_9;
					break;
			}
		
		if (buttonState)
			GPIOC->ODR |= GPIO_ODR_7;    // Blue Light
		else 
			GPIOC->ODR &= ~GPIO_ODR_7;   // Off blue light
		
		state++;
		state %= 9 * DELAY;
	}		
}


// Programming manual - p. 85+
void STInit(){
	SystemCoreClockUpdate();
	SysTick->LOAD = (SystemCoreClock*5) / 10 - 1; // 100 ms
	SysTick->VAL =  (SystemCoreClock*5) / 10 - 1; // initial value of timer
	//SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // vkluchit' timer 
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk; //- razrehit` obrabotchik prerivaniy
}

static void trafficLightOnTimer(void){
	// PC6 - Red
	// PC7 - Blue
	// PC8 - Yellow
	// PC9 - Green
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	int state = 0;
	while (1) {
		int buttonState = checkButton();
		
		if ( SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ) {// operatsia chtenia stavit 0
			switch (state){
				case 0: // Red light
					GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BR_8;
					state = 1;
					break;
				case 1: // Yellow light
					GPIOC->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BR_6;
					state = 2;
					break;
				case 2: // Green light
					GPIOC->BSRR = GPIO_BSRR_BS_9 | GPIO_BSRR_BR_8;
					state = 3;
					break;
				case 3: // Yellow light
					GPIOC->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BR_9;
					state = 0;
					break;
			}
		}
		if (buttonState)
			GPIOC->ODR |= GPIO_ODR_7;    // Blue Light
		else 
			GPIOC->ODR &= ~GPIO_ODR_7;   // Off blue light
		
	}		
}


volatile counter = 0; // vneshnie peremennie
void SysTick_Handler(void){ // timer interrupt handler
	counter++;
	counter%=9;
}

static void trafficLightOnTimerInterrupt(void){
	// PC6 - Red
	// PC7 - Blue
	// PC8 - Yellow
	// PC9 - Green
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	while (1) {
		
		int buttonState = checkButton();
		if ( SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ) {
			// cnt == 0   -> state 0
			// cnt == 3   -> state 1
			// cnt == 4   -> state 2
			// cnt == 8   -> state 3  ==> cnt = 0
			switch (counter){
				case 0: // Red light
					GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BR_8;
					break;
				case 3: // Yellow light
					GPIOC->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BR_6;
					break;
				case 4: // Green light
					GPIOC->BSRR = GPIO_BSRR_BS_9 | GPIO_BSRR_BR_8;
					break;
				case 8: // Yellow light
					GPIOC->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BR_9;
					break;
			}
		}
		if (buttonState)
			GPIOC->ODR |= GPIO_ODR_7;    // Blue Light
		else 
			GPIOC->ODR &= ~GPIO_ODR_7;   // Off blue light
		
	}		
}
/*
vihod
PC12

PA15
		PA4		PA5  - vhod
*/


int main(void){
	//runningLight();
	//buttonBlueLight();
	//trafficLight();
	STInit();
	trafficLightOnTimerInterrupt();
	return 0;
	
}
