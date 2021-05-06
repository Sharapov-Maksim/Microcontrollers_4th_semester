#include <stm32f0xx.h>

#define FILTER_MAX_CNT 10

// Programming manual - p. 85+
void STInit(){
	SystemCoreClockUpdate();
	SysTick->LOAD = (SystemCoreClock) / 1000 - 1; // 1 ms
	SysTick->VAL =  (SystemCoreClock) / 1000 - 1; // initial value of timer
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk; //- razrehit` obrabotchik prerivaniy
}

volatile static int button1 = 0; // vneshnie peremennie
volatile static int button2 = 0;
volatile static int button3 = 0;
volatile static int button4 = 0;
volatile static int stateOfKeyboardChecker = 0;
volatile static int c1 = 0;
volatile static int c2 = 0;
volatile static int c3 = 0;
volatile static int c4 = 0;
volatile static int state1 = 0;
volatile static int state2 = 0;
volatile static int state3 = 0;
volatile static int state4 = 0;

/*void SysTick_Handler_for_holding_buttons(void){ // timer interrupt handler
	if(state == 0) {
		GPIOA->BSRR = GPIO_BSRR_BR_15;
		GPIOC->BSRR = GPIO_BSRR_BS_12;
		if (GPIOA->IDR & GPIO_IDR_4)
			button1 = 1;
		else{
			button1 = 0;
		}
		if (GPIOA->IDR & GPIO_IDR_5)
			button2 = 1;
		else{
			button2 = 0;
		}
		GPIOC->BSRR = GPIO_BSRR_BR_12;
	}
	else{
		GPIOC->BSRR = GPIO_BSRR_BR_12;
		GPIOA->BSRR = GPIO_BSRR_BS_15;
		if (GPIOA->IDR & GPIO_IDR_4)
			button3 = 1;
		else{
			button3 = 0;
		}
		if (GPIOA->IDR & GPIO_IDR_5)
			button4 = 1;
		else{
			button4 = 0;
		}
		GPIOA->BSRR = GPIO_BSRR_BR_15;
	}
	state++;
	state %= 2;
}*/



void SysTick_Handler(void){ 
	if(stateOfKeyboardChecker == 0) {
		GPIOA->BSRR = GPIO_BSRR_BR_15;
		GPIOC->BSRR = GPIO_BSRR_BS_12;
		if (GPIOA->IDR & GPIO_IDR_4){
			if(state1 == 0){
				c1++;
			}
			else{
				state1 = 1;
				c1 = 0;
			}
			if (c1 == FILTER_MAX_CNT) {button1++;}
		}
		else{
			if (state1 == 1){
				c1++;
			}
			else{
				state1 = 0;
				c1 = 0;
			}
		}
		if (GPIOA->IDR & GPIO_IDR_5){
				if(state2 == 0){
				c2++;
			}
			else{
				state2 = 1;
				c2 = 0;
			}
			if (c2 == FILTER_MAX_CNT) {button2++;}

		}
		else{
			if (state2 == 1){
				c2++;
			}
			else{
				state2 = 0;
				c2 = 0;
			}
		}
		GPIOC->BSRR = GPIO_BSRR_BR_12;
	}
	else{
		GPIOC->BSRR = GPIO_BSRR_BR_12;
		GPIOA->BSRR = GPIO_BSRR_BS_15;
		if (GPIOA->IDR & GPIO_IDR_4){
			if(state3 == 0){
				c3++;
			}
			else{
				state3 = 1;
				c3 = 0;
			}
			if (c3 == FILTER_MAX_CNT) {button3++;}
		}
		else{
			if (state3 == 1){
				c3++;
			}
			else{
				state3 = 0;
				c3 = 0;
			}
		}
		if (GPIOA->IDR & GPIO_IDR_5){
				if(state4 == 0){
				c4++;
			}
			else{
				state4 = 1;
				c4 = 0;
			}
			if (c4 == FILTER_MAX_CNT) {button4++;}

		}
		else{
			if (state4 == 1){
				c4++;
			}
			else{
				state4 = 0;
				c4 = 0;
			}
		}GPIOA->BSRR = GPIO_BSRR_BR_15;
	}
	stateOfKeyboardChecker++;
	stateOfKeyboardChecker %= 2;
}


/*
vihod
PC12

PA15
		PA4		PA5  - vhod
*/

static void lightsOnKeyboard(void) {
	/*
	nuzen antidrebezgoviy filtr
	mojno jdat posle pervogo fronta -- ploxo (>50 ms)
	
	
	*/
	
	
	while(1){
				if(button3 == 1){
					GPIOC->BSRR = GPIO_BSRR_BS_6; // Red
				}
				else{
					GPIOC->BSRR = GPIO_BSRR_BR_6;
					button3 = 0;
				}
				if(button1 == 1){
					GPIOC->BSRR = GPIO_BSRR_BS_9; // Green
				}
				else{
					GPIOC->BSRR = GPIO_BSRR_BR_9;
					button1 = 0;
				}
							
				if(button2 == 1){
					GPIOC->BSRR = GPIO_BSRR_BS_8; //Yellow
				}
				else{
					GPIOC->BSRR = GPIO_BSRR_BR_8;
					button2 = 0;
				}
				if(button4 == 1){
					GPIOC->BSRR = GPIO_BSRR_BS_7; //Blue
				}
				else{
					GPIOC->BSRR = GPIO_BSRR_BR_7;
					button4 = 0;
				}
		}
}

//SSM 
//SSI = 1 
// SPE - po zaversheniyu nastroyki
// BR - skorost` interfeisa -- 	stavim 111
// MSTR = 1
// CPOL = 0 | 1
// CPHA = 0 | 1
// DS = 1111
// FRF = 0
// 


// DATA - PB15
// PB13 - CLCK
// LE - PA8
static void SPI_init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	
	
	// podat` taktirovanie na SPI
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
	//nastraivaem interfeis
	SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA;
	SPI2->CR2 = SPI_CR2_DS;
	SPI2->CR1 |= SPI_CR1_SPE;
	
	
	
	// daem taktirovanie na vihody
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//GPIOB->AFR[0] |= Fn << 4 * Pn; // Pn < 8
	//GPIOB->AFR[1] |= Fn << 4 * (Pn - 8); // Pn >= 8
	GPIOB->AFR[1] |= (0 << 4 * (13 - 8)) | (0 << 4 * (15 - 8));
	
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1;
}




int main(void){
	/*RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR5_1;
	STInit();*/
	//lightsOnKeyboard();
	SPI_init();
	while(1)
	{
		if ( !(SPI2->SR & SPI_SR_BSY)){
			GPIOA->BSRR = GPIO_BSRR_BR_8;
			SPI2->DR = 0xFF11;
		}
		while (SPI2->SR & SPI_SR_BSY);
		GPIOA->BSRR = GPIO_BSRR_BS_8;
		
		}
	return 0;
	
}
