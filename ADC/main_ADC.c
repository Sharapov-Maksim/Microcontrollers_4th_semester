#include <stm32f0xx.h>
#include <stdlib.h>
#define FILTER_MAX_CNT 10
#define DMA_ADC_BUFFER_SIZE 128


typedef struct PatternS{
	unsigned short frames[8];
	char showframe[8];
	char frames_cnt;
} PatternS;

typedef struct Display{
	uint8_t raws[8];
	uint8_t frame;
} Display;

void pushNextFrame(Display* dis){
	unsigned short data = (dis->raws[dis->frame]<<8) | (1 << dis->frame);
	SPI2->DR = data;
	dis->frame++;
	dis->frame%=8;
}


// Programming manual - p. 85+
void STInit(){
	SystemCoreClockUpdate();
	SysTick->LOAD = (SystemCoreClock) / 500 - 1; // 1 ms
	SysTick->VAL =  (SystemCoreClock) / 500 - 1; // initial value of timer
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
// Display variables
volatile static int frame = 0;
volatile static PatternS patternStruct;
//volatile static int isDataPushedToSPI = 1;
volatile static Display displayS;
volatile static uint32_t ADC_array[DMA_ADC_BUFFER_SIZE];
volatile int half_of_DMA_array;



void ButtonsCheker(void){
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


void ChangeDisplayFrame(){
	frame++;
	frame%=patternStruct.frames_cnt;
}

void PushFrame(int curr_frame, PatternS pat){
		if (!pat.showframe[curr_frame]) SPI2->DR = 0;
		else SPI2->DR = pat.frames[curr_frame];
}

void SysTick_Handler(void){ 
	ButtonsCheker();
}


static void SPI_init(){
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
	
	// Enable interrupts from SPI2 in core
	NVIC_EnableIRQ(SPI2_IRQn);
	
	//Control Register
	SPI2->CR2 |= SPI_CR2_RXNEIE;
	
	SPI2->DR = 0x0111; // to catch first interrupt
}


void SPI2_IRQHandler(){
	if (SPI2->SR & SPI_SR_RXNE){ // Data was received
		GPIOA->BSRR = GPIO_BSRR_BS_8;  // LE = 1
		volatile uint32_t a = SPI2->DR;
		pushNextFrame(&displayS);
		GPIOA->BSRR = GPIO_BSRR_BR_8;
	}
}


void adc_calibration(){
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN by setting ADDIS*/
	/* (3) Clear DMAEN */
	/* (4) Launch the calibration by setting ADCAL */
	/* (5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		/* For robust implementation, add here time-out management */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{
	}
}

void adc_enable(){
	/* (1) Ensure that ADRDY = 0 */
	/* (2) Clear ADRDY */
	/* (3) Enable the ADC */
	/* (4) Wait until ADC ready */
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
	{
		ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
	{
		ADC1->CR |= ADC_CR_ADEN;
	}
	ADC1->ISR |= ADC_ISR_ADRDY;
}



void displayADC(){
	ADC1->CR |= ADC_CR_ADSTART;
	while (1){
		
		int ADC_Result = 0;
		for(int i = 0; i<DMA_ADC_BUFFER_SIZE/2;i++){
			ADC_Result += ADC_array[i + half_of_DMA_array * (DMA_ADC_BUFFER_SIZE/2)];
		}
		ADC_Result /= (DMA_ADC_BUFFER_SIZE/2);
		
		volatile uint32_t ADC_level = (ADC_Result & 0xE0)>>5;
		
		for(int i = 0; i < 40000; i++){}
		
		displayS.raws[0] = 1 << ADC_level;
		
		for(int i = 7; i>0; i--){
			displayS.raws[i] = displayS.raws[i-1];
		}
	}
}

void adc_Init(){
	adc_calibration();
	
	// taktirovanie na ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CR2 |= RCC_CR2_HSI14ON;
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
	// 00: ADCCLK (Asynchronous clock mode) - on reset
  ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE_Msk;

	ADC1->CFGR1  |= ADC_CFGR1_RES_1; // 8 bit resolution
	
	// DMA Machine   // <Ex.: p.941>
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;  // DMA enable
	ADC1->CFGR1 |= ADC_CFGR1_DMACFG; // DMA circular mode
	
	adc_enable();
	
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1; // PA1
	ADC1->CFGR1  |= ADC_CFGR1_CONT;   // Continuous conversion mode (CONT=1) <13.4.8>
	ADC1->CFGR1  |= ADC_CFGR1_OVRMOD;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Clock on PA1
	GPIOA->MODER |= GPIO_MODER_MODER1; // Analog mode on PA1
	
}

static void DMA_Init(){
	/* (1) Enable the peripheral clock on DMA */
	/* (2) Enable DMA transfer on ADC */
	/* (3) Configure the peripheral data register address */
	/* (4) Configure the memory address */
	/* (5) Configure the number of DMA tranfer to be performs on channel 1 */
	/* (6) Configure increment, size and interrupts + circular mode */
	/* (7) Enable DMA Channel 1 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN; /* (2) */
	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); /* (3) */
	DMA1_Channel1->CMAR = (uint32_t)(ADC_array); /* (4) */
	DMA1_Channel1->CNDTR = DMA_ADC_BUFFER_SIZE; /* (5) */
	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_0
												| DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_CIRC; /* (6) */
	DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */
	
	/* Configure NVIC for DMA */
	/* Enable Interrupt on DMA Channel 1 */
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


void DMA1_Channel1_IRQHandler(){
	//GPIOC->BSRR = GPIO_BSRR_BS_6;  // Red light
	half_of_DMA_array++;
	half_of_DMA_array%=2;
	if (DMA1->ISR & DMA_ISR_HTIF1)
		DMA1->IFCR |= DMA_IFCR_CHTIF1;
	if (DMA1->ISR & DMA_ISR_TCIF1)
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
}

int main(void){
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->MODER |= GPIO_MODER_MODER15_0;
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR5_1;
	
	//STInit();
	displayS.frame = 0;
	for (int i = 0; i < 8; i++){
		displayS.raws[i] = 0;
	}
	
	for(int i = 0; i<DMA_ADC_BUFFER_SIZE; i++){
		ADC_array[0] = 0;
	}
	
	SPI_init();
	
	adc_Init();
	
	
	DMA_Init();
	half_of_DMA_array = 1;
	
	displayADC();
	return 0;
	
}
