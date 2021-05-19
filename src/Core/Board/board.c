
#include "stm32f1xx.h"
#include "uart.h"
#include "can.h"
#include "tasks.h"

#include "board.h"


static void initRCC(void);
static void initGPIO(void);
static void initTimer(void);
static void initADC(void);

volatile uint16_t conv;                  // ADC temperature sensor convertion value

/**
 * @brief MCU initialization
 */
void initMCU(void)
{
    initRCC();
    initGPIO();
    initTimer();
    initUART();
    initADC();
    initCAN();
    initTasks();

}


/**
 * @brief RCC initialization
 *        System clock 64 MHz
 *        System clock source is PLL from HSI
 *        HCLK clock 64 MHz
 *        PCLK1 clock 32 MHz
 *        PCLK2 clock 64 MHz
 *        ADC clock 8 MH
 *
 *        systick timer delay 1 ms
 */
static void initRCC(void)
{
    // set flash latency two wait states
    FLASH->ACR |= FLASH_ACR_LATENCY_1;

    // wait FLASH->ACR to set
    while((FLASH->ACR & FLASH_ACR_LATENCY_1) == 0);

    // set HSI on
    RCC->CR |= RCC_CR_HSION;

    // wait HSI ready
    while((RCC->CR & RCC_CR_HSIRDY) == 0);

    // set PLL source to HSI and PLL mul to 16
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;
    RCC->CFGR |= RCC_CFGR_PLLMULL16;

    // enable PLL
    RCC->CR |= RCC_CR_PLLON;

    // wait PLL ready
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // set APB1 prescaler to 2
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // set system clock source to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // wait system clock ready
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // set system variable SystemCoreClock to current clock value
    SystemCoreClockUpdate();

    // set ADC precsaler to 8
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;

    // enable ADC interface
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // enable power interface
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // enable USART interface
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // enable DMA interface
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // enable TIM4 interface
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // enable CAN interface
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
}


/**
 * @brief GPIO initialization
 */
static void initGPIO(void)
{
    // enable PORTA clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // enable alternate function clock
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // UART PA9 - TX, PA10 - RX
    // PA9
    // Output mode, max speed 10 MHz
    GPIOA->CRH |= GPIO_CRH_MODE9_0;
    GPIOA->CRH &= ~GPIO_CRH_MODE9_1;

    // Alternate function output Push-pull
    GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
    GPIOA->CRH |= GPIO_CRH_CNF9_1;

    // PA10
    // Input mode
    GPIOA->CRH &= ~GPIO_CRH_MODE10;

    // Alternate function floating input
    GPIOA->CRH |= GPIO_CRH_CNF10_0;
    GPIOA->CRH &= ~GPIO_CRH_CNF10_1;

    // PORTA5 LED
    GPIOA->CRL = GPIO_CRL_MODE5_1;
    GPIOA->BSRR = GPIO_BSRR_BR5;

    // PA0 DAC
    // input mode analog mode
    GPIOA->CRL &= ~GPIO_CRL_MODE0;
    GPIOA->CRL &= ~GPIO_CRL_CNF0;

    // PA11 - CAN RX, PA12 - CAN TX
    // PA11
    // Input mode pull-up
    GPIOA->CRH &= ~GPIO_CRH_MODE11;
    GPIOA->CRH &= ~GPIO_CRH_CNF11;
    GPIOA->CRH |= GPIO_CRH_CNF11_1;
    GPIOA->ODR |= GPIO_ODR_ODR11;

    // PA12
    // Output mode, max speed 50 MHz
    GPIOA->CRH |= GPIO_CRH_MODE12;

    // Alternate function Push Pull
    GPIOA->CRH &= ~GPIO_CRH_CNF12_0;
    GPIOA->CRH |= GPIO_CRH_CNF12_1;
}


/**
 * @brief UART timer initialization
 */
static void initTimer()
{
    // set precsaler to get 1 MHz clock
    TIM4->PSC = 64;
    // set auto-reload to get 20 ms delay
    TIM4->ARR = 20000;

    // update
    TIM4->EGR |= TIM_EGR_UG;

    // reset status regiater
    TIM4->SR = 0;

    // enable interrupt
    TIM4->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM4_IRQn, 0x00);
    NVIC_EnableIRQ(TIM4_IRQn);
}


/**
 * @brief ADC initalization
 */
static void initADC()
{
    // enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // enable DMA
    ADC1->CR2 |= ADC_CR2_DMA;

    // DMA1 Channel 1 ADC
    // set data transfer direction read from peripheral
    DMA1_Channel1->CCR &= ~DMA_CCR_DIR;

    // set memory size 16 bits
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;

    // set peripheral size 16 bits
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;

    // set peripheral address
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));

    // set memory address
    DMA1_Channel1->CMAR = (uint32_t)(&conv);

    // set DMA buffer size
    DMA1_Channel1->CNDTR = 1;

    // set DMA transfer complete interrupt enable
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;

    // enable global DMA interrupt
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // enable DMA channel
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // enable temperature sensor
    ADC1->CR2 |= ADC_CR2_TSVREFE;

    // event selection to SWStart
    ADC1->CR2 |= ADC_CR2_EXTSEL;

    // start convertion on external event
    ADC1->CR2 |= ADC_CR2_EXTTRIG;

    // set sample time 239.5 cycles
    ADC1->SMPR1 |= ADC_SMPR1_SMP16;

    // select channel 16
    ADC1->SQR3 = ADC_SQR3_SQ1_4;

    // select continuous conversion mode
    ADC1->CR2 |= ADC_CR2_CONT;

    // start convertion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}


/**
 * @brief DMA interrupt handler
 */
void DMA1_Channel1_IRQHandler(void)
{
    // disable DMA channel
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    // clear DMA flags
    DMA1->IFCR = (DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1);

    // set DMA buffer size
    DMA1_Channel1->CNDTR = 1;

    // enable DMA channel
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}
