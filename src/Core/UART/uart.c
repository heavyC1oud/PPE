
#include "stm32f1xx.h"

#include "uart.h"

uart_buf_t rbuf;

static volatile uint8_t trCompleteF = 0;

/**
 * @brief UART initialization
 */
void initUART(void)
{
    // enable USART
    USART1->CR1 |= USART_CR1_UE;

    // set 115200 baud rate
    USART1->BRR = 0x022C;

    // enable TX DMA
    USART1->CR3 |= USART_CR3_DMAT;

    // DMA1 Channel 4 USART1 TX
    // enable memory increment mode
    DMA1_Channel4->CCR |= DMA_CCR_MINC;

    // set data transfer direction read from memory
    DMA1_Channel4->CCR |= DMA_CCR_DIR;

    // set peripheral address
    DMA1_Channel4->CPAR = (uint32_t)(&(USART1->DR));

    // enable RX DMA
    USART1->CR3 |= USART_CR3_DMAR;

    // DMA1 Channel 5 USART1 RX
    // enable memory increment mode
    DMA1_Channel5->CCR |= DMA_CCR_MINC;

    // set data transfer direction read from peripheral
    DMA1_Channel5->CCR &= ~DMA_CCR_DIR;

    // set peripheral address
    DMA1_Channel5->CPAR = (uint32_t)(&(USART1->DR));

    // set memory address
    DMA1_Channel5->CMAR = (uint32_t)(rbuf.data);

    // set DMA receive buffer size
    DMA1_Channel5->CNDTR = USART_BUF_SIZE;

    // enable DMA channel
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    // enable global USART interrupt
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);

    // enable idle interrupt
    USART1->CR1 |= USART_CR1_IDLEIE;

    // enable transmit complete interrupt
    USART1->CR1 |= USART_CR1_TCIE;

    // enable receiver
    USART1->CR1 |= USART_CR1_RE;
}


/**
 * @brief Send data array.
 *
 * @param data - pointer to data array
 * @param size - data count
 */
void uart_putdata(uint8_t* data, uint16_t size)
{
    trCompleteF = 0;

    // set memory address
    DMA1_Channel4->CMAR = (uint32_t)(data);

    // set data count
    DMA1_Channel4->CNDTR = size;

    // enable transmitter
    USART1->CR1 |= USART_CR1_TE;

    // enable DMA channel
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}


/**
 * @brief Get transmit complete flag
 *
 * @return - flag state, 0 - transmit not complete, 1 - transmit compele
 */
uint8_t getUartTF(void)
{
    return trCompleteF;
}


/**
 * @brief UART1 IRQ
 */
void USART1_IRQHandler(void)
{
    // receive complete
    if((USART1->SR & USART_SR_IDLE) != 0) {
        (void)USART1->DR;
        // start delay timer
        TIM4->CR1 &= ~TIM_CR1_CEN;
        TIM4->CNT = 0;
        TIM4->CR1 |= TIM_CR1_CEN;
    }

    // transmit complete
    if((USART1->SR & USART_SR_TC) != 0) {
        // disable DMA channel
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;

        // clear DMA flags
        DMA1->IFCR = (DMA_IFCR_CGIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTEIF4);

        // clear UART TC flag
        USART1->SR &= ~USART_SR_TC;

        // disable transmitter
        USART1->CR1 &= ~USART_CR1_TE;

        trCompleteF = 1;
    }
}


/**
 * @brief Receive timer interrupt handler
 */
void TIM4_IRQHandler(void)
{
    // disable timer
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->CNT = 0;

    // clear flags
    TIM4->SR = 0;

    // disable DMA channel
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    // calculate received data
    rbuf.count = USART_BUF_SIZE - DMA1_Channel5->CNDTR;

    // clear DMA flags
    DMA1->IFCR = (DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5);

    // set DMA receive buffer size
    DMA1_Channel5->CNDTR = USART_BUF_SIZE;

    // enable DMA channel
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}
