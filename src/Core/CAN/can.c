
#include "stm32f1xx.h"

#include "can.h"

static void can_putmailbox(uint8_t* data, uint8_t size);

/**
 * @brief CAN initalization
 */
void initCAN(void)
{
    // exit sleep mode
    CAN1->MCR &= ~CAN_MCR_SLEEP;

    // enable CAN configuration
    CAN1->MCR |= CAN_MCR_INRQ;

    // wait INAK bit
    while((CAN1->MSR & CAN_MSR_INAK) == 0);

    // disable autoretransmission
    CAN1->MCR |= CAN_MCR_NART;

    // enable loop back mode
    CAN1->BTR |= CAN_BTR_LBKM;

    // set CAN clock to 1 MHz
    // presc = 4, clock = 36 / 4 = 8MHz
    // SWJ = 1tq, TS1 = 13tq, TS2 = 2tq
    CAN1->BTR |= (0x03UL << CAN_BTR_BRP_Pos);

    CAN1->BTR &= ~CAN_BTR_TS1;
    CAN1->BTR |= (0x0CUL << CAN_BTR_TS1_Pos);

    CAN1->BTR &= ~CAN_BTR_TS2;
    CAN1->BTR |= (0x01UL << CAN_BTR_TS2_Pos);

    // disable CAN configuration
    CAN1->MCR &= ~CAN_MCR_INRQ;

    // wait INAK bit
    while((CAN1->MSR & CAN_MSR_INAK) != 0);

    // clear status flags
    CAN1->TSR |= (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_ALST0 | CAN_TSR_TERR0);

    // enable FIFO message pending interrupt enable
    CAN1->IER |= CAN_IER_FMPIE0;

    // enable global CAN RX interrupt
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    // enable CAN filter initialization
    CAN1->FMR |= CAN_FMR_FINIT;

    // deactivate filter 0
    CAN1->FA1R &= ~CAN_FA1R_FACT0;

    // single 32-bit scale configuration
    CAN1->FS1R |= CAN_FS1R_FSC0;

    // mask mode
    CAN1->FM1R &= ~CAN_FM1R_FBM0;

    // empty filter
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;

    // filter assignment to FIFO0
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0;

    // activate filter 0
    CAN1->FA1R |= CAN_FA1R_FACT0;

    // disable CAN filter initialization
    CAN1->FMR &= ~CAN_FMR_FINIT;
}


/**
 * @ brief Send up to 8 bytes of data through CAN interface
 *
 * @param data - pointer to data array
 * @param size - mailbox data count
 */
static void can_putmailbox(uint8_t* data, uint8_t size)
{
    if(data == 0) {
        return;
    }

    // check size
    if(size > MAX_CAN_BOX_DATA_SIZE) {
        return;
    }

    // wait mailbox
    while((CAN1->TSR & CAN_TSR_TME0) == 0);

    // data length code
    CAN1->sTxMailBox[0].TDTR = (size << CAN_TDT0R_DLC_Pos);

    CAN1->sTxMailBox[0].TDLR = 0;
    CAN1->sTxMailBox[0].TDHR = 0;

    for(uint8_t i = 0; i < size; i++) {
        if(i < REG_SIZE) {
            CAN1->sTxMailBox[0].TDLR |= (data[i] << (i * BYTE_SIZE));
        }
        else {
            CAN1->sTxMailBox[0].TDHR |= (data[i]) << ((i - REG_SIZE) * BYTE_SIZE);
        }
    }

    // set ID
    CAN1->sTxMailBox[0].TIR = (CAN_ID << CAN_TI0R_STID_Pos);

    // transmit mailbox request
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}


/**
 * @ brief Send data through CAN interface
 *
 * @param data - pointer to data array
 * @param size - data count
 */
void can_putdata(uint8_t* data, uint16_t size)
{
    if((data == 0) || (size == 0)) {
        return;
    }

    uint16_t pi = 0;

    do {
        if(size >= MAILBOX_SIZE) {
            // send data
            can_putmailbox(&data[pi], MAILBOX_SIZE);

            pi += MAILBOX_SIZE;
            size -= MAILBOX_SIZE;
        }
        else {
            // send data
            can_putmailbox(&data[pi], size);

            size = 0;
        }
    }
    while(size != 0);
}


/**
 * @brief CAN receive interrupt handler
 *        for test purpose only
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    uint16_t id = 0;
    uint16_t len = 0;
    uint8_t mail[8] = {0};

    (void)id;
    (void)mail;

    id = (CAN1->sFIFOMailBox[0].RIR >> CAN_RI0R_STID_Pos);
    len = (CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk);

    for(uint8_t i = 0; i < len; i++) {
        if(i < REG_SIZE) {
            mail[i] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> (i * BYTE_SIZE));
        }
        else {
            mail[i] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> ((i - REG_SIZE) * BYTE_SIZE));
        }
    }

    // clear receive
    CAN1->RF0R |= CAN_RF0R_RFOM0;
}
