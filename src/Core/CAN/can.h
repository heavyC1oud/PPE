
#ifndef _CAN_H_
#define _CAN_H_


typedef enum
{
    MAX_CAN_BOX_DATA_SIZE = 8,
    CAN_ID = 72,
    BYTE_SIZE = 8,
    MAILBOX_SIZE = 8,
    REG_SIZE = (sizeof(uint32_t) / sizeof(uint8_t)),
} can_param_t;


void initCAN(void);
void can_putdata(uint8_t* data, uint16_t size);

#endif  // _CAN_H_
