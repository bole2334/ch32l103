/********************************** (C) COPYRIGHT  *******************************
 * File Name          : iap.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/01/13
 * Description        : IAP
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "iap.h"
#include "string.h"
#include "core_riscv.h"

/******************************************************************************/

iapfun jump2app;
u32 Program_addr = FLASH_Base;
u32 Verify_addr = FLASH_Base;
u32 User_APP_Addr_offset = 0x5000;
u8 Verify_Star_flag = 0;
u8 Fast_Program_Buf[390];
u32 CodeLen = 0;
u8 End_Flag = 0;
u8 EP2_Rx_Buffer[USBD_DATA_SIZE+4];
#define  isp_cmd_t   ((isp_cmd  *)EP2_Rx_Buffer)
CanFDRxMsg CanFDRxStructure = {0};
u8 CanFD_IAP_StreamBuf[80];
u8 CanFD_IAP_StreamLen = 0;
u8 CanFD_IAP_StreamState = 0;

static u8 CANFD_IAP_DlcEncode(u8 len)
{
    if(len <= 8) return len;
    else if(len <= 12) return CANFD_DLC_BYTES_12;
    else if(len <= 16) return CANFD_DLC_BYTES_16;
    else if(len <= 20) return CANFD_DLC_BYTES_20;
    else if(len <= 24) return CANFD_DLC_BYTES_24;
    else if(len <= 32) return CANFD_DLC_BYTES_32;
    else if(len <= 48) return CANFD_DLC_BYTES_48;
    else return CANFD_DLC_BYTES_64;
}

static void CANFD_IAP_StreamReset(void)
{
    CanFD_IAP_StreamLen = 0;
    CanFD_IAP_StreamState = 0;
}

static u8 CANFD_IAP_Send_Msg(u8 *msg, u8 len)
{
    u8 mbox;
    u16 i = 0, timeout = 0;
    CanFDTxMsg CanFDTxStructure = {0};

    CanFDTxStructure.StdId = CANFD_IAP_TX_STDID;
    CanFDTxStructure.IDE = CAN_Id_Standard;
    CanFDTxStructure.RTR = CAN_RTR_Data;
    CanFDTxStructure.DLC = CANFD_IAP_DlcEncode(len);

    for(i = 0; i < len; i++) {
        CanFDTxStructure.Data[i] = msg[i];
    }

    mbox = CANFD_Transmit(CAN1, &CanFDTxStructure);

    while((CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) && (timeout < 0xFFF))
    {
        timeout++;
    }

    if(timeout == 0xFFF)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void CANFD_IAP_SendAck(u8 s)
{
    u8 ack_buf[6];

    ack_buf[0] = Uart_Sync_Head1;
    ack_buf[1] = Uart_Sync_Head2;
    ack_buf[2] = 0x00;
    ack_buf[3] = (s == ERR_ERROR) ? 0x01 : 0x00;
    ack_buf[4] = Uart_Sync_Head2;
    ack_buf[5] = Uart_Sync_Head1;
    CANFD_IAP_Send_Msg(ack_buf, 6);
}

static u8 CANFD_IAP_PacketExpectedLen(u8 cmd, u8 data_len)
{
    u8 expected = 8;

    if((cmd == CMD_IAP_ERASE) || (cmd == CMD_IAP_VERIFY))
    {
        expected += 4;
    }

    if((cmd == CMD_IAP_PROM) || (cmd == CMD_IAP_VERIFY))
    {
        expected += data_len;
    }

    return expected;
}

static u8 CANFD_IAP_PacketDeal(u8 *pkt, u8 pkt_len)
{
    u8 i, cmd, data_len, expected_len;
    u16 data_add = 0;

    if(pkt_len < 8)
    {
        return ERR_ERROR;
    }

    if((pkt[0] != Uart_Sync_Head1) || (pkt[1] != Uart_Sync_Head2))
    {
        return ERR_ERROR;
    }

    if((pkt[pkt_len - 2] != Uart_Sync_Head2) || (pkt[pkt_len - 1] != Uart_Sync_Head1))
    {
        return ERR_ERROR;
    }

    cmd = pkt[2];
    data_len = pkt[3];
    expected_len = CANFD_IAP_PacketExpectedLen(cmd, data_len);
    if((expected_len != pkt_len) || (data_len > 64))
    {
        return ERR_ERROR;
    }

    isp_cmd_t->UART.Cmd = cmd;
    isp_cmd_t->UART.Len = data_len;
    data_add += cmd;
    data_add += data_len;

    if((cmd == CMD_IAP_ERASE) || (cmd == CMD_IAP_VERIFY))
    {
        for(i = 0; i < 4; i++)
        {
            isp_cmd_t->other.buf[2 + i] = pkt[4 + i];
            data_add += pkt[4 + i];
        }
    }

    if((cmd == CMD_IAP_PROM) || (cmd == CMD_IAP_VERIFY))
    {
        u8 data_offset = ((cmd == CMD_IAP_VERIFY) ? 8 : 4);
        for(i = 0; i < data_len; i++)
        {
            isp_cmd_t->UART.data[i] = pkt[data_offset + i];
            data_add += pkt[data_offset + i];
        }
    }

    if(pkt[pkt_len - 4] != (u8)(data_add & 0xFF))
    {
        return ERR_ERROR;
    }

    if(pkt[pkt_len - 3] != (u8)(data_add >> 8))
    {
        return ERR_ERROR;
    }

    return UART_RecData_Deal();
}

void CANFD_IAP_Init(void)
{
    GPIO_InitTypeDef GPIO_InitSturcture = {0};
    CAN_InitTypeDef CAN_InitSturcture = {0};
    CANFD_InitTypeDef CANFD_InitSturcture = {0};
    CAN_FilterInitTypeDef CAN_FilterInitSturcture = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_CAN1, ENABLE);

    GPIO_InitSturcture.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitSturcture.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitSturcture.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitSturcture);

    GPIO_InitSturcture.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitSturcture.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitSturcture);

    CAN_InitSturcture.CAN_Mode = CAN_Mode_Normal;
    CAN_InitSturcture.CAN_SJW = CAN_SJW_4tq;
    CAN_InitSturcture.CAN_BS1 = CAN_BS1_15tq;
    CAN_InitSturcture.CAN_BS2 = CAN_BS2_8tq;
    CAN_InitSturcture.CAN_Prescaler = 2;
    CAN_Init(CAN1, &CAN_InitSturcture);

    CANFD_InitSturcture.CANFD_TTCM = DISABLE;
    CANFD_InitSturcture.CANFD_ABOM = DISABLE;
    CANFD_InitSturcture.CANFD_AWUM = DISABLE;
    CANFD_InitSturcture.CANFD_NART = ENABLE;
    CANFD_InitSturcture.CANFD_RFLM = DISABLE;
    CANFD_InitSturcture.CANFD_TXFP = DISABLE;
    CANFD_InitSturcture.CANFD_Mode = CAN_Mode_Normal;
    CANFD_InitSturcture.CANFD_SJW = CANFD_SJW_4tq;
    CANFD_InitSturcture.CANFD_BS1 = CANFD_BS1_6tq;
    CANFD_InitSturcture.CANFD_BS2 = CANFD_BS2_5tq;
    CANFD_InitSturcture.CANFD_Prescaler = 2;
    CANFD_InitSturcture.CANFD_BRS_TXM0 = ENABLE;
    CANFD_InitSturcture.CANFD_BRS_TXM1 = ENABLE;
    CANFD_InitSturcture.CANFD_BRS_TXM2 = ENABLE;
    CANFD_Init(CAN1, &CANFD_InitSturcture);

    CAN_FilterInitSturcture.CAN_FilterNumber = 0;
    CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitSturcture.CAN_FilterIdHigh = (CANFD_IAP_RX_STDID << 5);
    CAN_FilterInitSturcture.CAN_FilterIdLow = 0;
    CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0;
    CAN_FilterInitSturcture.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitSturcture.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitSturcture);

    CANFD_ReceiveFIFO_DMAAdr(CAN1, CAN_FIFO0, (u32)&CanFDRxStructure.Data[0]);
    CANFD_IAP_StreamReset();
}

void CANFD_Rx_Deal(void)
{
    u8 i, s;

    if(CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
    {
        return;
    }

    if(CANFD_Receive(CAN1, CAN_FIFO0, &CanFDRxStructure) != READY)
    {
        return;
    }

    if((CanFDRxStructure.IDE != CAN_Id_Standard) || (CanFDRxStructure.StdId != CANFD_IAP_RX_STDID))
    {
        return;
    }

    for(i = 0; i < CanFDRxStructure.DLC; i++)
    {
        u8 rx = CanFDRxStructure.Data[i];
        if(CanFD_IAP_StreamState == 0)
        {
            if(rx == Uart_Sync_Head1)
            {
                CanFD_IAP_StreamBuf[0] = rx;
                CanFD_IAP_StreamLen = 1;
                CanFD_IAP_StreamState = 1;
            }
        }
        else if(CanFD_IAP_StreamState == 1)
        {
            if(rx == Uart_Sync_Head2)
            {
                CanFD_IAP_StreamBuf[1] = rx;
                CanFD_IAP_StreamLen = 2;
                CanFD_IAP_StreamState = 2;
            }
            else if(rx == Uart_Sync_Head1)
            {
                CanFD_IAP_StreamBuf[0] = rx;
                CanFD_IAP_StreamLen = 1;
            }
            else
            {
                CANFD_IAP_StreamReset();
            }
        }
        else
        {
            if(CanFD_IAP_StreamLen >= sizeof(CanFD_IAP_StreamBuf))
            {
                CANFD_IAP_StreamReset();
                continue;
            }

            CanFD_IAP_StreamBuf[CanFD_IAP_StreamLen++] = rx;

            if(CanFD_IAP_StreamLen >= 4)
            {
                u8 expected = CANFD_IAP_PacketExpectedLen(CanFD_IAP_StreamBuf[2], CanFD_IAP_StreamBuf[3]);
                if(expected > sizeof(CanFD_IAP_StreamBuf))
                {
                    CANFD_IAP_StreamReset();
                    continue;
                }

                if(CanFD_IAP_StreamLen == expected)
                {
                    s = CANFD_IAP_PacketDeal(CanFD_IAP_StreamBuf, CanFD_IAP_StreamLen);
                    if(s != ERR_End)
                    {
                        CANFD_IAP_SendAck(s);
                    }
                    CANFD_IAP_StreamReset();
                }
                else if(CanFD_IAP_StreamLen > expected)
                {
                    CANFD_IAP_StreamReset();
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      CH32_IAP_Program
 *
 * @brief   adr - the date address
 *          buf - the date buffer
 *
 * @return  none
 */
void CH32_IAP_Program(u32 adr, u32* buf)
{
    u8 i;

    FLASH_BufReset();
    for(i=0; i<64; i++){
        FLASH_BufLoad(adr+4*i, buf[i]);
    }
    FLASH_ProgramPage_Fast(adr);
}

/*********************************************************************
 * @fn      RecData_Deal
 *
 * @brief   USB deal data
 *
 * @return  ERR_ERROR - ERROR
 *          ERR_SUCCESS - SUCCESS
 *          ERR_End - End
 */
u8 RecData_Deal(void)
{
     u8 i, s, Lenth;

     Lenth = isp_cmd_t->other.buf[1];

     switch ( isp_cmd_t->other.buf[0]) {
     case CMD_IAP_ERASE:
         FLASH_Unlock_Fast();
         s = ERR_SUCCESS;
         break;

     case CMD_IAP_PROM:
         for (i = 0; i < Lenth; i++) {
             Fast_Program_Buf[CodeLen + i] = isp_cmd_t->program.data[i];
         }
         CodeLen += Lenth;
         if (CodeLen >= 256) {
             FLASH_Unlock_Fast();
             FLASH_ErasePage_Fast(Program_addr);
             CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
             CodeLen -= 256;
             for (i = 0; i < CodeLen; i++) {
                 Fast_Program_Buf[i] = Fast_Program_Buf[256 + i];
             }

             Program_addr += 0x100;

         }
         s = ERR_SUCCESS;
         break;

     case CMD_IAP_VERIFY:
         if (Verify_Star_flag == 0) {
             Verify_Star_flag = 1;
            if(CodeLen != 0)
            {
                for (i = 0; i < (256 - CodeLen); i++) {
                    Fast_Program_Buf[CodeLen + i] = 0xff;
                }

                FLASH_ErasePage_Fast(Program_addr);
                CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
                CodeLen = 0;             
            }
         }

         s = ERR_SUCCESS;
         for (i = 0; i < Lenth; i++) {
             if (isp_cmd_t->verify.data[i] != *(u8*) (Verify_addr + i)) {
                 s = ERR_ERROR;
                 break;
             }
         }

         Verify_addr += Lenth;

         break;

     case CMD_IAP_END:
         Verify_Star_flag = 0;
         End_Flag = 1;
         Program_addr = FLASH_Base;
         Verify_addr = FLASH_Base;
         FLASH_ErasePage_Fast(CalAddr & 0xFFFFFF00);
         FLASH->CTLR |= ((uint32_t)0x00008000);  //FLASH_Lock_Fast
         FLASH->CTLR |= ((uint32_t)0x00000080);  //FLASH_Lock
         s = ERR_End;
         break;

     case CMD_JUMP_IAP:

         s = ERR_SUCCESS;
         break;
     default:
         s = ERR_ERROR;
         break;
     }

     return s;
}

/*********************************************************************
 * @fn      UART_RecData_Deal
 *
 * @brief   UART deal data
 *
 * @return  ERR_ERROR - ERROR
 *          ERR_SUCCESS - SUCCESS
 *          ERR_End - End
 */
u8 UART_RecData_Deal(void)
{
    u8 i, s, Lenth;

    Lenth = isp_cmd_t->UART.Len;
    switch ( isp_cmd_t->UART.Cmd) {
    case CMD_IAP_ERASE:

        FLASH_Unlock_Fast();
        s = ERR_SUCCESS;
        break;

    case CMD_IAP_PROM:
        for (i = 0; i < Lenth; i++) {
            Fast_Program_Buf[CodeLen + i] = isp_cmd_t->UART.data[i];
        }
        CodeLen += Lenth;
        if (CodeLen >= 256) {
            FLASH_Unlock_Fast();
            FLASH_ErasePage_Fast(Program_addr);
            CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
            CodeLen -= 256;
            for (i = 0; i < CodeLen; i++) {
                Fast_Program_Buf[i] = Fast_Program_Buf[256 + i];
            }

            Program_addr += 0x100;

        }
        s = ERR_SUCCESS;
        break;

    case CMD_IAP_VERIFY:

        if (Verify_Star_flag == 0)
        {
        Verify_Star_flag = 1;
          if(CodeLen != 0)
          {
            for (i = 0; i < (256 - CodeLen); i++)
            {
                Fast_Program_Buf[CodeLen + i] = 0xff;
            }
            FLASH_ErasePage_Fast(Program_addr);
            CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
            CodeLen = 0;
          }
        }
        s = ERR_SUCCESS;
        for (i = 0; i < Lenth; i++) {
            if (isp_cmd_t->UART.data[i] != *(u8*) (Verify_addr + i)) {
                s = ERR_ERROR;
                break;
            }
        }

        Verify_addr += Lenth;
        break;

    case CMD_IAP_END:
        Verify_Star_flag = 0;
        End_Flag = 1;
        Program_addr = FLASH_Base;
        Verify_addr = FLASH_Base;
        FLASH_ErasePage_Fast(CalAddr & 0xFFFFFF00);
        FLASH->CTLR |= ((uint32_t)0x00008000);
        FLASH->CTLR |= ((uint32_t)0x00000080);

        s = ERR_End;
        break;

    case CMD_JUMP_IAP:

        s = ERR_SUCCESS;
        break;
    default:
        s = ERR_ERROR;
        break;
    }

    return s;
}
/*********************************************************************
 * @fn      GPIO_Cfg_init
 *
 * @brief   GPIO init
 *
 * @return  none
 */
void GPIO_Cfg_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      GPIO_Cfg_Float
 *
 * @brief   GPIO float
 *
 * @return  none
 */
void GPIO_Cfg_Float(void)
{
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOC);
    GPIO_AFIODeInit();
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB | RCC_PB2Periph_GPIOC, DISABLE);
}

/*********************************************************************
 * @fn      PA0_Check
 *
 * @brief   Check PA0 state
 *
 * @return  1 - IAP
 *          0 - APP
 */
u8 PA0_Check(void)
{
    u8 i, cnt=0;

    GPIO_Cfg_init();

    for(i=0; i<10; i++){
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0) cnt++;
        Delay_Ms(5);
    }

    if(cnt>6) return 0;
    else return 1;
}

/*********************************************************************
 * @fn      USART2_CFG
 *
 * @brief   baudrate:UART2 baudrate
 *
 * @return  none
 */
void USART2_CFG(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_PB2PeriphClockCmd( RCC_PB2Periph_GPIOA, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_USART2,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
}
/*********************************************************************
 * @fn      UART2_SendMultiyData
 *
 * @brief   Deal device Endpoint 3 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void UART2_SendMultiyData(u8* pbuf, u8 num)
{
    u8 i = 0;

    while(i<num)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, pbuf[i]);
        i++;
    }
}
/*********************************************************************
 * @fn      UART2_SendMultiyData
 *
 * @brief   USART2 send date
 *
 * @param   pbuf - Packet to be sent
 *          num - the number of date
 *
 * @return  none
 */
void UART2_SendData(u8 data)
{
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, data);
}

/*********************************************************************
 * @fn      Uart2_Rx
 *
 * @brief   Uart2 receive date
 *
 * @return  none
 */
u8 Uart2_Rx(void)
{
    while( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData( USART2);
}

/*********************************************************************
 * @fn      UART_Rx_Deal
 *
 * @brief   UART Rx data deal
 *
 * @return  none
 */
void UART_Rx_Deal(void)
{
    u8 i, s;
    u16 Data_add = 0;

    if (Uart2_Rx() == Uart_Sync_Head1)
    {
        if (Uart2_Rx() == Uart_Sync_Head2)
        {
            isp_cmd_t->UART.Cmd = Uart2_Rx();
            Data_add += isp_cmd_t->UART.Cmd;
            isp_cmd_t->UART.Len = Uart2_Rx();
            Data_add += isp_cmd_t->UART.Len;

            if(isp_cmd_t->UART.Cmd == CMD_IAP_ERASE ||isp_cmd_t->UART.Cmd == CMD_IAP_VERIFY)
            {
                isp_cmd_t->other.buf[2] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[2];
                isp_cmd_t->other.buf[3] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[3];
                isp_cmd_t->other.buf[4] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[4];
                isp_cmd_t->other.buf[5] = Uart2_Rx();
                Data_add += isp_cmd_t->other.buf[5];
            }
            if ((isp_cmd_t->other.buf[0] == CMD_IAP_PROM) || (isp_cmd_t->other.buf[0] == CMD_IAP_VERIFY))
            {
                for (i = 0; i < isp_cmd_t->UART.Len; i++) {
                    isp_cmd_t->UART.data[i] = Uart2_Rx();
                    Data_add += isp_cmd_t->UART.data[i];
                }
            }
            if (Uart2_Rx() == (uint8_t)(Data_add & 0xFF))
            {
                if(Uart2_Rx() == (uint8_t)(Data_add >>8))
                {
                    if (Uart2_Rx() == Uart_Sync_Head2)
                    {
                        if (Uart2_Rx() == Uart_Sync_Head1)
                        {
                            s = UART_RecData_Deal();

                            if (s != ERR_End)
                            {
                                UART2_SendData(Uart_Sync_Head1);
                                UART2_SendData(Uart_Sync_Head2);
                                UART2_SendData(0x00);
                                if (s == ERR_ERROR)
                                {
                                    UART2_SendData(0x01);
                                }
                                else
                                {
                                    UART2_SendData(0x00);
                                }
                                UART2_SendData(Uart_Sync_Head2);
                                UART2_SendData(Uart_Sync_Head1);
                            }
                        }
                    }
                }
            }
        }
    }
}

void SW_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      SW_Handler
 *
 * @brief   This function handles Software exception.
 *
 * @return  none
 */
void SW_Handler(void) {
    __asm("li  a6, 0x5000");
    __asm("jr  a6");

    while(1);
}
