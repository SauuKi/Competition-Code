#include "Serial.h"

uint8_t Serial_TxPacket[4];

int RXD_I[RXD_LEN];                  //功能码1帧中的 dx/dy
int TYPE_I[TYPE_LEN];                  //功能码4帧中的位置信息
uint8_t Serial_RxFlag = 0;           //一帧接收完毕标志位
uint8_t Func_Code= 0;                //功能码
static uint8_t rx_frame[FRAME_LEN];  //存放最近一帧视觉数据
static uint8_t rx_index = 0;         //当前帧内字节计数
///////////////////////////////////////////////////////////////////////

/*-----------------初始化函数-----------------*/
void Serial_Init(void)
{
    /* 1. 时钟开启 ----------------------------------------------------------*/
    RCC_AHB1PeriphClockCmd(SERIAL_GPIO_CLK, ENABLE);   /* GPIO 在 AHB1 */
    RCC_APB1PeriphClockCmd(SERIAL_USART_CLK, ENABLE);  /* USART3 在 APB1 */

    /* 2. GPIO 配置 ---------------------------------------------------------*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /* TX : PC10 复用推挽 */
    GPIO_InitStructure.GPIO_Pin   = SERIAL_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;          /* F4 必须设为 AF */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      /* 50 MHz 足够 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         /* 推挽输出 */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;          /* 上拉 */
    GPIO_Init(SERIAL_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(SERIAL_GPIO_PORT, GPIO_PinSource10, GPIO_AF_USART3);

    /* RX : PC11 复用上拉输入 */
    GPIO_InitStructure.GPIO_Pin   = SERIAL_RX_PIN;
    GPIO_Init(SERIAL_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(SERIAL_GPIO_PORT, GPIO_PinSource11, GPIO_AF_USART3);

    /* 3. USART 参数 --------------------------------------------------------*/
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(SERIAL_USART, &USART_InitStructure);

    /* 4. 中断配置 ----------------------------------------------------------*/
    USART_ITConfig(SERIAL_USART, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
	
    /* 5. 使能串口 ----------------------------------------------------------*/
    USART_Cmd(SERIAL_USART, ENABLE);
}
///////////////////////////////////////////////////////////////////////////////

/*----------------- 对外函数 -----------------*/
void Serial_Vision_SendByte(uint8_t Byte)
{
    USART_SendData(SERIAL_USART, Byte);
    while (USART_GetFlagStatus(SERIAL_USART, USART_FLAG_TXE) == RESET);
}

void Serial_Vision_SendArray(uint8_t *Array, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
        Serial_Vision_SendByte(Array[i]);
}

void Serial_Vision_SendString(char *String)
{
    while (*String) Serial_Vision_SendByte(*String++);
}

static uint32_t Serial_Vision_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void Serial_Vision_SendNumber(uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
    {
        uint8_t digit = (Number / Serial_Vision_Pow(10, Length - i - 1)) % 10;
        Serial_Vision_SendByte(digit + '0');
    }
}

int fputc(int ch, FILE *f)
{
    Serial_Vision_SendByte(ch);
    return ch;
}

void Serial_Vision_Printf(char *format, ...)
{
    char buf[100];
    va_list ap;
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);
    Serial_Vision_SendString(buf);
}

void Serial_Vision_SendPacket(void)
{
    uint8_t txb[2];
    Serial_Vision_SendByte(0xFF);
    for (uint16_t w = 0; w < Tx_length; w++)
    {
        txb[0] = (Serial_TxPacket[w] >> 8) & 0xFF;
        txb[1] =  Serial_TxPacket[w] & 0xFF;
        Serial_Vision_SendArray(txb, 2);
    }
    Serial_Vision_SendByte(0xFE);
}
///////////////////////////////////////////////////////////////////////////////////

/*----------------- 视觉解包函数（任务侧调用） -----------------*/
void Serial_Vision_Unpack(void)
{
    uint16_t header = ((uint16_t)rx_frame[0] << 8) | rx_frame[1];
    uint16_t tail   = ((uint16_t)rx_frame[FRAME_LEN - 2] << 8) | rx_frame[FRAME_LEN - 1];
    if (header != FRAME_HEADER || tail != FRAME_TAIL) return;

    uint8_t sum = 0;
    for (uint8_t i = 2; i <= 10; i++) sum += rx_frame[i];
    if (sum != rx_frame[11]) return;

    if (rx_frame[2] != 1&&rx_frame[2] != 4) return;   //如果功能码为1或4，才进行解包

    Func_Code = rx_frame[2];                          //记录功能码，后续根据功能码进行不同处理

    if(Func_Code == 1)
    {
        RXD_I[0] = (int32_t)(((uint32_t)rx_frame[3] << 24) |
                            ((uint32_t)rx_frame[4] << 16) |
                            ((uint32_t)rx_frame[5] << 8)  |
                            rx_frame[6]);
        RXD_I[1] = (int32_t)(((uint32_t)rx_frame[7] << 24) |
                            ((uint32_t)rx_frame[8] << 16) |
                            ((uint32_t)rx_frame[9] << 8)  |
                            rx_frame[10]);
    }
    else if(Func_Code == 4)
    {
        TYPE_I[0] = (int32_t)(((uint32_t)rx_frame[3] << 24) |
                         ((uint32_t)rx_frame[4] << 16) |
                         ((uint32_t)rx_frame[5] << 8)  |
                         rx_frame[6]);
        TYPE_I[1] = (int32_t)(((uint32_t)rx_frame[7] << 24) |
                         ((uint32_t)rx_frame[8] << 16) |
                         ((uint32_t)rx_frame[9] << 8)  |
                         rx_frame[10]);
    }
}
//////////////////////////////////////////////////////////////////////////////////

/*----------------- 中断接收 -----------------*/
void SERIAL_IRQHandler(void)
{
    if (USART_GetITStatus(SERIAL_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(SERIAL_USART);

        /* 只负责拼出完整帧，解包放到任务侧 */
        if (rx_index == 0)
        {
            if (data == ((FRAME_HEADER >> 8) & 0xFF))
            {
                rx_frame[0] = data;
                rx_index = 1;
            }
        }
        else if (rx_index == 1)
        {
            if (data == (FRAME_HEADER & 0xFF))
            {
                rx_frame[1] = data;
                rx_index = 2;
            }
            else if (data != ((FRAME_HEADER >> 8) & 0xFF))
            {
                rx_index = 0;   //不是帧头，重新等待 0xAB
            }
            /* data==0xAB 时按连续帧头处理，继续等下一个字节 */
        }
        else
        {
            rx_frame[rx_index] = data;
            rx_index++;
            if (rx_index == FRAME_LEN)
            {
                rx_index = 0;
                Serial_RxFlag = 1;
            }
        }
    }
}


