#include "Serial.h"

uint8_t Serial_TxPacket[4];

int RXD_I[RXD_LEN];                    //用于存储需要使用的整数数据
float RXD_F[RXD_LEN];				   //用于存储需要使用的浮点数数据
uint8_t Serial_RxFlag = 0;             //数据接收完毕标志位
static uint16_t rx_index = 0;          
char Rx_data[MAX_PARTS][MAX_LEN];      
static char  rx_buffer[RX_BUF_SIZE];

volatile uint8_t chassis_cmd = 0;
volatile uint8_t chassis_cmd_flag = 0;

/*-----------------中间处理函数-----------------*/
static char *my_strdup(const char *s)
{
    char *d = (char *)malloc(strlen(s) + 1);
    if (d) strcpy(d, s);
    return d;
}

static void cut_string(const char *str, char delimiter,
                       char result[MAX_PARTS][MAX_LEN])
{
    char *token;
    char *str_copy = my_strdup(str);
    int count = 0;

    token = strtok(str_copy, &delimiter);
    while (token && count < MAX_PARTS)
    {
        strncpy(result[count++], token, MAX_LEN - 1);
        result[count - 1][MAX_LEN - 1] = '\0';
        token = strtok(NULL, &delimiter);
    }
    free(str_copy);
}
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

/*----------------- 数据转换函数 -----------------*/
void CharToInt(char Rx_data[MAX_PARTS][MAX_LEN],int RXD_I[RXD_LEN])//将字符串数组转换成int类型数组
{
 	for(int i=0;i<RXD_LEN;i++)
 {
	RXD_I[i]=atoi(Rx_data[i]);
 }
}

void CharToFloat(char Rx_data[MAX_PARTS][MAX_LEN],float RXD_F[RXD_LEN])
{
	for(int i=0;i<RXD_LEN;i++)
 {
	RXD_F[i]=atof(Rx_data[i]);
 }
}
//////////////////////////////////////////////////////////////////////////////////

/*----------------- 中断接收 -----------------*/
void SERIAL_IRQHandler(void)
{
    if (USART_GetITStatus(SERIAL_USART, USART_IT_RXNE) != RESET)
    {
        char data = USART_ReceiveData(SERIAL_USART);
        /* 底盘指令检测 */
        if ((uint8_t)data >= 0xA0 && (uint8_t)data <= 0xA3) {
            chassis_cmd = (uint8_t)data;
            chassis_cmd_flag = 1;
            return;
        }
        if (rx_index < RX_BUF_SIZE - 1)
        {
            rx_buffer[rx_index++] = data;
            if (data == '\n')
            {
                rx_buffer[rx_index] = '\0';
                Serial_RxFlag = 1;
                rx_index = 0;
                cut_string(rx_buffer, ',', Rx_data);
            }
        }
        else
        {
            rx_index = 0;
        }
    }
}

/* 底盘应答发送 */
void Chassis_SendRSP(uint8_t rsp)
{
    Serial_Vision_SendByte(rsp);
}

