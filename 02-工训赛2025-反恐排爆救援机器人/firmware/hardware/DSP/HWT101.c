#include "HWT101.h"

__IO bool HWT101_rxFrameFlag = false;
__IO uint8_t HWT101_rxCmd[HWT101_FIFO_SIZE] = {0};   // DMA接收缓冲区


/*--------------命令集--------------*/
static const uint8_t unlock_reg[] = {0xFF,0xAA,0x69,0x88,0xB5};
static const uint8_t reset_Z[]    = {0xFF,0xAA,0x76,0x00,0x00};
static const uint8_t set_1000Hz[] = {0xFF,0xAA,0x03,0x0D,0x00};
static const uint8_t set_baud[]   = {0xFF,0xAA,0x04,0x07,0x00};
static const uint8_t save_cfg[]   = {0xFF,0xAA,0x00,0x00,0x00};
static const uint8_t restart[]    = {0xFF,0xAA,0x00,0xFF,0x00};
static const uint8_t calsw_in[]   = {0xFF,0xAA,0x01,0x01,0x00};
static const uint8_t calsw_out[]  = {0xFF,0xAA,0x01,0x00,0x00};

/*-------------- ���ͷ�װ --------------*/
static void SendByte(uint8_t b)
{
    USART_SendData(USART2, b);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}
static void SendArray(const uint8_t *a, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) SendByte(a[i]);
    SendByte(0x0D); SendByte(0x0A);
    volatile uint32_t t = 168000; while(t--); // �� 200 ms @168 MHz
}

void HWT_Settings(void)
{
    SendArray(unlock_reg, 5); volatile uint32_t t=168000; while(t--);
    SendArray(reset_Z,    5); t=168000; while(t--);
    SendArray(set_1000Hz,  5); t=168000; while(t--);
    SendArray(set_baud,   5); t=168000; while(t--);
    SendArray(save_cfg,   5); t=168000; while(t--);
    SendArray(restart,    5); t=168000; while(t--);
}
void HWT_CALSW_IN(void)  { SendArray(calsw_in,  5); }
void HWT_CALSW_OUT(void) { SendArray(calsw_out, 5); }

/*=====================================================
 *  NVIC 初始化：仅使能 USART2 中断
 *====================================================*/
void HWT101_nvic_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*=====================================================
 *  时钟/引脚复用/外设时钟 初始化
 *====================================================*/
void HWT101_clock_init(void)
{
    /* 使能 SYSCFG 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* 使能 GPIOA 时钟（PA2/PA3） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* 使能 USART2 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* 使能 DMA1 时钟（USART2_RX 使用 DMA1_Stream5） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* PA2/PA3 复用为 USART2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  // PA2 -> TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  // PA3 -> RX
}

/*=====================================================
 *  USART2 + DMA1_Stream5 初始化
 *====================================================*/
void HWT101_usart_init(void)
{
    HWT101_clock_init();

    /*---------------- GPIO 初始化 ----------------*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PA2 USART2_TX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 USART2_RX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*---------------- USART2 配置 ----------------*/
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 230400;
    USART_InitStructure.USART_WordLength        = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits          = USART_StopBits_1;
    USART_InitStructure.USART_Parity            = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode              = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStructure);

    /*---------------- DMA1_Stream5 RX 配置 ----------------*/
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Stream5);
	
    while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);

    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;          // USART2_RX
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)HWT101_rxCmd;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = HWT101_FIFO_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);

    /*---------------- 中断 & 使能 ----------------*/
    HWT101_nvic_init();
	
	USART2->SR;
	USART2->DR;
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);

    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Stream5, ENABLE);
	
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    USART_Cmd(USART2, ENABLE);
}

/*=====================================================
 *  板级初始化
 *====================================================*/
void HWT101_board_init(void)
{
    HWT101_usart_init();
	HWT_Settings();
}

//接收状态

int16_t mpu;

uint8_t Serial_RxFla_Data_processing=0;			//定义接收边框坐标数据包标志位
uint8_t Serial_RxFlag_mpu=0;			//定义接收边框坐标数据包标志位
uint8_t Serial_RxFlag_mpu_1=0;			//定义接收边框坐标数据包标志位
uint8_t Serial_RxFlag_mpu_2=0;			//定义接收边框坐标数据包标志位

uint8_t pRxPacket_mpu = 0;		//边框//定义表示当前接收数据位置的静态变量
int16_t Serial_RxPacket_Mpu[40];				//定义接收数据包数组，数据包格式

float Yaw=0;

__IO uint8_t HWT101_rxCount = 0;

/*=====================================================
 *  数据解析
 *====================================================*/
void MPUData_processing(void)
{
		Yaw = (short)(((short)HWT101_rxCmd[18] << 8) | HWT101_rxCmd[17]) / 32768.0f * 180.0f;
}

/*=====================================================
 *  简易收发（保持与原 API 一致）
 *====================================================*/
void HWT101(void)
{
    USART_Cmd(USART2, ENABLE);
    delay_ms(20);
    MPUData_processing();
    USART_Cmd(USART2, DISABLE);
}

/*=====================================================
 *  USART2 中断服务程序（原 UART4_IRQHandler 改名）
 *====================================================*/
void USART2_IRQHandler(void)
{
    /* 错误处理 */
    if (USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET) {
        USART_ReceiveData(USART2);
        USART_ClearFlag(USART2, USART_FLAG_PE);
    }
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET) {
        USART_ReceiveData(USART2);
        USART_ClearFlag(USART2, USART_FLAG_ORE);
    }
    if (USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET) {
        USART_ReceiveData(USART2);
        USART_ClearFlag(USART2, USART_FLAG_FE);
    }

    /* IDLE 中断处理 */
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
        volatile uint16_t temp;
        temp = USART2->SR;
        temp = USART2->DR;
        (void)temp;

        DMA_Cmd(DMA1_Stream5, DISABLE);
        while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);

        DMA_ClearFlag(DMA1_Stream5,
                      DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 |
                      DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 |
                      DMA_FLAG_FEIF5);

        DMA1_Stream5->M0AR  = (uint32_t)HWT101_rxCmd;
        DMA_SetCurrDataCounter(DMA1_Stream5, HWT101_FIFO_SIZE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
}
