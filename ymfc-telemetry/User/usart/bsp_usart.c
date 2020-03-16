#include "bsp_usart.h"

//static void NVIC_Configuration(void)
//{
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//	// 嵌套向量中断控制器组选择
//	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
//	// 抢占优先级
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	// 子优先级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	// 使能中断
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	// 初始化配置NVIC
//	NVIC_Init(&NVIC_InitStructure);
//}

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// 打开串口 GPIO 的时钟
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

	// 打开串口外设的时钟
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// 将 USART Tx 的 GPIO 配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	// 将 USART Rx 的 GPIO 配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* 配置串口的工作模式 */
	// 配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// 配置帧数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	// 串口中断优先级配置
//	NVIC_Configuration();

	// 使能串口接收中断
//	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	// 使能串口
	USART_Cmd(DEBUG_USARTx, ENABLE);
}

/**
 * 发送一个字节
 */
void Usart_SendByte(USART_TypeDef * pUSARTx, uint8_t data)
{
	USART_SendData(pUSARTx, data);
	while ( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
 * 发送字符串
 */
void Usart_SendStr(USART_TypeDef * pUSARTx, const char *str)
{
	for (; *str != '\0'; )
		Usart_SendByte(pUSARTx, *str++);
	while ( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET );
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __in_getchar(FILE *f)

#else
#define PUTCHAR_PROTOTYPE int fputc(int c, FILE *f)
$define GETCHAR_PROTOTYPE int fgetc(FILE *f)

#endif

/**
 * 重定向C库函数 printf 到串口
 * 重定向后可以使用 printf 函数
 */
int _write(int fd, char *pBuffer, int size)
{
	for (int i = 0; i < size; i++)
	{
		USART_SendData(DEBUG_USARTx, (uint8_t)(*(pBuffer + i)));
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);
	}
	return size;
}

int getchar(void)
{
	while ( USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);
	return (uint8_t)USART_ReceiveData(DEBUG_USARTx);
}
