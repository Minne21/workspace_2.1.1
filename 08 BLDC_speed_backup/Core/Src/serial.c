/*

* uart_dma.c
*
* Non-blocking UART + DMA implementation using STM32 HAL
*
* * Designed for USART2 by default (huart2). Adjust the extern if needed.
* * Uses HAL_UARTEx_ReceiveToIdle_DMA for RX (available in HAL).
* * TX uses HAL_UART_Transmit_DMA; when TX completes, we restart DMA with next chunk.
*
* Notes:
* * CubeMX must configure DMA for USART2 RX and TX.
* * The RX path moves the bytes reported by the HAL "RxEvent" callback into a software FIFO.
* * The TX path is a software ring buffer; UART_DMA_Send copies into it and triggers DMA if idle.
*
* Integration:
* * Add this file and uart_dma.h to your project.
* * Call UART_DMA_Init() after MX_USART2_UART_Init();
* * Ensure vector table calls HAL callbacks (default CubeMX does).
* * If your project uses different UART handle names, replace 'huart2' usage or change the extern below.
    */

#include "serial.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

// Change the UART handle if you use another UART instance
extern UART_HandleTypeDef huart2;      // created by CubeMX
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* Buffers */
static uint8_t rx_dma_buf[UART_DMA_RX_DMA_BUF_SIZE];   // memory used by DMA (circular)
static uint8_t rx_fifo[UART_DMA_RX_FIFO_SIZE];        // software FIFO for received bytes
static volatile size_t rx_fifo_head = 0;
static volatile size_t rx_fifo_tail = 0;

static uint8_t tx_buf[UART_DMA_TX_BUF_SIZE];          // software TX ring
static volatile size_t tx_head = 0;
static volatile size_t tx_tail = 0;
static volatile uint8_t tx_dma_active = 0;

/* Helper macros */
#define INC_WRAP(idx, size)  ( ((idx) + 1) % (size) )
#define MIN(a,b) (( (a) < (b) ) ? (a) : (b))

/* Forward declarations of weak callbacks (app may override) */
__weak void UART_DMA_RxPacketCallback(const uint8_t *data, size_t len) { (void)data; (void)len; }
__weak void UART_DMA_TxCompleteCallback(void) {}

/* Internal helpers ---------------------------------------------------------*/
static inline size_t rx_fifo_free_space(void)
{
	if (rx_fifo_head >= rx_fifo_tail) return (UART_DMA_RX_FIFO_SIZE - (rx_fifo_head - rx_fifo_tail) - 1);
	return (rx_fifo_tail - rx_fifo_head - 1);
}

static inline size_t rx_fifo_count(void)
{
	if (rx_fifo_head >= rx_fifo_tail) return (rx_fifo_head - rx_fifo_tail);
	return (UART_DMA_RX_FIFO_SIZE - (rx_fifo_tail - rx_fifo_head));
}

static inline size_t tx_free_space(void)
{
	if (tx_head >= tx_tail) return (UART_DMA_TX_BUF_SIZE - (tx_head - tx_tail) - 1);
	return (tx_tail - tx_head - 1);
}

static inline size_t tx_count(void)
{
	if (tx_head >= tx_tail) return (tx_head - tx_tail);
	return (UART_DMA_TX_BUF_SIZE - (tx_tail - tx_head));
}

/* Copy bytes into rx FIFO (used by RxEvent callback) */
static void rx_fifo_push_bytes(const uint8_t *src, size_t len)
{
	size_t free = rx_fifo_free_space();
	size_t tocpy = MIN(free, len);
	for (size_t i = 0; i < tocpy; ++i)
	{
		rx_fifo[rx_fifo_head] = src[i];
		rx_fifo_head = (rx_fifo_head + 1) % UART_DMA_RX_FIFO_SIZE;
	}
	UART_DMA_RxPacketCallback(src, tocpy);
}

/* Start next TX DMA transfer if there is queued data and TX is idle */
static void tx_start_dma_if_needed(void)
{
	if (tx_dma_active) return;
	size_t cnt = tx_count();
	if (cnt == 0) return;

	/* compute contiguous chunk from tail to end of buffer */
	size_t chunk;
	if (tx_tail < tx_head)
		chunk = tx_head - tx_tail;
	else
		chunk = UART_DMA_TX_BUF_SIZE - tx_tail;

	/* start DMA transmit */
	tx_dma_active = 1;
	if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&tx_buf[tx_tail], (uint16_t)chunk) != HAL_OK)
	{
		/* failure -> mark inactive so future attempts can try again */
		tx_dma_active = 0;
	}
}

/* Public API ---------------------------------------------------------------*/

HAL_StatusTypeDef UART_DMA_Init(void)
{
	/* Ensure the HAL UART handle is ready */
	if (huart2.Instance == NULL) return HAL_ERROR;

	/* Start Rx: ReceiveToIdle in circular buffer */
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, UART_DMA_RX_DMA_BUF_SIZE) != HAL_OK)
	{
		return HAL_ERROR;
	}
	/* Optionally disable half-transfer interrupt to reduce chattiness */
	if (huart2.hdmarx != NULL) __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

	/* flush FIFOs/indices */
	rx_fifo_head = rx_fifo_tail = 0;
	tx_head = tx_tail = 0;
	tx_dma_active = 0;
	return HAL_OK;
}

size_t UART_DMA_Send(const uint8_t *data, size_t len)
{
	if (data == NULL || len == 0) return 0;

	size_t queued = 0;
	for (size_t i = 0; i < len; ++i)
	{
		size_t next = (tx_head + 1) % UART_DMA_TX_BUF_SIZE;
		if (next == tx_tail)
		{
			/* buffer full - stop queuing (caller may retry) */
			break;
		}
		tx_buf[tx_head] = data[i];
		tx_head = next;
		queued++;
	}

	/* If TX DMA idle, start a transfer */
	tx_start_dma_if_needed();
	return queued;
}


size_t UART_DMA_Available(void)
{
	return rx_fifo_count();
}

int UART_DMA_Peek(void)
{
	if (rx_fifo_count() == 0) return -1;
	return rx_fifo[rx_fifo_tail];
}

int UART_DMA_Read(void)
{
	if (rx_fifo_count() == 0) return -1;
	int c = rx_fifo[rx_fifo_tail];
	rx_fifo_tail = (rx_fifo_tail + 1) % UART_DMA_RX_FIFO_SIZE;
	return c;
}

void UART_DMA_FlushRx(void)
{
	rx_fifo_head = rx_fifo_tail = 0;
}

/* Simple printf-like helper (uses local stack buffer) */
size_t UART_DMA_Printf(const char *fmt, ...)
{
	char tmp[256];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
	va_end(ap);
	if (n <= 0) return 0;
	if ((size_t)n > sizeof(tmp)) n = sizeof(tmp);
	return UART_DMA_Send((const uint8_t*)tmp, (size_t)n);
}

/* Exposure helpers */
const uint8_t *UART_DMA_RxDmaBufferPtr(void) { return rx_dma_buf; }
size_t UART_DMA_RxDmaBufferSize(void) { return (size_t)UART_DMA_RX_DMA_BUF_SIZE; }

/* HAL Callbacks ------------------------------------------------------------*/

/*

* HAL will call this when RxToIdle event occurs (HAL_UARTEx_ReceiveToIdle_DMA)
* The Size parameter provided by HAL indicates how many new bytes are available
* in the DMA buffer since the last callback.
*
* We must move those bytes into the rx FIFO for the application to consume.
  */
static uint16_t old_pos = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                uint16_t pos)
{
    if (huart != &huart2)
        return;

    if (pos > old_pos)
    {
        rx_fifo_push_bytes(&rx_dma_buf[old_pos],
                           pos - old_pos);
    }
    else
    {
        rx_fifo_push_bytes(&rx_dma_buf[old_pos],
                           UART_DMA_RX_DMA_BUF_SIZE - old_pos);

        if (pos > 0)
        {
            rx_fifo_push_bytes(&rx_dma_buf[0],
                               pos);
        }
    }

    old_pos = pos;
}
/*

* HAL UART TX complete callback - invoked when a DMA TX finishes.
* We advance the tail by the transmitted size (accessible via huart->TxXferSize)
* and trigger the next chunk if any.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		/* Advance tail by last transfer size */
		uint16_t last_size = huart->TxXferSize;
		tx_tail = (tx_tail + last_size) % UART_DMA_TX_BUF_SIZE;
		tx_dma_active = 0;

		/* If more data present -> start next chunk */
		if (tx_count() > 0) tx_start_dma_if_needed();
		else UART_DMA_TxCompleteCallback();
	}
}

/* Optional: handle error callback to recover TX DMA state */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		/* try to recover TX DMA flag and restart if data pending */
		tx_dma_active = 0;
		if (tx_count() > 0) tx_start_dma_if_needed();
	}
}


/**
 * @brief  Safely processes numeric RPM input, echo-types it cleanly,
 *         and ensures no ghost digits remain in the buffer.
 */

#define CMD_BUF_SIZE 16

volatile int target_rpm_cmd = 0;

void UART_ProcessCommands(void)
{
    static char cmd_buf[CMD_BUF_SIZE];
    static uint8_t idx = 0;

    while (UART_DMA_Available())
    {
        int c = UART_DMA_Read();

        if (c < 0)
            return;

        char ch = (char)c;

        /* ENTER */
        if (ch == '\r' || ch == '\n')
        {
            UART_DMA_Printf("\r\n");

            if (idx > 0)
            {
                cmd_buf[idx] = 0;

                target_rpm_cmd = atoi(cmd_buf);

                UART_DMA_Printf(
                    "Target RPM = %d\r\n",
                    target_rpm_cmd);
            }

            memset(cmd_buf, 0, sizeof(cmd_buf));
            idx = 0;

            UART_DMA_Printf("Enter target RPM: ");

            continue;
        }

        /* BACKSPACE */
        if (ch == '\b' || ch == 127)
        {
            if (idx > 0)
            {
                idx--;

                UART_DMA_Printf("\b \b");
            }

            continue;
        }

        /* DIGITS ONLY */
        if (ch >= '0' && ch <= '9')
        {
            if (idx < CMD_BUF_SIZE - 1)
            {
                cmd_buf[idx++] = ch;

                /* Echo */
                UART_DMA_Send((uint8_t *)&ch, 1);
            }
        }
    }
}
/* End of file */
