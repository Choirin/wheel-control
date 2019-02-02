/*
* The STM32 makes receiving chars into a large circular buffer simple
* and requires no CPU time. The UART receiver DMA must be setup as CIRCULAR.
*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#define CIRC_BUF_SZ       64  /* must be power of two */
static uint8_t rx_dma_circ_buf[CIRC_BUF_SZ];
static UART_HandleTypeDef *huart_cobs;
static uint32_t rd_ptr;

#define DMA_WRITE_PTR ( (CIRC_BUF_SZ - huart_cobs->hdmarx->Instance->NDTR) & (CIRC_BUF_SZ - 1) )

void msgrx_init(UART_HandleTypeDef *huart)
{
    huart_cobs = huart;
    HAL_UART_Receive_DMA(huart_cobs, rx_dma_circ_buf, CIRC_BUF_SZ);
    rd_ptr = 0;
}

bool msgrx_circ_buf_is_empty(void) {
    if(rd_ptr == DMA_WRITE_PTR) {
        return true;
    }
    return false;
}

uint8_t msgrx_circ_buf_get(void) {
    uint8_t c = 0;
    if(rd_ptr != DMA_WRITE_PTR) {
        c = rx_dma_circ_buf[rd_ptr++];
        rd_ptr &= (CIRC_BUF_SZ - 1);
    }
    return c;
}