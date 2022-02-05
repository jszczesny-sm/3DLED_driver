// Peripheral usage
#include "stm32f4xx_hal.h"

#include "ws2812b.h"



static inline uint8_t scale8(uint8_t x, uint8_t scale) {
  return ((uint16_t)x * scale) >> 8;
}

// Set a single color (RGB) to index
void led_set_RGB(Layers *layer, uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
	layer->rgb_arr[3 * index] = scale8(g, 0xB0); // g;
	layer->rgb_arr[3 * index + 1] = r;
	layer->rgb_arr[3 * index + 2] = scale8(b, 0xF0); // b;
}

// Set all colors to RGB
void led_set_all_RGB(Layers *layer, uint8_t r, uint8_t g, uint8_t b) {
  for(uint_fast8_t i = 0; i < NUM_PIXELS; ++i) led_set_RGB(layer, i, r, g, b);
}

// Shuttle the data to the LEDs!
void led_render(Layers *layer) {
  if(layer->wr_buf_p != 0 || layer->dma->State != HAL_DMA_STATE_READY) {
    // Ongoing transfer, cancel!
    for(uint8_t i = 0; i < WR_BUF_LEN; ++i) layer->wr_buf[i] = 0;
    layer->wr_buf_p = 0;
    HAL_TIM_PWM_Stop_DMA(layer->timer, layer->channel);
    return;
  }
  // Ooh boi the first data buffer half (and the second!)
  for(uint_fast8_t i = 0; i < 8; ++i) {
	  layer->wr_buf[i     ] = PWM_LO << (((layer->rgb_arr[0] << i) & 0x80) > 0);
	  layer->wr_buf[i +  8] = PWM_LO << (((layer->rgb_arr[1] << i) & 0x80) > 0);
	  layer->wr_buf[i + 16] = PWM_LO << (((layer->rgb_arr[2] << i) & 0x80) > 0);
	  layer->wr_buf[i + 24] = PWM_LO << (((layer->rgb_arr[3] << i) & 0x80) > 0);
	  layer->wr_buf[i + 32] = PWM_LO << (((layer->rgb_arr[4] << i) & 0x80) > 0);
	  layer->wr_buf[i + 40] = PWM_LO << (((layer->rgb_arr[5] << i) & 0x80) > 0);
  }
  HAL_TIM_PWM_Start_DMA(layer->timer, layer->channel, (uint32_t *)layer->wr_buf, WR_BUF_LEN);
  layer->wr_buf_p = 2; // Since we're ready for the next buffer
}

