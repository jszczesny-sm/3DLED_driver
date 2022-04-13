#ifndef _LED_DRIVER_SK6812
#define _LED_DRIVER_SK6812

#include <stdint.h>

#define PWM_HI (66)
#define PWM_LO (33)

// LED parameters
#define NUM_BPP (3) // WS2812B
#define NUM_PIXELS (256)
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)



// LED write buffer
#define WR_BUF_LEN (NUM_BPP * 8 * 2)

typedef struct Layers_struct
{
	TIM_HandleTypeDef *timer;
	DMA_HandleTypeDef *dma;
	uint8_t channel;

	// LED color buffer
	uint8_t rgb_arr[NUM_BYTES];

	// LED write buffer
	uint16_t wr_buf[WR_BUF_LEN];
	uint_fast8_t wr_buf_p;

}Layers;

void led_set_RGB(Layers *layer, uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void led_set_all_RGB(Layers *layer, uint8_t r, uint8_t g, uint8_t b);
void led_render(Layers *layer);

#endif
