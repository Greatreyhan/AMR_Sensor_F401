/*
 * H711.c
 *
 *  Created on: Jan 5, 2024
 *      Author: greatreyhan
 */

#include "HX711.h"

void hx711_init(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin){
  hx711->clk_gpio = clk_gpio;
  hx711->clk_pin = clk_pin;
  hx711->dat_gpio = dat_gpio;
  hx711->dat_pin = dat_pin;

  GPIO_InitTypeDef  gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = clk_pin;
  HAL_GPIO_Init(clk_gpio, &gpio);
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = dat_pin;
  HAL_GPIO_Init(dat_gpio, &gpio);

}

void set_scale(hx711_t *hx711, float Ascale, float Bscale){
	hx711->Ascale = Ascale;
	hx711->Bscale = Bscale;
}

void set_gain(hx711_t *hx711, uint8_t Again, uint8_t Bgain){
	switch (Again) {
			case 128:
				hx711->Again = 1;
				break;
			case 64:
				hx711->Again = 3;
				break;
		}
	hx711->Bgain = 2;
}

void set_offset(hx711_t *hx711, long offset, uint8_t channel){
	if(channel == CHANNEL_A) hx711->Aoffset = offset;
	else hx711->Boffset = offset;
}

uint8_t shiftIn(hx711_t *hx711, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
    	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, SET);
        if(bitOrder == 0)
            value |= HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) << i;
        else
            value |= HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) << (7 - i);
        HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, RESET);
    }
    return value;
}

bool is_ready(hx711_t *hx711) {
	if(HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_RESET){
		return 1;
	}
	return 0;
}

void wait_ready(hx711_t *hx711) {
	while (!is_ready(hx711)) {
		HAL_Delay(0);
	}
}

long read(hx711_t *hx711, uint8_t channel){
	wait_ready(hx711);
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	noInterrupts();

	data[2] = shiftIn(hx711, 1);
	data[1] = shiftIn(hx711, 1);
	data[0] = shiftIn(hx711, 1);

	uint8_t gain = 0;
	if(channel == 0) gain = hx711->Again;
	else gain = hx711->Bgain;

	for (unsigned int i = 0; i < gain; i++) {
		HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, SET);
		HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, RESET);
	}

	interrupts();

	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	value = ( (unsigned long)(filler) << 24
			| (unsigned long)(data[2]) << 16
			| (unsigned long)(data[1]) << 8
			| (unsigned long)(data[0]) );

	return (long)(value);
}

long read_average(hx711_t *hx711, int8_t times, uint8_t channel) {
	long sum = 0;
	for (int8_t i = 0; i < times; i++) {
		sum += read(hx711, channel);
		HAL_Delay(0);
	}
	return sum / times;
}

double get_value(hx711_t *hx711, int8_t times, uint8_t channel) {
	long offset = 0;
	if(channel == CHANNEL_A) offset = hx711->Aoffset;
	else offset = hx711->Boffset;
	return read_average(hx711, times, channel) - offset;
}

void tare(hx711_t *hx711, uint8_t times, uint8_t channel) {
	read(hx711, channel);
	double sum = read_average(hx711, times, channel);
	set_offset(hx711, sum, channel);
}

void tare_all(hx711_t *hx711, uint8_t times) {
	tare(hx711, times, CHANNEL_A);
	tare(hx711, times, CHANNEL_B);
}

float get_weight(hx711_t *hx711, int8_t times, uint8_t channel) {
	read(hx711, channel);
	float scale = 0;
	if(channel == CHANNEL_A) scale = hx711->Ascale;
	else scale = hx711->Bscale;
	return get_value(hx711, times, channel) / scale;
}

// User Defined Function

void hx711_start(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin){
//	snprintf(Buffer, sizeof(Buffer), "HX711 initialization\n\r");
//	HAL_UART_Transmit(&huart6,(uint8_t*)Buffer, strlen(Buffer), 100);

	/* Initialize the hx711 sensors */
	hx711_init(hx711, clk_gpio, clk_pin, dat_gpio, dat_pin);

	/* Configure gain for each channel (see datasheet for details) */
	set_gain(hx711, 128, 32);

	/* Set HX711 scaling factor (see README for procedure) */
	set_scale(hx711, 1, 1);

	/* Tare weight */
	tare_all(hx711, 10);

//	snprintf(Buffer, sizeof(Buffer), "HX711 module has been initialized\n");
//	HAL_UART_Transmit(&huart6,(uint8_t*)Buffer, strlen(Buffer), 100);
}

void hx711_calibration(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin){
//	snprintf(Buffer, sizeof(Buffer), "HX711 initialization\n\r");
//	HAL_UART_Transmit(&huart6,(uint8_t*)Buffer, strlen(Buffer), 100);
	/* Initialize the hx711 sensors */
	hx711_init(hx711, clk_gpio, clk_pin, dat_gpio, dat_pin);

	/* Configure gain for each channel (see datasheet for details) */
	set_gain(hx711, 128, 32);

	/* Set HX711 scaling factor (see README for procedure) */
	set_scale(hx711, 1, 1);

	/* Tare weight */
	tare_all(hx711, 10);

//	snprintf(Buffer, sizeof(Buffer), "HX711 module has been initialized\n");
//	HAL_UART_Transmit(&huart6,(uint8_t*)Buffer, strlen(Buffer), 100);
}

float hx711_measure_channel(hx711_t hx711, uint8_t channel){
	float weight = 0;
	// Measure the weight for channel A
	weight = get_weight(&hx711, 10, channel);
	// Weight cannot be negative
	//	weight = (weight < 0.00) ? 0 : weight;
	//	sprintf(buffer, "Weight : %lf \n",weight);

//	snprintf(Buffer, sizeof(Buffer), "Weight %f \r\n", weight);
//	HAL_UART_Transmit(&huart6,(uint8_t*)Buffer, strlen(Buffer), 100);
	return weight;
}

float hx711_measure_weight(hx711_t hx711){
	long weightA = 0;
	long weightB = 0;

	// Measure the weight for channel A
	weightA = get_weight(&hx711, 10, CHANNEL_A);
	// Weight cannot be negative
	weightA = (weightA < 0) ? 0 : weightA;

	// Measure the weight for channel B
	weightB = get_weight(&hx711, 10, CHANNEL_B);
	// Weight cannot be negative
	weightB = (weightB < 0) ? 0 : weightB;

	return weightA;
}
