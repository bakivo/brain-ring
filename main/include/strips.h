/*
 * strips.h
 *
 *  Created on: 18/03/2021
 *      Author: user
 */

#ifndef MAIN_INCLUDE_STRIPS_H_
#define MAIN_INCLUDE_STRIPS_H_
#include "driver/rmt.h"

#define PIXELS_MAX 100
#define LEDS_MAX 4

typedef enum {
	STRIP_ENUM_ZERO,
	WS2812_GRB,
	SK6812_RGBW,
	STRIP_ENUM_TOTAL
} strip_name_t;


typedef struct pixel_s {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t white;
} pixel_t;

/*
 * @brief set new strip associated with allocated RMT channel
 *
 * @param[in] rmt_channel: value of rmt_channel_t enumeration
 * @param[in] name: type of strip taken from enumeration
 * @param[in] pixels_num: initial number of pixels
 * @return
 *		- ESP_OK: success
 *		- ESP_ERR_INVALID_ARG: invalid parameters
 *		- ESP_FAIL: failed because other error occurred
 */
esp_err_t set_new_strip(rmt_channel_t rmt_channel, strip_name_t name, uint16_t pixels_num);

uint16_t get_pixels_num();
/*
 * @brief set number of pixels to control
 *
 * @param[in] pixels_number: number of pixels
 * @param[in] name: type of strip taken from enumeration
 * @return
 *		- ESP_OK: success
 *		- ESP_ERR_INVALID_ARG: invalid parameters
 *		- ESP_FAIL: failed because other error occurred
 */
esp_err_t set_number_controlled_pixels(uint16_t pixels_number);

/*
 * @brief set corresponding bytes of strip's buffer for a specific pixel.
 *
 * @param[in] index: of a pixel to set
 * @param[in] pixel: pointer to a structure pixel with red, green, blue, white colors defined
 * @return
 *		- ESP_OK: Set RGBW for a specific pixel successfully
 *		- ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
 *		- ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
 */
esp_err_t set_pixel(const uint16_t index, const pixel_t *pixel);

/**
    * @brief Refresh memory colors to entire strip's pixels
    *
    * @param none:
    *
    * @return
    *      - ESP_OK: Refresh successfully
    *      - ESP_ERR_TIMEOUT: Refresh failed because of timeout
    *      - ESP_FAIL: Refresh failed because some other error occurred
    *
    * @note:
    *      After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.
    */
esp_err_t refresh_strip();

void clear_strip();

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
esp_err_t hsv2rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);

#endif /* MAIN_INCLUDE_STRIPS_H_ */
