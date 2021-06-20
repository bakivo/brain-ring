/*
 * strips.c
 *
 *  Created on: 18/03/2021
 *      Author: user
 */
#include "strips.h"
#include "esp_log.h"
#include <string.h>

#define WS2812_T0H_NS (350)
#define WS2812_T0L_NS (1000)
#define WS2812_T1H_NS (1000)
#define WS2812_T1L_NS (350)
#define WS2812_RESET_US (280)

#define SK6812_T0H_NS (300)
#define SK6812_T0L_NS (900)
#define SK6812_T1H_NS (600)
#define SK6812_T1L_NS (600)
#define SK6812_RESET_US (80)


static const char *TAG_STRIP = "strip";

#define STRIP_CHECK(a, str, goto_tag, ret_value, ...)                             \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG_STRIP, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

static uint32_t t0h_ticks = 0;
static uint32_t t1h_ticks = 0;
static uint32_t t0l_ticks = 0;
static uint32_t t1l_ticks = 0;
static uint16_t pixels_num_to_off = PIXELS_MAX;

enum {
	RED,
	GREEN,
	BLUE
};

typedef struct {
	strip_name_t name;
	uint8_t leds_num;
	uint8_t colors_order[3];
	bool dedicated_white;
} strip_params_t;

typedef struct {
	uint16_t pixels_num;
	strip_params_t params;
	uint8_t buffer[PIXELS_MAX * LEDS_MAX];
	rmt_channel_t rmt_channel;
} strip_t;

static strip_t strip;
static strip_t *strip_ptr = NULL;

void set_strip_ticks(strip_name_t name)
{
    uint32_t counter_clk_hz = 0;
    rmt_get_counter_clock(strip.rmt_channel, &counter_clk_hz);
    // ns -> ticks
    float ratio = (float)counter_clk_hz / 1e9;
    switch(name) {
    	case WS2812_GRB:
    		t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    		t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    		t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    		t1l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    		break;
    	case SK6812_RGBW:
    		t0h_ticks = (uint32_t)(ratio * SK6812_T0H_NS);
			t0l_ticks = (uint32_t)(ratio * SK6812_T0L_NS);
			t1h_ticks = (uint32_t)(ratio * SK6812_T1H_NS);
			t1l_ticks = (uint32_t)(ratio * SK6812_T0L_NS);
    		break;
    	default:
    		break;
    	}
}
/*
 * @brief Order of bytes encoding red, green and blue may vary from strip to strip.
 */
void set_colors_order(strip_name_t name)
{
    switch(name) {
	case WS2812_GRB:
	case SK6812_RGBW:
		strip.params.colors_order[0] = GREEN;
		strip.params.colors_order[1] = RED;
		strip.params.colors_order[2] = BLUE;
		break;
	default:
		strip.params.colors_order[0] = RED;
		strip.params.colors_order[1] = GREEN;
		strip.params.colors_order[2] = BLUE;
		break;
    }
}

void set_strip_type(strip_name_t name)
{
	strip.params.name = name;
	strip.params.leds_num = 3;
	strip.params.dedicated_white = false;
	if (name == SK6812_RGBW){
		strip.params.leds_num = 4;
		strip.params.dedicated_white = true;
	}
	set_strip_ticks(name);
	set_colors_order(name);
}

esp_err_t hsv2rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
	//ESP_LOGI(TAG_STRIP,"hsv2rgb: %d %d %d", h, s, v);
	esp_err_t ret;
	STRIP_CHECK( (s <= 100) && (v <= 100), "invalid argument: value or saturation", err, ESP_ERR_INVALID_ARG );
    h %= 360; // h -> [0,360]
    uint8_t rgb_max = v * 2.55f;
    uint8_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint8_t i = h / 60;
    uint8_t diff = h % 60;

    // RGB adjustment amount by hue
    uint8_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief Convert RGB data to RMT format.
 *
 * @note For WS2812, R,G,B each contains 256 different choices (i.e. uint8_t)
 *
 * @param[in] src: source data, to converted to RMT format
 * @param[in] dest: place where to store the convert result
 * @param[in] src_size: size of source data
 * @param[in] wanted_num: number of RMT items that want to get
 * @param[out] translated_size: number of source data that got converted
 * @param[out] item_num: number of RMT items which are converted from source data
 */
static void IRAM_ATTR rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size, size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ t0h_ticks, 1, t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ t1h_ticks, 1, t1l_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
    return;
}

esp_err_t set_new_strip(rmt_channel_t rmt_channel, strip_name_t name, uint16_t pixels_num)
{
	esp_err_t ret;
	STRIP_CHECK(strip_ptr == NULL, "Strip is already created", err, ESP_FAIL);
    STRIP_CHECK(pixels_num <= PIXELS_MAX, "pixels number out of the maximum number supported", err, ESP_ERR_INVALID_ARG);
    STRIP_CHECK((name < STRIP_ENUM_TOTAL) && (name > STRIP_ENUM_ZERO), "strip type is out of the range supported", err, ESP_ERR_INVALID_ARG);

    strip_ptr = &strip;
	strip.rmt_channel = rmt_channel;
	strip.pixels_num = pixels_num;
	pixels_num_to_off = pixels_num;
	set_strip_type(name);

	STRIP_CHECK(rmt_translator_init(rmt_channel, rmt_adapter) == ESP_OK, "RMT translator initialization failed", err, ESP_FAIL);
	return ESP_OK;
err:
	return ret;
}

esp_err_t set_pixel(const uint16_t index, const pixel_t *pixel)
{
	esp_err_t ret;
    STRIP_CHECK(index < PIXELS_MAX, "index out of the maximum number of pixels", err, ESP_ERR_INVALID_ARG);
    STRIP_CHECK(index < strip.pixels_num, "index out of range of controlled pixels", err, ESP_ERR_INVALID_ARG);

    uint32_t offset = index * strip.params.leds_num;

	strip.buffer[offset + strip.params.colors_order[0]] = pixel->r;
	strip.buffer[offset + strip.params.colors_order[1]] = pixel->g;
	strip.buffer[offset + strip.params.colors_order[2]] = pixel->b;
	// some strips like SK6812 have dedicated white led needing 4 bytes for control
	if (strip.params.dedicated_white){
		strip.buffer[offset + 3] = pixel->white;
	}
	return ESP_OK;
err:
	return ret;
}

void change_strip_type(strip_name_t name)
{
	set_strip_type(name);
}

uint16_t get_pixels_num()
{
	return strip.pixels_num;
}

esp_err_t set_number_controlled_pixels(uint16_t pixels_number)
{
	esp_err_t ret;
	STRIP_CHECK(pixels_number <= PIXELS_MAX, "pixels number exceeded maximum", err, ESP_ERR_INVALID_ARG);
	// set pixels amount
	strip.pixels_num = pixels_number;
	pixels_num_to_off = pixels_number;
	return ESP_OK;
err:
	return ret;
}

esp_err_t refresh_strip()
{
	esp_err_t ret = ESP_OK;
	STRIP_CHECK(rmt_write_sample(strip.rmt_channel, strip.buffer, strip.pixels_num * strip.params.leds_num, false) == ESP_OK, "transmitting of RMT samples failed", err, ESP_FAIL);
	return ret;
err:
	return ret;
}

void clear_strip()
{
    // Write zero to turn off all leds
    memset(strip.buffer, 0, pixels_num_to_off * strip.params.leds_num);
    refresh_strip();
}
