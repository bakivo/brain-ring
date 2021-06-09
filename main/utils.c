/*
 * utils.c
 *
 *  Created on: 9/06/2021
 *      Author: v.bakaev
 */
#include "utils.h"
#include <sys/param.h>

esp_err_t rgb2hsv(uint32_t *h, uint32_t *s, uint32_t *v, uint32_t r, uint32_t g, uint32_t b) {
	if ( (r > 255) || (g > 255) || (b > 255)) {
		return ESP_ERR_INVALID_ARG;
	}
	float R, G, B;
	R = r / 255.0;
	G = g / 255.0;
	B = b / 255.0;
	float cmax = MAX(R, MAX(G,B));
	float cmin = MIN(R, MIN(G,B));
	float diff = cmax - cmin;

	if( cmax == cmin ) {
		*h = 0;
	} else if( cmax == R ) {
		*h = (uint32_t)( 60 * ( (G - B) / diff ) + 360 ) % 360;
	} else if( cmax == G ) {
		*h = (uint32_t)( 60 * ( (B - R) / diff ) + 120 ) % 360;
	} else if( cmax == B ) {
		*h = (uint32_t)( 60 * ( (R - G) / diff ) + 240 ) % 360;
	}

	if( cmax == 0 ) {
		*s = 0;
	} else {
		*s = ((diff / cmax) * 100);
	}

	*v = (cmax * 100);
	return ESP_OK;
}



