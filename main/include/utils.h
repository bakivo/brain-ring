/*
 * utils.h
 *
 *  Created on: 9/06/2021
 *      Author: user
 */

#ifndef MAIN_INCLUDE_UTILS_H_
#define MAIN_INCLUDE_UTILS_H_

#include "esp_system.h"
esp_err_t rgb2hsv(uint32_t *h, uint32_t *s, uint32_t *v, uint32_t r, uint32_t g, uint32_t b);



#endif /* MAIN_INCLUDE_UTILS_H_ */

