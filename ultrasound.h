#ifndef ULTRASSOM_H
#define ULTRASSOM_H

#include "stdint.h"
#include "gpio.h"

/*
 * Initialize the ultrasound sensor
 * GPIO_t GPIO_func         => GPIO who the sensor is connected
 * int TG_PIN_f             => BIT to the toggle pin
 * int EC_PIN_f             => BIT to the echo pin
 */
void Ultrasound_Init(const GPIO_t GPIO_ult, int TG_PIN, int EC_PIN);

/*
 * Delay in 'v' microseconds
 * uint32_t *ticks          => pointer to the ticks variable
 * uint32_t v               => time of the delay, in microseconds
 */
void delay(uint32_t *ticks, uint32_t v);

/*
 * Read the ultrasonic sensor and return the distance in CM
 * uint32_t *ticks          => pointer to the ticks variable
 * uint32_t timeout         => time in microseconds to stop wait for the echo, it changes the max sensor range
 */
int Ultrasound_Read_Cm(const GPIO_t GPIO_ult, int TG_PIN, int EC_PIN, uint32_t *ticks, uint32_t timeout);

#endif
