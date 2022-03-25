#include "ultrasound.h"

void Ultrasound_Init(const GPIO_t GPIO_ult, int TG_PIN, int EC_PIN) {
    GPIO_Init(GPIO_ult, EC_PIN, TG_PIN);
}

void delay(uint32_t *ticks, uint32_t v) {
    uint32_t lim = *ticks+v;       // Missing processing of overflow here

    while ( *ticks < lim ) {}
}

int Ultrasound_Read_Cm(const GPIO_t GPIO_ult, int TG_PIN, int EC_PIN, uint32_t *ticks, uint32_t timeout) {
    GPIO_WritePins(GPIO_ult, TG_PIN, 0);
    delay(ticks, 1);
    GPIO_WritePins(GPIO_ult, 0, TG_PIN);
    delay(ticks, 1);
    GPIO_WritePins(GPIO_ult, TG_PIN, 0);
    
    uint32_t previousMicros = *ticks;
    while(!(GPIO_ReadPins(GPIO_ult)&EC_PIN) && (*ticks - previousMicros) <= timeout);   // wait for the echo pin HIGH or timeout
    previousMicros = *ticks;
    
    while((GPIO_ReadPins(GPIO_ult)&EC_PIN) && (*ticks - previousMicros) <= timeout);    // wait for the echo pin LOW or timeout
    uint32_t duration = *ticks - previousMicros;
    int dist = (duration*10) / 58; // distance in CM
    return dist;
}
