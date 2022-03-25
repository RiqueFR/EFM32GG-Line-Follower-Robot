/** ***************************************************************************
 * @file    main.c
 * @brief   Robo Labirinto for EFM32GG_STK3700
 * @version 1.0
******************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "em_device.h"
#include "pt.h"
#include "pwm.h"
#include "lcd.h"
#include "adc.h"
#include "uart.h"
#include "ultrasound.h"
#include "clock_efm32gg2.h"

#include "gpio.h"


#define PIN1 0b00000100  // PWMD
#define PIN2 0b01000000  // PWME
#define PRETO 4095*0.95
#define MAX_PWM 0xFFFF
#define FRENTE 0
#define DIREITA 1
#define TRAS 2
#define ESQUERDA 3
#define DIST 15


#define TG_PIN 0b00100000
#define EC_PIN 0b00010000


uint32_t valE = 0, valD = 0, valT = 0;
uint32_t pwmE, pwmD;
static char status = 'I';
static int cruz = 0;
static char string[50] = "\0";
static char string_ult[50] = "\0";
static int dist = 0;


struct pt pt1;
uint32_t threshold1,period1=10000;

struct pt pt2;
uint32_t threshold2,period2=20000;

struct pt pt3;
uint32_t threshold3,period3=15000;

struct pt pt4;
uint32_t threshold4,period4=17000;


#define TickDivisor 100000        // 10 microseconds
static uint32_t ticks = 0;

void SysTick_Handler(void){
    ticks++;
}

void Delay(uint32_t v) {
    uint32_t lim = ticks+v;       // Missing processing of overflow here
    while ( ticks < lim ) {}
}


float pwm_value(int adc_value){
    return (float)(1.0/adc_value) * 500 * MAX_PWM;
}

void uart_adc(const char* pal, int int_val) {
    char string[50];
    string[0] = '\0';
    strcat(string, pal);
    string[4] = ' ';
    string[5] = '\0';
    itoa(int_val, &string[5], 10);
    UART_SendString(string);
    UART_SendChar('\n');
    UART_SendChar('\r');
}

PT_THREAD(Blinker1(struct pt *pt)) {

    PT_BEGIN(pt);

    while(1) {
        // Processing
        valE = ADC_Read(ADC_CH0);
        valD = ADC_Read(ADC_CH1);
        valT = ADC_Read(ADC_CH3);
        dist = Ultrasound_Read_Cm(GPIOC, TG_PIN, EC_PIN, &ticks, 600);
        threshold1 = ticks+period1;
        PT_WAIT_UNTIL(pt,ticks>=threshold1);
    }

    (void) PT_YIELD_FLAG; // to silence compiler warning

    PT_END(pt);

}

PT_THREAD(Blinker2(struct pt *pt)) {

    PT_BEGIN(pt);

    while(1) {
        // Processing
        uart_adc("ADCD", valD);
        uart_adc("ADCE", valE);
        uart_adc("ADCT", valT);
        UART_SendString("SITU: ");
        UART_SendChar(status);
        UART_SendChar('\n');
        UART_SendChar('\r');
        UART_SendString("CRUZAMENTO: ");
        UART_SendChar(cruz + '0');
        UART_SendChar('\n');
        UART_SendChar('\r');
        UART_SendString("DIST: ");

        itoa(dist, string_ult, 10);
        UART_SendString(string_ult);
        UART_SendChar('\n');
        UART_SendChar('\r');

        UART_SendChar('\n');
        UART_SendChar('\r');
        threshold2 = ticks+period2;
        PT_WAIT_UNTIL(pt,ticks>=threshold2);
    }

    (void) PT_YIELD_FLAG; // to silence compiler warning

    PT_END(pt);

}

PT_THREAD(Blinker3(struct pt *pt)) {

    PT_BEGIN(pt);

    static int dist = 0;

    while(1) {
        // Processing
        if((valD > PRETO) && (valE > PRETO)) {
            pwmE = MAX_PWM * 0.7;
            pwmD = MAX_PWM * 0.7;
            status = 'P';
            cruz = 1;
        } else if(valT > PRETO && cruz == 1) {
            pwmE = 0;
            pwmD = 0;
            
            status = 'R';
            threshold3 = ticks+400000;
            PT_WAIT_UNTIL(pt,ticks>=threshold3);

            dist = Ultrasound_Read_Cm(GPIOC, TG_PIN, EC_PIN, &ticks, 600);
            itoa(dist, string, 10);
            LCD_WriteString(string);
            if (dist > DIST) { // direita livre
                while (valE < (PRETO)) {
                    pwmE = MAX_PWM/2;
                    pwmD = 0;
                    status = 'D';
                    threshold3 = ticks+period3;
                    PT_WAIT_UNTIL(pt,ticks>=threshold3);
                }

                while (valE > (PRETO)) {
                    pwmE = MAX_PWM/2;
                    pwmD = 0;
                    status = 'd';
                    threshold3 = ticks+period3;
                    PT_WAIT_UNTIL(pt,ticks>=threshold3);
                }
                
                // volta servo para o lugar
                // virou para direita
            } else { // direita bloqueada

                // verifica frente
                status = 'r';
                threshold3 = ticks+400000;
                PT_WAIT_UNTIL(pt,ticks>=threshold3);

                dist = Ultrasound_Read_Cm(GPIOC, TG_PIN, EC_PIN, &ticks, 600);
                itoa(dist, string, 10);
                LCD_WriteString(string);
                
                if (dist < DIST) { // verifica se frente NAO está livre, ai verifica esquerda
                    
                    // verifica esquerda
                    status = 'R';
                    threshold3 = ticks+400000;
                    PT_WAIT_UNTIL(pt,ticks>=threshold3);

                    dist = Ultrasound_Read_Cm(GPIOC, TG_PIN, EC_PIN, &ticks, 600);
                    itoa(dist, string, 10);
                    LCD_WriteString(string);
                    if(dist > DIST) {
                        while (valD < (PRETO)) {
                            pwmD = MAX_PWM/2;
                            pwmE = 0;
                            status = 'E';
                            threshold3 = ticks+period3;
                            PT_WAIT_UNTIL(pt,ticks>=threshold3);
                        }

                        while (valD > (PRETO)) {
                            pwmD = MAX_PWM/2;
                            pwmE = 0;
                            status = 'e';
                            threshold3 = ticks+period3;
                            PT_WAIT_UNTIL(pt,ticks>=threshold3);
                        }
                        // volta servo para o lugar
                        // virou para esquerda

                    } else {
                        while (valE < (PRETO)) {
                            pwmE = MAX_PWM/2;
                            pwmD = 0;
                            status = 'T';
                            threshold3 = ticks+period3;
                            PT_WAIT_UNTIL(pt,ticks>=threshold3);
                        }
                        while (valE > (PRETO)) {
                            pwmE = MAX_PWM/2;
                            pwmD = 0;
                            status = 't';
                            threshold3 = ticks+period3;
                            PT_WAIT_UNTIL(pt,ticks>=threshold3);
                        }
                        while (valE < (PRETO)) {
                            pwmE = MAX_PWM/2;
                            pwmD = 0;
                            status = 'T';
                            threshold3 = ticks+period3;
                            PT_WAIT_UNTIL(pt,ticks>=threshold3);
                        }
                        while (valE > (PRETO)) {
                            pwmE = MAX_PWM/2;
                            pwmD = 0;
                            status = 't';
                            threshold3 = ticks+period3;
                            PT_WAIT_UNTIL(pt,ticks>=threshold3);
                        }
                        // volta servo para o lugar
                        // retornou
                    }
                }
            }

            // sai do cruzamento
            while (valT > PRETO) {
                pwmE = MAX_PWM/2;
                pwmD = MAX_PWM/2;
                status = 'G';
                threshold3 = ticks+period3;
                PT_WAIT_UNTIL(pt,ticks>=threshold3);
            }
            cruz = 0;

        } else {
            pwmE = pwm_value(valE);
            pwmD = pwm_value(valD);
            status = 'I';
        }
        
        threshold3 = ticks+period3;
        PT_WAIT_UNTIL(pt,ticks>=threshold3);
    }

    (void) PT_YIELD_FLAG; // to silence compiler warning

    PT_END(pt);

}

PT_THREAD(Blinker4(struct pt *pt)) {

    PT_BEGIN(pt);

    while(1) {
        // Processing
        PWM_Write(TIMER0, 1, pwmD);
        PWM_Write(TIMER1, 0, pwmE);
        threshold4 = ticks+period4;
        PT_WAIT_UNTIL(pt,ticks>=threshold4);
    }

    (void) PT_YIELD_FLAG; // to silence compiler warning

    PT_END(pt);

}

int main(void) {

    ClockConfiguration_t clockconf;

    // Set clock source to external crystal: 48 MHz
    (void) SystemCoreClockSet(CLOCK_HFXO,1,1);

#if 1
    ClockGetConfiguration(&clockconf);
#endif

    // Configure SysTick
    SysTick_Config(SystemCoreClock/TickDivisor);

    ADC_Init(5000);
    ADC_ConfigChannel(ADC_CH0, 0);
    ADC_ConfigChannel(ADC_CH1, 0);
    ADC_ConfigChannel(ADC_CH3, 0);

    /* Configure LCD */
    LCD_Init();

    LCD_SetAll();
    LCD_ClearAll();

    UART_Init();

    GPIO_Init(GPIOD, 0, PIN1|PIN2);

    PWM_Init(TIMER0, 3, PWM_PARAMS_ENABLECHANNEL1); // PIN1
    PWM_Init(TIMER1, 4, PWM_PARAMS_ENABLECHANNEL0); // PIN2

    Ultrasound_Init(GPIOC, TG_PIN, EC_PIN);

    // Enable IRQs
    __enable_irq();

    PT_INIT(&pt1);
    PT_INIT(&pt2);
    PT_INIT(&pt3);
    PT_INIT(&pt4);

    while (1) {
        Blinker1(&pt1);
        Blinker2(&pt2);
        Blinker3(&pt3);
        Blinker4(&pt4);
    }
}
