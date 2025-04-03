#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "sysctl_pll.h"
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "driverlib/fpu.h"

#include "sampling.h"

volatile uint16_t draw_buffer[128];

void sample_init() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3); // GPIO setup for analog input AIN3


    // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE)
    + 1; // round up
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
    pll_divisor);

    // choose ADC1 sequence 0; disable before configuring
    ADCSequenceDisable(ADC1_BASE, 0);

    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger

    // in the 0th step, sample channel 3 (AIN3)
    // enable interrupt, and make it the end of sequence
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

    // enable the sequence. it is now sampling
    ADCSequenceEnable(ADC1_BASE, 0);

    // enable sequence 0 interrupt in the ADC1 peripheral
    ADCIntEnable(ADC1_BASE,0);

    IntPrioritySet(INT_ADC1SS0, SAMPLING_INT_PRIORITY); // set ADC1 sequence 0 interrupt priority

    // enable ADC1 sequence 0 interrupt in int. controller
    //setup the draw buffer
    IntEnable(INT_ADC1SS0);
    uint8_t i;
    for(i=0;i<128;++i) {
        draw_buffer[i] = 0;
    }
}


#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
// index wrapping macro
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
// latest sample index
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines

void ADC_ISR(void)
{
    // clear ADC1 sequence0 interrupt flag in the ADCISC register
    //<...>;
    ADCIntClear(ADC1_BASE,0);

//    ADCSequenceDataGet();
    // check for ADC FIFO overflow
    if(ADC1_OSTAT_R & ADC_OSTAT_OV0) {
    gADCErrors++; // count errors
    ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
    }
    gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1);
    // read sample from the ADC1 sequence 0 FIFO
    uint16_t sample = *(uint16_t*)(ADC1_SSFIFO0_R);
    gADCBuffer[gADCBufferIndex] = sample; // read from sample sequence 0
}


void draw_grid(tContext* context,tRectangle* rectFullScreen,uint16_t max_width,uint16_t max_height) {
    GrContextForegroundSet(context, ClrBlack);
    GrRectFill(context, rectFullScreen); // fill screen with black
    GrContextForegroundSet(context, ClrRoyalBlue); // blue line
    uint8_t i;
    for(i=PIXELS_PER_DIV-5; i<max_width;i+=PIXELS_PER_DIV) {
        //this works because we are working with a square display
        GrLineDrawV(context,i,0,max_height);
        GrLineDrawH(context,0,max_width,i);
    }
    int max_size = LCD_VERTICAL_MAX -1;
    char was_high = 0x1;
    uint16_t last_y = 0;
    GrContextForegroundSet(context, ClrYellow); // blue line
    for(i=0;i<128;++i) {
           if(draw_buffer[i] > 4338) {
               if(was_high==0x0) {
                   GrLineDrawV(context,i,0,max_height);
               } else {
                   GrPixelDraw(context,i,0);
               }
               was_high = 0x1;
           } else {
               if(was_high==0x1) {
                   GrLineDrawV(context,i,0,max_height);
               } else {
                   GrPixelDraw(context,i,max_size);
               }
               was_high=0x0;
           }
    }
    GrFlush(context); // flush the frame buffer to the LCD
}

void update_draw_buffer() {
    uint8_t i;
    int32_t last_index = gADCBufferIndex;
    for(i=0;i<128;++i) {
        draw_buffer[i] = gADCBuffer[ADC_BUFFER_WRAP(gADCBufferIndex -i)];
    }
}
