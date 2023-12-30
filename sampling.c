/*
 * sampling.c
 *
 * ECE 3849 Lab 2
 * Adam Grabowski, Michael Rideout
 * Created on April 16, 2022
 *
 * ECE 3849 Lab ADC handling
 */

// XDCtools Header files
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

// BIOS Header files
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "Crystalfontz128x128_ST7735.h"
#include "math.h"
#include "peripherals.h"

// KISS FFT header files
#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

// KISS FFT constants
#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

// ADC globals
uint32_t gADCSamplingRate;                              // [Hz] actual ADC sampling rate
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];          // circular buffer
volatile uint32_t gADCErrors;                           // number of missed ADC deadlines
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1; // latest sample index

// waveform globals
volatile uint32_t trigger_value;
volatile uint16_t trigger_samples[ADC_TRIGGER_SIZE];
volatile int16_t processedWaveform[ADC_TRIGGER_SIZE];
volatile uint16_t fft_samples[NFFT];

// state globals
volatile bool spectrumMode = false;             // determines the mode of the oscilloscope
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2};   // array of voltage scale per division
float fScale;

// initialize ADC hardware
void ADC_Init(void)
{
    // GPIO setup for analog input AIN3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    // initialize ADC1 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1;  // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor);                      // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // initialize ADC1 sampling sequence
    ADCSequenceDisable(ADC1_BASE, 0);                                                   // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);                          // specify the "timer" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH3);  // in the 0th step, sample channel 3 (AIN3)

    // enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0);    // enable the sequence.  it is now sampling
    ADCIntEnable(ADC1_BASE, 0);         // enable sequence 0 interrupt in the ADC1 peripheral
}

// ADC interrupt service routine
void ADC_ISR(void)
{
    ADC1_ISC_R = ADC_ISC_IN0;           // clears ADC interrupt flag

    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }

    gADCBuffer[
               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
               ] = ADC1_SSFIFO0_R;          // read sample from the ADC1 sequence 0 FIFO
}

// search for sample trigger
void triggerSearch(void)
{
    int32_t trigger_index;
    int32_t i;

    // goes backwards through the whole gADCBuffer array, and finds the zero-crossing point index and shifts it
    trigger_index = gADCBufferIndex - LCD_HORIZONTAL_MAX/2;

    if (risingSlope) { // rising slope trigger search
        for (i = 0; i < ADC_BUFFER_SIZE/2; i++, trigger_index--) {
            if (gADCBuffer[ADC_BUFFER_WRAP(trigger_index)] <= trigger_value &&
                    gADCBuffer[ADC_BUFFER_WRAP(trigger_index + 1)] > trigger_value) {
                break; // if found, stop looking
            }
        }
    }
    else { // falling slope trigger search
        for (i = 0; i < ADC_BUFFER_SIZE/2; i++, trigger_index--) {
            if (gADCBuffer[ADC_BUFFER_WRAP(trigger_index)] >= trigger_value &&
                    gADCBuffer[ADC_BUFFER_WRAP(trigger_index + 1)] < trigger_value) {
                break; // if found, stop looking
            }
        }
    }

    if (i == ADC_BUFFER_SIZE/2) { // if trigger not found, set to previous value
        trigger_index = gADCBufferIndex - LCD_HORIZONTAL_MAX/2;
    }

    // local buffer retrieves 128 samples of the gADCBuffer from the trigger_index previously found
    for (i = 0; i < ADC_TRIGGER_SIZE; i++){
        trigger_samples[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger_index - (ADC_TRIGGER_SIZE - 1) + i)];
    }
}

// returns zero-crossing point of the ADC waveform by finding the max and min points, averaging them
uint32_t zeroCrossPoint(void)
{
    int max = 0;
    int min = 10000;

    int i;
    for (i = 0; i < ADC_BUFFER_SIZE; i++){
        if (gADCBuffer[i] > max){
            max = gADCBuffer[i];
        }

        if (gADCBuffer[i] < min){
            min = gADCBuffer[i];
        }
    }

    return (max+min)/2;
}

// TI-RTOS processing task function
void processingTask_func(UArg arg1, UArg arg2)
{
    IntMasterEnable(); // enable interrupts

    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE];                 // KISS FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;                             // KISS FFT buffer size
    kiss_fft_cfg cfg;                                                   // KISS FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT];                            // complex waveform and spectrum buffers
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size);   // init Kiss FFT
    int i;

    static float w[NFFT]; // window function
    for (i = 0; i < NFFT; i++) {
        // blackman window
        w[i] = (0.42f - 0.5f * cosf(2*PI*i/(NFFT-1)) + 0.08f * cosf(4*PI*i/(NFFT-1)));
    }

    while(true){
        Semaphore_pend(semProcessing, BIOS_WAIT_FOREVER); // from waveform

        if (spectrumMode){
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            for (i = 0; i < NFFT; i++) { // generate an input waveform
                in[i].r = ((float)fft_samples[i] - trigger_value) * w[i];   // real part of waveform
                in[i].i = 0;                                                // imaginary part of waveform
            }

            Semaphore_post(sem_cs);

            kiss_fft(cfg, in, out); // compute FFT

            // convert first 128 bins of out[] to dB for display
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            for (i = 0; i < ADC_TRIGGER_SIZE - 1; i++) {
                processedWaveform[i] = (int)roundf(128 - 10*log10f(out[i].r*out[i].r +out[i].i*out[i].i));
            }

            Semaphore_post(sem_cs);
        } else {
            // determines fScale
            fScale = (VIN_RANGE/(1 << ADC_BITS))*(PIXELS_PER_DIV/fVoltsPerDiv[stateVperDiv]);
            int i;

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            for (i = 0; i < ADC_TRIGGER_SIZE - 1; i++) {
                processedWaveform[i] = ((int)(ADC_TRIGGER_SIZE/2) - (int)roundf(fScale*(int)(trigger_samples[i] - trigger_value)));
            }

            Semaphore_post(sem_cs);
        }

        Semaphore_post(semWaveform);    // to waveform
        Semaphore_post(semDisplay);     // to display
    }
}

// TI-RTOS waveform task function
void waveformTask_func(UArg arg1, UArg arg2)
{
    IntMasterEnable(); // enable interrupts

    while(true){
        Semaphore_pend(semWaveform, BIOS_WAIT_FOREVER); // from processing

        trigger_value = zeroCrossPoint(); // Dynamically finds the ADC_OFFSET
        if (spectrumMode){
            int i;
            int buffer_ind = gADCBufferIndex;

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            for (i = 0; i < NFFT; i++){
                fft_samples[i] = gADCBuffer[ADC_BUFFER_WRAP(buffer_ind - NFFT + i)];
            }
            Semaphore_post(sem_cs);

        } else {
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section

            triggerSearch(); // searches for trigger

            Semaphore_post(sem_cs);
        }

        Semaphore_post(semProcessing); // to processing
    }
}
