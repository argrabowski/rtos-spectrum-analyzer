# TI-RTOS Spectrum Analyzer

## Overview

This repository contains the source code for a real-time embedded systems project developed using the Texas Instruments Real-Time Operating System (TI-RTOS). The project involves porting a [Digital Oscilloscope](https://github.com/argrabowski/digital-oscilloscope) to TI-RTOS and implementing a spectrum analyzer.

- **Lab Name:** Real-Time Embedded Systems Lab 2
- **Authors:** Adam Grabowski, Michael Rideout
- **Lab Date:** April 14, 2022

## Description

The project involves the following major components:

1. **Porting Oscilloscope to TI-RTOS:**
   - The code base which implementes a digital oscilloscope is ported to TI-RTOS.
   - The ADC ISR is configured as a Hardware Interrupt (Hwi) object using the M3 specific Hwi module.

2. **Waveform, Processing, and Display Tasks:**
   - Three tasks are implemented for waveform, processing, and display.
   - Semaphores are used to protect critical sections and signal tasks.

3. **Button Handling:**
   - User commands from the button mailbox control oscilloscope settings.
   - A clock object is used to schedule periodic button scanning.

4. **Spectrum Analyzer Mode:**
   - The project includes a spectrum analyzer mode using the Kiss FFT package.
   - The FFT is computed on the captured waveform to measure the magnitude of the input signal versus its frequency.

## File Structure

- **main.c:** The main entry point, containing initialization and the main loop.
- **peripherals.c/h:** Peripheral initialization and utility functions.
- **Crystalfontz128x128_ST7735.c/h:** Driver for the LCD display.
- **sysctl_pll.c/h:** System clock configuration functions.
- **rtos.cfg:** Configuration file for TI-RTOS.

## Getting Started

1. Clone the repository:

    ```bash
    git clone https://github.com/your-username/rtos-spectrum-analyzer.git
    ```

2. Open the project in your preferred development environment.

3. Ensure that TI-RTOS and necessary dependencies are installed.

4. Build and flash the code to your target board.

## Dependencies

- TI-RTOS
- Kiss FFT Package
- TivaWare Peripheral Driver Library

## Configuration

Adjust project configuration parameters in the `rtos.cfg` file.
