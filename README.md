# STM32_RF_modulator

A simpel SSB and NFM modulator implemented on an STM32F103 microcontroller. For RF generation a AD9850 DDS is used. NFM is fully working but SSB still needs some quality improvements.  
The goal of this project is to create a HAM Radio transciver for under 50€. The documentation is not finished yet.

## Using the code
* The peripherals are configured via STM32CubeMX.
* To open the software project you need uVision 5.

## Hardware
You need:
* NUCLEO-F103RB board
* AD9850 board
* 12MHz Crystal. This must be soldered on the uC board (X3). R35 and R37 must be bridged. In my case the oscillator worked without C33 and C34.
* Audio source

![Pinout](/Pinout.PNG)

## Quality
There are some audio exampels in the repository.  
Update: The FM quality could be further improved while reducing the modulator code. The preemphasis filter is now implemented in the DC blocking filter. 
This picture shows the NFM spectrum with the compressor enabled.  
![Compressor on](/nfm_with_compressor.PNG)  
This picture shows the NFM spectrum with no compressor.  
![Compressor off](/nfm_no_compressor.PNG)  
Spectrum of USB modulation.  
![USB](/USB.PNG)  
  
Inspired from: https://github.com/threeme3/QCX-SSB
