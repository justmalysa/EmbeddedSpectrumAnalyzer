# Embedded Spectrum Analyzer

## Overview

This repository contains source code for spectrum analyzer running on STM32F469 Discovery board.
Time-domain and frequency-domain graphs are displayed on LCD screen in real time. 
Signal samples are collected using built-in 12-bit analog-to-digital converter with 20kHz sampling rate.
Design was tested using signal generator described below. 

Link to presentation:

https://drive.google.com/drive/u/0/folders/1ffZ-vcdK80NhrOqD0A0DbXpTGKb4eeP2?fbclid=IwAR2y_DDZqQxDjsFQYuY5I8UU-GQMYoR-F7UneR6ak9hCfSS3dZfcxXPa4sI

## Getting started

Project works under following versions of software:
- STM32CubeMX 5.4.0
- STM32CubeIDE 1.5.0
- TouchGFX 4.10.0

To run the project get the latest version by cloning
the repository to the existing CubeIDE workspace directory: 
```
git clone git@github.com:justmalysa/EmbeddedSpectrumAnalyzer.git
```
Then open the `EmbeddedSpectrumAnalyzer/TouchGFX/SpectrumAnalyzer.touchgfx` 
in TouchGFX Designer and click `Generate code` in the upper right corner.
Next import the `EmbeddedSpectrumAnalyzer` project to the CubeIDE workspace
(File -> Import -> General -> Existing Projects into Workspace).
After that, open the project in CubeIDE and build it.


## Link to the signal_generator on KL05
https://github.com/przemyslaw-grobecki/signal_generator
