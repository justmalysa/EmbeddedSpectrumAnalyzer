# Embedded Spectrum Analyzer

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


## Link to the signal_generator on KLO5
https://github.com/przemyslaw-grobecki/signal_generator
