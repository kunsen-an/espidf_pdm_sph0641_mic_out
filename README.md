# ESP32 PDM Microphone (SPH0641LU4H-1) and built-in DAC/PDM output Example

This is based on ESP-IDF i2s_adc_dac Example.
Adapted to PlatformIO IDE for VSCode

Please refer to https://kunsen.net/ for details.

---

* This is an example of:
    * Recording sound from PDM Microphone with ultrasonic support (SPH0641LU4H-1)
    * Replay the recorded sound via PDM or DAC
	* Sampling frequency is 48kHz or 96kHz
    * Play an audio file in flash
    
---

* Run this example
	* Set partition table to "partitions_adc_dac_example.csv" in platformio.ini
---

* This example will execute the following steps:
    1. Erase flash
    2. Record audio from PDM microphone and save in flash
    3. Read flash and replay the sound via PDM or DAC
    4. Play an example audio file(file format: 8bit/single channel)
    5. Loop back to step 3
  
---
  
* Hardware connection:
	Please refer to https://kunsen.net


	
---

* Note:
	* DAC can only play 8-bit data, so the wav file data are scaled to 8-bit data.
	* I2S DMA can only output 16-bit/32-bit data to DAC, DAC will only take the highest 8-bit data and output accordingly. 
	* Before I2S DMA can output data stream to DAC, the data format should be converted to 16-bit or 32-bit by padding zeros.



