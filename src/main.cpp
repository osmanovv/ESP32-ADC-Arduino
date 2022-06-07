/**
 * Working with ESP32 Audio Sampling
 * https://www.toptal.com/embedded/esp32-audio-sampling
 */

#include <Arduino.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include "driver/adc.h"

volatile int16_t adc, dac = 0;

portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t adcHandlerTask;
hw_timer_t * adcTimer = NULL; // our timer

int IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  dac = local_adc1_read(ADC1_CHANNEL_0);
  
  /* push DAC */
  dacWrite(DAC1, highByte(dac));
  dacWrite(DAC2, lowByte(dac)); 

  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  adcTimer = timerBegin(3, 80, true); // 80 MHz / 80 = 1 MHz hardware clock for easy figuring
  timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adcTimer, 45, true); // Interrupts when counter == 45, i.e. 22.222 times a second
  timerAlarmEnable(adcTimer);
}

void loop() {
  // put your main code here, to run repeatedly:
}