#include <Arduino.h>

#define VTP_PIN A2

volatile int adc_Vtp; // ADC reading of thermopile voltage
volatile float avg_adc_Vtp = 0.0; // Exponential moving average of thermopile voltage ADC samples

volatile int count = 0;

unsigned long startMillis = 0;
unsigned long currentMillis;
const unsigned long period = 1;  // in ms

void setup() {
  pinMode(VTP_PIN, INPUT);

  Serial.begin(9600);

  cli(); // Disable interrupts

  TCCR2A = 0x02; // Set Timer2 to Mode 2 (non-PWM, CTC on OCR2A)
  TCCR2B = 0x05; // Set clock prescaler to 128
  OCR2A = 0x7C; // Set count to 124 for 1ms interrupt
  TIMSK2 = 0x02; // Set interrupt on OCR2A compare match

  sei(); // Enable interrupts
}

ISR (TIMER2_COMPA_vect) {
  cli(); // Disable interrupts

  adc_Vtp = analogRead(VTP_PIN);

  // EMA filter formula: alpha*newSample + (1-alpha)*lastEMAvalue, with alpha=1/16 (to divide by power of 2)
  avg_adc_Vtp = (adc_Vtp + 15 * avg_adc_Vtp) / 16;

  count++;

  sei(); // Enable interrupts
}

void loop() {
  currentMillis = millis();
  if ((currentMillis - startMillis) >= period) {
    // Serial.print("Count: ");
    // Serial.print(count);
    // Serial.print(", Avg: ");
    Serial.println(avg_adc_Vtp);
    // Serial.println(adc_Vtp);
    startMillis = currentMillis;
  }
}

