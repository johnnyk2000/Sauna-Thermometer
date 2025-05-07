#include <Arduino.h>

#define VPTAT_PIN A0
#define VTH_PIN A1
#define VTP_PIN A2

volatile float avg_adc_Vtp = 0.0; // Exponential moving average of thermopile voltage ADC samples

// volatile int count = 0;

unsigned long dispBtempMillis = 0; // Start time for displaying body temperature
unsigned long measBtempMillis = 0; // Start time for measuring body temperature
unsigned long updateHRMillis = 0; // Time that heart rate was last updated
unsigned long updateAtempMillis = 0; // Time that ambient temperature was last updated
// unsigned long startMillis = 0;

const unsigned long dispBtempPeriod = 3000; // Display body temperature for 3sec before reverting to ready state
const unsigned long measBtempPeriod = 1000; // Measure (sample and average) body temperature for 1sec
unsigned long updateHRPeriod = 1000; // Update heart rate reading every 1sec (when valid)
unsigned long updateAtempPeriod = 30000; // Update ambient temperature reading every 30sec
// const unsigned long period = 1;  // in ms

bool dispBtemp = false; // Flag for displaying body temperature
bool measBtemp = false; // Flag for measuring body temperature

int atemp = 0; // Ambient temperature reading

void setup() {
  pinMode(VTP_PIN, INPUT);

  Serial.begin(9600);

  cli(); // Disable interrupts

  TCCR2A = 0x02; // Set Timer2 to Mode 2 (non-PWM, CTC on OCR2A)
  TCCR2B = 0x05; // Set clock prescaler to 128
  OCR2A = 0x7C; // Set count to 124 for 1ms interrupt
  TIMSK2 = 0x02; // Set interrupt on OCR2A compare match

  sei(); // Enable interrupts

  /* TODO: Setup PulseSensor */

  /* TODO: Setup OLED and RNBD */
}

ISR (TIMER2_COMPA_vect) {
  cli(); // Disable interrupts

  int adc_Vtp = analogRead(VTP_PIN); // ADC reading of thermopile voltage

  // EMA filter formula: alpha*newSample + (1-alpha)*lastEMAvalue, with alpha=1/16 (to divide by power of 2)
  avg_adc_Vtp = (adc_Vtp + 15 * avg_adc_Vtp) / 16;

  // count++;

  sei(); // Enable interrupts
}

void loop() {
  unsigned long currentMillis = millis();

  /* TODO: Calculate sensor temp (from thermistor), body temp, and heart rate bpm */
  int stemp = 0;
  float btemp = 0;
  int bpm = 0;
  /* TODO: Calculate sensor temp (from thermistor), body temp, and heart rate */

  /* BODY TEMPERATURE */
  if (dispBtemp) { // Displaying body temperature
    if ((currentMillis - dispBtempMillis) >= dispBtempPeriod) { // Display period elapsed
      // TODO: print "---" to OLED to indicate ready state
      dispBtemp = false;
      avg_adc_Vtp = 0; // Reset ADC exponential moving average for body temperature to zero
      // This is to prevent measurement from starting again right away due to capacitor at ADC input not discharging fully
    }
  }
  else if (measBtemp) { // Measuring body temperature
    if ((currentMillis - measBtempMillis) >= measBtempPeriod) { // Measurement period elapsed
      if (btemp >= 32 && btemp <= 43) { // Valid body temperature reading
        // TODO: print btemp to OLED and RNBD serial
      }
      measBtemp = false;
      dispBtemp = true; // Start displaying
      dispBtempMillis = currentMillis;
    }
  }
  else if (btemp >= 32 && btemp <= 43) { // Valid body temperature reading
    // TODO: print "Measuring" to OLED
    measBtemp = true; // Start measuring
    measBtempMillis = currentMillis;
  }

  /* HEART RATE */
  if ((bpm >= 30 && bpm <= 250) && ((currentMillis - updateHRMillis) >= updateHRPeriod)) { // Valid heart rate reading and update period elapsed
    // TODO: print heart rate to OLED and RNBD serial

    updateHRMillis = currentMillis;
  }

  /* AMBIENT TEMPERATURE */
  if ((currentMillis - updateAtempMillis) >= updateAtempPeriod) { // Update period elapsed
    int adc_ptat = analogRead(VPTAT_PIN); // ADC reading of PTAT voltage
    int atempNew = round((adc_ptat / 12.4) - 241.6); // Calculate updated ambient temperature TODO: is this correct??

    if (atempNew != atemp) { // Ambient temperature reading changed
      atemp = atempNew;
      // TODO: print atemp to OLED and RNBD serial
    }
    
    updateAtempMillis = currentMillis;
  }

  // if ((currentMillis - startMillis) >= period) {
  //   // Serial.print("Count: ");
  //   // Serial.print(count);
  //   // Serial.print(", Avg: ");
  //   Serial.println(avg_adc_Vtp);
  //   // Serial.println(adc_Vtp);
  //   startMillis = currentMillis;
  // }
}

