#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Adafruit_GFX.h>
#include <PulseSensorPlayground.h>

#define VPTAT_PIN A0
#define VTH_PIN A1
#define VTP_PIN A2
#define HR_PIN A6
#define LED_PIN 5

// Linear fit parameters for calculating body temperature at each ambient temperature (20, 21, ..., 85°C)
// TODO: change after calibration
// const float Btemp_line_const[] = {-1.7194, -1.7874, -1.8560, -1.9254, -1.9954, -2.0662, -2.1377, -2.2099, -2.2828, -2.3564, -2.4308, 
//                                   -2.5059, -2.5818, -2.6584, -2.7358, -2.8139, -2.8928, -2.9725, -3.0530, -3.1342, -3.2162, -3.2990, 
//                                   -3.3826, -3.4670, -3.5522, -3.6382, -3.7250, -3.8126, -3.9011, -3.9904, -4.0805, -4.1715, -4.2633, 
//                                   -4.3560, -4.4495, -4.5439, -4.6391, -4.7352, -4.8322, -4.9301, -5.0289, -5.1286, -5.2291, -5.3306, 
//                                   -5.4330, -5.5363, -5.6405, -5.7456, -5.8517, -5.9587, -6.0667, -6.1756, -6.2854, -6.3962, -6.5080, 
//                                   -6.6207, -6.7345, -6.8492, -6.9649, -7.0815, -7.1992, -7.3179, -7.4376, -7.5583, -7.6800, -7.8028};
// const float Btemp_line_slope = 0.08046; // Slope is same for all ambient temperatures
const float Btemp_line_const1[] = {-21.3701, -22.2146, -23.0677, -23.9296, -24.8002, -25.6796, -26.5680, -27.4653, -28.3716, -29.2870, -30.2115, 
                                   -31.1452, -32.0882, -33.0404, -34.0021, -34.9732, -35.9538, -36.9440, -37.9438, -38.9532, -39.9725, -41.0015, 
                                   -42.0404, -43.0892, -44.1481, -45.2170, -46.2960, -47.3853, -48.4847, -49.5945, -50.7147, -51.8453, -52.9865, 
                                   -54.1382, -55.3006, -56.4736, -57.6575, -58.8522, -60.0578, -61.2743, -62.5019, -63.7406, -64.9905, -66.2516, 
                                   -67.5240, -68.8078, -70.1030, -71.4098, -72.7281, -74.0580, -75.3997, -76.7532, -78.1185, -79.4957, -80.8849, 
                                   -82.2861, -83.6995, -85.1251, -86.5630, -88.0132, -89.4758, -90.9508, -92.4385, -93.9387, -95.4516, -96.9773};
const float Btemp_line2 = 0.08991;
const float Btemp_line3 = 68.4766;

// Voltage across thermistor, converted to ADC value, for each ambient temperature (20, 21, ..., 85°C)
// TODO: change after calibration
const int Vth_adc_table[] = {995, 951, 908, 868, 830, 794, 760, 727, 696, 666, 638, 611, 586, 562, 539, 517, 496, 476, 456, 438, 421, 404, 
                             389, 373, 359, 345, 332, 319, 307, 296, 285, 274, 264, 255, 245, 236, 228, 220, 212, 205, 197, 191, 184, 178, 
                             172, 166, 160, 155, 149, 145, 140, 135, 131, 126, 122, 118, 115, 111, 107, 104, 101, 98, 95, 92, 89, 86};
const int Vth_table_len = 66;

volatile float avg_adc_Vtp = 0.0; // Exponential moving average of thermopile voltage ADC samples
volatile float avg_adc_Vptat = 0.0; // Exponential moving average of PTAT voltage ADC samples

// volatile int count = 0;

unsigned long dispBtempMillis = 0; // Start time for displaying body temperature
unsigned long measBtempMillis = 0; // Start time for measuring body temperature
unsigned long dispHRMillis = 0; // Start time for displaying body temperature
unsigned long measHRMillis = 0; // Start time for measuring body temperature
unsigned long updateAtempMillis = 0; // Time that ambient temperature was last updated
unsigned long ledMillis = 0; // Start time of LED blink period
// unsigned long startMillis = 0;

const unsigned long dispBtempPeriod = 3000; // Display body temperature for 3sec before reverting to ready state
const unsigned long measBtempPeriod = 1000; // Measure (sample and average) body temperature for 1sec
const unsigned long dispHRPeriod = 3000; // Display heart rate for 3sec before reverting to ready state
const unsigned long measHRPeriod = 1000; // Measure heart rate for 1sec
const unsigned long updateAtempPeriod = 30000; // Update ambient temperature reading every 30sec
const unsigned long ledOnPeriod = 300; // LED on for 0.3sec per blink period
const unsigned long ledPeriod = 1000; // LED blink total period is 1sec
// const unsigned long period = 1;  // in ms

bool dispBtemp = false; // Flag for displaying body temperature
bool measBtemp = false; // Flag for measuring body temperature
bool dispHR = false; // Flag for displaying heart rate
bool measHR = false; // Flag for measuring heart rate
bool ledWarningBtemp = false; // Flag for flashing warning LED for high body temperature
bool ledWarningHR = false; // Flag for flashing warning LED for high heart rate
bool dispInit = false; // Flag for displaying measurements upon initialization

int atemp = 0; // Ambient temperature reading
int stemp = 0; // Sensor temperature reading
int hrThreshold = 550; // Threshold for detecting pulses in heart rate measurement

SSD1306AsciiWire oled;
PulseSensorPlayground pulseSensor;

void setup() {
  pinMode(VPTAT_PIN, INPUT);
  pinMode(VTH_PIN, INPUT);
  pinMode(VTP_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();

  cli(); // Disable interrupts

  TCCR2A = 0x02; // Set Timer2 to Mode 2 (non-PWM, CTC on OCR2A)
  TCCR2B = 0x05; // Set clock prescaler to 128
  OCR2A = 0x7C; // Set count to 124 for 1ms interrupt
  TIMSK2 = 0x02; // Set interrupt on OCR2A compare match

  sei(); // Enable interrupts

  /* SETUP PULSESENSOR */
  pulseSensor.analogInput(HR_PIN);
  pulseSensor.setThreshold(hrThreshold);
  pulseSensor.begin();

  /* SETUP OLED */
  oled.begin(&Adafruit128x32, 0x3C); // Start the OLED display
  oled.setFont(System5x7);

  dispInit = true;
  updateAtempMillis = millis();
}

/* INTERRUPT SERVICE ROUTINE FOR SAMPLING THERMOPILE VOLTAGE */
ISR (TIMER2_COMPA_vect) {
  cli(); // Disable interrupts

  int adc_Vtp = analogRead(VTP_PIN); // ADC reading of thermopile voltage

  // EMA filter formula: alpha*newSample + (1-alpha)*lastEMAvalue, with alpha=1/16 (to divide by power of 2)
  avg_adc_Vtp = (adc_Vtp + 15 * avg_adc_Vtp) / 16;

  int adc_Vptat = analogRead(VPTAT_PIN); // ADC reading of PTAT voltage

  // EMA filter formula: alpha*newSample + (1-alpha)*lastEMAvalue, with alpha=1/100=0.01
  avg_adc_Vptat = (adc_Vptat + 99 * avg_adc_Vptat) / 100;

  // count++;

  sei(); // Enable interrupts
}

void loop() {
  unsigned long currentMillis = millis();

  /* DISPLAY MEASUREMENTS UPON INITIALIZATION */
  if (dispInit) { // Just initialized
    if ((currentMillis - updateAtempMillis) >= 1000) { // 1sec elapsed after initialization
      atemp = round(0.3946 * avg_adc_Vptat - 242.3); // TODO: check this
      oled.setCursor(0, 1);
      oled.print(F("Ambient temp: "));
      oled.print(atemp);
      oled.print(F("\200C ")); // \200 is octal escape for degree symbol

      oled.setCursor(0, 2);
      oled.print(F("Body temp: ---      "));

      oled.setCursor(0, 3);
      oled.print(F("Heart rate: ---    "));

      updateAtempMillis = currentMillis;
      dispInit = false;
    }
    else { // Just initialized and 1sec not yet elapsed
      return; // Don't start rest of program yet
    }
  }

  /* AMBIENT TEMPERATURE */
  if ((currentMillis - updateAtempMillis) >= updateAtempPeriod) { // Update period elapsed
    // Vptat = 12.3733*Tamb + 2998.08 [mV] -> Tamb = Vptat/12.3733 - 242.30
    int atempNew = round(0.3946 * avg_adc_Vptat - 242.3); // Calculate updated ambient temperature TODO: check this

    if (atempNew != atemp) { // Ambient temperature reading changed
      atemp = atempNew;
      // TODO: print atemp to OLED and RNBD serial
      oled.setCursor(0, 1);
      oled.print(F("Ambient temp: "));
      oled.print(atemp);
      oled.print(F("\200C ")); // \200 is octal escape for degree symbol

      oled.setCursor(0, 3);
      oled.print(F("Sensor temp: "));
      oled.print(stemp);
      oled.print(F("\200C   "));
    }

    float Vptat = avg_adc_Vptat * 5.0 / 1024;
    oled.setCursor(0, 1);
    oled.print(F("Vptat: "));
    oled.print(Vptat, 4);
    oled.print(F("V "));
  
    updateAtempMillis = currentMillis;
  }

  int adc_Vth = analogRead(VTH_PIN); // ADC reading of voltage across thermistor

  // Only proceed if Vth is within the specified operating range
  if (adc_Vth <= 1019 && adc_Vth >= 85) { // Limits are the values corresponding to 19 and 86°C TODO: change after calibration

    /* CALCULATE SENSOR AMBIENT TEMPERATURE */
    // Binary nearest neighbor search to map Vth reading to temperature; note Vth_adc_table is in descending order
    int left = 0;
    int right = Vth_table_len;
    int mid;
    do {
      mid = left + floor((right-left)/2);
      if (Vth_adc_table[mid] > adc_Vth)
        left = mid + 1;
      else
        right = mid;
    } while(left < right); // At the end, right holds the index of the predecessor (smaller neighbor)

    if (right == 0)
      stemp = 20;
    else if (right == Vth_table_len)
      stemp = 19 + Vth_table_len;
    else if ((adc_Vth - Vth_adc_table[right]) <= (Vth_adc_table[right-1] - adc_Vth)) // Vth reading is closer to the predecessor than to the successor (larger neighbor)
      stemp = 20 + right; // Use the predecessor index
    else
      stemp = 19 + right; // Use the successor index (20 + right-1) = 19 + right

    /* CALCULATE BODY TEMPERATURE */
    float btemp = 0;
    if (adc_Vth <= 1019 && adc_Vth >= 85) { // Limits are the values corresponding to 19 and 86°C TODO: change after calibration
      // btemp = ((avg_adc_Vtp / 1024 * 5000 - 3719) / 675 - Btemp_line_const[stemp - 20]) / Btemp_line_slope;
      // Reduced above computations by absorbing constants into the line parameters
      btemp = avg_adc_Vtp * Btemp_line2 - Btemp_line3 - Btemp_line_const1[stemp - 20]; // TODO: change after calibration
    }

    /* CALCULATE HEART RATE */

    int bpm = pulseSensor.getBeatsPerMinute();

    /* TODO: Compare/average(?) the sensor temp (from thermistor) and ambient temp (from PTAT) */

    /* BODY TEMPERATURE */
    if (dispBtemp) { // Displaying body temperature
      if ((currentMillis - dispBtempMillis) >= dispBtempPeriod) { // Display period elapsed
        // TODO: print "---" to OLED to indicate ready state
        oled.setCursor(0, 2);
        oled.print(F("Body temp: ---      "));
        ledWarningBtemp = false;
        if (!ledWarningHR)
          digitalWrite(LED_PIN, LOW);
        dispBtemp = false;
        avg_adc_Vtp = 0; // Reset ADC exponential moving average for body temperature to zero
        // This is to prevent measurement from starting again right away due to capacitor at ADC input not discharging fully
      }
    }
    else if (measBtemp) { // Measuring body temperature
      if ((currentMillis - measBtempMillis) >= measBtempPeriod) { // Measurement period elapsed
        if (btemp >= 32 && btemp <= 43) { // Valid body temperature reading
          // TODO: print btemp to OLED and RNBD serial
          float Vtp = avg_adc_Vtp * 5.0 / 1024;
          oled.setCursor(0, 2);
          oled.print(F("Body temp: "));
          oled.print(Vtp, 4);
          oled.print(F("V "));
          if (btemp >= 39.4) { // Body temperature too high, flash warning LED
            ledWarningBtemp = true;
            if (!ledWarningHR)
              ledMillis = currentMillis;
          }
        }
        else {
          oled.setCursor(0, 2);
          oled.print(F("Body temp: ---      "));
        }
        measBtemp = false;
        dispBtemp = true; // Start displaying
        dispBtempMillis = currentMillis;
      }
    }
    else { // Start measuring body temperature again
      // TODO: print "Measuring" to OLED
      oled.setCursor(0, 2);
      oled.print(F("Body temp: Measuring"));
      measBtemp = true; // Start measuring
      measBtempMillis = currentMillis;
    }

    /* HEART RATE */
    if (dispHR) { // Displaying heart rate
      if ((currentMillis - dispHRMillis) >= dispHRPeriod) { // Display period elapsed
        // TODO: print "---" to OLED to indicate ready state
        oled.setCursor(0, 3);
        oled.print(F("Heart rate: ---      "));
        ledWarningHR = false;
        if (!ledWarningBtemp)
          digitalWrite(LED_PIN, LOW);
        dispHR = false;
      }
    }
    else if (measHR) { // Measuring heart rate
      if ((currentMillis - measHRMillis) >= measHRPeriod) { // Measurement period elapsed
        if (bpm >= 30 && bpm <= 250) { // Valid heart rate reading
          // TODO: print heart rate to OLED and RNBD serial
          oled.setCursor(0, 3);
          oled.print(F("Heart rate: "));
          oled.print(bpm);
          oled.print(F(" BPM   "));
          if (bpm >= 180) { // Heart rate too high, flash warning LED
            ledWarningHR = true;
            if (!ledWarningBtemp)
              ledMillis = currentMillis;
          }
        }
        else {
          oled.setCursor(0, 3);
          oled.print(F("Heart rate: ---      "));
        }
        measHR = false;
        dispHR = true; // Start displaying
        dispHRMillis = currentMillis;
      }
    }
    else if (bpm >= 30 && bpm <= 250) { // Valid heart rate reading
      // TODO: print "Measuring" to OLED
      oled.setCursor(0, 3);
      oled.print(F("Heart rate: Measuring"));
      measHR = true; // Start measuring
      measHRMillis = currentMillis;
    }

    /* WARNING LED */
    if (ledWarningBtemp || ledWarningHR) {
      if ((currentMillis - ledMillis) < ledOnPeriod)
        digitalWrite(LED_PIN, HIGH); // Blink on
      else if ((currentMillis - ledMillis) < ledPeriod)
        digitalWrite(LED_PIN, LOW); // Blink off
      else
        ledMillis = currentMillis; // Repeat blink period
    }

  }
}

