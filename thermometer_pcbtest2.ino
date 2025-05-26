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
// Fit of our data
// const float Btemp_line_const[] = {-1.7439, -1.8128, -1.8824, -1.9527, -2.0238, -2.0955, -2.1680, -2.2413, -2.3152, -2.3899, -2.4654, 
//                                   -2.5415, -2.6185, -2.6962, -2.7747, -2.8539, -2.9339, -3.0147, -3.0963, -3.1787, -3.2619, -3.3459, 
//                                   -3.4306, -3.5162, -3.6026, -3.6899, -3.7779, -3.8668, -3.9565, -4.0471, -4.1385, -4.2308, -4.3239, 
//                                   -4.4179, -4.5127, -4.6084, -4.7050, -4.8025, -4.9009, -5.0002, -5.1004, -5.2014, -5.3034, -5.4063, 
//                                   -5.5102, -5.6149, -5.7206, -5.8273, -5.9349, -6.0434, -6.1529, -6.2633, -6.3747, -6.4871, -6.6005, 
//                                   -6.7148, -6.8302, -6.9465, -7.0638, -7.1822, -7.3015, -7.4219, -7.5433, -7.6657, -7.7892, -7.9137};
// const float Btemp_line_slope = 0.08160; // Slope is same for all ambient temperatures
// Divide Btemp_line_const by Btemp_line_slope
const float Btemp_line_const1[] = {-21.3710, -22.2155, -23.0686, -23.9305, -24.8012, -25.6806, -26.5690, -27.4664, -28.3727, -29.2881, -30.2127, 
                                   -31.1464, -32.0894, -33.0418, -34.0035, -34.9746, -35.9552, -36.9455, -37.9453, -38.9548, -39.9741, -41.0031, 
                                   -42.0421, -43.0910, -44.1499, -45.2188, -46.2979, -47.3872, -48.4867, -49.5965, -50.7168, -51.8474, -52.9886, 
                                   -54.1404, -55.3028, -56.4759, -57.6598, -58.8545, -60.0602, -61.2768, -62.5044, -63.7432, -64.9931, -66.2543, 
                                   -67.5268, -68.8106, -70.1059, -71.4127, -72.7310, -74.0611, -75.4028, -76.7563, -78.1216, -79.4989, -80.8882, 
                                   -82.2895, -83.7030, -85.1286, -86.5665, -88.0168, -89.4794, -90.9546, -92.4422, -93.9426, -95.4555, -96.98133};
const float Btemp_line2 = 0.08865; // 5000 / (1024 * 675 * Btemp_line_slope)
const float Btemp_line3 = 67.7560; // 3732 / (675 * Btemp_line_slope)

// Manufacturer provided typical curve
const float Btemp_line_const2[] = {-21.3701, -22.2146, -23.0677, -23.9296, -24.8002, -25.6796, -26.5680, -27.4653, -28.3716, -29.2870, -30.2115, 
                                   -31.1452, -32.0882, -33.0404, -34.0021, -34.9732, -35.9538, -36.9440, -37.9438, -38.9532, -39.9725, -41.0015, 
                                   -42.0404, -43.0892, -44.1481, -45.2170, -46.2960, -47.3852, -48.4847, -49.5945, -50.7147, -51.8453, -52.9865, 
                                   -54.1382, -55.3006, -56.4736, -57.6575, -58.8522, -60.0577, -61.2743, -62.5019, -63.7406, -64.9905, -66.2516, 
                                   -67.5240, -68.8078, -70.1030, -71.4097, -72.7281, -74.0580, -75.3997, -76.7531, -78.1184, -79.4957, -80.8849, 
                                   -82.2861, -83.6995, -85.1251, -86.5630, -88.0131, -89.4757, -90.9508, -92.4384, -93.9387, -95.4516, -96.9773};
const float Btemp_line4 = 0.08991; // 5000 / (1024 * 675 * Btemp_line_slope) slope is 0.08046
const float Btemp_line5 = 68.7160; // 3732 / (675 * Btemp_line_slope)

// Voltage across thermistor, converted to ADC value, for each ambient temperature (20, 21, ..., 85°C)
// TODO: change after calibration
// Fit of our data incorporating hardcoded changes in bandgap current
const int Vth_adc_table1[] = {1103, 1055, 1009, 965, 924, 885, 848, 812, 778, 746, 719, 689, 661, 635, 609, 585, 562, 540, 519, 500, 480, 462, 
                              444, 427, 411, 397, 382, 368, 355, 342, 330, 318, 307, 296, 285, 275, 266, 256, 248, 239, 231, 223, 215, 208, 
                              201, 195, 188, 182, 176, 170, 165, 160, 155, 150, 145, 140, 136, 132, 128, 124, 120, 116, 113, 110, 106, 103};
// Fit of our data assuming constant bandgap current (43uA)
const int Vth_adc_table2[] = {1101, 1053, 1008, 965, 924, 885, 848, 813, 779, 747, 717, 688, 660, 633, 608, 584, 561, 539, 518, 498, 479, 461, 
                              444, 427, 411, 396, 381, 367, 354, 341, 329, 317, 306, 295, 285, 275, 265, 256, 247, 239, 231, 223, 215, 208, 
                              201, 195, 188, 182, 176, 171, 165, 160, 155, 150, 145, 141, 136, 132, 128, 124, 121, 117, 113, 110, 107, 104};
// Manufacturer provided typical curve incorporating hardcoded changes in bandgap current
const int Vth_adc_table3[] = {1101, 1052, 1005, 961, 919, 879, 841, 804, 770, 737, 709, 680, 651, 624, 599, 574, 551, 529, 507, 488, 469, 451, 
                              433, 416, 400, 386, 371, 357, 343, 330, 319, 307, 296, 285, 275, 265, 255, 246, 237, 229, 221, 213, 206, 199, 
                              192, 185, 179, 173, 167, 162, 156, 151, 146, 142, 137, 133, 128, 124, 120, 117, 113, 109, 106, 103, 100, 97};
// Manufacturer provided typical curve assuming constant bandgap current (43uA)
const int Vth_adc_table4[] = {1104, 1054, 1008, 963, 921, 881, 842, 806, 772, 739, 708, 678, 650, 623, 597, 573, 550, 527, 506, 486, 467, 448, 
                              431, 414, 398, 383, 368, 354, 341, 328, 316, 304, 293, 282, 272, 262, 253, 244, 235, 227, 219, 211, 204, 197, 
                              190, 184, 178, 172, 166, 160, 155, 150, 145, 140, 136, 131, 127, 123, 119, 115, 112, 108, 105, 102, 99, 96};
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
// const unsigned long updateAtempPeriod = 30000; // Update ambient temperature reading every 30sec
const unsigned long updateAtempPeriod = 1000; 
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

  // Serial.begin(9600);
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
  oled.setFont(Callibri11);

  dispInit = true;
  updateAtempMillis = millis();
}

/* INTERRUPT SERVICE ROUTINE FOR SAMPLING THERMOPILE VOLTAGE */
ISR (TIMER2_COMPA_vect) {
  cli(); // Disable interrupts

  int adc_Vtp = analogRead(VTP_PIN); // ADC reading of thermopile voltage

  // EMA filter formula: alpha*newSample + (1-alpha)*lastEMAvalue, with alpha=1/16 (to divide by power of 2)
  avg_adc_Vtp = (adc_Vtp + 99 * avg_adc_Vtp) / 100;

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
      // oled.setCursor(0, 1);
      // oled.print(F("Ambient temp: "));
      // oled.print(atemp);
      // oled.print(F("\200C ")); // \200 is octal escape for degree symbol

      // oled.setCursor(0, 2);
      // oled.print(F("Body temp: ---      "));

      // oled.setCursor(0, 3);
      // oled.print(F("Heart rate: ---    "));

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
    // int atempNew = round(0.3946 * avg_adc_Vptat - 242.3); // Calculate updated ambient temperature TODO: check this

    // // if (atempNew != atemp) { // Ambient temperature reading changed
    //   atemp = atempNew;
    //   // TODO: print atemp to OLED and RNBD serial
    //   oled.setCursor(0, 1);
    //   oled.print(F("Ambient temp: "));
    //   oled.print(atemp);
    //   oled.print(F("\200C ")); // \200 is octal escape for degree symbol

    //   oled.setCursor(0, 3);
    //   oled.print(F("Sensor temp: "));
    //   oled.print(stemp);
    //   oled.print(F("\200C   "));
    // // }

    float Vptat = avg_adc_Vptat * 5.0 / 1024;
    // oled.setCursor(0, 1);
    // oled.print(F("Vptat: "));
    // oled.print(Vptat, 4);
    // oled.print(F("V "));

    float atemp1 = 69.9 * Vptat - 217;
    float atemp2 = 89.5 * Vptat - 287;
    float atemp3 = 66.4 * Vptat - 208;
  
    oled.setCursor(0,0);
    oled.print(F("PT1: "));
    oled.print(atemp1, 2);

    oled.setCursor(56,0);
    oled.print(F("PT2: "));
    oled.println(atemp2, 2);

    oled.print(F("PT4: "));
    oled.print(atemp3, 2);

    updateAtempMillis = currentMillis;

    int adc_Vth = analogRead(VTH_PIN); // ADC reading of voltage across thermistor
    
  }

  int adc_Vth = analogRead(VTH_PIN); // ADC reading of voltage across thermistor

  // Only proceed if Vth is within the specified operating range
  // if (adc_Vth <= 1019 && adc_Vth >= 85) { // Limits are the values corresponding to 19 and 86°C TODO: change after calibration

    /* CALCULATE SENSOR AMBIENT TEMPERATURE */
    // Binary nearest neighbor search to map Vth reading to temperature; note Vth_adc_table is in descending order
    int left = 0;
    int right = Vth_table_len;
    int mid;
    // do {
    //   mid = left + floor((right-left)/2);
    //   if (Vth_adc_table1[mid] > adc_Vth)
    //     left = mid + 1;
    //   else
    //     right = mid;
    // } while(left < right); // At the end, right holds the index of the predecessor (smaller neighbor)

    // if (right == 0)
    //   stemp = 20;
    // else if (right == Vth_table_len)
    //   stemp = 19 + Vth_table_len;
    // else if ((adc_Vth - Vth_adc_table1[right]) <= (Vth_adc_table1[right-1] - adc_Vth)) // Vth reading is closer to the predecessor than to the successor (larger neighbor)
    //   stemp = 20 + right; // Use the predecessor index
    // else
    //   stemp = 19 + right; // Use the successor index (20 + right-1) = 19 + right

    // oled.setCursor(0,0);
    // oled.print(F("TH1: "));
    // oled.print(stemp);

    // // Binary nearest neighbor search to map Vth reading to temperature; note Vth_adc_table is in descending order
    // left = 0;
    // right = Vth_table_len;
    // do {
    //   mid = left + floor((right-left)/2);
    //   if (Vth_adc_table2[mid] > adc_Vth)
    //     left = mid + 1;
    //   else
    //     right = mid;
    // } while(left < right); // At the end, right holds the index of the predecessor (smaller neighbor)

    // if (right == 0)
    //   stemp = 20;
    // else if (right == Vth_table_len)
    //   stemp = 19 + Vth_table_len;
    // else if ((adc_Vth - Vth_adc_table2[right]) <= (Vth_adc_table2[right-1] - adc_Vth)) // Vth reading is closer to the predecessor than to the successor (larger neighbor)
    //   stemp = 20 + right; // Use the predecessor index
    // else
    //   stemp = 19 + right; // Use the successor index (20 + right-1) = 19 + right

    // oled.setCursor(56,0);
    // oled.print(F("TH2: "));
    // oled.println(stemp);

    // Binary nearest neighbor search to map Vth reading to temperature; note Vth_adc_table is in descending order
    left = 0;
    right = Vth_table_len;
    do {
      mid = left + floor((right-left)/2);
      if (Vth_adc_table3[mid] > adc_Vth)
        left = mid + 1;
      else
        right = mid;
    } while(left < right); // At the end, right holds the index of the predecessor (smaller neighbor)

    if (right == 0)
      stemp = 20;
    else if (right == Vth_table_len)
      stemp = 19 + Vth_table_len;
    else if ((adc_Vth - Vth_adc_table3[right]) <= (Vth_adc_table3[right-1] - adc_Vth)) // Vth reading is closer to the predecessor than to the successor (larger neighbor)
      stemp = 20 + right; // Use the predecessor index
    else
      stemp = 19 + right; // Use the successor index (20 + right-1) = 19 + right

    oled.setCursor(65,24);

    oled.print(F("TH3: "));
    oled.print(stemp);

    // // Binary nearest neighbor search to map Vth reading to temperature; note Vth_adc_table is in descending order
    // left = 0;
    // right = Vth_table_len;
    // do {
    //   mid = left + floor((right-left)/2);
    //   if (Vth_adc_table4[mid] > adc_Vth)
    //     left = mid + 1;
    //   else
    //     right = mid;
    // } while(left < right); // At the end, right holds the index of the predecessor (smaller neighbor)

    // if (right == 0)
    //   stemp = 20;
    // else if (right == Vth_table_len)
    //   stemp = 19 + Vth_table_len;
    // else if ((adc_Vth - Vth_adc_table4[right]) <= (Vth_adc_table4[right-1] - adc_Vth)) // Vth reading is closer to the predecessor than to the successor (larger neighbor)
    //   stemp = 20 + right; // Use the predecessor index
    // else
    //   stemp = 19 + right; // Use the successor index (20 + right-1) = 19 + right

    // oled.setCursor(65,24);
    // oled.print(F("TH4: "));
    // oled.println(stemp);

    /* CALCULATE BODY TEMPERATURE */
    float btemp1 = 0;
    float btemp2 = 0;
    // if (adc_Vth <= 1019 && adc_Vth >= 85) { // Limits are the values corresponding to 19 and 86°C TODO: change after calibration
    if (adc_Vth <= 1154 && adc_Vth >= 93) { // approx. limits
      // btemp = ((avg_adc_Vtp / 1024 * 5000 - 3732) / 675 - Btemp_line_const[stemp - 20]) / Btemp_line_slope;
      // Reduced above computations by absorbing constants into the line parameters
      btemp1 = avg_adc_Vtp * Btemp_line2 - Btemp_line3 - Btemp_line_const1[stemp - 20]; // TODO: change after calibration
      btemp2 = avg_adc_Vtp * Btemp_line4 - Btemp_line5 - Btemp_line_const2[stemp - 20];
    }

    /* CALCULATE HEART RATE */

    // int bpm = pulseSensor.getBeatsPerMinute();

    /* TODO: Compare/average(?) the sensor temp (from thermistor) and ambient temp (from PTAT) */

    /* BODY TEMPERATURE */
  if ((currentMillis - measBtempMillis) >= measBtempPeriod) {
    float Vtp = avg_adc_Vtp * 5.0 / 1024;
    // oled.setCursor(0, 0);
    // oled.print(F("TP1: "));
    // oled.print(btemp1, 2);
    // oled.setCursor(56, 0);
    // oled.print(F("TP1: "));
    // oled.println(btemp2, 2);
    measBtempMillis = currentMillis;
  }

    // if (dispBtemp) { // Displaying body temperature
    //   if ((currentMillis - dispBtempMillis) >= dispBtempPeriod) { // Display period elapsed
    //     // TODO: print "---" to OLED to indicate ready state
    //     oled.setCursor(0, 2);
    //     oled.print(F("Body temp: ---      "));
    //     ledWarningBtemp = false;
    //     if (!ledWarningHR)
    //       digitalWrite(LED_PIN, LOW);
    //     dispBtemp = false;
    //     avg_adc_Vtp = 0; // Reset ADC exponential moving average for body temperature to zero
    //     // This is to prevent measurement from starting again right away due to capacitor at ADC input not discharging fully
    //   }
    // }
    // else if (measBtemp) { // Measuring body temperature
    //   if ((currentMillis - measBtempMillis) >= measBtempPeriod) { // Measurement period elapsed
    //     // if (btemp >= 32 && btemp <= 43) { // Valid body temperature reading
    //       // TODO: print btemp to OLED and RNBD serial
    //       float Vtp = avg_adc_Vtp * 5.0 / 1024;
    //       oled.setCursor(0, 2);
    //       oled.print(F("Body temp: "));
    //       oled.print(Vtp, 4);
    //       oled.print(F("V "));
    //       // if (btemp >= 39.4) { // Body temperature too high, flash warning LED
    //       //   ledWarningBtemp = true;
    //       //   if (!ledWarningHR)
    //       //     ledMillis = currentMillis;
    //       // }
    //     // }
    //     // else {
    //     //   oled.setCursor(0, 2);
    //     //   oled.print(F("Body temp: ---      "));
    //     // }
    //     measBtemp = false;
    //     dispBtemp = true; // Start displaying
    //     dispBtempMillis = currentMillis;
    //   }
    // }
    // else { // Start measuring body temperature again
    //   // TODO: print "Measuring" to OLED
    //   oled.setCursor(0, 2);
    //   oled.print(F("Body temp: Measuring"));
    //   measBtemp = true; // Start measuring
    //   measBtempMillis = currentMillis;
    // }

    /* HEART RATE */
    // if (dispHR) { // Displaying heart rate
    //   if ((currentMillis - dispHRMillis) >= dispHRPeriod) { // Display period elapsed
    //     // TODO: print "---" to OLED to indicate ready state
    //     oled.setCursor(0, 3);
    //     oled.print(F("Heart rate: ---      "));
    //     ledWarningHR = false;
    //     if (!ledWarningBtemp)
    //       digitalWrite(LED_PIN, LOW);
    //     dispHR = false;
    //   }
    // }
    // else if (measHR) { // Measuring heart rate
    //   if ((currentMillis - measHRMillis) >= measHRPeriod) { // Measurement period elapsed
    //     if (bpm >= 30 && bpm <= 250) { // Valid heart rate reading
    //       // TODO: print heart rate to OLED and RNBD serial
    //       oled.setCursor(0, 3);
    //       oled.print(F("Heart rate: "));
    //       oled.print(bpm);
    //       oled.print(F(" BPM   "));
    //       if (bpm >= 180) { // Heart rate too high, flash warning LED
    //         ledWarningHR = true;
    //         if (!ledWarningBtemp)
    //           ledMillis = currentMillis;
    //       }
    //     }
    //     else {
    //       oled.setCursor(0, 3);
    //       oled.print(F("Heart rate: ---      "));
    //     }
    //     measHR = false;
    //     dispHR = true; // Start displaying
    //     dispHRMillis = currentMillis;
    //   }
    // }
    // else if (bpm >= 30 && bpm <= 250) { // Valid heart rate reading
    //   // TODO: print "Measuring" to OLED
    //   oled.setCursor(0, 3);
    //   oled.print(F("Heart rate: Measuring"));
    //   measHR = true; // Start measuring
    //   measHRMillis = currentMillis;
    // }

    /* WARNING LED */
    // if (ledWarningBtemp || ledWarningHR) {
    //   if ((currentMillis - ledMillis) < ledOnPeriod)
    //     digitalWrite(LED_PIN, HIGH); // Blink on
    //   else if ((currentMillis - ledMillis) < ledPeriod)
    //     digitalWrite(LED_PIN, LOW); // Blink off
    //   else
    //     ledMillis = currentMillis; // Repeat blink period
    // }

  // }
}

