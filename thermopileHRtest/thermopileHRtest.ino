#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PulseSensorPlayground.h>

#define tPilePin A0
#define HRpin A1
#define thermistorPin A2
#define ptatPin A3

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the OLED name
SSD1306AsciiWire oled;
PulseSensorPlayground pulseSensor;

const int tPileSamples = 50;
unsigned long timeTracker = 0;
int threshold = 510;

int starttime = 0;
int endtime = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(tPilePin, INPUT);
  pinMode(thermistorPin, INPUT);
  pinMode(ptatPin, INPUT);

  oled.begin(&Adafruit128x32, 0x3C); //Start the OLED display
  oled.setFont(Callibri11);

  pulseSensor.analogInput(A1);
  pulseSensor.blinkOnPulse(LED_BUILTIN);
  pulseSensor.setThreshold(threshold);
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }

  delay(500);

}

void loop() {
  // put your main code here, to run repeatedly:
  //int summer1 = 0;

  // for (int i = 0; i < tPileSamples; i++) {
  //   summer1+=analogRead(tPilePin);
  //   delay(10);
  // }
  // int tPileAnalog = summer1 / tPileSamples; // take avg of samples

  Serial.print("Start Time: ");
  starttime = millis();
  Serial.print(starttime);

  int tPileAnalog = analogRead(tPilePin);
  int bpm = pulseSensor.getBeatsPerMinute();
  int thermistorAnalog = analogRead(thermistorPin);
  int ptatAnalog = analogRead(ptatPin);

  float tPileVoltage = tPileAnalog * (5.0/1023.0);
  float thermistorVoltage = thermistorAnalog * (5.0/1023.0);
  float ptatVoltage = ptatAnalog * (5.0/1023.0);
  //Serial.print("Thermopile Analog Value: ");
  //Serial.print(tPileAnalog);
  Serial.print("    Thermopile Voltage Value: ");
  Serial.print(tPileVoltage, 4);
  //Serial.print("    Mistor Analog Value: ");
  //Serial.print(thermistorAnalog);
  Serial.print("    Mistor Voltage Value: ");
  Serial.print(thermistorVoltage, 4);
  Serial.print ("   PTAT: ");
  Serial.print(ptatVoltage, 4);
  Serial.print("    HR: ");
  Serial.println(bpm);
  



  if (millis() - timeTracker > 1000) {
    oled.clear();
    oled.setCursor(0,0);
    // oled.print(F("TP A Val: "));
    // oled.println(tPileAnalog, DEC);
    //oled.setCursor(0, 24);
    oled.print(F("Vtp: "));
    oled.println(tPileVoltage, 4);

    oled.setCursor(60, 0);
    oled.print(F("Vam: "));
    oled.println(thermistorVoltage, 4);

    // oled.print(F("TM A Val: "));
    // oled.println(thermistorAnalog, DEC);
    oled.print(F("Vam: "));
    oled.println(thermistorVoltage, 4);
    oled.print(F("VPTAT: "));
    oled.println(ptatVoltage, 4);

    oled.setCursor(78, 24);
    oled.print(F("HR: "));
    oled.println(bpm);
    timeTracker = millis();
  }


}
