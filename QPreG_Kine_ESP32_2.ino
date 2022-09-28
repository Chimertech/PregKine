#include "Wire.h"
#include "SPI.h"
#include "TFT_22_ILI9225.h"
#include "logo.h"
#include "RTClib.h"
#include "Preferences.h"
#include "limits.h"

#define TFT_RST 2
#define TFT_RS  15
#define TFT_CLK 18
#define TFT_SDI 23
#define TFT_CS  4
#define TFT_LED 0
#define TFT_BRIGHTNESS 200

#define Time_potPin 34
#define Temp_potPin 35
#define Heater_ONPin 16
#define Heater_OFFPin 17
#define Test_Pin 36
#define NTC_Pin 39
#define Lid_Pin 1
#define Heater_relayPin 25
//#define Fan_relayPin 1
#define Colorimeter_relayPin 32
#define Turbidity_relayPin 12
#define Buzzer_relayPin 13
SPIClass vspi(VSPI);
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);
RTC_DS1307 rtc;
Preferences preferences;
long duration;// holds the duration of timer
long rememTime;// holds current run time
int relayState = 0; // holds relay state
int heaterState = LOW;
int Time_potValue, Temp_potValue, durationCalc, TemperatureCalc, remaining, testState, Lid_State;
const int maxTime = 1000, minTime = 0, maxTemp = 800, minTemp = 0; //Seconds
const int tempTarget =  100, tempMin =  120, timerLimit = 10, buzzerKZ = 500 ; // Celcius, Kilo Hertz
const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned long startSecs = 0;

bool isRTCTimeOlderThanCompileTime(DateTime rtcTime , DateTime compileTime) {
  Serial.println("RTC Time " + rtcTime.timestamp());
  Serial.println("Compile Time " + compileTime.timestamp());
  return (rtcTime < compileTime);
}

void renderWelcomePage() {

  tft.setFont(Terminal12x16);
  tft.drawText(20, 20, "Q PreG - Kine", COLOR_TOMATO);
  tft.setFont(Terminal6x8);
  tft.drawText(35, 50, "Under Validation", COLOR_CYAN);
  tft.drawText(40, 65, "Version - 1.0.1", COLOR_CYAN);
  delay(300);

  for (uint8_t i = 0; i < 100; i++)
    tft.drawPixel(random(tft.maxX()), random(tft.maxY()), random(0xffff));
  tft.setFont(Terminal12x16);
  tft.drawText(30, 90, "Chimertech");
  tft.drawText(25, 115, "Private LTD");
  tft.drawBitmap(15, 130, Logo, 150, 90, COLOR_DARKGREEN);
  delay(2000);
  tft.clear();
}

float calculateTemp() {
  int Vo;
  float R1 = 1000;
  float logR2, R2, T, TC, Tf;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  Vo = analogRead(NTC_Pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  TC = T - 273.15;
  return TC;
}

void renderBatteryLevelAndQuadPie() {

  DateTime now = rtc.now();
  int batt;
  tft.fillRectangle( 0, 0, 175, 20, COLOR_BLUE);
  tft.setFont(Terminal6x8);
  tft.setBackgroundColor(COLOR_BLUE);

  tft.drawText(60, 3, String (now.day()) + "/" + String (now.month()) + "/" + String (now.year()) + "");

  //Need to fix for leading zeros
  tft.drawText(63, 13, now.hour() < 12  ? String(now.hour()) + ":" + String (now.second()) + " AM " :
               ( now.hour() == 12 ? String(now.hour()) + ":" + String (now.minute()) + " PM " :
                 String(now.hour() - 12 ) + ":" + String (now.minute()) + ":" + String (now.second()) + "PM "));

  batt = 60; //constrain(map(analogRead (27) , 2200, 2300, 0, 100 ), 0, 100); // Li-Ion battery should have minimum of 3.6V and a max of 4.2v to run Quadmastest.
  // 3.6V corresponds to 655 digital and 4.2v to 880.

  //tft.setFont(Terminal6x8);

  if (batt >= 60 ) {
    tft.drawBitmap(143, 3, Batt , 30, 15, COLOR_GREEN);
    tft.drawText(146, 7, String (batt) + "%", COLOR_GREEN);
  }
  else if (batt >= 20 ) {
    tft.drawBitmap(143, 3, Batt , 30, 15, COLOR_ORANGE);
    tft.drawText(146, 7, String (batt) + "%", COLOR_ORANGE);
  }
  else if (batt >= 10 ) {
    tft.drawBitmap(143, 3, Batt , 30, 15, COLOR_RED);
    tft.drawText(146, 7, String (batt) + "%", COLOR_RED);
  }
  else {
    //    checkBatteryAndRenderPrompt();
  }
  tft.setBackgroundColor(COLOR_BLACK);
  tft.setFont(Terminal12x16);
}

long calculateTimeLeft(long currentSecs, long startSecs) {

  if (startSecs == 0)
    return INT_MAX;

  long timeElapsed = currentSecs - startSecs;

  return timerLimit > timeElapsed ? timerLimit - timeElapsed : 0;
}
void MainTemplate() {
  int timeLeft;
  int TC;
  int SetTemp;

  tft.drawRectangle(0, 21, 175, 36, COLOR_BLUE);
  tft.drawRectangle(0, 36, 175, 51, COLOR_BLUE);
  tft.drawRectangle(0, 51, 175, 66, COLOR_BLUE);
  tft.drawRectangle(0, 66, 175, 81, COLOR_BLUE);
  tft.drawRectangle(0, 81, 175, 96, COLOR_BLUE);
  tft.drawRectangle(0, 96, 175, 111, COLOR_BLUE);
  tft.drawLine(115, 21, 115, 96, COLOR_BLUE);

  Time_potValue = analogRead(Time_potPin) / 10; // reads the value of the potentiometer (value between 0 and 4095)
  durationCalc = map(Time_potValue, 0, 100, minTime, maxTime);// convert A0 value to time set at minTime and maxTime

  Temp_potValue = analogRead(Temp_potPin) / 10;
  TemperatureCalc = map(Temp_potValue, 0, 100, minTemp, maxTemp);

  TC = TemperatureCalc;  //calculateTemp();

  timeLeft = calculateTimeLeft(millis() / 1000, startSecs);
  Serial.print("Time_potValue");
  tft.setBackgroundColor(COLOR_BLACK);
  Serial.print("Time set: ");
  Serial.print(durationCalc / 60, 1);
  Serial.print (" Mints ");
  Serial.print("Set Temp: ");
  Serial.print(TemperatureCalc);
  Serial.print (" C ");
  tft.setFont(Terminal6x8);
  tft.drawText(4, 26, "Set Time", COLOR_GOLD);
  tft.drawText(120, 26, String (durationCalc / 60) + " M  ", COLOR_GOLD);
  tft.drawText(4, 41, "Set Temperature" , COLOR_GOLD);
  tft.drawText(130, 41,  String (TemperatureCalc) + " C  ", COLOR_GOLD);
  tft.drawText(4, 56, "Live Temperature" , COLOR_GOLD);
  tft.drawText(130, 56, String (TC) + " C  ", COLOR_GOLD);
}
void relaycontrol() {

  if (digitalRead(Heater_ONPin) == LOW) {
    duration = durationCalc;
    rememTime = millis() / 1000;
    relayState = 1;// change relay state to ON
  }
  if (( millis() / 1000 - rememTime) > duration || digitalRead(Heater_OFFPin) == LOW )  {
    relayState = 0;// change relay state to OFF
  }
  if (digitalRead(Test_Pin) == LOW) {
    testState = 1;
  }

  if (relayState == 1) {
    int TC;
    Temp_potValue = analogRead(Temp_potPin) / 10;
    TemperatureCalc = map(Temp_potValue, 0, 100, minTemp, maxTemp);
    TC = TemperatureCalc;

    if (tempTarget > TC) {
      digitalWrite(Heater_relayPin, HIGH);
    }
    else if (tempMin < TC) {
      digitalWrite(Heater_relayPin, LOW);
    }
    Serial.println(TC);
    Serial.print(" Remaining: ");
    remaining = (getRtime());
    Serial.print(remaining / 60, 1);
    Serial.print(" Mints ");
    tft.setFont(Terminal6x8);
    tft.drawText(4, 71, "Heater Status ", COLOR_RED);
    tft.drawText(130, 71, " ON  ", COLOR_RED);
    tft.drawText(4, 86, "Remaining Time   ", COLOR_RED);
    tft.drawText(120, 86, String (remaining / 60) + " M ", COLOR_RED);
  }
  else {
    tft.setFont(Terminal6x8);
    tft.drawText(4, 71, "Heater Status ", COLOR_GREEN);
    tft.drawText(130, 71, " OFF ", COLOR_GREEN);
    tft.drawText(4, 86, "Press Heater ON", COLOR_GREEN);
    tft.drawText(120, 86, "           ", COLOR_BLACK);
    digitalWrite(Heater_relayPin, LOW);
  }
}
/*
  void triggerBuzzer() {
  tone(BUZZERPIN, buzzerKZ);
  delay(2000);
  noTone(BUZZERPIN);
  }
*/
void setup() {
  pinMode(Test_Pin, INPUT_PULLUP);
  pinMode(Heater_ONPin, INPUT_PULLUP);
  pinMode(Heater_OFFPin, INPUT_PULLUP);
  pinMode(NTC_Pin, INPUT);
  pinMode(Lid_Pin, INPUT_PULLUP);
  pinMode(Heater_relayPin, OUTPUT);
  pinMode(Colorimeter_relayPin, OUTPUT);
  pinMode(Turbidity_relayPin, OUTPUT);
  pinMode(BUZZERPIN, OUTPUT);
#if defined(ESP32)
  vspi.begin();
  tft.begin(vspi);
#else
  tft.begin();
#endif
  Serial.begin(115200);

  renderWelcomePage();

  DateTime compileTime = DateTime(F(__DATE__), F(__TIME__));

  if (! rtc.begin() || ! rtc.isrunning() ) {
    Serial.println("Couldn't find RTC Or RTC is NOT running, Setting RTC time as compile time");
    rtc.adjust(compileTime);
  }
  else if ( isRTCTimeOlderThanCompileTime( rtc.now(), compileTime ) ) {
    Serial.println("RTC time Older than Compile time, Setting RTC time as compile time");
    rtc.adjust(compileTime);
  }
  tft.clear();
  delay(100);
}

void loop() {
  int TC;
  preferences.begin("my-app", false);
  unsigned int testNumber = preferences.getUInt("test-number", 0);
  unsigned int temperature = preferences.getUInt("temperature", 0);
  unsigned int colorimeter_value = preferences.getUInt("colorimeter", 0);
  unsigned int Turbidity_value = preferences.getUInt("turbidity", 0);

  Temp_potValue = analogRead(Temp_potPin) / 10;
  TemperatureCalc = map(Temp_potValue, 0, 100, minTemp, maxTemp);
  TC = TemperatureCalc;
  //TC = calculateTemp();

  renderBatteryLevelAndQuadPie();
  MainTemplate();
  relaycontrol();

  if (testState == 1) {
    for (int i = 0; i < 4; i++) {
      MainTemplate();
      renderBatteryLevelAndQuadPie();
      relaycontrol();
      digitalWrite(Colorimeter_relayPin, HIGH);
      tft.setFont(Terminal12x16);
      int sensorValue = analogRead(26);
      float Colorimeter_voltage = sensorValue * (5.0 / 1023.0);
      Serial.println(Colorimeter_voltage);
      double Transmission = (Colorimeter_voltage / 4.15);
      double param, result;
      double colorimeter_value;
      colorimeter_value = 2 - log10(Transmission);
      tft.drawText(30, 115, "Colorimeter");
      tft.fillRectangle(7, 135, 176, 150, COLOR_BLACK);
      tft.drawText (15, 135,   "  OD =  " + String (colorimeter_value, 2  ), COLOR_YELLOW);
      tft.drawText(10, 175, (" Wait for 5sec "), COLOR_YELLOW);
      tft.fillRectangle( 0, 195, 176, 220, COLOR_DARKBLUE);
      tft.setBackgroundColor(COLOR_DARKBLUE);
      tft.drawText(5, 200, " Testing..... " + String (), COLOR_YELLOW);
      tft.setBackgroundColor(COLOR_BLACK);
      delay(1000);
    }
    digitalWrite(Colorimeter_relayPin, LOW);
    for (int i = 0; i < 4; i++) {
      MainTemplate();
      renderBatteryLevelAndQuadPie();
      relaycontrol();
      tft.setFont(Terminal12x16);
      digitalWrite(Turbidity_relayPin, HIGH);
      int sensorValue = analogRead(33);
      float Turbidity_voltage = sensorValue * (5.0 / 1023.0);
      Serial.println(Turbidity_voltage);
      double Transmission = (Turbidity_voltage / 4.15);
      double param, result;
      double Turbidity_value;
      Turbidity_value = 2 - log10(Transmission);
      tft.drawText(35, 155, "Turbidity");
      tft.fillRectangle(7, 175, 176, 190, COLOR_BLACK);
      tft.drawText(10, 175,  "  NTU = "  + String (Turbidity_value, 2  ), COLOR_YELLOW);
      delay(1000);
    }
    renderBatteryLevelAndQuadPie();
    digitalWrite(Turbidity_relayPin, LOW);
    testNumber++;
    Serial.printf("Current Test Number : %u\n", testNumber);
    preferences.putUInt("test-number", testNumber);
    preferences.end();
    tft.fillRectangle( 0, 195, 176, 220, COLOR_DARKBLUE);
    tft.setBackgroundColor(COLOR_DARKBLUE);
    tft.drawText(5, 200, "Test  No : " + String (testNumber), COLOR_GREEN);
    tft.setBackgroundColor(COLOR_BLACK);
    testState = 0;
  }
  else {
    renderBatteryLevelAndQuadPie();
    relaycontrol();
    tft.setFont(Terminal12x16);
    tft.fillRectangle( 0, 195, 176, 220, COLOR_DARKBLUE);
    tft.setBackgroundColor(COLOR_DARKBLUE);
    tft.drawText(5, 200, "Test  No : " + String (testNumber), COLOR_GREEN);
    tft.setBackgroundColor(COLOR_BLACK);
    tft.drawText(30, 115, "Colorimeter");
    tft.drawText(1, 135, String ("  Press Test ON  ") + "", COLOR_YELLOW);
    tft.drawText(35, 155, "Turbidity");
    tft.drawText(1, 175, String ("  Press Test ON  ") + "", COLOR_YELLOW);
  }
  Serial.println();
  delay(500);
}

int getRtime() {
  return duration - (millis() / 1000 - rememTime);
}
