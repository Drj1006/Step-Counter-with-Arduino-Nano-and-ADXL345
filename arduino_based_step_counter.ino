#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

const int buttonPinOnOff = 7;  
const int buttonPinPause = 8;  

unsigned long debounceDelay = 50;
unsigned long pressStartTime = 0;
bool longPressDetected = false;
bool isPaused = true; 
bool isOn = false;   

int steps = 0;
int flag = 0;
float threshold = 11.6; 
float xval[100] = {0}, yval[100] = {0}, zval[100] = {0};
float xavg, yavg, zavg;

void calibrate() {
  float sum = 0, sum1 = 0, sum2 = 0;
  sensors_event_t event;
  
  for (int i = 0; i < 100; i++) {
    accel.getEvent(&event);
    xval[i] = event.acceleration.x;
    sum += xval[i];
  }
  xavg = sum / 100.0;

  for (int j = 0; j < 100; j++) {
    accel.getEvent(&event);
    yval[j] = event.acceleration.y;
    sum1 += yval[j];
  }
  yavg = sum1 / 100.0;


  for (int q = 0; q < 100; q++) {
    accel.getEvent(&event);
    zval[q] = event.acceleration.z;
    sum2 += zval[q];
  }
  zavg = sum2 / 100.0;
}

#define SAMPLE_SIZE 5  

float smoothData(float newData, float previousAvg) {
  static float buffer[SAMPLE_SIZE] = {0};
  static int index = 0;
  float sum = 0;


  buffer[index] = newData;
  index = (index + 1) % SAMPLE_SIZE;

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    sum += buffer[i];
  }

  return sum / SAMPLE_SIZE;
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  if (!accel.begin()) {
    Serial.println("Couldn't find the ADXL345 sensor.");
    while (1);
  }

  calibrate();

  pinMode(buttonPinOnOff, INPUT_PULLUP);
  pinMode(buttonPinPause, INPUT_PULLUP);
}

void loop() {
  handlePowerButton();

  handlePauseResetButton();

  if (isOn && !isPaused) {
    countSteps();
  }
}

void handlePowerButton() {
  static unsigned long lastPowerPress = 0;
  if (digitalRead(buttonPinOnOff) == LOW && millis() - lastPowerPress > debounceDelay) {
    lastPowerPress = millis();
    isOn = !isOn;

    if (isOn) {
      lcd.clear();
      lcd.print("Powering on...");
      lcd.backlight();
      delay(1000);
      steps = 0; 
      lcd.clear();
      lcd.print("Steps: 0");
    } else {
      lcd.clear();
      lcd.print("Powering off...");
      lcd.noBacklight();
      delay(1000);
      lcd.clear();
    }
  }
}

void handlePauseResetButton() {
  static unsigned long lastPausePress = 0;

  if (digitalRead(buttonPinPause) == LOW && millis() - lastPausePress > debounceDelay) {
    lastPausePress = millis();

    if (!longPressDetected) {
      pressStartTime = millis();
      longPressDetected = true;
    }

    if (millis() - pressStartTime > 1000) {
      steps = 0;
      lcd.clear();
      lcd.print("Steps: 0");
      lcd.setCursor(0, 1);
      lcd.print("Reset Done");
      delay(1000);
      lcd.clear();
      lcd.print("Steps: 0");
      isPaused = true;
      longPressDetected = false;
    }
  } else if (digitalRead(buttonPinPause) == HIGH) {
  
    if (longPressDetected && (millis() - pressStartTime) < 1000) {
      isPaused = !isPaused;
      lcd.setCursor(0, 1);
      if (isPaused) {
        lcd.print("Paused     ");
      } else {
        lcd.print("            "); 
      }
    }
    longPressDetected = false;
  }
}

void countSteps() {
  sensors_event_t event;
  accel.getEvent(&event);

  float totvect = sqrt(event.acceleration.x * event.acceleration.x +
                       event.acceleration.y * event.acceleration.y +
                       event.acceleration.z * event.acceleration.z);

  float smoothedTotVect = smoothData(totvect, 0);

  if (smoothedTotVect > threshold && flag == 0) {
    steps++;
    flag = 1;
  } else if (smoothedTotVect < threshold && flag == 1) {
    flag = 0;
  }

  lcd.setCursor(0, 0);
  lcd.print("Steps: ");
  lcd.print(steps);
}