#include <SoftwareSerial.h>
#include <Wire.h>
#include <SHT2x.h>
#include <Adafruit_NeoPixel.h>
#include <Microduino_Motor.h>
#define BLYNK_PRINT Serial // Enables Serial Monitor

#include "userDef.h"
#include "sensor.h"
#include "oled.h"
#include "WiFiBlynk.h"

float pm25 = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600); // See the connection status in Serial Monitor
  // pmSerial.begin(2400);   //首先启动软串口
  EspSerial.begin(115200);
  Blynk.begin(auth, wifi, SSID, PASS);

  temHumtimer.setInterval(2000L, senTempHumi);
  delay(10);
  
  // button input
//  pinMode(pushButton, INPUT);
//  pinMode(pushButton2, INPUT);

  strip.begin();  //初始化必要函数
  strip.show(); // Initialize all pixels to 'off'

  pinMode(BzPin, OUTPUT);
  
  delay(10);

  MotorLeft.Fix(1);
  MotorRight.Fix(1);
  MotorBottom.Fix(1);

  delay(500);

  slow = 0;
  OFF = 0;
}

void loop() {
  MotorLeft.Driver(MotorLeft.GetData(255, 0, CHAN_LEFT));
//  delay(100);
//  MotorRight.Driver(MotorRight.GetData(20, 0, CHAN_RIGHT));
//  delay(100);
//  MotorBottom.Driver(MotorBottom.GetData(20, 0, CHAN_LEFT));
  
  Blynk.run();
  temHumtimer.run();
  updateTempHumi();
  updateMode(mod);

  if (pm25_time > millis()) pm25_time = millis();
  if (millis() - pm25_time > INTERVAL_pm25) {   
    sensorPM25=PM25();
    pm25 = sensorPM25;
    pm25_time = millis();    //更新计时器
  }
  
  // read the input pin:
//  int buttonState = digitalRead(pushButton);
//  delay(1);
//  int buttonState2 = digitalRead(pushButton2);
//  delay(1);

  // 如果当前按钮1被持续点击则不做反应
//  if (prevState != buttonState) {
//    if (buttonState == 0) {
//      ++mod;
//      mod = mod % 3;
//      tone(BzPin, 1000, 75);
//      Serial.println(mod);
//    }
//    prevState = buttonState;
//  }
//
//  // 如果当前按钮2被持续点击则不做反应(有问题)
//  if (prevState2 != buttonState2) {
//    if (buttonState2 == 0) {
////      Serial.println("Button2Click");
//    }
//    prevState2 = buttonState2;
//  }
}

void updateFanWithFanMode() {
  if(modifyFan == true){
    modifyFan = false;
    MotorLeft.Driver(MotorLeft.GetData(throttle, 0, CHAN_LEFT));
    MotorRight.Driver(MotorRight.GetData(throttle, 0, CHAN_RIGHT));
    MotorBottom.Driver(MotorBottom.GetData(throttle, 0, CHAN_LEFT));
    temhum = millis();    //更新计时器
    return;
  }
  
  if (temhum > millis()) temhum = millis();
  if (millis() - temhum > INTERVAL_temhum) {
    const int readin = sensor_tem;
    // < 32 255
    if(readin >= 32){
      throttle = 255;
    } 
    // < 30 128
    else if(readin >= 30){
      throttle = 128;
    }
    // < 28 64
    else if(readin >= 28){
      throttle = 64;
    }
  //   20     < 25
    else{
      throttle = 20;
    }

    MotorLeft.Driver(MotorLeft.GetData(throttle, 0, CHAN_LEFT));
    MotorRight.Driver(MotorRight.GetData(throttle, 0, CHAN_RIGHT));
    MotorBottom.Driver(MotorBottom.GetData(throttle, 0, CHAN_LEFT));
    temhum = millis();    //更新计时器
  }
  return;
}

void updateFanWithPurificationMode() {
  if(modifyFan == true){
    modifyFan = false;
    MotorLeft.Driver(MotorLeft.GetData(throttle, 0, CHAN_LEFT));
    MotorRight.Driver(MotorRight.GetData(throttle, 0, CHAN_RIGHT));
    MotorBottom.Driver(MotorBottom.GetData(throttle, 0, CHAN_LEFT));
    return;
  }
  // < 200 255
  if(pm25 >= 200){
    throttle = 255;
  }
  // < 100 128
  else if(pm25 >= 100){
    throttle = 128;
  }
  // < 75  64
  else if(pm25 >= 75){
    throttle = 64;
  }
  //   20     < 75
  else{
    throttle = 20;
  }
  
  throttle = 128;
  MotorLeft.Driver(MotorLeft.GetData(throttle, 0, CHAN_LEFT));
  MotorRight.Driver(MotorRight.GetData(throttle, 0, CHAN_RIGHT));
  MotorBottom.Driver(MotorBottom.GetData(throttle, 0, CHAN_LEFT));
  return;
}

void FanWithUmbMode() {
  int sensorValue = analogRead(A6);
  delay(1);
  if (sensorValue <= 900) {
    throttle = 255;
    MotorLeft.Driver(MotorLeft.GetData(throttle, 0, CHAN_LEFT));
    MotorRight.Driver(MotorRight.GetData(throttle, 0, CHAN_RIGHT));
    MotorBottom.Driver(MotorBottom.GetData(throttle, 0, CHAN_LEFT));
  }
  else{
    throttle = 0;
    MotorLeft.Driver(MotorLeft.GetData(throttle, 0, CHAN_LEFT));
    MotorRight.Driver(MotorRight.GetData(throttle, 0, CHAN_RIGHT));
    MotorBottom.Driver(MotorBottom.GetData(throttle, 0, CHAN_LEFT));
  return;
  }
}


