#include <SoftwareSerial.h>
#include <Wire.h>
#include <SHT2x.h>
#include <Adafruit_NeoPixel.h>
#include <Microduino_Motor.h>

//#include <ESP8266_HardSer.h>
//#include <BlynkSimpleShieldEsp8266_HardSer.h>
//
//#if defined(__AVR_ATmega32U4__)
//#define BLYNK_PRINT Serial
//#define EspSerial Serial1
//#else if defined(__AVR_ATmega328P__) || (__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega128RFA1__)
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(2, 3); // RX, TX
//#define BLYNK_PRINT mySerial
//#define EspSerial Serial
//#endif

#define PIN 11    //连接引脚
#define LEDquantity 1 //LED灯珠数量

SoftwareSerial pmSerial(12, 13);  //PM2.5传感器通讯软串口
#define INTERVAL_pm25             200
unsigned long pm25_time = millis();

int prevState = 1;
int prevState2 = 1;

#define INTERVAL_temhum           1250
unsigned long temhum = millis();

// init mod
// 0 => 风扇
// 1 => 净化
// 2 => 雨伞
int mod = 0;

// init button port
#define pushButton 2 //传感器接senhub D2接口
#define pushButton2 3

const int BzPin =  10;
boolean State = LOW;

// 风扇控制(占用D5,6,7,8) 需要调整hub插口
#if defined(__AVR_ATmega32U4__) || (__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega128RFA1__)
#define motor_pin0A 8
#define motor_pin0B 6
#define motor_pin1A 7
#define motor_pin1B 5
#else
#define motor_pin0A 6
#define motor_pin0B 8
#define motor_pin1A 5
#define motor_pin1B 7
#endif
Motor MotorLeft(motor_pin0A, motor_pin0B);
Motor MotorRight(motor_pin1A, motor_pin1B);
Motor MotorBottom(A0, B0);
#define MAX_THROTTLE 255
#define MAX_STEERING 200
int16_t throttle = 0;
int16_t steering = 0;

boolean slow;
boolean OFF;
// 风扇控制结束

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDquantity, PIN, NEO_GRB + NEO_KHZ800);

////ESP8266 wifi(EspSerial);
//char auth[] = "9eb84af6803d45b1aef11a08fbefb575";

float pm25 = 0;

void setup() {
  Serial.begin(9600); // See the connection status in Serial Monitor
  pmSerial.begin(2400);   //首先启动软串口
  Wire.begin();
  // button input
  pinMode(pushButton, INPUT);
  pinMode(pushButton2, INPUT);

  strip.begin();  //初始化必要函数
  strip.show(); // Initialize all pixels to 'off'

  pinMode(BzPin, OUTPUT);

//  EspSerial.begin(115200);
  delay(10);
//  Blynk.begin(auth, wifi, "123321", "12345678");

  MotorLeft.Fix(1);
  MotorRight.Fix(1);
  MotorBottom.Fix(1);

  delay(1000);

  slow = 0;
  OFF = 0;
}

void loop() {
  // read the input pin:
  int buttonState = digitalRead(pushButton);
  delay(1);
  int buttonState2 = digitalRead(pushButton2);
  delay(1);

  // 如果当前按钮1被持续点击则不做反应
  if (prevState != buttonState) {
    if (buttonState == 0) {
      ++mod;
      mod = mod % 3;
      tone(BzPin, 1000, 75);
      Serial.println(mod);
    }
    prevState = buttonState;
  }

  // 如果当前按钮2被持续点击则不做反应(有问题)
  if (prevState2 != buttonState2) {
    if (buttonState2 == 0) {
//      Serial.println("Button2Click");
    }
    prevState2 = buttonState2;
  }
    switch (mod) {
    case 0:
      strip.setPixelColor(0, 0, 0, 255);
      strip.show();
      updateFanWithFanMode();
      break;
    case 1:
      strip.setPixelColor(0, 0, 255, 0);
      strip.show();
      updateFanWithPurificationMode();
      break;
    default:
      strip.setPixelColor(0, 255, 255, 255);
      strip.show();
      FanWithUmbMode();
      break;
  }

//  Blynk.run();
}

void updateFanWithFanMode() {
  if (temhum > millis()) temhum = millis();
  if (millis() - temhum > INTERVAL_temhum) {
//    Serial.print("Humidity(%RH): ");
//    Serial.print(SHT2x.GetHumidity());
//    Serial.print("     Temperature(C): ");
//    Serial.println(SHT2x.GetTemperature());
    // < 32 255
    if(SHT2x.GetTemperature() >= 32){
      throttle = 255;
    } 
    // < 30 128
    else if(SHT2x.GetTemperature() >= 30){
      throttle = 128;
    }
    // < 28 64
    else if(SHT2x.GetTemperature() >= 28){
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

  if (pm25_time > millis()) pm25_time = millis();
  if (millis() - pm25_time > INTERVAL_pm25) {
    pm25 = PM25();
    pm25_time = millis();    //更新计时器
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

float PM25() {
  int data_s = 0;    //串口接收数据
  int num = -1;      //串口接收数据计数
  int sum = 0;       //校验和
  int cal[5];        //接收数据缓存
  float dustDensity = 0;  //PM2.5浓度

  pmSerial.listen();
  while (1) {
    if (pmSerial.available() > 0) { //串口缓存有数据
      data_s = pmSerial.read();   //读取串口缓存数据
      if (data_s == 0xAA) {        //得到数据帧起始位
        num = 0;                  //开始计数
      }
      else if (num >= 0) {
        cal[num++] = data_s; //读到数据，计数+1，数据保存到缓存中
        if (num == 6) {        //读到数据帧最后一位
          sum = cal[0] + cal[1] + cal[2] + cal[3];  //计算校验和
          if (sum == cal[4] && cal[5] == 0xFF) {    //校验和匹配，数据帧最后一位为0xFF，说明接收的数据帧正常
            dustDensity = (cal[0] * 256 + cal[1]) * (5.0 / 1024) * 550; //计算PM2.5浓度，单位ug/m3
          }
          else {     //接收的数据不正常
            dustDensity = 0;    //浓度清零
          }
          break;
        }
      }
    }
  }
  pmSerial.stopListening();
  return dustDensity ;
}

//BLYNK_READ(V0) {
//  Blynk.virtualWrite(V0, SHT2x.GetTemperature());
//}
//
//BLYNK_READ(V1) {
//  Blynk.virtualWrite(V1, SHT2x.GetHumidity());
//}
//
//BLYNK_READ(V2) {
//  Blynk.virtualWrite(V2, pm25);
//}
//
//BLYNK_READ(V3) {
//  Blynk.virtualWrite(V3, mod);
//}


