#include <SoftwareSerial.h>
#include <Wire.h>
#include <SHT2x.h>
#include <Adafruit_NeoPixel.h>
#include <Microduino_Motor.h>

#define PIN 12    //连接引脚
#define LEDquantity 1 //LED灯珠数量

SoftwareSerial pmSerial(4, 5);  //PM2.5传感器通讯软串口
#define INTERVAL_pm25             200
unsigned long pm25_time = millis();
int prevState = 1;
int prevState2 = 1;

// init mod
// 0 => 风扇
// 1 => 净化
// 2 => 雨伞
int mod = 0;

// init button port
int pushButton = 6; //传感器接senhub D6接口
int pushButton2 = 8;

const int BzPin =  10;
boolean State = LOW;

// 风扇控制(占用5,6,7,8) 需要调整hub插口
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
#define MAX_THROTTLE 255
#define MAX_STEERING 200
int16_t throttle = 0;
int16_t steering = 0;

boolean slow;
boolean OFF;
// 风扇控制结束

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDquantity, PIN, NEO_GRB + NEO_KHZ800);

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
}

void loop() {
  // read the input pin:
  int buttonState = digitalRead(pushButton);
  delay(1);
  int buttonState2 = digitalRead(pushButton2);
  delay(1);
  // print out the state of the button:

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

  // 如果当前按钮1被持续点击则不做反应
  if (prevState != buttonState) {
    if (buttonState == 0) {
      Serial.println("Button1Click");
      Serial.print("Humidity(%RH): ");
      Serial.print(SHT2x.GetHumidity());
      Serial.print("     Temperature(C): ");
      Serial.println(SHT2x.GetTemperature());
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
      Serial.println("Button2Click");
    }
    prevState2 = buttonState2;
  }

  if (pm25_time > millis()) pm25_time = millis();
  if (millis() - pm25_time > INTERVAL_pm25) {
    Serial.println(PM25()) ;
    pm25_time = millis();    //更新计时器
  }
}

void updateFanWithFanMode() {
  return;
}

void updateFanWithPurificationMode() {
  return;
}

void FanWithUmbMode() {
  int sensorValue = analogRead(A0);
  delay(1);
  if (sensorValue <= 900) {
    Serial.println("TMD");
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


