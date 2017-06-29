#define SSID "123321"
#define PASS "12345678"

char auth[] = "bcefc62b0fc84260b3e89eaa3cc7fdc9";

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

////SoftwareSerial pmSerial(12, 13);  //PM2.5传感器通讯软串口
//#define INTERVAL_pm25             200
//unsigned long pm25_time = millis();

//int prevState = 1;
//int prevState2 = 1;

#define INTERVAL_temhum           200
unsigned long temhum = millis();

// init mod
// 0 => 风扇
// 1 => 净化
// 2 => 雨伞
int mod = 0;

// init button port
//#define pushButton 2 //传感器接senhub D2接口
//#define pushButton2 3

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

void updateFanWithFanMode();
void updateFanWithPurificationMode();
void FanWithUmbMode();





