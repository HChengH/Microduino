#include <SoftwareSerial.h>
#include <SHT2x.h>

#define INTERVAL_pm25             200 
unsigned long pm25_time = millis();

SoftwareSerial pmSerial(12,13);   //PM2.5传感器通讯软串口


float sensor_tem,sensor_hum,sensor_mode;

float sensorPM25;

int fanSpeed;

int mode;

boolean modifyFan = false;


float PM25() {
  uint8_t rxData = 0;    //串口接收数据
  int8_t index = -1;      //串口接收数据计数
  uint8_t checksum = 0;       //校验和
  uint8_t buf[5];        //接收数据缓存
  float dustDensity = 0;  //PM2.5浓度
  uint32_t pmTimer = millis() + 300;
  
  pmSerial.begin(2400);   //首先启动软串口
  
  while(millis() < pmTimer) { 
    if(pmSerial.available() > 0) { //串口缓存有数据
      rxData = pmSerial.read();   //读取串口缓存数据
      if(rxData == 0xAA) {         //得到数据帧起始位
        index = 0;                  //开始计数
      }
      else if(index >= 0) {    
        buf[index++] = rxData;  //数据保存到缓存中
        if(index == 6) {         //读到数据帧最后一位
          checksum = buf[0] + buf[1] + buf[2] + buf[3];  //计算校验和
          if(checksum == buf[4] && buf[5] == 0xFF) {     //校验和匹配，数据帧最后一位为0xFF，说明接收的数据帧正常
            dustDensity = (buf[0]*256 + buf[1])*(5.0/1024)*550;   //计算PM2.5浓度，单位ug/m3
          }
          else {     //接收的数据不正常
            dustDensity = 0;    //浓度清零
          }
          break;
        }
      }
      pmTimer = millis() + 300;
    }   
  }
  pmSerial.end();     //关闭软串口
  return dustDensity; //返回值
}


void updateMode(int mode) {
  sensor_mode = mode;
}

void updateTempHumi() {
  sensor_tem = SHT2x.GetTemperature();
  sensor_hum = SHT2x.GetHumidity();
}
