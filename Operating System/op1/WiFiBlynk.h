#include <ESP8266_HardSer.h>
#include <BlynkSimpleShieldEsp8266_HardSer.h>
#include <SimpleTimer.h>

// Set ESP8266 Serial object
#define EspSerial Serial1

ESP8266 wifi(EspSerial);

SimpleTimer temHumtimer;

void senTempHumi() {
  Blynk.virtualWrite(V0, sensor_tem);
  Blynk.virtualWrite(V1, sensor_hum); 
}

BLYNK_READ(V3) {
  Blynk.virtualWrite(V3, sensor_mode);
}

BLYNK_READ(V2) {
  Blynk.virtualWrite(V2, sensorPM25);
  //BLYNK_PRINT.println(sensorPM25);
}

BLYNK_WRITE(V5) {
  sensor_mode++;
  sensor_mode = sensor_mode%3;
  switch (sensor_mode) {
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
  //BLYNK_PRINT.println(sensorPM25);
}

BLYNK_WRITE(V4) {
  fanSpeed = param.asInt();
  modifyFan = true;
  //BLYNK_PRINT.println(sensorPM25);
}
