#include <Arduino.h>
#include <Controllino.h>

#include<avr/wdt.h>

#include <SPI.h>
#include <Ethernet.h>

#include <ArduinoHA.h>

//Ethernet
byte mac[] = { 0x62, 0x75, 0x6E, 0x6E, 0x79, 0x69 };
byte ip[] = { 192, 168, 0, 50 };

//HASS
#define BROKER_ADDR     IPAddress(192,168,0,69)

EthernetClient client;
HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device);
HASwitch led("led", false); // "led" is unique ID of the switch. You should define your own ID.

void onBeforeSwitchStateChanged(bool state, HASwitch* s)
{

}

void onSwitchStateChanged(bool state, HASwitch* s)
{
    digitalWrite(CONTROLLINO_D1, (state ? HIGH : LOW));
}

//IO

bool statusD0 = false;

void blink() {
  statusD0 = !statusD0;
  digitalWrite(CONTROLLINO_D0, statusD0);
}

void printTime() {
  Serial.print((int)Controllino_GetDay()); Serial.print("-");
  Serial.print((int)Controllino_GetMonth());Serial.print("-");
  Serial.print((int)Controllino_GetYear());Serial.print("\t");
  Serial.print((int)Controllino_GetHour());Serial.print(":");
  Serial.print((int)Controllino_GetMinute());Serial.print(":");
  Serial.println((int)Controllino_GetSecond());
}

int prevSeconds = 0;
bool oneSecondPassed() {
  if (prevSeconds != (int) Controllino_GetSecond()) {
    prevSeconds = (int) Controllino_GetSecond();
    return 1;
  }
  return 0;
}

void setup() {
  Serial.begin(9600);

  //watchdog
  wdt_enable(WDTO_4S);

  //IO
  pinMode(CONTROLLINO_D0, OUTPUT);

  //RTC
  Controllino_RTC_init();
  // if (spiffs) or if DateTimeRTC > compile markers
  Controllino_SetTimeDateStrings(__DATE__, __TIME__);

  //Ethernet
  Ethernet.begin(mac, ip);

  //HASS
  // set device's details (optional)
  device.setName("Domoriks Brain");
  device.setSoftwareVersion("1.0.0");

  // set icon (optional)
  led.setIcon("mdi:lightbulb");
  led.setName("My LED");

  // handle switch state
  led.onBeforeStateChanged(onBeforeSwitchStateChanged); // optional
  led.onStateChanged(onSwitchStateChanged);
  mqtt.begin(BROKER_ADDR);
}

void loop() {
  //Docker
  Ethernet.maintain();
  mqtt.loop();

  //Triggers every second
  if (oneSecondPassed()) {
    blink();
    printTime();
  }

  //feed the dog
  while (true); //make the dog angry
  wdt_reset();
}
