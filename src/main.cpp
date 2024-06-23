/*  Trabalho Final Internet Of Things - Junho 2024
    Professor Marcelo Trindade Rebonatto
    
    Monitoramento de Frota de Veículos
    Autores:
      Enzo Zavorski Delevatti
      Gabriel Tibola Castagnera
      Juan Pinto Loureiro
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"
#include "wifi_password.hpp"

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL2ATdqFknC"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "8COhjnKC4gM9OFqMbZN6-ZnAF0HAXn6H"
/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#define DHTPIN 4
#define DIPLAY_A 26
#define DIPLAY_B 32
#define DIPLAY_C 13
#define DIPLAY_D 12
#define DIPLAY_E 14
#define DIPLAY_F 25
#define DIPLAY_G 33
#define ACCELERATIONPIN 35
#define GEAR_UP 22
#define GEAR_DOWN 23

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

BlynkTimer timer;

const double maxLitersPerSercond = 0.0125;
const int maxGear = 5;
const u8_t gearDisplay[] = {0x79, 0x24, 0x30, 0x19, 0x12, 0x02};

int gear, acceleration;
bool lastStateGearUp, lastStateGearDown;
double transmition[] = {0.0583, 0.1167, 0.175, 0.2333, 0.2917, 0.35};

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  double temperature = map(dht.readTemperature(), 0, 40, 70, 120);
  double acceleration = map(analogRead(ACCELERATIONPIN), 0, 4095, 0, 100);
  double speed = transmition[gear] * acceleration;
  double consumption = speed / (maxLitersPerSercond * (acceleration / 100.0));

  Serial.print("Speed: ");
  Serial.println(speed);
  Serial.print("Litters per second: ");
  Serial.println((maxLitersPerSercond * (acceleration / 100.0)));
  Serial.print("Consumption: ");
  Serial.println(consumption);

  Blynk.virtualWrite(V2, consumption / 1000); // KM/L
  Blynk.virtualWrite(V1, speed * 3.6);        // KM/H
  Blynk.virtualWrite(V3, temperature);        // C°
}

// Reads for gear inputs
void update_gears()
{
  int gearUp = digitalRead(GEAR_UP);
  int gearDown = digitalRead(GEAR_DOWN);

  if (!lastStateGearUp && gearUp)
  {
    if (gear < maxGear)
      gear++;
    update_gear_diplay();
  }

  if (!lastStateGearDown && gearDown)
  {
    if (gear > 0)
      gear--;
    update_gear_diplay();
  }

  lastStateGearUp = gearUp;
  lastStateGearDown = gearDown;
}

void update_gear_diplay()
{
  u8_t selectedGear = gearDisplay[gear];
  digitalWrite(DIPLAY_A, selectedGear & 0x01);
  digitalWrite(DIPLAY_B, selectedGear & 0x02);
  digitalWrite(DIPLAY_C, selectedGear & 0x04);
  digitalWrite(DIPLAY_D, selectedGear & 0x08);
  digitalWrite(DIPLAY_E, selectedGear & 0x10);
  digitalWrite(DIPLAY_F, selectedGear & 0x20);
  digitalWrite(DIPLAY_G, selectedGear & 0x40);
}

void setup()
{
  // Var initial values
  gear = 0;
  acceleration = 0;
  lastStateGearUp = false;
  lastStateGearDown = false;

  // Debug console
  Serial.begin(115200);
  pinMode(DIPLAY_A, OUTPUT);
  pinMode(DIPLAY_B, OUTPUT);
  pinMode(DIPLAY_C, OUTPUT);
  pinMode(DIPLAY_D, OUTPUT);
  pinMode(DIPLAY_E, OUTPUT);
  pinMode(DIPLAY_F, OUTPUT);
  pinMode(DIPLAY_G, OUTPUT);
  dht.begin();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
  timer.setInterval(100L, update_gears);
  update_gear_diplay();
}

void loop()
{
  Blynk.run();
  timer.run();
}