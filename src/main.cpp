/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL2ATdqFknC"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "8COhjnKC4gM9OFqMbZN6-ZnAF0HAXn6H"
void update_gear_diplay();
double getAccelaration(double currentSpeed, double percentAccelaration);
const double maxLitersPerSercond = 0.0125;
const double transmition[] = {49320, 24660, 16440, 12330, 19720, 8220};
const double maxSpeed[] = {5.55, 11.11, 16.67, 22.22, 27.78, 33.33};
double lastSpeed = 0;
int gear = 0;
int ignition = 0;
const double weight = 328800; // N
const int maxGear = 5;
bool lastStateGearUp = false;   // last buttons states
bool lastStateGearDown = false; // ^
/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"
#include "wifi_password.hpp"
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
BlynkTimer::Handle cutIgnition;

void ignitionHandler()
{
  ignition = 0;
}
// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  int value = param.asInt();
  Serial.println(value);
  if (value)
  {
    if (cutIgnition.isEnabled())
    {
      cutIgnition.disable();
    }
    cutIgnition = timer.setTimeout(5e3L, ignitionHandler);
  }
  else
  {
    if (cutIgnition.isEnabled())
    {
      cutIgnition.disable();
    }
    ignition = 1;
  }
  digitalWrite(2, value);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
}
int state = LOW;
// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  double temperature = map(dht.readTemperature(), 0, 40, 70, 120);

  double accelerationPercent = map(analogRead(ACCELERATIONPIN), 0, 4095, 0, 100);
  double accelaration = getAccelaration(lastSpeed, accelerationPercent / 100.0);
  Serial.print("final accelaration: ");
  Serial.println(accelaration);

  lastSpeed = lastSpeed + accelaration;
  if (lastSpeed > maxSpeed[gear])
  {
    lastSpeed = maxSpeed[gear];
  }
  else if (lastSpeed < 0)
  {
    lastSpeed = 0;
  }
  double consumo = ignition != 0 && (accelerationPercent > 0 || lastSpeed == 0) ? lastSpeed / (maxLitersPerSercond * (accelerationPercent / 100.0))
                                                                                : 30000;
  Blynk.virtualWrite(V2, consumo / 1000);
  Blynk.virtualWrite(V1, lastSpeed * 3.6);
  Blynk.virtualWrite(V3, temperature);
}

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
void setup()
{
  // Debug console
  cutIgnition = timer.setTimeout(5e3L, []()
                                 { ignition = 0; });
  cutIgnition.disable();
  Serial.begin(9600);
  pinMode(2, OUTPUT);
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
  Blynk.connect();
  Blynk.syncVirtual(V0);
}
void loop()
{
  Blynk.run();
  timer.run();
}
const u8_t gearDisplay[] = {0x79, 0x24, 0x30, 0x19, 0x12, 0x02};

void update_gear_diplay()
{
  if (gear > 5)
  {
    gear = 5;
  }
  else if (gear < 0)
  {
    gear = 0;
  }

  u8_t selectedGear = gearDisplay[gear];
  digitalWrite(DIPLAY_A, selectedGear & 0x01);
  digitalWrite(DIPLAY_B, selectedGear & 0x02);
  digitalWrite(DIPLAY_C, selectedGear & 0x04);
  digitalWrite(DIPLAY_D, selectedGear & 0x08);
  digitalWrite(DIPLAY_E, selectedGear & 0x10);
  digitalWrite(DIPLAY_F, selectedGear & 0x20);
  digitalWrite(DIPLAY_G, selectedGear & 0x40);
}
double getAccelaration(double currentSpeed, double percentAccelaration)
{
  const double p = 1.077;               // kg/m3
  const double truckArea = 2.44 * 4.11; // m2
  const double c = 0.65;
  double friction = 0.0041 + (0.0000041 * (currentSpeed / 1.60934));
  double da = 0.5 * p * currentSpeed * currentSpeed * truckArea * c; // da -> N arrasto
  double rx = friction * weight;                                     // rx -> N resistencia a rolagem
  double totaOfResistant = da + rx;
  double maxGearSpeed = maxSpeed[gear];
  double maxAcceleration = (transmition[gear] * percentAccelaration);
  double percentSpeed = currentSpeed / maxGearSpeed;
  double accelarationFactor = 0;
  if (percentAccelaration <= 0 || ignition == 0)
  {
    accelarationFactor = -0.5;
  }
  else if (percentSpeed < 0.325)
  {
    accelarationFactor = 0.1 + (percentSpeed / 0.325) * 0.9;
  }
  else if (percentSpeed <= 0.625)
  {
    accelarationFactor = 1.0;
  }
  else if (percentSpeed <= 1.0)
  {
    accelarationFactor = 1.0 - ((percentSpeed - 0.625) / 0.375) * 0.5;
  }
  else
  {
    accelarationFactor = 0;
  }
  double accelaration = percentAccelaration > 0 ? maxAcceleration * accelarationFactor
                                                : transmition[gear] * accelarationFactor;
  Serial.print("speed: ");
  Serial.println(currentSpeed);
  Serial.print("air fricction: ");
  Serial.println(da);
  Serial.print("resistant: ");
  Serial.println(totaOfResistant);
  Serial.print("accelarationFactor: ");
  Serial.println(accelarationFactor);
  Serial.print("MaxAccelaration: ");
  Serial.println(maxAcceleration);
  Serial.print("accelaration: ");
  Serial.println(accelaration);
  return (accelaration - totaOfResistant) / (weight / 9.8);
}