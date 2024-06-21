/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL2ATdqFknC"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "8COhjnKC4gM9OFqMbZN6-ZnAF0HAXn6H"

const int maxLitersPerSercond = 0.05;
double transmition[] = {0.2, 0.4, 0.6, 0.8, 1, 1.2};
int gear = 0;
int accelaration = 0;
const int maxGear = 5;
/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"

#define DHTPIN 4
#define ACCELERATIONPIN 35

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "LCI01";
char pass[] = "up3@wz01";

BlynkTimer timer;

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();
  // Update state
  Blynk.virtualWrite(V1, value);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  double temperature = map(dht.readTemperature(), 0, 40, 70, 120);

  Serial.println();
  accelaration = map(analogRead(ACCELERATIONPIN), 0, 4095, 0, 100);
  double speed = transmition[gear] * accelaration;
  double consumo = (speed / 1000) / (accelaration * maxLitersPerSercond / 100); 
  Blynk.virtualWrite(V2, consumo);
  Blynk.virtualWrite(V1, speed * 3.6);
  Blynk.virtualWrite(V3, temperature);
}

void setup()
{
  // Debug console
  Serial.begin(115200);
  dht.begin();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}

void loop()
{
  Blynk.run();
  timer.run();
}
