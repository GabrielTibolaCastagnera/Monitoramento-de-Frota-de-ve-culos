/*  Trabalho Final Internet Of Things - Junho 2024
    Professor Marcelo Trindade Rebonatto
    
    Monitoramento de Frota de Veículos
    Autores:
      Enzo Zavorski Delevatti
      Gabriel Tibola Castagnera
      Juan Pinto Loureiro
*/
#define BLYNK_TEMPLATE_ID "TMPL2ATdqFknC"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "8COhjnKC4gM9OFqMbZN6-ZnAF0HAXn6H"

/**
 * @brief Atualiza o diplay com a marcha selecionada
 * 
 */
void update_gear_diplay();

/**
 * @brief Função que retorna a aceleração atual do veículo, levando em conta a marcha, 
 * velocidade atual, quando está pisando no acelerador
 * 
 * @param currentSpeed 
 * @param percentAccelaration 
 * @return double 
 */
double getAccelaration(double currentSpeed, double percentAccelaration);

/**
 * @brief máximo de fluxo de combustível da bomba em Litros / segundo
 * 
 */
const double maxLitersPerSercond = 0.0125;

/**
 * @brief aceleração máxima em Newtons (Kg*m/s^2) para cada marcha
 * 
 */
const double transmition[] = {49320, 24660, 16440, 12330, 19720, 8220};
/**
 * @brief máxima velocidade para cada marcha em m/s
 * 
 */
const double maxSpeed[] = {5.55, 11.11, 16.67, 22.22, 27.78, 33.33};

/**
 * @brief última velocidade registrada
 * 
 */
double lastSpeed = 0;

/**
 * @brief marcha atual
 * 
 */
int gear = 0;

/**
 * @brief indica se a ignição está ativa ou não
 * 
 */
int ignition = 0;

/**
 * @brief Peso do veículo em Newtons
 * 
 */
const double weight = 328800; // N

/**
 * @brief índice da marcha mais pesada
 * 
 */
const int maxGear = 5;

/**
 * @brief informa o último estado do botão de subir marcha
 * 
 */
bool lastStateGearUp = false;   // last buttons states

/**
 * @brief informa o último estado do botão de descer marcha
 * 
 */
bool lastStateGearDown = false;

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"
#include "wifi_password.hpp"
/**
 * @brief Pino do sensor de Temperatura
 * 
 */
#define DHTPIN 4
//Mapeamento das saidas dos LEDs do diplay de 7 segmentos
#define DIPLAY_A 26
#define DIPLAY_B 32
#define DIPLAY_C 13
#define DIPLAY_D 12
#define DIPLAY_E 14
#define DIPLAY_F 25
#define DIPLAY_G 33

/**
 * @brief Pino do Acelerador
 * 
 */
#define ACCELERATIONPIN 35

/**
 * @brief Pino do botão para subir marcha
 * 
 */
#define GEAR_UP 22

/**
 * @brief Pino do botão para descer marcha
 * 
 */
#define GEAR_DOWN 23

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

/**
 * @brief Handler do evento para cortar a ignição
 * 
 */
BlynkTimer::Handle cutIgnition;

/**
 * @brief função para cortar a ignição
 * 
 */
void ignitionHandler()
{
  ignition = 0;
}

/**
 * @brief Handler para saber se foi enviado um evento para cortar a ignição
 * 
 */
BLYNK_WRITE(V0)
{
  int value = param.asInt();
  //trata se foi dado o comando de cortar ignição
  if (value)
  {
    if (cutIgnition.isEnabled())
    {
      cutIgnition.disable();
    }
    // corta a ignição após um intervalo de 5 segundos
    cutIgnition = timer.setTimeout(5e3L, ignitionHandler);
  }
  else
  {
    if (cutIgnition.isEnabled())
    {
      cutIgnition.disable();
    }
    //retoma a ignição
    ignition = 1;
  }
  //aciona a lâmpada caso a ignição será cortada
  digitalWrite(2, value);
}
int sended_notification = 0;
/**
 * @brief Atualiza a velocidade, temperatura e consumo a cada segundo
 * 
 */
void myTimerEvent()
{
  //Ganho na leitura para simular uma temperatura mais real de um motor
  double temperature = map(dht.readTemperature(), 0, 40, 70, 120);
  //Quanto está sendo pisado no acelerador de 0 a 100%
  double accelerationPercent = map(analogRead(ACCELERATIONPIN), 0, 4095, 0, 100);
  //Consulta a aceleração
  double accelaration = getAccelaration(lastSpeed, accelerationPercent / 100.0);
  Serial.print("final accelaration: ");
  Serial.println(accelaration);

  lastSpeed = lastSpeed + accelaration;
  //limita a velocidade entre 0 e o máximo da marcha
  if (lastSpeed > maxSpeed[gear])
  {
    lastSpeed = maxSpeed[gear];
  }
  else if (lastSpeed < 0)
  {
    lastSpeed = 0;
  }
  if(lastSpeed > 27.76 && !sended_notification){
    Blynk.logEvent("velocidade_alta", "Velocidade acima de 100Km/h");
    sended_notification = 1;
  }
  else{
    sended_notification = 0;
  }
  //cálculo de consumo. Se a ignição estiver cortada ou o veículo estiver em movimento e sem acelerar, o consumo será infito, mas limitado em 30 km/L 
  //Caso contrário, dividir a última velocidade (m/s) pelo fluxo atual de combustível (L/s) tem como resultado o consumo em m/L
  double consumo = ignition != 0 && (accelerationPercent > 0 || lastSpeed == 0) ? lastSpeed / (maxLitersPerSercond * (accelerationPercent / 100.0))
                                                                                : 30000;
  // Atualiza os pinos Virtuais
  Blynk.virtualWrite(V2, consumo / 1000);
  Blynk.virtualWrite(V1, lastSpeed * 3.6);
  Blynk.virtualWrite(V3, temperature);
}

/**
 * @brief Atualiza a marcha a cada 100 milisegundos
 * 
 */
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
  // Setup a function to be called every 100 milliseconds
  timer.setInterval(100L, update_gears);

  update_gear_diplay();

  Blynk.connect();
  //sincroniza a ignição com o comando dado
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
  
  //cálculo da força de fricção
  double friction = 0.0041 + (0.0000041 * (currentSpeed / 1.60934));

  //cálculo do arrasto aerodinâmico
  double da = 0.5 * p * currentSpeed * currentSpeed * truckArea * c; // da -> N arrasto

  //cálculo da fricção total com o peso do veículo
  double rx = friction * weight;                                     // rx -> N resistencia a rolagem

  //resistência total
  double totaOfResistant = da + rx;
  //máxima velocidade da marcha
  double maxGearSpeed = maxSpeed[gear];
  //máxima aceleração
  double maxAcceleration = (transmition[gear] * percentAccelaration);

  double percentSpeed = currentSpeed / maxGearSpeed;
  double accelarationFactor = 0;

  /**
   * @brief calcula o fator de aceleração na marcha,
   * levando em consiração a velocidade atual e a marcha,
   * para calcular em que faixa de eficiência do motor está
   * e que aceleração pode entregar entre 10 a 100% da aceleração máxima
   * de acordo com o que está sendo requerido do pedal.
   * Caso for nada, será uma aceleração negativa, simulando um freio motor
   */
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
  //aceleração final (N)
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
  // desconta as forças de resistência e divide pela massa do veículo, obtendo a aceleração em m/s^2 
  return (accelaration - totaOfResistant) / (weight / 9.8);
}