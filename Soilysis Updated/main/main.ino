/*
   Soilysis e-nose
   Created by Ryan J. Ward <ryan.ward@liverpool.ac.uk>
*/
#include "SCD30.h"
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "Wire.h"
#include "String.h"
/* eCO2 Sensor */
#include "EEPROM.h"
#include "sensirion_common.h"
#include "sgp30.h"
#include "Multichannel_Gas_GMXXX.h"
//#include "DFRobot_MICS.h"

#define LOOP_TIME_INTERVAL_MS  1000
#define BASELINE_IS_STORED_FLAG  (0X55)
#define NOSensor A8
#define NDSensor A9
#define MQ4Pin A4

/********************/
String receivedMsg = "";
HardwareSerial& receiver(Serial1);
char c = ' ';

/* All gas sensors */
int counter = 0;

/* Nova PM 2.5 & 10 sensor */
SoftwareSerial novaPMserial(10, 11);
static unsigned char novaBuf[25];
float novaNow;

/* NPK Sensor */
#define RE 15
#define DE 14
const byte nitro_inquiry_frame[] = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x01, 0xE4, 0x0C};
const byte phos_inquiry_frame[] = {0x01, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xB5, 0xCC};
const byte pota_inquiry_frame[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xC0};
byte values[11];
int nitro[10] = { -1};
int phos[10] = { -1};
int pota[10] = { -1};
SoftwareSerial modbus(50, 52);
/* Moisture sensor */
#define moistureSensorPin A0
float moistureValues[20] = {0};
bool moistureRecording;
int moistureCounter;
float features[13] = {0};
/* PH sensor */
#define SensorPin A2            //pH meter Analog output to Arduino Analog Input 0
#define Offset 34.47058824 //41.02740741      //deviation compensate
#define K -15.23529412
#define LED 13
int pHArray[40];
int pHArrayIndex = 0;
bool requestingPH;

int dustSensorPin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
/* FECS41-250 */

/* Ultrasonic atomiser */
#define AtomiserPin A3
int state;
bool high;
unsigned long time_now = 0;
unsigned long time_begin = 0;
float timeOn = 1000;
float timeOff = 1000;
float wait_start = 30000;
float wait_end = 200000;
bool atomising = false;

/* Multichannel gas sensor */
GAS_GMXXX<TwoWire> gas;

void setup()
{
  Serial.begin(9600);
  /* NPK Sensor */
  modbus.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  /* Nova PM 2.5 & 10 sensor */
  novaPMserial.begin(9600);
  novaPMserial.setTimeout(200);
  novaPMserial.readBytesUntil(0xAB, novaBuf, 20);
  novaNow = millis();
  /* Moisture Sensor */
  moistureRecording = false;
  moistureCounter = 0;
  /* PH Sensor */
  requestingPH = false;
  /* Bluetooth module setup */
  /* Multichannal gas sensor  */
  while (!Serial);
  Serial.println("Initilised!");
  receiver.begin(9600);
  receiver.println("Connected!");
  /* Bluetooth module setup end */
  /* CO2, Temperature and Humidity Sensor*/
  Wire.begin();
  scd30.initialize();
  /* Dust sensors (PM 1) */
  pinMode(dustSensorPin, INPUT);
  /* Multichannel gas sensor */
  gas.begin(Wire, 0x08);
  /* PH Sensor */
  pinMode(LED, OUTPUT);
  /* eCO2 Sensor */
  s16 err;
  u16 scaled_ethanol_signal, scaled_h2_signal;
  /*For wio link!*/
#if defined(ESP8266)
  pinMode(15, OUTPUT);
  digitalWrite(15, 1);
  //  Serial.println("Set wio link power!");
  delay(500);
#endif

  /*  Init module,Reset all baseline,The initialization takes up to around 15 seconds, during which
      all APIs measuring IAQ(Indoor air quality ) output will not change.Default value is 400(ppm) for co2,0(ppb) for tvoc*/
  while (sgp_probe() != STATUS_OK) {
    Serial.println("SGP failed");
    while (1);
  }
  /*Read H2 and Ethanol signal in the way of blocking*/
  err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                          &scaled_h2_signal);
  if (err == STATUS_OK) {
    //Serial.println("get ram signal!");
  } else {
    Serial.println("error reading signals");
  }
  set_baseline();
  /* Ultrasonic Atomiser */
  pinMode(AtomiserPin, OUTPUT);
  digitalWrite(AtomiserPin, LOW);
  state = 0;
  high = true;
  time_now = millis();
  time_begin = millis();
  delay(2000);
  Serial.println("Started!");
}

void loop()
{
  /* Carbon Dioxide, Temperature, Humidity */
  CO2_temperature_humidity_sensor();
  /* PM 1 */
  dust_sensor();
  /* PM 2.5 & PM 10*/
  nova_pm();
  /* Deal with the gas sensors, perfoms a moving average before adding it to the array */
  gas_sensors();
  /* Deal with the eCO2 sensor */
  eCO2_sensor();
  /* Deal with the nitrogen, phosphorus and potassium sensor */
  //  NPK_sensor();
  /* Deal with the mositure sensor*/
  if (moistureRecording) moisture_sensor();
  /* Deal with the PH sensor */
  if (requestingPH) PH_sensor();

 /* if (Serial.available())
  {
    c = Serial.read();
    receiver.write(c);
    Serial.write(c);
  }*/
  /* Bluetooth receiver*/
  if (receiver.available())
  {
    receivedMsg = receiver.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(receivedMsg);
    // If requesting NPK
    if (receivedMsg.equals("0"))
    {
      /* Deal with the nitrogen, phosphorus and potassium sensor */
      NPK_sensor();
    } // If requesting moisture
    else if (receivedMsg.equals("1"))
    {
      moistureRecording = true;
    } // If requesting PH
    else if (receivedMsg.equals("2"))
    {
      Serial.println("Requing PH");
      requestingPH = true;
    } // If requsting Soilysis
    else if (receivedMsg.equals("3"))
    {
      char toSend[3];
      toSend[0] = 'X'; toSend[1] = ','; toSend[3] = 'X';
      receiver.write(toSend);
      atomising = true;

      digitalWrite(AtomiserPin, LOW);
      state = 0;
      high = true;
      time_now = millis();
      time_begin = millis();
    }
    /* Bluetooth Receiver end */
  }

  if (atomising)
  {
    atomise();
    transmit_features();
  }

  //transmit_features_wire();
}

void transmit_features()
{
  /* Sends data every 500 ms to stop buffer overflows */

  String buf = "S,";

  for (int i = 0; i < 13; ++ i) buf += String(features[i], 2) + ",";
  buf += "E, ";

   Serial.println("Sending: " + buf);
  char toSend[buf.length()];
  buf.toCharArray(toSend, buf.length());
  receiver.write(toSend);
}

void transmit_features_wire()
{
  /* Sends data every 500 ms to stop buffer overflows */

  String buf = "S,";

  for (int i = 0; i < 13; ++ i) buf += String(features[i], 2) + ",";
  buf += "E, ";

  Serial.println(buf);
}

void atomise()
{
  if (millis() - time_begin > wait_start)
  {
    state = 1;
  }

  if (millis() - time_begin > wait_end)
  {
    state = 0;
    atomising = false;
    pinMode(AtomiserPin, OUTPUT);
    digitalWrite(AtomiserPin, LOW);
    state = 0;
    high = true;
    time_now = millis();
    time_begin = millis();
    String fin = "F,FFFF";
    char toSend[fin.length()];
    fin.toCharArray(toSend, fin.length());
    receiver.write(toSend);
  }

  /* Control the state of the atomisation */
  if (state == 1)
  {
    if (millis() - time_now > timeOff && high) // wait in stop mode
    {
      digitalWrite(AtomiserPin, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      time_now = millis();
      high = !high;
    }

    if (millis() - time_now > timeOn && !high) // wait in atomising mode
    {
      digitalWrite(AtomiserPin, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      time_now = millis();
      high = !high;
    }
  } else {
    digitalWrite(AtomiserPin, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    high = true;
  }
}

float getResistanceMQ4mean()
{
  float v = 0;
  for (int i = 0; i < 50; ++i) v += analogRead(MQ4Pin);
  v = v / 50.0f;
  return -3.556e-06 * v + 0.1785f;
}

void eCO2_sensor()
{
  s16 err = 0;
  u16 tvoc_ppb, co2_eq_ppm;
  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
  if (err == STATUS_OK) {
    features[7] = tvoc_ppb / 1000.0f;
    features[8] = co2_eq_ppm;
  } else {
    Serial.println("error reading IAQ values\n");
  }
  store_baseline();
}

void PH_sensor()
{
  static float pHValue, voltage;

  pHArray[pHArrayIndex++] = analogRead(SensorPin);
  voltage = avergearray(pHArray, 40) * 5.0 / 1024;
  // pHValue = -19.18518519 * voltage + Offset;
  pHValue = K * voltage + Offset;

  if (pHArrayIndex == 40)
  {
    pHArrayIndex = 0;
    String buf = "C, " + String(pHValue, 2) + ", ";
    char toSend[buf.length()];
    buf.toCharArray(toSend, buf.length());
    receiver.write(toSend);
    Serial.println(buf);
    requestingPH = false;
  }
}

void moisture_sensor()
{
  char str[16];
  /* Half a second duty cycle for more reliable readings */
  moistureValues[moistureCounter] = -0.3846153 * analogRead(A0) + 288.5;
  moistureCounter++;

  /* If we have taken the required number of samples, stop taking readings and send via bluetooth */
  if (moistureCounter == 20) {
    moistureRecording = false;
    moistureCounter = 0;
    /* Calculate mean moisture value */
    float meanReading = 0;
    for (int i = 0; i < 20; ++i) meanReading = meanReading + moistureValues[i];
    meanReading = meanReading / 20.0f;

    if (meanReading > 100) meanReading = 100;
    else if (meanReading < 0) meanReading = 0;

    String buf = "B, " + String(meanReading, 2) + ", ";
    /* Send via bluetooth */
    char toSend[buf.length()];
    buf.toCharArray(toSend, buf.length());
    receiver.write(toSend);
    Serial.println(buf);
  }
}

void NPK_sensor()
{
  // Create a array for processing (minor noise in the signals)
  modbus.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  //char str[16];
  for (int i = 0; i < 10; ++i)
  {
    nitro[i] = nitrogen();
    delay(300);
    phos[i] = phosphorous();
    delay(300);
    pota[i] =  potassium();
    delay(300);
   // itoa(10 - i, str, 10);
   // receiver.write(str);
  }
  // Sort the arrays to calculate the median value
  qsort(nitro, sizeof(nitro) / sizeof(nitro[0]), sizeof(nitro[0]), sort_desc);
  qsort(phos, sizeof(phos) / sizeof(phos[0]), sizeof(phos[0]), sort_desc);
  qsort(pota, sizeof(pota) / sizeof(pota[0]), sizeof(pota[0]), sort_desc);
  // Get the medain values and send via bluetooth
  String buf = "A, ";
  buf += nitro[5];
  buf += ", ";
  buf += phos[5];
  buf += ", ";
  buf += pota[5];
  buf += ",";
  //Serial.println(buf);
  char toSend[buf.length()];
  buf.toCharArray(toSend, buf.length());
  receiver.write(toSend);
  // Serial.println(buf);
  modbus.end();
  novaPMserial.begin(9600);
  novaPMserial.setTimeout(200);
  novaPMserial.readBytesUntil(0xAB, novaBuf, 20);
  novaNow = millis();
}

void gas_sensors()
{
  features[6] = getResistanceMQ4mean();
  features[9] = 0.8732 * gas.getGM102B() - 84.91;
  features[10] = gas.getGM302B();
  features[11] = gas.getGM502B();
  features[12] = -0.008508 * gas.getGM702B() + 3.632;
}

void nova_pm()
{
  if (millis() - novaNow >= 1000)
  {
    novaPMserial.readBytesUntil(0xAB, novaBuf, 20);
    /* Extract PM 2.5 */
    features[4] = ((novaBuf[3] * 256) + novaBuf[2]) / 10;
    /* Extract PM 10 */
    features[5] = ((novaBuf[5] * 256) + novaBuf[4]) / 10;
    /* Reset Duty Cycle */
    novaNow = millis();
  }
}


void CO2_temperature_humidity_sensor()
{
  float result[3] = {0};

  if (scd30.isAvailable()) {
    scd30.getCarbonDioxideConcentration(result);
    // Carbon Dioxide Concentration
    features[0] = -6.285e-07f * result[0] + 0.04269f;
    // Temperature
    features[1] = result[1];
    // Humidity
    features[2] = result[2];
  }
}

/* Needs a sampling time of 30 seconds */
void dust_sensor()
{
  duration = pulseIn(dustSensorPin, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if ((millis() - starttime) > 30000) // 30 Seconds sampling time
  {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0); // Integer percentage 0=>100
    // Dust sensor concenstration
    features[3] = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}

int nitrogen() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(100);
  if (modbus.write(nitro_inquiry_frame, sizeof(nitro_inquiry_frame)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    /* Read the response frame for the NPK sensor */
    for (byte i = 0; i < 7; ++i) values[i] = modbus.read();
  }
  return values[4]; // returns the Nigtrogen value only, which is stored at location 4 in the array
}

int phosphorous() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(100);
  if (modbus.write(phos_inquiry_frame, sizeof(phos_inquiry_frame)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    /* Read the response frame for the NPK sensor */
    for (byte i = 0; i < 7; ++i) values[i] = modbus.read();
  }
  return values[4];
}

int potassium() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(100);
  if (modbus.write(pota_inquiry_frame, sizeof(pota_inquiry_frame)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    /* Read the response frame for the NPK sensor */
    for (byte i = 0; i < 7; ++i) values[i] = modbus.read();
  }
  return values[4];
}


/* Prints out a string of the features to the serial monitor
   Carbon Dioxide Concentration, Temperature (C), Humidity (%)
*/
void print_array()
{
  String output = "";
  for (int i = 0; i < (sizeof(features) / sizeof(features[0])); ++i)
  {
    output.concat(features[i]);
    output.concat(", ");
  }
  Serial.println(output);
}

int sort_desc(const void *cmp1, const void *cmp2)
{
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  return a > b ? -1 : (a < b ? 1 : 0);
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}


void array_to_u32(u32* value, u8* array) {
  (*value) = (*value) | (u32)array[0] << 24;
  (*value) = (*value) | (u32)array[1] << 16;
  (*value) = (*value) | (u32)array[2] << 8;
  (*value) = (*value) | (u32)array[3];
}

void u32_to_array(u32 value, u8* array) {
  if (!array) {
    return;
  }
  array[0] = value >> 24;
  array[1] = value >> 16;
  array[2] = value >> 8;
  array[3] = value;
}

/*
    Reset baseline per hour,store it in EEPROM;
*/
void  store_baseline(void) {
  static u32 i = 0;
  u32 j = 0;
  u32 iaq_baseline = 0;
  u8 value_array[4] = {0};
  i++;
  // Serial.println(i);
  if (i == 3600) {
    i = 0;
    if (sgp_get_iaq_baseline(&iaq_baseline) != STATUS_OK) {
      Serial.println("get baseline failed!");
    } else {
      Serial.println(iaq_baseline, HEX);
      Serial.println("get baseline");
      u32_to_array(iaq_baseline, value_array);
      for (j = 0; j < 4; j++) {
        EEPROM.write(j, value_array[j]);
        Serial.print(value_array[j]);
        Serial.println("...");
      }
      EEPROM.write(j, BASELINE_IS_STORED_FLAG);
    }
  }
}

/*  Read baseline from EEPROM and set it.If there is no value in EEPROM,retrun .
    Another situation: When the baseline record in EEPROM is older than seven days,Discard it and return!!
*/
void set_baseline(void) {
  u32 i = 0;
  u8 baseline[5] = {0};
  u32 baseline_value = 0;
  for (i = 0; i < 5; i++) {
    baseline[i] = EEPROM.read(i);
    //   Serial.print(baseline[i], HEX);
    // Serial.print("..");
  }
  //Serial.println("!!!");
  if (baseline[4] != BASELINE_IS_STORED_FLAG) {
    //Serial.println("There is no baseline value in EEPROM");
    return;
  }
  /*
      if(baseline record in EEPROM is older than seven days)
      {
      return;
      }
  */
  array_to_u32(&baseline_value, baseline);
  sgp_set_iaq_baseline(baseline_value);
  //Serial.println(baseline_value, HEX);
}
