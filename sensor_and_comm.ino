#include <Wire.h>    // I2C library
#include "MHZ19.h"
#include "Adafruit_HTU21DF.h"
#include <WiFi.h>
#include "time.h"
#include <PubSubClient.h>
#include "ccs811.h"  // CCS811 library

#define RX_PIN 16
#define TX_PIN 17
#define PWMPIN 3
#define BAUDRATE 9600
#define ROOM_NUMBER 2
#define CO2_ENABLED 1
#define USE_MHZ 1

// time server for proper communication via mqtt 
const char* ntpServer = "192.168.179.18"; 
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
unsigned long epochTime;

WiFiClient espClient;
PubSubClient pbclient(espClient);

MHZ19 myMHZ19;
HardwareSerial mySerial(2);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
CCS811 ccs811(23);

const char *ssid = "FRITZ!Box Fon WLAN 7141";
const char *password = "my_super_safe_wifi_password_123";

// mqtt server to communicate
const char* mqttServer = "192.168.179.11";
const int mqttPort = 1883;
const char* mqttUser = "mqtt_user";
const char* mqttPassword = "my_super_safe_mqtt_password_123";

unsigned long startTime = 0;

void setup() {
  // ------------------------------------------ setup sensor ------------------------------------------------------

  // Enable serial
  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network
  WiFi.begin(ssid, password);

  Serial.println("Waiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);

  // Enable I2C
  Wire.begin();

  // enable comm to co2
  mySerial.begin(BAUDRATE);
  myMHZ19.begin(mySerial);

  // Check sensors
  delay(500);
  int sensors_work = are_sensors_working();
  int checkedTimes = 0;
  while (!sensors_work && checkedTimes < 10) {
    checkedTimes += 1;
    sensors_work = are_sensors_working();
    delay(1000);
  }
  if (!sensors_work) {
    Serial.println("Sensors dont work");
    while (1);
  }


  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  startTime = getTime();
}

void loop() {
  unsigned long currentTime = getTime();
  while (((currentTime % 86400) % 3600) % 30 != 6 * ROOM_NUMBER) {
    Serial.println("idleing " + String(currentTime));
    delay(1000);
    currentTime = getTime();
  }
  Serial.println("reading and sending" + String(currentTime));

  // ----------------loop for co2--------------------------------------------------------------------------------
  int CO2 = 0;
  int errorCodeCo2 = 0;
  if (CO2_ENABLED) {
    if (USE_MHZ) {
      CO2 = myMHZ19.getCO2();
      errorCodeCo2 = myMHZ19.errorCode;
    } else {
      uint16_t eco2, etvoc, errstat, raw;
      ccs811.read(&eco2, &etvoc, &errstat, &raw);
      CO2 = (int) eco2;
    }
  }
  // ----------------loop for temp/humid--------------------------------------------------------------------------------

  float temp = htu.readTemperature();
  float hum = htu.readHumidity();

  // ----------------loop for sending--------------------------------------------------------------------------------
  sendDataToMaster(CO2, errorCodeCo2, hum, temp);

  if (getTime() - startTime > 86400 * 3) {
    ESP.restart();
  }

  delay(23000);
}

//--------------- helper --------------------------------------------------------------------------------

void sendDataToMaster(int CO2, int errorCodeCo2, float humidity, float temperature) {
  String co2String = "";
  if (CO2_ENABLED) {
    co2String = getCo2String(CO2, errorCodeCo2);
  }
  String temperatureString = getTemperatureString(temperature);
  String humidityString = getHumidityString(humidity);

  // send to mqtt
  // init mqtt

  initPBClient();
  if (CO2_ENABLED) {
    sendIntDataMQTT(pbclient, "co2", CO2);
  }
  sendFloatDataMQTT(pbclient, "humidity", humidity, 1, 4);
  sendFloatDataMQTT(pbclient, "temperature", temperature, 1, 3);

  pbclient.disconnect();
}

void sendIntDataMQTT(PubSubClient &pbclient, String dataType, int dataPoint) {
  char intstr[16];
  itoa(dataPoint, intstr, 10);
  String mqttTopicString = "myvalues/";
  mqttTopicString.concat(ROOM_NUMBER);
  mqttTopicString.concat("/");
  mqttTopicString.concat(dataType);
  Serial.println(mqttTopicString);
  Serial.println(intstr);
  int result = pbclient.publish(&mqttTopicString[0], intstr);
  Serial.print("sent " + dataType + " to MQTT, message: ");
  Serial.println(result);
}

void sendFloatDataMQTT(PubSubClient &pbclient, String dataType, float dataPoint, int decimalPrecision, int dataLength) {

  String mqttTopicString = "myvalues/";
  mqttTopicString.concat(ROOM_NUMBER);
  mqttTopicString.concat("/");
  mqttTopicString.concat(dataType);

  char dataChars[dataLength];
  dtostrf(dataPoint, 3, decimalPrecision, dataChars);

  Serial.println(mqttTopicString);
  Serial.println(dataChars);
  int result = pbclient.publish(&mqttTopicString[0], dataChars);

  Serial.print("sent " + dataType + " to MQTT, message: ");
  Serial.println(result);
}

void initPBClient() {
  pbclient.setServer(mqttServer, mqttPort);

  while (!pbclient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (pbclient.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
    } else {
      Serial.print("MQTT failed with state: ");
      Serial.println(pbclient.state());
      delay(100);
    }
  }
}

String wrapDataString(String mydata) {
  String wrappedData = String(getTime()) + "_" + mydata + "-" + ROOM_NUMBER + "/data";
  return wrappedData;
}

String getHumidityString(float humidity) {
  Serial.println("Humidity: " + String(humidity));
  String result = "humidity=" + String(humidity);
  return wrapDataString(result);
}

String getTemperatureString(float temperature) {
  Serial.println("Temperature: " + String(temperature));
  String result = "temperature=" + String(temperature);
  return wrapDataString(result);
}

String getCo2String(int CO2, int errorCode) {
  String result;
  if (myMHZ19.errorCode  == RESULT_OK) {
    Serial.print("MHZ19c: ");
    Serial.print("co2=");
    Serial.print(CO2);
    Serial.print(" ppm  ");
    Serial.println();

    String co2Data = "co2=" + String(CO2);
    result = wrapDataString(co2Data);
  } else {
    Serial.println("Something went wrong (if MHZ), Error Code: ");
    Serial.println(errorCode);
    String errorCodeMessage = "Something went wrong: " + String(myMHZ19.errorCode);
    result = wrapDataString(errorCodeMessage);
  }
  return result;

}

// --------------------- check wifi response  -------------------------

void check_response_wifi(WiFiClient *client) {

  int maxloops = 5;

  //wait for the server's reply to become available
  while (!client->available() && maxloops < 1000)
  {
    maxloops++;
    delay(1); //delay 1 msec
  }
  if (client->available() > 0)
  {
    //read back one line from the server
    String line = client->readStringUntil('\r');
    Serial.println(line);
  }
  else
  {
    Serial.println("client.available() timed out ");
  }

  Serial.println("Closing connection.");
}


// --------------------- check sensor and get time -------------------------

int are_sensors_working() {
  int co2_sensor_works = 0;
  int htu_sensor_works = 0;

  // check co2
  if (CO2_ENABLED) {
    if (USE_MHZ) {
      int CO2 = myMHZ19.getCO2();

      if (myMHZ19.errorCode  == RESULT_OK) {
        Serial.println("Co2 received successfully");
        co2_sensor_works = 1;
      } else {
        Serial.println("Co2 result was not okay, Response Code: ");
        Serial.println(myMHZ19.errorCode);
        while (1);
      }
    } else {
      ccs811.set_i2cdelay(50);
      bool ok = ccs811.begin();
      if ( !ok ) {
        Serial.println("setup: CCS811 begin FAILED");
      } else {
        co2_sensor_works = 1;
      }

      // Start co2 measuring
      if ( !ccs811.start(CCS811_MODE_1SEC) ) {
        Serial.println("setup: CCS811 start FAILED");
        co2_sensor_works = 0;
      }
    }
  }

  // start temp/humid measuring
  if (!htu.begin()) {
    Serial.println("Check circuit. HTU21D not found!");
  } else {
    Serial.println("HTU21D works");
    htu_sensor_works = 1;
  }
  if (CO2_ENABLED) {
    return co2_sensor_works & htu_sensor_works;
  } else {
    return htu_sensor_works;
  }
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}
