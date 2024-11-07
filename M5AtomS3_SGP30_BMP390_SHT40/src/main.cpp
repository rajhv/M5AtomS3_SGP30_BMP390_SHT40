/*************************************************** 
  Za dodajanje library pojdi na https://registry.platformio.org/
  kjer najdeš library in pod Installation dobiš navodilo, kako jo dodaš
  v platformio ter #include
 ****************************************************/

#include <Arduino.h>
#include <M5AtomS3.h>
#include "Adafruit_SHT4x.h"
#include "Adafruit_BMP3XX.h"
#include "bme68xLibrary.h"
#include "SparkFun_SGP30_Arduino_Library.h" 
#include "ArduinoMqttClient.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_wpa2.h>
#include "time.h"

//#include "index.h"

//#define BUTTON_GPIO GPIO_NUM_2
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp390;
SGP30 sgp30; //create an object of the SGP30 class

Adafruit_SHT4x sht4 = Adafruit_SHT4x();


//const char* ssid     = "REK-PRIVAT";
//const char* password = "********";
//REK-PRIVAT
//uprek2015
//MAC >> 70:04:1D:CD:C3:DC


const char *ssid = "eduroam"; // Eduroam 
#define EAP_IDENTITY "vojko.rajh@upr.si" 
#define EAP_PASSWORD "********" 


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
WiFiServer server(80);

const char broker[] = "1.1.1.1"; // ip od domačega ruterja
int        port     = 1883;

JsonDocument doc;

uint32_t send_interval = millis();

char HTML_CONTENT;


const char* ntpServer =
    "time1.aliyun.com";  // Set the connect NTP server.  设置连接的NTP服务器
const long gmtOffset_sec     = 0;
const int daylightOffset_sec = 7200;

void printLocalTime() {  // Output current time.  输出当前时间
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {  // Return 1 when the time is successfully
                                     // obtained.  成功获取到时间返回1
        USBSerial.println("Failed to obtain time");
        return;
    }
    USBSerial.println(
        &timeinfo,
        "%A, %B %d \n%Y %H:%M:%S");  // Serial port output date and
                                     // time.  串口输出日期和时间
}


void printWifiStatus() {
  USBSerial.print("You're connected to the network: ");  USBSerial.println(WiFi.SSID());
  USBSerial.println();
  // print your board's IP address:
  USBSerial.print("IP Address: ");
  USBSerial.println(WiFi.localIP());

  // print the received signal strength:
  USBSerial.print("signal strength (RSSI):");
  USBSerial.print(WiFi.RSSI());
  USBSerial.println(" dBm");
}


void connectWiFi() {
    // attempt to connect to WiFi network:
  WiFi.disconnect(true); //disconnect form wifi to set new wifi connection
  USBSerial.print("Attempting to connect to WPA SSID: ");
  USBSerial.println(ssid);
  WiFi.mode(WIFI_STA);
  USBSerial.print("MAC >> ");
  USBSerial.println(WiFi.macAddress());
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(ssid);

  delay(10);
  USBSerial.println(WiFi.status());
    USBSerial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        USBSerial.print(".");
        delay(100);
    }
 

  printWifiStatus();
}


void connectMQTT() {

  USBSerial.print("Attempting to connect to the MQTT broker: ");
  USBSerial.println(broker);

  //mqttClient.setUsernamePassword("dacus","dacus");

  mqttClient.connect(broker, port);
  delay(10);
  USBSerial.println(mqttClient.connectError());
  //mqttClient.setCleanSession(true);
  //mqttClient.setKeepAliveInterval(60);


  USBSerial.println("You're connected to the MQTT broker!");
  USBSerial.println();

}


void setup() {
  M5.begin();
  Wire.begin(2,1);

  USBSerial.begin(115200);
  delay(2000);
  USBSerial.println("Setup started..............");
  

  delay(10);

  USBSerial.println(F("Initialized"));

  USBSerial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    USBSerial.println("Couldn't find SHT4x");
  while (1);
  }
  USBSerial.println("Found SHT4x sensor");
  USBSerial.print("Serial number 0x");
  USBSerial.println(sht4.readSerial(), HEX);

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
 
  
  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_NO_HEATER);

  if (!bmp390.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    USBSerial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  } else {
    // Set up oversampling and filter initialization
    bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp390.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  
  //Initialize sensor
  if (!sgp30.begin()) {
    USBSerial.println("No SGP30 Detected. Check connections.");
    while (1);
  } else {
    //Initializes sensor for air quality readings
    //measureAirQuality should be called in one second increments after a call to initAirQuality
    sgp30.initAirQuality();
  }
  

//lipo.begin();


  connectWiFi();
  connectMQTT();

  //server.begin(); 
  
delay(1000); //počakaj sekundo na senzorje

configTime(gmtOffset_sec, daylightOffset_sec,
               ntpServer);  // init and get the time.  初始化并设置NTP
printLocalTime();

}

void loop() {
  
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    connectMQTT();
  }

  //USBSerial.println("++++++++++");
  sensors_event_t humidity, temp;

  uint32_t timestamp = millis();


  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    // read the HTTP request header line by line
    while (client.connected()) {
      if (client.available()) {
        String HTTP_header = client.readStringUntil('\n');  // read the header line of HTTP request

        if (HTTP_header.equals("\r"))  // the end of HTTP request
          break;

        USBSerial.print("<< ");
        USBSerial.println(HTTP_header);  // print HTTP request to Serial Monitor
      }
    }


    // send the HTTP response
    // send the HTTP response header
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");  // the connection will be closed after completion of the response
    client.println();                     // the separator between HTTP header and body

    // send the HTTP response body
    String html = String(HTML_CONTENT);
    html.replace("TEMPERATURE_MARKER", String(temp.temperature, 2)); // replace the marker by a real value

    client.println(html);
    client.flush();

    // give the web browser time to receive the data
    delay(10);

    // close the connection:
    client.stop();
  }
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  doc["Temp"] = temp.temperature;
  doc["Hum"] = humidity.relative_humidity;
  
  //USBSerial.print("TEMPERATURA: "); USBSerial.println(temp.temperature);
  //USBSerial.print("VLAŽNOST: "); USBSerial.println(humidity.relative_humidity);
  timestamp = millis() - timestamp;

  if (!bmp390.performReading()) {
    USBSerial.println("Failed to perform reading :(");
    //return;
  }
  doc["Press"] = bmp390.pressure/100.0;
  
  //measure CO2 and TVOC levels
  sgp30.measureAirQuality();
  doc["CO2"] = sgp30.CO2;
  doc["tVOC"] = sgp30.TVOC;

  //USBSerial.print("Send Interval:"); USBSerial.println(send_interval);
  //USBSerial.print("millis:"); USBSerial.println(millis());

  if ((millis() - send_interval) > 3000) {
    //USBSerial.println("MQTT was sent!");
    mqttClient.poll(); //vzdržuje povezavo z mqtt brokerjem
    mqttClient.beginMessage("M5AtomS3_1", (unsigned long)measureJson(doc));
    serializeJson(doc, mqttClient);
    mqttClient.endMessage();
    send_interval = millis();
  };
  

  delay(100);
  //USBSerial.println("--------------------");
 
 printLocalTime();

}


