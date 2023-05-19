#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>

#include <DHT.h>

#include <InfluxDb.h>


#define INFLUXDB_HOST "192.168.1.64"   //Enter IP of device running Influx Database
#define WIFI_SSID "xxxx"              //Enter SSID of your WIFI Access Point
#define WIFI_PASS "xxxx"          //Enter Password of your WIFI Access Point
// - (GND) to GND
// + (VDD) to 3.3V
// SCL to D1
// SDA to D2 


ESP8266WiFiClass WiFiMulti; // not ure if we should stick with this lib 
Influxdb influx(INFLUXDB_HOST);
Adafruit_BMP085 bme; //I2C
// Put DHT init here
#define DHTPIN 5     // Digital pin connected to the DHT sensor
// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

// current temperature & humidity, updated in loop()
float t = 0.0;
float h = 0.0;

//BMP 180
float temperature;
float pressure;

// Initilize DHT11,
void initDHT(){
    dht.begin();
}

// Initialize BME280 , directly lifted 
void initBME(){
  if (!bme.begin(0x76)) {
    Serial.println("No valid bmp085 sensor found");
    while (1);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println(".");
  }

  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  influx.setDb("yellowIOT");
  initBME();// Init the HW
  initDHT();// init the DHT hw
  Serial.println("Setup Complete.");
}

int recordnr = 0; // why ? 

void loop() {
  // float temperature = bme.readTemperature();
  // float humidity = bme.readHumidity();
  // float newP = dht.readPressure()/100.0F;
  float newT = dht.readTemperature();
  float newH = dht.readHumidity();
  
  int loopCount;  
  loopCount++;

  InfluxData row("iots");

  Serial.println(newT);
  Serial.println(newH);
  // Serial.println(newP);
  
  row.addTag("Sensor", "DHT11");
  row.addValue("temperaturedht", newT);
  row.addValue("humidity", newH);
  
  row.addTag("Sensor", "BMP085");
  // row.addTag("RecordNr", String(recordno));
  // row.addValue("temperature", temperature);
  // row.addValue("pressure",pressure);
  row.addValue("rssi", WiFi.RSSI());
  
  influx.write(row);
  
  // Serial.print(String(recordno));
  Serial.println(" written to local InfluxDB instance");
  
  Serial.println();
  delay(60000); // we should either modem sleep or light sleep 
}
