#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_NeoPixel.h>
#include <DHT20.h>
#include <LiquidCrystal_I2C.h>
#include "HCSR04.h";
#include "SoftServo.h";
#include <Arduino.h>;

#define WLAN_SSID       "ACLAB"
#define WLAN_PASS       "ACLAB2023"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "hoang451"
#define AIO_KEY         ""  

// Define your tasks here
void TaskBlink(void *pvParameters);
void TaskTemperatureHumidity(void *pvParameters);
void TaskSoilMoistureAndRelay(void *pvParameters);
void TaskLightAndLED(void *pvParameters);
void TaskAutoFan(void *pvParameters);
void TaskDistanceAndServo(void *pvParameters);
void TaskMQTT(void *pvParameters);

//Define your components here
Adafruit_NeoPixel rgb(4, D5, NEO_GRB + NEO_KHZ800);
DHT20 dht20;
LiquidCrystal_I2C lcd(33,16,2);
UltraSonicDistanceSensor ultrasonic(D9, D10);
SoftServo myservo;
WiFiClient client;

float objectCelsius = 20.0;
float objectHumidity;
float objectSoilMoisture;
float objectLight;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe ledControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humi");
Adafruit_MQTT_Publish soilmoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilmoisture");
Adafruit_MQTT_Publish light = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/light");

void setup() {

  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200); 
  dht20.begin();
  lcd.begin();
  rgb.begin();

  myservo.attach(D2); // servo nối với cổng D2
  myservo.asyncMode();
  myservo.delayMode();

  myservo.tick();
  myservo.write(90);
  
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  mqtt.subscribe(&ledControl);

  xTaskCreate( TaskBlink, "Task Blink" ,2048  ,NULL  ,2 , NULL);
  xTaskCreate( TaskTemperatureHumidity, "Task Temperature" ,2048  ,NULL  ,2 , NULL);
  xTaskCreate( TaskSoilMoistureAndRelay, "Task Soil Moisture and Relay" ,2048  ,NULL  ,2 , NULL);
  xTaskCreate( TaskLightAndLED, "Task Light LED" ,2048  ,NULL  ,2 , NULL);
  xTaskCreate( TaskAutoFan, "Task Auto Open Fan" ,2048  ,NULL  ,2 , NULL);
  xTaskCreate( TaskDistanceAndServo, "Task Distance And Servo" ,2048  ,NULL  ,2 , NULL);
  xTaskCreate( TaskMQTT, "Task MQTT", 2048, NULL, 2, NULL);
  
  //Now the task scheduler is automatically started.
  Serial.printf("Basic Multi Threading Arduino Example\n");
}

void loop() {
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/



void TaskBlink(void *pvParameters) {  // This is a task.
  //uint32_t blink_delay = *((uint32_t *)pvParameters);

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  

  while(1) {                          
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED ON
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED OFF
    delay(1000);
  }
}


void TaskTemperatureHumidity(void *pvParameters) {  // This is a task.
  //uint32_t blink_delay = *((uint32_t *)pvParameters);

  while(1) {                          
    Serial.println("Task Temperature and Humidity");
    dht20.read();
    Serial.println(dht20.getTemperature());
    Serial.println(dht20.getHumidity());
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(dht20.getTemperature());
    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(dht20.getHumidity());

    delay(1000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Soil Moisture: ");
    lcd.setCursor(0, 1);
    lcd.print(analogRead(A0));

    delay(1000);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Light: ");
    lcd.setCursor(0, 1);
    lcd.print(analogRead(A1));

    delay(1000);
  }
}

void TaskSoilMoistureAndRelay(void *pvParameters) {  // This is a task.
  //uint32_t blink_delay = *((uint32_t *)pvParameters);

  pinMode(D3, OUTPUT);

  while(1) {                          
    Serial.println("Task Soild and Relay");
    Serial.println(analogRead(A0));
    
    if(analogRead(A0) > 500){
      digitalWrite(D3, LOW);
    }
    if(analogRead(A0) < 50){
      digitalWrite(D3, HIGH);
    }
    delay(1000);
  }
}


void TaskLightAndLED(void *pvParameters) {  // This is a task.
  //uint32_t blink_delay = *((uint32_t *)pvParameters);

  while(1) {                          
    Serial.println("Task Light and LED");
    Serial.println(analogRead(A1));
    
    if(analogRead(A1) < 350){
      // rgb.setPixelColor(0, rgb.Color(255,0,0));
      // rgb.setPixelColor(1, rgb.Color(0,255,0));
      // rgb.setPixelColor(2, rgb.Color(0,0,255));
      // rgb.setPixelColor(3, rgb.Color(255,255,0));
      // rgb.show();
      {
        rgb.setPixelColor(0, rgb.Color(255,0,0));
        rgb.setPixelColor(1, rgb.Color(255,0,0));
        rgb.setPixelColor(2, rgb.Color(255,0,0));
        rgb.setPixelColor(3, rgb.Color(255,0,0));
        rgb.show();
      }
      delay(1000);
      {
        rgb.setPixelColor(0, rgb.Color(255,255,0));
        rgb.setPixelColor(1, rgb.Color(255,255,0));
        rgb.setPixelColor(2, rgb.Color(255,255,0));
        rgb.setPixelColor(3, rgb.Color(255,255,0));
        rgb.show();
      }
      delay(1000);
      {
        rgb.setPixelColor(0, rgb.Color(0,0,255));
        rgb.setPixelColor(1, rgb.Color(0,0,255));
        rgb.setPixelColor(2, rgb.Color(0,0,255));
        rgb.setPixelColor(3, rgb.Color(0,0,255));
        rgb.show();
      }
    }
    if(analogRead(A1) > 550){
      rgb.setPixelColor(0, rgb.Color(0,0,0));
      rgb.setPixelColor(1, rgb.Color(0,0,0));
      rgb.setPixelColor(2, rgb.Color(0,0,0));
      rgb.setPixelColor(3, rgb.Color(0,0,0));
      rgb.show();
    }
    delay(1000);
  }
}

void TaskAutoFan(void *pvParameters) {  // This is a task.
  //uint32_t blink_delay = *((uint32_t *)pvParameters);

  while(1) {                          
    Serial.println("Task Auto Open Fan");
    dht20.read();
    float temperature = dht20.getTemperature();
    // Serial.println(temperature);

    if (temperature > 27.5) {
      analogWrite(D7, 100);
    } else if (temperature > 27) {
      analogWrite(D7, 63);
    } else {
      analogWrite(D7, 0);
    }
    delay(1000);
  }
}

void TaskDistanceAndServo(void *pvParameters) {  // This is a task.
  //uint32_t blink_delay = *((uint32_t *)pvParameters);

  while(1) {                          
    Serial.println("Task Distance And Servo");

    if ((ultrasonic.measureDistanceCm() < 10)) {
      myservo.tick();
      myservo.write(180);
      delay(3000);
      myservo.tick();
      myservo.write(90);
    } else {
      myservo.tick();
      myservo.write(90);
    }
  }
}

void TaskMQTT(void *pvParameters) {
  MQTT_connect();
  while(1){
    dht20.read();
    
    Adafruit_MQTT_Subscribe *subscription;

    while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &ledControl) {
        Serial.print(F("Got: "));
        Serial.println((char *)ledControl.lastread);

        if (!strcmp((char*) ledControl.lastread, "ON")) {
          Serial.println("ON LED");
          onLed();
        }
        else {
          Serial.println("OFF LED");
          offLed();
        }
      }
    }

    objectCelsius = dht20.getTemperature();
    if (!temperature.publish(objectCelsius)) {
      Serial.println(F("Temperature Failed"));
    }
    else {
      Serial.println(F("Temperature OK!"));
      objectCelsius += 10;
      if(objectCelsius >= 60)
        objectCelsius = 1;
    }

    objectHumidity = dht20.getHumidity();
    if (!humidity.publish(objectHumidity)) {
      Serial.println(F("Humidity Failed"));
    }
    else {
      Serial.println(F("Humidity OK!"));
    }

    objectSoilMoisture = analogRead(A0);
    if (!soilmoisture.publish(objectSoilMoisture)) {
      Serial.println(F("Soil Moisture Failed"));
    }
    else {
      Serial.println(F("Soil Moisture OK!"));
    }

    objectLight = analogRead(A1);
    if (!light.publish(objectLight)) {
      Serial.println(F("Light Failed"));
    }
    else {
      Serial.println(F("Light OK!"));
    }
  }
}

void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();

    delay(5000); // wait 5 seconds

    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
}

void onLed() {
  rgb.setPixelColor(0, rgb.Color(255,255,255));
  rgb.setPixelColor(1, rgb.Color(255,255,255));
  rgb.setPixelColor(2, rgb.Color(255,255,255));
  rgb.setPixelColor(3, rgb.Color(255,255,255));
  rgb.show();
}

void offLed() {
  rgb.setPixelColor(0, rgb.Color(0,0,0));
  rgb.setPixelColor(1, rgb.Color(0,0,0));
  rgb.setPixelColor(2, rgb.Color(0,0,0));
  rgb.setPixelColor(3, rgb.Color(0,0,0));
  rgb.show();
}