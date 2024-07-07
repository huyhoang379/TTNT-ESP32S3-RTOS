/* Basic Multi Threading Arduino Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// Please read file README.md in the folder containing this example.

// #include "LiquidCrystal_I2C.h";
// #include "DHT20.h";

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// #include "HCSR04.h";
// #include "SoftServo.h";

// khởi tạo LED màu chân D3
Adafruit_NeoPixel rgb(4, D3, NEO_GRB + NEO_KHZ800);

// LiquidCrystal_I2C lcd(0x21,16,2); // địa chỉ màn hình là 0x21 hexa
// DHT20 DHT;

// UltraSonicDistanceSensor ultrasonic(D9, D10); //cảm biến nối với D9-D10
// SoftServo myservo;

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define ANALOG_INPUT_PIN 1


// Define two tasks for Blink & AnalogRead.
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);
TaskHandle_t analog_read_task_handle;  // You can (don't have to) use this to be able to manipulate a task from somewhere else.

// The setup function runs once when you press reset or power on the board.
void setup() {

  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // Cau hinh chan analog
  pinMode(A0, INPUT);

  rgb.begin();

  // Cấu hình chân PWM
  // pinMode(D5, OUTPUT);

  // //Initialize the LCD
  // lcd.init();
  // // Turn on the blacklight
  // lcd.backlight();

  // myservo.attach(D2); // servo nối với cổng D2
  // myservo.asyncMode();
  // myservo.delayMode();

  // myservo.tick();
  // myservo.write(90);

  // Set up two tasks to run independently.
  uint32_t blink_delay = 1000;  // Delay between changing state on LED pin
  xTaskCreate(
    TaskBlink, "Task Blink"  // A name just for humans
    ,
    2048  // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,
    (void *)&blink_delay  // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,
    2  // Priority
    ,
    NULL  // Task handle is not used here - simply pass NULL
  );

  // This variant of task creation can also specify on which core it will be run (only relevant for multi-core ESPs)
  xTaskCreatePinnedToCore(
    TaskAnalogRead, "Analog Read", 2048  // Stack size
    ,
    NULL  // When no parameter is used, simply pass NULL
    ,
    1  // Priority
    ,
    &analog_read_task_handle  // With task handle we will be able to manipulate with this task.
    ,
    ARDUINO_RUNNING_CORE  // Core on which the task will run
  );

  Serial.printf("Basic Multi Threading Arduino Example\n");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop() {
  // Doc gia tri cam bien
  int sensorValue = analogRead(A0);

  // Điều khiển đèn LED
  if (sensorValue < 1000) { // Bật đèn LED khi ánh sáng thấp
    // digitalWrite(D3, HIGH);

    rgb.setPixelColor(0, rgb.Color(255,255,255));
    rgb.setPixelColor(1, rgb.Color(255,255,255));
    rgb.setPixelColor(2, rgb.Color(255,255,255));
    rgb.setPixelColor(3, rgb.Color(255,255,255));


    rgb.show();
  } else {
    // digitalWrite(D3, LOW); // Tắt đèn LED khi ánh sáng cao

    rgb.setPixelColor(0, rgb.Color(0,0,0));
    rgb.setPixelColor(1, rgb.Color(0,0,0));
    rgb.setPixelColor(2, rgb.Color(0,0,0));
    rgb.setPixelColor(3, rgb.Color(0,0,0));
    rgb.show();
  }

  // In ra gia tri anh sang
  Serial.print("Light: ");
  Serial.println(sensorValue); // In voi 2 chu so thap phan

  delay(1000); // Cho 1 giay

  // Thay đổi chu kỳ duty cycle PWM
  // for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
  //   analogWrite(D5, dutyCycle); // Điều chỉnh tốc độ quay của quạt
  //   delay(10); // Chờ 10 mili giây
  // }

  // // Thay đổi chu kỳ duty cycle PWM theo hướng ngược lại
  // for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
  //   analogWrite(D5, dutyCycle); // Điều chỉnh tốc độ quay của quạt
  //   delay(10); // Chờ 10 mili giây
  // }

  // cảm biến ánh sáng gắn vào cổng A0
  // if ((analogRead(A0)/10.24 < 30)) {
  //   rgb.fill(rgb.Color(255,255,255));
  //   rgb.show();
  // } else {
  //   rgb.fill(rgb.Color(0,0,0));
  //   rgb.show();
  // }

  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("Hello World");
  // lcd.setCursor(0, 1);
  // lcd.print("500 ae");
  // delay(5000);

  // DHT.read();
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print((DHT.getTemperature()));
  // lcd.setCursor(0, 1);
  // lcd.print((DHT.getHumidity()));
  // if ((DHT.getTemperature() > 28)) { // nhiệt độ lớn hơn 28
  //   analogWrite(D5, 250);  // quay quạt 100%
  // } else {
  //   analogWrite(D5, 0); // tắt quạt
  // }
  // delay(5000);

  // if ((ultrasonic.measureDistanceCm() < 10)) {
  //   myservo.tick();
  //   myservo.write(180);
  //   delay(3000);
  //   myservo.tick();
  //   myservo.write(90);
  // } else {
  //   myservo.tick();
  //   myservo.write(90);
  // }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters) {  // This is a task.
  uint32_t blink_delay = *((uint32_t *)pvParameters);

  /*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  If you want to know what pin the on-board LED is connected to on your ESP32 model, check
  the Technical Specs of your board.
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) {                          // A Task shall never return or exit.
    Serial.println("TaskBlink: LED_ON");
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    // arduino-esp32 has FreeRTOS configured to have a tick-rate of 1000Hz and portTICK_PERIOD_MS
    // refers to how many milliseconds the period between each ticks is, ie. 1ms.
    delay(blink_delay);
    Serial.println("TaskBlink: LED_OFF");
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(blink_delay);
  }
}

void TaskAnalogRead(void *pvParameters) {  // This is a task.
  (void)pvParameters;
  // Check if the given analog pin is usable - if not - delete this task
  if (digitalPinToAnalogChannel(ANALOG_INPUT_PIN) == -1) {
    Serial.printf("TaskAnalogRead cannot work because the given pin %d cannot be used for ADC - the task will delete itself.\n", ANALOG_INPUT_PIN);
    analog_read_task_handle = NULL;  // Prevent calling vTaskDelete on non-existing task
    vTaskDelete(NULL);               // Delete this task
  }

  /*
  AnalogReadSerial
  Reads an analog input on pin A3, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A3, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

  for (;;) {
    // read the input on analog pin:
    int sensorValue = analogRead(A1);
    // print out the value you read:
    Serial.print("TaskAnalogRead: ");
    Serial.println(sensorValue);
    delay(1000);  // 100ms delay
  }
}