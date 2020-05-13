#define CONFIG_ESP32_ENABLE_COREDUMP 1
#include "esp_task_wdt.h"
#include <CircularBuffer.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "task.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define MAX_ACCEL_SAMPLES_PER_SECOND 100 // We're assuming max of 60hz

// PIN declerations
#define PIN_BATTERY_ADC A13

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Task Mutex
SemaphoreHandle_t xSemaphore = NULL;

// Display + IMU
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_LIS3DH imu;
boolean accel_started = false;

//Accel data buffer
CircularBuffer<float, MAX_ACCEL_SAMPLES_PER_SECOND * 4> accel_data;

// Tasks
void TaskReadBattery( void *pvParameters );
void TaskReadAccel( void *pvParameters );
void TaskUpdateDisplay (void *pvParameters );
void TaskUpdateStepCount( void *pvParameters );
void TaskWifi( void *pvParameters );

// Common data
unsigned int stepCount = 92;
unsigned short batteryLevel = 0; // between 0 and 100, 0 is unknown

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  // Display
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // IMU
  imu = Adafruit_LIS3DH();
  if (imu.begin()) {   // 0x18, change this to 0x19 for alternative i2c address
    Serial.println("Started Gyro");
    imu.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
    //imu.setDataRate(LIS3DH_DATARATE_POWERDOWN); // by default, we power down
    imu.setDataRate(LIS3DH_DATARATE_100_HZ);
    imu.setFifoMode(LIS3DH_FIFO_MODE);
    imu.setClick(1, 120); // single click, threshold 110
    accel_started = true;
  }
  else {
    Serial.println("Failed to start Gyro");
  }

  // Task setup
  SetupTasks();
  CreateTask("AccelRead", TaskReadAccel);
  CreateTask("Wifi", TaskWifi);
  CreateTask("ReadBattery", TaskReadBattery);
  CreateTask("UpdateStepCount", TaskUpdateStepCount);
  CreateTask("UpdateDisplay", TaskUpdateDisplay);
}


void loop() {
  for (;;) {
    A2Task* task = GetNextReadyTaskFromList();
    if (task == NULL) {
      //Serial.println("Nothing to do");
      // go to low power mode for a second
      TickType_t ticks = GetTicksUntilNextReadyTask();
      if (ticks == 0) {
        Serial.print("We have no tasks@");
        Serial.print(xTaskGetTickCount());
        Serial.println();
        vTaskDelay(1);
        // Go into deep sleep?
      }
      else if (ticks > 1) {
        Serial.print("\tMain Loop Sleeping for ");
        Serial.println(ticks);
        vTaskDelay(ticks);
      }
      continue;
    }
    Serial.print("Executing @");
    Serial.print(xTaskGetTickCount());
    Serial.print("->");
    PrintTask(task);

    // Execute the task that was picked
    task->function(NULL);
  }
}

void TaskWifi( void *pvParameters ) {
  static boolean wifi_connecting = false;
  static unsigned short wifi_connecting_count = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (wifi_connecting_count == 50) { //if we've been connecting for more than 25 seconds
      wifi_connecting = false;
      SleepFor(60 * 1000); // wait a minute before trying again
      return;
    }
    if (wifi_connecting) { // if we're connecting
      SleepFor(500); // wait half a second
      wifi_connecting_count ++;
    }
    else { // start connecting
      WiFi.begin(ssid, password);
      wifi_connecting = true;
      wifi_connecting_count = 0;
      Serial.println("Establishing connection to WiFi..");
    }
    return;
  }
  // Turn off this task until we need it again?
  DisableTask();
  // We are connected, do stuff
}

void TaskReadBattery( void *pvParameters ) {
  const int battery_taskDelay = 10000;
  const float low_voltage = 3.27;
  const float high_voltage = 4.3;

  SleepFor(battery_taskDelay);  // one tick delay (15ms) in between reads for stability
  Serial.println("Reading battery");
  int value = analogRead(PIN_BATTERY_ADC);
  float voltage = (value / 4095.0) * 2 * 3.3 * 1.1;
  batteryLevel = map(voltage, low_voltage, high_voltage, 1, 100);
  Serial.print(value);
  Serial.print("\t volts:");
  Serial.print(voltage);
  Serial.print("\tBattery level");
  Serial.print(batteryLevel);
  Serial.println();

}

void TaskReadAccel( void *pvParameters ) {
  static unsigned int accel_task_read_fruitless = 0;
  unsigned int accel_task_count_read = 0;
  // we're going to shoot for 15 samples each time
  static unsigned int accel_task_delay_rate = 1000 / MAX_ACCEL_SAMPLES_PER_SECOND * 15; 
  SleepFor(accel_task_delay_rate); // This won't sleep until we exit
  if (!accel_started || !imu.haveNewData()) {
    accel_task_read_fruitless ++;
    return;
  }
  Serial.print("ReadAccel");
  Serial.print(accel_data.size());

  if (accel_task_read_fruitless > 0) accel_task_delay_rate ++; // adjust our tick rate as needed
  accel_task_read_fruitless = 0;
  accel_task_count_read = 0;
  while (imu.haveNewData() && !accel_data.isFull()) {
    sensors_event_t event;
    imu.getEvent(&event);
    float square_mag = pow(event.acceleration.y, 2) + pow(event.acceleration.x, 2) + pow(event.acceleration.z, 2);
    float mag = sqrt(square_mag);
    accel_data.push(mag);
    accel_task_count_read ++;
  }
  if (imu.getClick()) {
    stepCount ++;
  }
  Serial.print("\t");
  Serial.print(accel_task_count_read);
  Serial.println();
}


void TaskUpdateStepCount( void *pvParameters ) {
  static int stepcountask_delay_rate = 1000;
  SleepFor(stepcountask_delay_rate); // This won't sleep until we exit
  // don't bother updating the step count if we don't have enough data
  if (accel_data.size() < MAX_ACCEL_SAMPLES_PER_SECOND) {
    return;
  }
  int buffer_size = accel_data.size();
  if (buffer_size > MAX_ACCEL_SAMPLES_PER_SECOND * 1.5 && stepcountask_delay_rate > 5) stepcountask_delay_rate--;
  // TODO: figure out if we aren't moving enough- turn ourselves off if we are
  Serial.print("TaskUpdateStepCount: ");
  Serial.println(buffer_size);
  for (int i = 0; i < buffer_size; i++) {
    accel_data.pop();
  }
}

void TaskUpdateDisplay (void *pvParameters ) {
  static int display_task_delay = 500;
  static int display_prev_hash = 0;

  SleepFor(display_task_delay); // This won't sleep until we exit
  int new_hash = (batteryLevel + 1) * (stepCount + 1);
  Serial.print("TaskUpdateDisplay");
  if (new_hash == display_prev_hash) {
    Serial.println("- skipped");
    return;
  }
  display_prev_hash = new_hash;

  int16_t x, y = 0;
  uint16_t w, h = 0;
  const int16_t step_count_box_x = SCREEN_WIDTH / 2; // this is where our box starts
  const int16_t step_count_box_y = 12; // this is where our box starts
  const int16_t step_count_box_w = SCREEN_WIDTH - step_count_box_x;
  const int16_t step_count_box_h = SCREEN_HEIGHT - step_count_box_y;
  const int16_t step_count_text_size = 2;

  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  // Draw the battery level (ie 95%)
  char batteryLevelText[10];
  if (batteryLevel > 0) sprintf(batteryLevelText, "%d%%", batteryLevel);
  else sprintf(batteryLevelText, "None");
  display.getTextBounds(batteryLevelText, 0, 0, &x, &y, &w, &h); // the longest string
  display.setCursor(SCREEN_WIDTH - w - 1, 0);     // Start at top-left corner
  display.print(batteryLevelText);
  if (stepCount < 100) display.setTextSize(step_count_text_size + 1);      // Normal 1:1 pixel scale
  else display.setTextSize(step_count_text_size);      // Normal 1:1 pixel scale
  char step_count_text[5];
  memset(step_count_text, ' ', 5); // make sure to zero everything to zero?
  unsigned int temp_step_count = stepCount > 9999 ? 9999 : stepCount; // we max out at 9999
  
  if (!accel_started) sprintf(step_count_text, "NONE");
  else if (temp_step_count > 99 && temp_step_count < 1000) {
    itoa(temp_step_count, &step_count_text[1], 10);
    step_count_text[0] = '0';
  }
  else itoa(temp_step_count, step_count_text, 10);
  
  Serial.print("\tStep text: ");
  for (int i = 0; i < 4; i++) Serial.print(step_count_text[i]);
  display.setTextSize(step_count_text_size + 1);
  display.getTextBounds("5", 0, 0, &x, &y, &w, &h);
  int16_t step_count_spacing_x = (step_count_box_w - (w * 2)) / 4;
  int16_t step_count_spacing_y = (step_count_box_h - (h * 2)) / 4;
  int16_t step_count_col2_x = step_count_box_x + w + step_count_spacing_x * 3;
  int16_t step_count_row2_y = step_count_box_y + h + step_count_spacing_y * 3;
  display.setCursor(step_count_box_x + step_count_spacing_x, step_count_box_y + step_count_spacing_y);
  display.write(step_count_text[0]);
  display.setCursor(step_count_col2_x, step_count_box_y + step_count_spacing_y);
  display.write(step_count_text[1]);
  display.setCursor(step_count_box_x + step_count_spacing_x, step_count_row2_y);
  display.write(step_count_text[2]);
  display.setCursor(step_count_col2_x, step_count_row2_y);
  display.write(step_count_text[3]);

  // Draw the face
  display.drawRect(15,15, 10, 20, SSD1306_WHITE);
  display.drawRect(30,15, 10, 20, SSD1306_WHITE);
  display.drawLine(17, 50, 38, 50, SSD1306_WHITE),

  /*display.getTextBounds(step_count_text, 0, 0, &x, &y, &w, &h); // the longest string
    Serial.print("\t X,Y: ");
    Serial.print(step_count_box_x + (w / 2));
    Serial.print(step_count_box_y + (h / 2));
    display.print(step_count_text);*/

  // Make sure to put it out to the display
  Serial.print("\tDisplay");
  display.display();
  Serial.println("END");

}
