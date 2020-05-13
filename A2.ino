#define CONFIG_ESP32_ENABLE_COREDUMP 1
#include "esp_task_wdt.h"
#include <CircularBuffer.h>
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "nextdns.h"
#include "task.h"
#define DEBUG 0

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
CircularBuffer<float, MAX_ACCEL_SAMPLES_PER_SECOND * 3> accel_data;

// Tasks
void TaskReadBattery( void *pvParameters );
void TaskReadAccel( void *pvParameters );
void TaskUpdateDisplay (void *pvParameters );
void TaskUpdateStepCount( void *pvParameters );
void TaskWifi( void *pvParameters );

// Common data
unsigned int stepCount = 0;
unsigned short batteryLevel = 0; // between 0 and 100, 0 is unknown
int happiness = 100; // Max of -100 to 100

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(50);

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
    imu.setFifoMode(LIS3DH_FIFO_BYPASS);
    imu.setDataRate(LIS3DH_DATARATE_100_HZ);
    imu.setFifoMode(LIS3DH_FIFO_STREAM);
    imu.setDataRate(LIS3DH_DATARATE_100_HZ);
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
#if DEBUG
        Serial.print("We have no tasks@");
        Serial.print(xTaskGetTickCount());
        Serial.println();
#endif
        vTaskDelay(1);
        // Go into deep sleep?
      }
      else if (ticks > 1) {
#if DEBUG
        Serial.print("\tMain Loop Sleeping for ");
        Serial.println(ticks);
#endif
        vTaskDelay(ticks);
      }
      continue;
    }
#if DEBUG
    Serial.print("Executing @");
    Serial.print(xTaskGetTickCount());
    Serial.print("->");
    PrintTask(task);
#endif
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
  static int old_happiness = 0;
  SleepFor(15 * 1000); // wait 15 seconds
  if (old_happiness == happiness) return;
    
  old_happiness = happiness;
  if (happiness < angry) {
    // Turn off dns parental controls
    NextDNSTurnOnParentalControls();
  }
  if (happiness > happy) {
    // Turn on dns parental controls
    NextDNSTurnOffParentalControls();
  }
}

void TaskReadBattery( void *pvParameters ) {
  const int battery_taskDelay = 10000;
  const float low_voltage = 3.27;
  const float high_voltage = 4.3;

  SleepFor(battery_taskDelay);  // one tick delay (15ms) in between reads for stability

  int value = analogRead(PIN_BATTERY_ADC);
  float voltage = (value / 4095.0) * 2 * 3.3 * 1.1;
  batteryLevel = map(voltage, low_voltage, high_voltage, 1, 100);
#if DEBUG
  Serial.println("Reading battery");
  Serial.print(value);
  Serial.print("\t volts:");
  Serial.print(voltage);
  Serial.print("\tBattery level");
  Serial.print(batteryLevel);
  Serial.println();
#endif
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
#if DEBUG
  Serial.print("ReadAccel ");
  Serial.print(accel_data.size());
#endif

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
#if DEBUG
  Serial.print("\t");
  Serial.print(accel_task_count_read);
  Serial.println();
#endif
}

// we get an unhappy every second there isn't a step
const int angry = -100; // once you get below this, you're in for a ride
const int happy = 100; // once you get above this, it's as happy as can be
const int max_happy = 200; // don't change happy once it's above this or below negative this

void TaskUpdateStepCount( void *pvParameters ) {
  // I've implemented enough convolution in my lifetime (EE Signal processing)
  // I'm just going to go the easy way
  //https://github.com/tttapa/Arduino-Filters/
  // Sampling frequency
  const double f_s = MAX_ACCEL_SAMPLES_PER_SECOND; // Hz
  const float min_sample_power = 13.0; // the minimum that we need to do
  const int min_samples_between_steps = 100 * 300 / 1000; //300ms
  // Cut-off frequency (-3 dB)
  const double f_c = 40; // Hz
  // Normalized cut-off frequency
  const double f_n = 2 * f_c / f_s;
  // Fifth-order Butterworth filter
  static auto butter_filter = butter<5>(f_n);
  static int stepcountask_delay_rate = 1000;
  const unsigned short happy_counter_max = MAX_ACCEL_SAMPLES_PER_SECOND;
  static unsigned short happy_counter = happy_counter_max;
  static boolean x_2to1_up = false; // if the sample at x[-2] < x[-1]
  static boolean x_1to0_up = false; // if the sample at x[-1] < x[0]
  static float x_1_value = 0;
  static unsigned int samples_until_ready_to_step = 0;

  // The actual task
  SleepFor(stepcountask_delay_rate); // This won't sleep until we exit
  // don't bother updating the step count if we don't have enough data
  if (accel_data.size() < MAX_ACCEL_SAMPLES_PER_SECOND) {
    return;
  }
  int buffer_size = accel_data.size();
  if (buffer_size > MAX_ACCEL_SAMPLES_PER_SECOND * 1.5 && stepcountask_delay_rate > 5) stepcountask_delay_rate--;
  // TODO: figure out if we aren't moving enough- turn ourselves off if we are
#if DEBUG
  Serial.print("TaskUpdateStepCount: ");
  Serial.println(buffer_size);
#endif
  for (int i = 0; i < buffer_size; i++) {
    if (samples_until_ready_to_step > 0) samples_until_ready_to_step --;
    float raw_data = accel_data.pop();
    float f_data = butter_filter(raw_data); // filtered data
    //Rolling average with a 3:2 ratio in importance
    float avg_data = (f_data * 3 + x_1_value * 2) / 5;
    float final_data = avg_data;
    x_1to0_up = x_1_value < final_data;
    // Detect a step
    if (samples_until_ready_to_step == 0 && x_1_value > min_sample_power && !x_1to0_up && x_2to1_up) {
      //We've detected a step
      stepCount += 1;
      if (happiness < 1) happiness ++; // if we're below happy, we get an extra boost
      happiness += 2; // we get two happiness with a step
      happy_counter = happy_counter_max;
      samples_until_ready_to_step = min_samples_between_steps;
    }
    // if the counter went off, we go down on the happiness meter
    if (happy_counter == 0) {
      happy_counter = happy_counter_max;
      happiness --;
    }
    else happy_counter --;
    // pass the info on
    x_2to1_up = x_1to0_up;
    x_1_value = final_data;
#if DEBUG
    Serial.print(raw_data);
    Serial.print("\t");
    Serial.print(f_data);
    Serial.print("\t");
    Serial.print(final_data);
    Serial.print("\t");
    Serial.print(found_step);
    Serial.println();
#endif
  }
  if (happiness > max_happy) happiness = max_happy;
  else if (happiness < -1 * max_happy) happiness = -1 * max_happy;
}

void TaskUpdateDisplay (void *pvParameters ) {
  static int display_task_delay = 500;
  static int display_prev_hash = 0;

  SleepFor(display_task_delay); // This won't sleep until we exit
  int new_hash = (batteryLevel + 1) * (stepCount + 1) * (happiness == 0 ? 1 : happiness);
#if DEBUG
  Serial.print("TaskUpdateDisplay");
#endif
  if (new_hash == display_prev_hash) {
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
  else if (temp_step_count < 10 || (temp_step_count > 99 && temp_step_count < 1000)) {
    itoa(temp_step_count, &step_count_text[1], 10);
    step_count_text[0] = '0';
  }
  else itoa(temp_step_count, step_count_text, 10);

  display.setTextSize(step_count_text_size + 1);
  display.getTextBounds("5", 0, 0, &x, &y, &w, &h);
  int16_t step_count_spacing_x = (step_count_box_w - (w * 2)) / 3;
  int16_t step_count_spacing_y = (step_count_box_h - (h * 2)) / 3;
  int16_t step_count_col2_x = step_count_box_x + w + step_count_spacing_x * 2;
  int16_t step_count_row2_y = step_count_box_y + h + step_count_spacing_y * 2;
  display.setCursor(step_count_box_x + step_count_spacing_x, step_count_box_y + step_count_spacing_y);
  display.write(step_count_text[0]);
  display.setCursor(step_count_col2_x, step_count_box_y + step_count_spacing_y);
  display.write(step_count_text[1]);
  display.setCursor(step_count_box_x + step_count_spacing_x, step_count_row2_y);
  display.write(step_count_text[2]);
  display.setCursor(step_count_col2_x, step_count_row2_y);
  display.write(step_count_text[3]);

  // Draw the face
  const uint16_t face_box_border = 5;
  const uint16_t face_box_x = face_box_border;
  const uint16_t face_box_y = face_box_border + 5;
  const uint16_t face_box_w = SCREEN_WIDTH - step_count_box_x - face_box_border;
  const uint16_t face_box_h = SCREEN_HEIGHT - face_box_y - face_box_border;
  const uint16_t face_eye_w = face_box_w / 3; // Each eye is a third of the box width
  const uint16_t face_eye_box_h = 2 * face_box_h / 3; // Each eye box is two thirds of the box height
  const uint16_t face_eye_padding_y = face_eye_box_h / 6;
  const uint16_t face_eye_h = face_eye_box_h - (face_eye_padding_y * 2); // Each eye bo is two thirds of the box height
  const uint16_t face_eye_padding_x = (face_box_w - face_eye_w * 2) / 3;
  const uint16_t face_pupil_w = face_eye_w / 3;
  const uint16_t face_pupil_h = face_eye_h / 3;
  //Draw the eyes
  const uint16_t face_eye_x1 = face_box_x + face_eye_padding_x;
  const uint16_t face_eye_y1 = face_box_y + face_eye_padding_y;
  const uint16_t face_eye_mid_y = face_eye_y1 + (face_eye_h / 2);
  display.drawRect(face_eye_x1, face_eye_y1, face_eye_w, face_eye_h, SSD1306_WHITE);
  const uint16_t face_eye_x2 = face_eye_x1 + face_eye_w + face_eye_padding_x;
  display.drawRect(face_eye_x2, face_eye_y1, face_eye_w, face_eye_h, SSD1306_WHITE);

  // Draw angry eyebrows
  if (happiness < angry) {
    display.drawLine(face_eye_x1, face_eye_y1, face_eye_x1 + face_eye_w, face_eye_mid_y, SSD1306_WHITE);
    display.drawLine(face_eye_x2, face_eye_mid_y, face_eye_x2 + face_eye_w, face_eye_y1, SSD1306_WHITE);
  }

  // Draw the pupils
  boolean eye_blink = random(5) == 1 && abs(happiness) < (happy / 3); // 1 in ten chance
  static boolean invert_x = false;
  static boolean invert_y = false;
  if (random(10) == 1) invert_x = !invert_x; // 1 in ten chance
  if (random(10) == 1) invert_y = !invert_y; // 1 in ten chance

  if (eye_blink) {
    display.drawLine(face_eye_x1, face_eye_mid_y, face_eye_x1 + face_eye_w, face_eye_mid_y, SSD1306_WHITE);
    display.drawLine(face_eye_x2, face_eye_mid_y, face_eye_x2 + face_eye_w, face_eye_mid_y, SSD1306_WHITE);
  }
  else {
    uint16_t pupil_x_mod = invert_x ? face_eye_w - face_pupil_w - 2 : 2;
    uint16_t pupil_y_mod = invert_y ? face_eye_h - face_pupil_h - 4 : 4;
    display.fillRect(face_eye_x1 + pupil_x_mod, face_eye_y1 + pupil_y_mod, face_pupil_w, face_pupil_h, SSD1306_WHITE);
    display.fillRect(face_eye_x2 + pupil_x_mod, face_eye_y1 + pupil_y_mod, face_pupil_w, face_pupil_h, SSD1306_WHITE);
  }

  // Mouth
  const uint16_t face_mouth_h = face_box_h / 3; // the height of the mouth box
  const uint16_t face_mouth_w = face_box_w; // the width of the mouth box
  const uint16_t face_mouth_x = face_box_x + (face_box_w - face_mouth_w); // the y of the left upper corner of the mouth box

  const uint16_t face_mouth_y = face_box_y + (face_box_h - face_mouth_h); // the y of the left upper corner of the mouth box
  const uint16_t face_mouth_mid_y = face_mouth_y + (face_mouth_h / 2); // the y of the middle mouth box
  const uint16_t face_mouth_mid_x = face_mouth_x + (face_mouth_w / 2); // the x of the middle mouth box
  // Draw the mouth
  int face_happiness = map(happiness, angry, happy, face_mouth_h / 2, face_mouth_h / -2);
  if (abs(face_happiness) < face_mouth_h / 4) {
    display.drawLine(face_mouth_x, face_mouth_mid_y + face_happiness, face_mouth_mid_x - (face_mouth_w / 3), face_mouth_mid_y, SSD1306_WHITE);
    display.drawLine(face_mouth_mid_x - (face_mouth_w / 3), face_mouth_mid_y, face_mouth_mid_x + (face_mouth_w / 3), face_mouth_mid_y, SSD1306_WHITE);
    display.drawLine(face_mouth_x + face_mouth_w, face_mouth_mid_y + face_happiness, face_mouth_mid_x + (face_mouth_w / 3), face_mouth_mid_y, SSD1306_WHITE);
  }
  else {
    display.drawLine(face_mouth_x, face_mouth_mid_y + face_happiness, face_mouth_mid_x, face_mouth_mid_y, SSD1306_WHITE);
    display.drawLine(face_mouth_x + face_mouth_w, face_mouth_mid_y + face_happiness, face_mouth_mid_x, face_mouth_mid_y, SSD1306_WHITE);
  }
  // Print happiness level for debugging
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Happy"));
  display.print(happiness);
  display.display();
}
