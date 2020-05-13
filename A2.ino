#include "esp_task_wdt.h"
#include <CircularBuffer.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_SSD1306.h>


#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define CONFIG_FREERTOS_UNICORE

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
CircularBuffer<float, MAX_ACCEL_SAMPLES_PER_SECOND * 2> accel_data;

// Tasks
void TaskReadBattery( void *pvParameters );
void TaskReadAccel( void *pvParameters );
void TaskUpdateDisplay (void *pvParameters );
void TaskUpdateStepCount( void *pvParameters );

TaskHandle_t xTaskReadAccelHandle;
TaskHandle_t xTaskUpdateStepCountHandle;
TaskHandle_t xTaskReadBatteryHandle;

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("Tick rate");
  Serial.println(portTICK_RATE_MS); // how many ms a tick is
  Serial.println(portTICK_PERIOD_MS);
  Serial.println(configTICK_RATE_HZ); // how many times a tick happens a second

  //Create the semaphore
  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore); // make sure to set it up as available
  Serial.print("TASK MAX");
  Serial.println(configMAX_PRIORITIES);

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
    accel_started = true;
  }
  else {
    Serial.println("Failed to start Gyro");
  }

  // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskReadBattery
    ,  "TaskReadBattery"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &xTaskReadBatteryHandle
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    TaskReadAccel
    ,  "TaskReadAccel"   // A name just for humans
    ,  4024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  configMAX_PRIORITIES - 5 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &xTaskReadAccelHandle
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    TaskUpdateDisplay
    ,  "TaskUpdateDisplay"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  configMAX_PRIORITIES - 9  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    TaskUpdateStepCount
    ,  "TaskUpdateStepCount"   // A name just for humans
    ,  8024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  configMAX_PRIORITIES - 7 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &xTaskUpdateStepCountHandle
    ,  ARDUINO_RUNNING_CORE);
}

void loop() {
  // This is empty since we use tasks
  static int last_ticks = 0;
  int tick_count = xTaskGetTickCount();
  if (tick_count % 500 == 0 && tick_count != last_ticks) {
    Serial.print("TICKS: ");
    Serial.println(xTaskGetTickCount());
    last_ticks = tick_count;
    xSemaphoreGive(xSemaphore);
  }


}

int batteryLevel = 0; // between 0 and 100, 0 is unknown
void TaskReadBattery( void *pvParameters ) {
  const int taskDelay = 10000;
  const float low_voltage = 3.27;
  const float high_voltage = 4.3;
  for (;;) // A Task shall never return or exit.
  {
    vTaskDelay(taskDelay);  // one tick delay (15ms) in between reads for stability
    Serial.println("Reading battery");
    boolean has_sema = xSemaphoreTake(xSemaphore, taskDelay / 2);
    if (!has_sema) continue;
    int value = analogRead(PIN_BATTERY_ADC);
    float voltage = (value / 4095.0) * 2 * 3.3 * 1.1;
    batteryLevel = map(voltage, low_voltage, high_voltage, 1, 100);
    Serial.print(value);
    Serial.print("\t volts:");
    Serial.print(voltage);
    Serial.print("\tBattery level");
    Serial.print(batteryLevel);
    Serial.println();
    xSemaphoreGive(xSemaphore);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level);
    vTaskDelay(taskDelay);  // one tick delay (15ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

  }
}

void TaskReadAccel( void *pvParameters ) {
  // Decleration for the LIS3DH accelerometer
  unsigned int task_read_fruitless = 0;
  unsigned int task_count_read = 0;
  unsigned int task_delay_rate = 1000 / MAX_ACCEL_SAMPLES_PER_SECOND;
  // Setup the gyro
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, task_delay_rate);
    xLastWakeTime = xTaskGetTickCount();
    if (!accel_started || !imu.haveNewData()) {
      task_read_fruitless ++;
      continue;
    }
    else {
      Serial.println("ReadAccel");
      boolean has_sema = xSemaphoreTake(xSemaphore, task_delay_rate / 2);
      if (has_sema) {
        //Serial.print("TaskReadAccel");
        //Serial.print(task_read_fruitless);
        if (task_read_fruitless > 0) task_delay_rate ++; // adjust our tick rate as needed
        task_read_fruitless = 0;
        task_count_read = 0;
        while (imu.haveNewData()) {
          sensors_event_t event;
          imu.getEvent(&event);
          double square_mag = ((double)pow(event.acceleration.y, 2)) + pow(event.acceleration.x, 2) + pow(event.acceleration.z, 2);
          float mag = sqrt(square_mag);
          accel_data.push(mag);
          task_count_read ++;
        }
        //Serial.print(" = ");
        //Serial.print(task_count_read);
        if (task_count_read > 1 && task_delay_rate > 3) task_delay_rate --;
        xSemaphoreGive(xSemaphore);
        //Serial.println();
      }
      else {
        vTaskDelay(1);  // one tick delay (1ms)
        task_read_fruitless ++;
      }
    }
  }
}

unsigned int stepCount = 32;
void TaskUpdateStepCount( void *pvParameters ) {
  int task_delay_rate = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, task_delay_rate);
    xLastWakeTime = xTaskGetTickCount();

    boolean has_sema = xSemaphoreTake(xSemaphore, task_delay_rate / 2);
    if (!has_sema) continue;
    Serial.print("StepCount:");
    Serial.println(accel_data.size());
    stepCount += 1;
    // don't bother updating the step count if we don't have enough data
    if (accel_data.size() < MAX_ACCEL_SAMPLES_PER_SECOND) {
      xSemaphoreGive(xSemaphore);
      continue;
    }

    int buffer_size = accel_data.size();
    if (buffer_size > MAX_ACCEL_SAMPLES_PER_SECOND * 1.5 && task_delay_rate > 5) task_delay_rate--;
    // TODO: figure out if we aren't moving enough- turn ourselves off if we are
    Serial.print("TaskUpdateStepCount: ");
    Serial.println(buffer_size);
    for (int i = 0; i < buffer_size; i++) {
      accel_data.pop();
    }
    xSemaphoreGive(xSemaphore);
  }
}

void SuspendAllTasks(TaskHandle_t curr_task) {
  Serial.println("Resume SUSPEND");
  if (curr_task != xTaskReadAccelHandle) {
    vTaskSuspend(xTaskReadAccelHandle);
  }
  if (curr_task != xTaskUpdateStepCountHandle) {
    vTaskSuspend(xTaskUpdateStepCountHandle);
  }
  //xTaskReadBatteryHandle
  if (curr_task != xTaskReadBatteryHandle) {
    vTaskSuspend(xTaskReadBatteryHandle);
  }
}
void ResumeAllTasks(TaskHandle_t curr_task) {
  Serial.println("Resume START");
  if (curr_task != xTaskReadAccelHandle) {
    vTaskResume(xTaskReadAccelHandle);
  }
  //xTaskUpdateStepCountHandle
  if (curr_task != xTaskUpdateStepCountHandle) {
    vTaskResume(xTaskUpdateStepCountHandle);
  }
  //xTaskReadBatteryHandle
  if (curr_task != xTaskReadBatteryHandle) {
    vTaskResume(xTaskReadBatteryHandle);
  }
  Serial.println("Resume END");
}

void TaskUpdateDisplay (void *pvParameters ) {
  TaskHandle_t curr_task = xTaskGetIdleTaskHandle();
  // Show initial display buffer contents on the screen --
  // TODO: show my logo not theirs?
  vTaskDelay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.display();

  int taskDelay = 500;
  int16_t x, y = 0;
  uint16_t w, h = 0;
  const int16_t step_count_box_x = SCREEN_WIDTH / 2; // this is where our box starts
  const int16_t step_count_box_y = 10; // this is where our box starts
  const int16_t step_count_box_w = SCREEN_WIDTH - step_count_box_x;
  const int16_t step_count_box_h = SCREEN_HEIGHT - step_count_box_y;
  const int16_t step_count_text_size = 2;

  int prev_hash = 0;
  for (;;)
  {
    vTaskDelay(taskDelay);
    int new_hash = (batteryLevel + 1) * (stepCount + 1);
    Serial.print("TaskUpdateDisplay");
    if (new_hash == prev_hash) {
      Serial.println("- skipped");
      continue;
    }
    boolean has_sema = xSemaphoreTake(xSemaphore, taskDelay / 2);
    if (!has_sema) continue;
    SuspendAllTasks(curr_task);

    prev_hash = new_hash;
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
    itoa(temp_step_count, step_count_text, 10);
    Serial.print("\tStep text: ");
    for (int i = 0; i < 4; i++) Serial.print(step_count_text[i]);
    /*
      display.setCursor(step_count_box_x + step_count_spacing_x, step_count_box_y + step_count_spacing_y);
      display.write(step_count_text[0]);

      display.setCursor(step_count_col2_x, step_count_box_y + step_count_spacing_y);
      display.write(step_count_text[1]);
      display.setCursor(step_count_box_x + step_count_spacing_x, step_count_row2_y);
      display.write(step_count_text[2]);
      display.setCursor(step_count_col2_x, step_count_row2_y);
      display.write(step_count_text[3]);
    */
    if (!accel_started) sprintf(step_count_text, "N/A");
    display.getTextBounds(step_count_text, 0, 0, &x, &y, &w, &h); // the longest string
    Serial.print("\t X,Y: ");
    Serial.print(step_count_box_x + (w / 2));
    Serial.print(step_count_box_y + (h / 2));

    display.setCursor(0, 0);

    // Make sure to put it out to the display
    Serial.print("\tDisplay");
    display.display();
    Serial.println("END");
    xSemaphoreGive(xSemaphore);
    ResumeAllTasks(curr_task);
  }
}
