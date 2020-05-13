#include "task.h"
#include "Arduino.h"

A2Task tasks[MAX_TASKS];
short current_task = -1;
unsigned short task_list_index = 0;
void SetupTasks() {
  // Clear all the task info
  Serial.println("SetupTask");
  memset(tasks, 0, MAX_TASKS * sizeof(A2Task));
}

void CreateTask(char* name, TaskFunction_t function) {
  Serial.println("Creating task");
  int i = 0;
  for (i = 0; i < MAX_TASKS; i++) {
    if (tasks[i].function == NULL) break;
  }
  if (i == MAX_TASKS) {
    Serial.println("I can't create more tasks");
    assert(false);
    return;
  }
  tasks[i].task_name = name;
  tasks[i].function = function;
  tasks[i].waitUntil = 0;
  tasks[i].enabled = true;
  tasks[i].ticks = 0;
}
void SleepFor(unsigned int ticks) {
  if (current_task == -1) return;
  tasks[current_task].waitUntil = xTaskGetTickCount() + ticks;
}
void DisableTask() {
  if (current_task == -1) return;
  tasks[current_task].enabled = false;
}
void EnableTask() {
  if (current_task == -1) return;
  tasks[current_task].enabled = true;
}

A2Task* GetNextReadyTaskFromList() {
  int count = 0;
  TickType_t ticks = xTaskGetTickCount();
  A2Task* task;
  while (count < MAX_TASKS) {
    count += 1;
    task_list_index += 1;
    task_list_index = task_list_index % MAX_TASKS;
    task = &tasks[task_list_index];
    Serial.print("Looking at ");
    Serial.println(task_list_index);
    if (task->function == NULL) continue;
    if (task->enabled == false) continue;
    if (task->waitUntil != 0 && task->waitUntil < ticks) continue;
    Serial.println("Picked");
  }
  if (count == MAX_TASKS) {
    current_task = -1;
    return NULL;
  }
  current_task = task_list_index;
  tasks[current_task].ticks ++;
  return &tasks[current_task];
}
