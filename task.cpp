#include "task.h"
#include "Arduino.h"
#define TASKDEBUG 0

A2Task tasks[MAX_TASKS];
short current_task = -1;
void SetupTasks() {
  // Clear all the task info
  memset(tasks, 0, MAX_TASKS * sizeof(A2Task));
}

void CreateTask(char* task_name, TaskFunction_t function) {

  int i = 0;
  for (i = 0; i < MAX_TASKS; i++) {
    if (tasks[i].function == NULL) break;
  }
  if (i == MAX_TASKS) {
    Serial.println("I can't create more tasks");
    assert(false);
    return;
  }
#if TASKDEBUG
  Serial.print("Creating task");
  Serial.print(" at ");
  Serial.print(i);
  Serial.println();
#endif
  tasks[i].task_name = task_name;
  tasks[i].function = function;
  tasks[i].waitUntil = 0;
  tasks[i].enabled = true;
  tasks[i].ticks = 0;
}
void SleepFor(unsigned int ticks) {
  if (current_task == -1) return;
  tasks[current_task].waitUntil = xTaskGetTickCount() + ticks;
#if TASKDEBUG
  Serial.print(tasks[current_task].task_name);
  Serial.print("\tTask sleeping for ");
  Serial.print(ticks);
  Serial.print("\t");
  Serial.print(tasks[current_task].waitUntil);
  Serial.println();
#endif
}

void DisableTask() {
  if (current_task == -1) return;
  tasks[current_task].enabled = false;
}
void EnableTask() {
  if (current_task == -1) return;
  tasks[current_task].enabled = true;
}

void PrintTask(A2Task* task) {
  if (task == NULL) {
    Serial.println("Bad task");
  }
  Serial.print(task->task_name);
  Serial.print("\t Enable:");
  Serial.print(task->enabled);
  Serial.print("\t Wait:");
  Serial.print(task->waitUntil);
  Serial.print("\t Ticks:");
  Serial.print(task->ticks);
  Serial.println();
}

A2Task* GetNextReadyTaskFromList() {
  unsigned int count = 0;
  TickType_t ticks = xTaskGetTickCount();
  unsigned short task_list_index = current_task;
  short next_ready_task = -1;
  // Next we need to find which task has been waiting the longest
  unsigned int task_wait_ticks = 0;
  short task_index_with_longest_wait = -1;
  A2Task* task;


  while (count <= MAX_TASKS) {
    count += 1;
    task_list_index += 1;
    task_list_index = max(0, task_list_index % MAX_TASKS); // make sure it's at least zero
    task = &tasks[task_list_index];
    if (task->function == NULL) continue;
    if (task->enabled == false) continue;
    // Find the task that has waited the longest
    if (task->waitUntil != 0) {
      if (task->waitUntil > ticks) continue;
      if (task_index_with_longest_wait == -1 || task_wait_ticks > task->waitUntil) {
        task_index_with_longest_wait = task_list_index;
        task_wait_ticks = task->waitUntil;
#if TASKDEBUG
        Serial.print(task_list_index);
        Serial.print(" has waited ");
        Serial.print(ticks - task_wait_ticks);
        Serial.println();
#endif
      }
    }
    // Find the next available tasks
    if (next_ready_task == -1) {
#if TASKDEBUG
      Serial.print(task_list_index);
      Serial.print(" is ready");
      Serial.println();
#endif
      next_ready_task = task_list_index;
    }
  }
  if (task_index_with_longest_wait != -1) current_task = task_index_with_longest_wait;
  else current_task = next_ready_task;
#if TASKDEBUG
  Serial.print("Picked ");
  Serial.println(current_task);
#endif
  if (current_task == -1) {
    return NULL;
  }

  tasks[current_task].ticks ++;
  tasks[current_task].waitUntil = 0;
  return &tasks[current_task];
}

TickType_t GetTicksUntilNextReadyTask() {
  TickType_t next_ticks = 0;
  A2Task* task;
  TickType_t curr_ticks = xTaskGetTickCount();
  for (int i = 0; i < MAX_TASKS; i ++) {
    task = &tasks[i];
    if (task->function == NULL) continue;
    if (task->enabled == false) continue;
    if (task->waitUntil == 0) continue;
    if (task->waitUntil < curr_ticks) return 1;
    if (task->waitUntil < next_ticks || next_ticks == 0) next_ticks = task->waitUntil;
  }
  if (next_ticks == 0) return 0;
#if TASKDEBUG
  Serial.print("We found a task to wait for @");
  Serial.print(curr_ticks);
  Serial.print("->");
  Serial.print(next_ticks);
  Serial.println();
#endif
  return next_ticks - curr_ticks;
}
