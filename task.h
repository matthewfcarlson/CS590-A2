#ifndef TASK_H
#define TASK_H
#include "esp_task_wdt.h"
struct _A2Task {
  char* task_name; // task name
  TaskFunction_t function; // the function to call
  bool enabled; // if we're turned on
  TickType_t waitUntil; // if we're waiting on something, 0 if we're not
  TickType_t ticks; // the number of ticks this task has received
};
#define MAX_TASKS 10
typedef struct _A2Task A2Task;

// ------------------------
// Functions
// ------------------------
// Setups the tasks
void SetupTasks();
// Creates a task
void CreateTask(char* name, TaskFunction_t function);
// RunNextReadyTask executes the next ready task from the list, return false if no tasks are ready
A2Task* GetNextReadyTaskFromList();
// Specifies the number of ticks to sleep until
void SleepFor(unsigned int ticks);
void DisableTask();
void EnableTask();
TickType_t GetTicksUntilNextReadyTask();
void PrintTask(A2Task* task);
#endif
