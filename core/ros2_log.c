#include "ros2_log.h"

#include <stdlib.h>
#include <stdio.h>

#include "xtimer.h"

#define MAX_LOG 1024

typedef struct {
  uint8_t evt;
  uint32_t time;
} log_elem_t;

static log_elem_t _log[MAX_LOG] = {0};
static size_t _counter = 0;

void ros2_log_add_event(ros2_log_event_t evt) {
  if(_counter < MAX_LOG) {
    _log[_counter].evt = (uint8_t)evt;
    _log[_counter].time = xtimer_now_usec();
    _counter++;
  }
}

void ros2_log_print(void) {
  printf("---- BEGIN LOG ----\n");

#define MACRO(str) \
  case str: printf(#str); break;

  for(size_t i = 0 ; i < _counter ; i++) {
    printf("%lu;", (long unsigned int)_log[i].time);

    switch((ros2_log_event_t)_log[i].evt) {

#include "ros2_log_defs.h"

      default:
        printf("%d", (int)_log[i].evt);
    }

    printf("\n");
  }

#undef MACRO

  printf("---- %lu / %lu ----\n", (long unsigned int)_counter, (long unsigned int)MAX_LOG);
  printf("---- END LOG ----\n");
}

void ros2_log_reset(void) {
  _counter = 0;
}
