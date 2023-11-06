#ifndef __DEMO_H__
#define __DEMO_H__

#include "stm32f4xx_hal.h"
typedef enum {
    DEMO_INIT,
    DEMO_FWD,
    DEMO_BWD,
    DEMO_LEFT,
    DEMO_RIGHT,
    DEMO_DIST_SENSE,
    DEMO_LINE_SENSE,
    DEMO_END
} DemoStates_E;

typedef struct DemoState_T {
    DemoStates_E state;
    void (*demoFunc)();
} DemoState_T;

extern uint8_t isSDemoStarted;
DemoStates_E nextDemoState();
DemoStates_E getDemoState();
void demoStateMachine();

void demoInit();
void demoEnd();
void demoFwd();
void demoBwd();
void demoLeft();
void demoRight();
void demoDistSense();
void demoLineSense();
#endif  // __DEMO_H__
