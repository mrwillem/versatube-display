#ifndef BUTTONS_H
#define BUTTONS_H

#include "hw_config.h"

extern volatile uint16_t ADCBuffer[3];
extern volatile uint8_t buttonnum;
void pushbuttoninit(void);
void update_buttons(void);

#endif //BUTTONS_H
