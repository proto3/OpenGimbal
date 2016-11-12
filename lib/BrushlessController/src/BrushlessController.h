#ifndef _BRUSHLESS_CONTROLLER_H_
#define _BRUSHLESS_CONTROLLER_H_

#include "Arduino.h"

class BrushlessController {
public:
    BrushlessController();
    void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t maxPWM);
private:
    int8_t pwmSinMotor[256];
};

#endif
