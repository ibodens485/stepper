#include "BetterStepper.h"

#include <Arduino.h>
#include "usqrt.h"

void BetterStepper::setDirection(StepperDirection direction) {
    this->direction = direction;
    digitalWrite(dirPin, direction ^ flipDirection);
}

void BetterStepper::step() {
    *port |= bitmask;
    *port &= ~bitmask;
}

BetterStepper::BetterStepper(int dirPin, int stepPin, bool flipDirection) {
    this->dirPin = dirPin;
    this->stepPin = stepPin;
    this->flipDirection = flipDirection;

    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);

    // precompute what digitalWrite would use, but do it now and only once so it's much faster when it counts
    bitmask = digitalPinToBitMask(stepPin);
    uint8_t portno = digitalPinToPort(stepPin);
    port = portOutputRegister(portno);

    setDirection(DIRECTION_FORWARD);
}

void BetterStepper::setMinSpeed(uint32_t minSpeedQ16) {
    this->minSpeedQ16 = minSpeedQ16;
}

uint32_t BetterStepper::getMinSpeed() {
    return minSpeedQ16;
}

void BetterStepper::setMaxSpeed(uint32_t maxSpeedQ16) {
    this->maxSpeedQ16 = maxSpeedQ16;
}

uint32_t BetterStepper::getMaxSpeed() {
    return maxSpeedQ16;
}

void BetterStepper::setAcceleration(uint32_t accelQ16) {
    this->accelRateQ16 = accelQ16;
}

uint32_t BetterStepper::getAcceleration() {
    return accelRateQ16;
}

void BetterStepper::setPosition(int32_t position) {
    this->position = position;
}

int32_t BetterStepper::getPosition() {
    return position;
}

void BetterStepper::startHoming(StepperDirection direction, uint32_t speedQ16, int32_t homePosition, uint32_t backOffSteps) {
    // TODO
}

void BetterStepper::endStopTriggered() {
    setPosition(endStopPosition);

    // TODO
}

// void BetterStepper::moveAt(int32_t speedQ16) {
//     StepperDirection wantDirection = (speedQ16 > 0) ? DIRECTION_FORWARD : DIRECTION_REVERSE;
//     if (wantDirection != direction) {
//         setDirection(direction);
//     }

//     if (speedQ16 < 0) {
//         speedQ16 = -speedQ16;
//     }

//     finite = false;
//     targetSpeedQ16 = speedQ16;

//     if (!running) {
//         currentSpeedQ16 = minSpeedQ16;
//         stepErrorQ16 = 0;
//         currentTick = 0;
//         running = true;
//     }
// }

void BetterStepper::moveBy(int32_t steps) {
    if (running) {
        return;
    }

    StepperDirection wantDirection = (steps > 0) ? DIRECTION_FORWARD : DIRECTION_REVERSE;
    if (wantDirection != direction) {
        setDirection(wantDirection);
    }

    if (steps < 0) {
        steps = -steps;
    }

    stepsToTake = steps;
    stepsTaken = 0;

    ticksAccel = (maxSpeedQ16 - minSpeedQ16) / accelRateQ16;

    uint32_t stepsAccel = (((int64_t)ticksAccel * (minSpeedQ16 + maxSpeedQ16)) >> 17);

    speedLimitQ16 = maxSpeedQ16;
    if (stepsAccel * 2 > stepsToTake) {
        stepsAccel = stepsToTake / 2;

        uint64_t v0sq_Q32 = (uint64_t)minSpeedQ16 * minSpeedQ16;
        uint64_t twoAS_Q32 = ((uint64_t)stepsToTake * accelRateQ16) << 16;
        speedLimitQ16 = usqrt_ll(v0sq_Q32 + twoAS_Q32);

        ticksAccel = (uint64_t)ticksAccel * (speedLimitQ16 - minSpeedQ16) / (maxSpeedQ16 - minSpeedQ16);
    }

    uint32_t stepsCoast = stepsToTake - 2 * stepsAccel;

    ticksCoast = ((int64_t)stepsCoast << 16) / maxSpeedQ16;
    ticksTotal = ticksCoast + 2 * ticksAccel;
    ticksDecel = ticksTotal - ticksAccel;

    stepErrorQ16 = 0;
    currentTick = 0;

    phase = PHASE_ACCEL;
    currentSpeedQ16 = minSpeedQ16;

    // finite = true;

    running = true;
}

void BetterStepper::moveTo(int32_t steps) {
    int32_t delta = steps - position;
    moveBy(delta);
}

// void BetterStepper::stop() {
//     if (!running) return;
//     finite = false;
//     targetSpeedQ16 = 0;
// }

void BetterStepper::stopImmediate() {
    if (!running) return;
    running = false;
}

void BetterStepper::waitUntilFinished() {
    while (running);
}

__attribute__((optimize("O2")))
void BetterStepper::tick() {
    if (!running)
        return;

    currentTick++;

    // if (finite) {
        switch (phase) {
            case PHASE_ACCEL:
                if (currentTick >= ticksAccel) {
                    phase = PHASE_COAST;
                    currentSpeedQ16 = speedLimitQ16;
                } else {
                    currentSpeedQ16 += accelRateQ16;
                }
                break;

            case PHASE_COAST:
                if (currentTick > ticksDecel) {
                    phase = PHASE_DECEL;
                    currentSpeedQ16 = speedLimitQ16 - accelRateQ16;
                }
                break;

            case PHASE_DECEL:
                if (currentTick > ticksTotal) {
                    phase = PHASE_DONE;
                    currentSpeedQ16 = minSpeedQ16;
                } else {
                    currentSpeedQ16 -= accelRateQ16;
                }
                break;

            case PHASE_DONE:
                currentSpeedQ16 = minSpeedQ16;
                break;
        }
    // } else {
    //     if (currentSpeedQ16 < targetSpeedQ16) {
    //         currentSpeedQ16 += accelRateQ16;
    //         if (currentSpeedQ16 > targetSpeedQ16) {
    //             currentSpeedQ16 = targetSpeedQ16;
    //         }
    //     } else if (currentSpeedQ16 > targetSpeedQ16) {
    //         if (currentSpeedQ16 - accelRateQ16 > minSpeedQ16) {
    //             currentSpeedQ16 = minSpeedQ16;
    //         } else {
    //             currentSpeedQ16 -= accelRateQ16;
    //         }
    //         if (currentSpeedQ16 < targetSpeedQ16) {
    //             currentSpeedQ16 = targetSpeedQ16;
    //         }
    //     }
    // }

    // step error will never store any non-fractional bits, so widen temporarily
    uint8_t stepsThisTime;
    {
        __uint24 newStepError = stepErrorQ16 + currentSpeedQ16;
        stepErrorQ16 = newStepError & 0xFFFF;
        stepsThisTime = newStepError >> 16;
    }

    stepsTaken += stepsThisTime;

    if (direction == DIRECTION_FORWARD) {
        position += stepsThisTime;
    } else {
        position -= stepsThisTime;
    }
    
    uint8_t mask = bitmask;

    for (uint8_t i = 0; i < stepsThisTime; i++) {
        *port |= mask;
        asm("nop");
        asm("nop");
        asm("nop");
        *port &= ~mask;
    }

    // if (finite) {
        if (stepsTaken == stepsToTake) {
            running = false;
            phase = PHASE_DONE;
        }
    // } else {
    //     if (currentSpeedQ16 == 0 && targetSpeedQ16 == 0) {
    //         running = false;
    //         phase = PHASE_DONE;
    //     }
    // }
}