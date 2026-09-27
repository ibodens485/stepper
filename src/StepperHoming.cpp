#include "StepperHoming.hpp"

namespace {

// Resolve reapproachSpeedQ16 == 0 into seekSpeedQ16 / 4, floored at the motor's minSpeed.
uint32_t resolveReapproachSpeed(const HomingParams& params, BetterStepper& motor) {
    uint32_t speed = (params.reapproachSpeedQ16 != 0) ? params.reapproachSpeedQ16 : (params.seekSpeedQ16 / 4);
    uint32_t minSpeed = motor.getMinSpeed();
    return (speed < minSpeed) ? minSpeed : speed;
}

int32_t signedSpeed(StepperDirection dir, uint32_t speedQ16) {
    return (dir == DIRECTION_FORWARD) ? (int32_t)speedQ16 : -(int32_t)speedQ16;
}

int32_t signedSteps(StepperDirection awayFromDir, uint32_t steps) {
    // "away from" the seek direction: back off in the opposite direction of the seek
    return (awayFromDir == DIRECTION_FORWARD) ? -(int32_t)steps : (int32_t)steps;
}

}  // namespace

void homeMotor(BetterStepper& motor, const HomingParams& params, volatile bool& endStopFlag) {
    uint32_t reapproachSpeedQ16 = resolveReapproachSpeed(params, motor);

    // --- seek ---
    endStopFlag = false;
    motor.moveAt(signedSpeed(params.direction, params.seekSpeedQ16));
    while (!endStopFlag);
    endStopFlag = false;

    if (params.backOffSteps == 0) {
        motor.setPosition(params.homePosition);
        return;
    }

    // --- back off ---
    motor.moveBy(signedSteps(params.direction, params.backOffSteps));
    motor.waitUntilFinished();

    // --- reapproach (slow, for a repeatable trigger point) ---
    motor.moveAt(signedSpeed(params.direction, reapproachSpeedQ16));
    while (!endStopFlag);
    endStopFlag = false;

    motor.setPosition(params.homePosition);
}

void homeMotorPair(BetterStepper& motorA, const HomingParams& paramsA, volatile bool& endStopFlagA,
                    BetterStepper& motorB, const HomingParams& paramsB, volatile bool& endStopFlagB) {
    uint32_t reapproachA = resolveReapproachSpeed(paramsA, motorA);
    uint32_t reapproachB = resolveReapproachSpeed(paramsB, motorB);

    // --- seek: start together, let each stop independently as it triggers (via its own
    // ISR calling stopImmediate() directly - see header), keep the other one running
    // until it triggers too ---
    endStopFlagA = false;
    endStopFlagB = false;
    motorA.moveAt(signedSpeed(paramsA.direction, paramsA.seekSpeedQ16));
    motorB.moveAt(signedSpeed(paramsB.direction, paramsB.seekSpeedQ16));

    while (!(endStopFlagA && endStopFlagB));
    endStopFlagA = false;
    endStopFlagB = false;

    bool backOffA = paramsA.backOffSteps != 0;
    bool backOffB = paramsB.backOffSteps != 0;

    if (!backOffA && !backOffB) {
        motorA.setPosition(paramsA.homePosition);
        motorB.setPosition(paramsB.homePosition);
        return;
    }

    // --- back off: start together, wait for both finite moves to finish ---
    // (a motor with backOffSteps == 0 simply doesn't move during this phase and is
    // already at its final home position once the loop below exits for the other one)
    if (backOffA) motorA.moveBy(signedSteps(paramsA.direction, paramsA.backOffSteps));
    if (backOffB) motorB.moveBy(signedSteps(paramsB.direction, paramsB.backOffSteps));
    while ((backOffA && motorA.running) || (backOffB && motorB.running));

    if (!backOffA) motorA.setPosition(paramsA.homePosition);
    if (!backOffB) motorB.setPosition(paramsB.homePosition);
    if (!backOffA && !backOffB) return;

    // --- reapproach: same start-together / independent-stop pattern as seek ---
    if (backOffA) motorA.moveAt(signedSpeed(paramsA.direction, reapproachA));
    if (backOffB) motorB.moveAt(signedSpeed(paramsB.direction, reapproachB));

    while ((backOffA && !endStopFlagA) || (backOffB && !endStopFlagB));
    endStopFlagA = false;
    endStopFlagB = false;

    if (backOffA) motorA.setPosition(paramsA.homePosition);
    if (backOffB) motorB.setPosition(paramsB.homePosition);
}