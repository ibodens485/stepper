#include "BetterStepper.h"

#include <Arduino.h>
#include "usqrt.h"

void BetterStepper::setDirection(bool direction) {
    this->direction = direction;
    digitalWrite(dirPin, direction);
}

BetterStepper::BetterStepper(int enPin, int dirPin, int stepPin, int stepsPerRevolution) {
}

void BetterStepper::enable() {
}

void BetterStepper::disable() {
}

void BetterStepper::setMaxSpeed(long maxSpeed) {
}

long BetterStepper::getMaxSpeed() {
    return 0;
}

void BetterStepper::setAcceleration(long acceleration) {
}

long BetterStepper::getAcceleration() {
    return 0;
}

void BetterStepper::setPosition(long position) {
}

long BetterStepper::getPosition() {
    return 0;
}

long BetterStepper::getSpeed() {
    return 0;
}

void BetterStepper::move(long steps) {
    targetPosition += steps;

    if ((steps > 0) != direction) {
        setDirection(!direction);
        steps = -steps;
    }

    stepsToTake = steps;
    stepsTaken = 0;

    timeAccel = (uint64_t)(maxSpeed - minSpeed) * 1000000 / accel;
    stepsAccel = (uint64_t)timeAccel * (minSpeed + maxSpeed) / 2000000;

    speedLimit = maxSpeed;
    if (stepsAccel * 2 > stepsToTake) {
        stepsAccel = stepsToTake / 2;

        speedLimit = usqrt_ll((uint64_t)minSpeed * minSpeed + (uint64_t)stepsToTake * accel);
        timeAccel = (uint64_t)timeAccel * (speedLimit - minSpeed) / (maxSpeed - minSpeed);
    }

    stepsCoast = stepsToTake - 2 * stepsAccel;

    timeCoast = 1000000LL * stepsCoast / maxSpeed;
    timeTotal = timeCoast + 2 * timeAccel;
    timeDecel = timeTotal - timeAccel;

    lastMicros = micros();
    stepError = 0;
    currentTime = 0;
}

void BetterStepper::moveRotations(long rotations) {
    move(((uint64_t)rotations * stepsPerRevolution) >> 16);
}

void BetterStepper::moveTo(long steps) {
}

void BetterStepper::moveToRotations(long rotations) {
}

void BetterStepper::stop() {
}

bool BetterStepper::run() {
    if (atTarget())
        return;

    // Check if long enough time has passed since last tick (remove for timer interrupts)
    long now = micros();
    if (now - lastMicros < tickMicros)
        return false;

    long delta = now - lastMicros;
    lastMicros = now;
    currentTime += delta;

    long currentSpeed;
    if (currentTime < timeAccel) {
        // accel
        currentSpeed = minSpeed + (long)((uint64_t)accel * currentTime / 1000000L);
    } else if (currentTime <= timeDecel) {
        // coast
        currentSpeed = speedLimit;
    } else if (currentTime <= timeTotal) {
        // decel
        currentSpeed = minSpeed + (long)((uint64_t)accel * (timeTotal - currentTime) / 1000000L);
    } else {
        // overshoot
        currentSpeed = minSpeed;
    }

    // Accumulate
    stepError += currentSpeed * delta;

    // Step when necessary
    if (stepError >= stepThreshold) {
        step();
        stepError -= stepThreshold;
        stepsTaken++;
        position++;
    }

    return true;
}

void BetterStepper::step() {
}