#include "Stepper.hpp"

#include <Arduino.h>
#include "usqrt.h"

void Stepper::setDirection(bool direction) {
    this->direction = direction;
    digitalWrite(dirPin, direction);
}

Stepper::Stepper(int enPin, int dirPin, int stepPin, int stepsPerRevolution) {
}

Stepper::~Stepper() {
}

void Stepper::enable() {
}

void Stepper::disable() {
}

void Stepper::setMaxSpeed(long maxSpeed) {
}

long Stepper::getMaxSpeed() {
    return 0;
}

void Stepper::setAcceleration(long acceleration) {
}

long Stepper::getAcceleration() {
    return 0;
}

void Stepper::setPosition(long position) {
}

long Stepper::getPosition() {
    return 0;
}

long Stepper::getSpeed() {
    return 0;
}

void Stepper::move(long steps) {
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

        speedLimit = usqrt((uint64_t)minSpeed * minSpeed + (uint64_t)stepsToTake * accel);
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

void Stepper::moveRotations(long rotations) {
    move(((uint64_t)rotations * stepsPerRevolution) >> 16);
}

void Stepper::moveTo(long steps) {
}

void Stepper::moveToRotations(long rotations) {
}

void Stepper::stop() {
}

bool Stepper::run() {
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

void Stepper::step() {
}