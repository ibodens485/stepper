#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <stdint.h>

#define DIRECTION_CW 1
#define DIRECTION_CCW 0

class BetterStepper {
   private:
    int enPin;
    int dirPin;
    int stepPin;

    bool enabled = false;
    bool direction = DIRECTION_CW;

    int stepsPerRevolution = 200 * 16;
    const long minSpeed = 100;   // steps/sec
    const long tickMicros = 20;  // resolution (every 100µs)
    long maxSpeed = 3200 * 4;    // steps/sec

    long lastMicros = 0;
    long currentTime = 0;
    long stepError = 0;
    const long stepThreshold = 1000000;  // fixed point denominator (1e6 for µs)

    int accel = 5000;

    long stepsTaken = 0;
    long stepsToTake = 0;

    long position = 0;
    long targetPosition = 0;

    long stepsAccel = 0;
    long stepsCoast = 0;
    long speedLimit = 0;

    long timeAccel = 0;
    long timeCoast = 0;
    long timeTotal = 0;
    long timeDecel = 0;

    void setDirection(bool direction);

   public:
    BetterStepper(int enPin, int dirPin, int stepPin, int stepsPerRevolution);
    ~BetterStepper() = default;

    void enable();
    void disable();

    void setMaxSpeed(long maxSpeed);
    long getMaxSpeed();
    void setAcceleration(long acceleration);
    long getAcceleration();

    void setPosition(long position);
    long getPosition();
    long getSpeed();

    void move(long steps);
    void moveRotations(long rotations);
    void moveTo(long steps);
    void moveToRotations(long rotations);

    void stop();

    bool run();

    inline bool atTarget() {
        return stepsTaken >= stepsToTake;
    }

    void step();
};

#endif