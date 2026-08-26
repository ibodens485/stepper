#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <stdint.h>

enum StepperDirection : bool {
    DIRECTION_REVERSE = 0,
    DIRECTION_FORWARD = 1
};

enum StepperPhase : uint8_t {
    PHASE_ACCEL,
    PHASE_COAST,
    PHASE_DECEL,
    PHASE_DONE,
};

class BetterStepper {
   private:
    // pin numbers for direction and step
    int dirPin;
    int stepPin;
    
    // user parameters
    __uint24 minSpeedQ16 = 0;
    __uint24 maxSpeedQ16 = 1L << 16;
    __uint24 accelRateQ16 = 1;
    bool flipDirection = false;

    // step pin info for fast access
    volatile uint8_t* port;
    uint8_t bitmask;

    // currently running direction (as the dirPin state is)
    StepperDirection direction = DIRECTION_FORWARD;

    // current position, in steps
    __int24 position = 0;
    
    // bool finite = true;

    // finite computed parameters
    __uint24 ticksAccel;
    __uint24 ticksCoast;
    __uint24 ticksDecel;
    __uint24 ticksTotal;
    __uint24 stepsToTake;
    __uint24 speedLimitQ16;
    
    // infinite computed parameters
    __int24 targetSpeedQ16 = 0;
    
    // tick accumulators / state
    __uint24 currentSpeedQ16;
    uint16_t stepErrorQ16 = 0;
    
    // tick state (finite)
    __uint24 currentTick;
    __uint24 stepsTaken = 0;
    StepperPhase phase = PHASE_DONE;

    __int24 endStopPosition = 0;
    bool homed = false;
    bool homing = false;
    
    void setDirection(StepperDirection direction);
    
    // advance by a single step
    void step();
    
   public:
    volatile bool running = false;

    BetterStepper(int dirPin, int stepPin, bool flipDirection = false);
    ~BetterStepper() = default;

    void setMinSpeed(uint32_t minSpeedQ16);
    uint32_t getMinSpeed();
    void setMaxSpeed(uint32_t maxSpeedQ16);
    uint32_t getMaxSpeed();
    void setAcceleration(uint32_t accelQ16);
    uint32_t getAcceleration();

    void setPosition(int32_t position);
    int32_t getPosition();

    void startHoming(StepperDirection direction, uint32_t speedQ16, int32_t homePosition = 0, uint32_t backOffSteps = 0);
    void endStopTriggered();

    // Move by a specified signed number of steps
    void moveBy(int32_t steps);
    // Move to a specified step position
    void moveTo(int32_t steps);

    // stop immediately
    void stopImmediate();
    
    // Move at a specified speed forever, until a call to either stop() or stopImmediate()
    // void moveAt(int32_t speedQ16);
    // start decelerating to a stop
    // void stop();

    // busy wait
    void waitUntilFinished();

    // Periodic tick called at a regular interval
    void tick();
};

#endif