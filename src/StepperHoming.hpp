#ifndef HOMING_HPP
#define HOMING_HPP

#include <stdint.h>
#include "BetterStepper.hpp"

// Homing sits on top of BetterStepper's public API (moveAt/stop/moveBy) rather than being
// part of the class itself - the class knows nothing about endstops or homing sequencing.
// These are blocking calls: they return once the motor(s) are homed. They assume a timer
// ISR is already calling motor.tick() at a regular interval in the background; these
// functions only poll state and issue moveAt()/moveBy() calls, they never call tick()
// themselves and never call stopImmediate() themselves either (see endstop wiring below).
//
// Endstop wiring: attach your own pin-change/external interrupt to each switch. Its ISR
// must call motor.stopImmediate() itself, then set a volatile bool you own, in that order:
//
//     void endstopISR_A() {
//         motorA.stopImmediate();
//         endStopFlagA = true;
//     }
//
// Calling stopImmediate() from the ISR (rather than leaving it to the polling loop below)
// matters: stopImmediate() is just one volatile write, so the motor stops within the same
// interrupt that detected the trigger. If instead the ISR only set the flag and the polling
// loop called stopImmediate() after observing it, the motor would keep stepping for however
// long it takes the main loop to get scheduled and notice - anything from a few tick()
// interrupts to a stalled overshoot, depending on what else is running. The flag itself is
// only used here to know when to move on to the next homing phase, not to stop the motor.
//
// Pass the flag in by reference. Keeping it outside BetterStepper means the class stays
// endstop-agnostic and these functions can poll two independent flags for the paired case.

// Parameters shared by both single- and paired-motor homing.
struct HomingParams {
    StepperDirection direction;    // direction to seek toward the switch
    uint32_t seekSpeedQ16;         // fast seek speed
    int32_t homePosition = 0;      // position value assigned once homing completes
    uint32_t backOffSteps = 0;     // steps to back off after the switch triggers;
                                    // 0 = no backoff/reapproach, finish on first trigger
    uint32_t reapproachSpeedQ16 = 0;  // slow speed for the repeatable second pass;
                                       // 0 defaults to seekSpeedQ16 / 4
};

// Home a single motor against its endstop. Blocks until complete.
//   motor        - the stepper to home
//   params       - homing parameters (see above)
//   endStopFlag  - volatile bool set by the endstop ISR (which must also call
//                  motor.stopImmediate() itself - see wiring note above); cleared by this
//                  function as it consumes each trigger
void homeMotor(BetterStepper& motor, const HomingParams& params, volatile bool& endStopFlag);

// Home two motors that share a linkage, starting their seek passes together so the two
// stay in sync throughout (rather than one finishing and stopping well before the other,
// which is what independent sequential homeMotor() calls would risk). Each motor still
// triggers, backs off, and reapproaches independently - if one reaches its switch first, it
// simply stops and waits while the other continues, then both back off/reapproach with the
// same start-together behavior. Blocks until both are complete.
void homeMotorPair(BetterStepper& motorA, const HomingParams& paramsA, volatile bool& endStopFlagA,
                    BetterStepper& motorB, const HomingParams& paramsB, volatile bool& endStopFlagB);

#endif