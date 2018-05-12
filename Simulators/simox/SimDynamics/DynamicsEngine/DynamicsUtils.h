#ifndef __DYNAMICS_UTILS__H__
#define __DYNAMICS_UTILS__H__

#include <iostream>
#include "../SimDynamics.h"

namespace SimDynamics
{

    class SIMDYNAMICS_IMPORT_EXPORT PIDController
    {
    public:
        PIDController(double gainP, double gainI, double gainD);

        double update(double error, double dt);

        void reset();
        void reset(double gainP, double gainI, double gainD);

        void debug();
        void getPID(double& storeP, double& storeI, double& storeD);

    private:
        double gainP;
        double gainI;
        double gainD;
        double errorSum;
        double lastError;
        double lastOutput;
    };

    // use bit field because enums are a pain
    union SIMDYNAMICS_IMPORT_EXPORT ActuationMode
    {
        struct
        {
            unsigned char position: 1;
            unsigned char velocity: 1;
            unsigned char torque: 1;
        } modes;
        unsigned char mode;
    };

    /**
     * For *torque* based motors.
     *
     * Position only:
     * position error --> [PID] --> joint
     *
     * Velocity only:
     * velocity error --> [PID] --> joint
     *
     * Torque only:
     * torque error --> [PID] --> joint
     *
     * Position + Velocity:
     *                       velocity error
     *                             |
     *                             v
     * position error -> [PID] -> (+) -> [PID] -> joint
     *
     * Position + Velocity + Torque:
     *                       velocity error  torque error
     *                             |               |
     *                             v               v
     * position error -> [PID] -> (+) -> [PID] -> (+) -> [PID] -> joint
     *
     */
    class SIMDYNAMICS_IMPORT_EXPORT TorqueMotorController
    {
    public:
        TorqueMotorController();
        TorqueMotorController(const PIDController& positionController,
                              const PIDController& velocityController,
                              const PIDController& torqueController);

        double update(double positionError, double velocityError, double torqueError, ActuationMode actuation, double dt);

    private:
        PIDController positionController;
        PIDController velocityController;
        PIDController torqueController;
    };

    /**
     * For *velocity* based motors (where target velocity == actual velocity).
     * We use this for Bullet motors.
     *
     * Note: Torque is ignored. This controler returns *velocities*.
     *
     *
     * Position only:
     * position error --> [PID] --> joint
     *
     * Velocity only:
     * velocity error -> [PID] --> joint
     *
     * Position + Velocity:
     *                       target velocity
     *                             |
     *                             v
     * position error -> [PID] -> (+) -> joint
     *
     */
    class SIMDYNAMICS_IMPORT_EXPORT VelocityMotorController
    {
    public:
        VelocityMotorController(double maxVelocity = -1.0, double maxAcceleration = -1.0, double maxJerk = -1);

        VelocityMotorController(const PIDController& positionController);

        void setCurrentVelocity(double vel);

        double update(double positionError, double targetVelocity, ActuationMode actuation, double dt);

        void reset();
        //! set new p,i,d values for position controller
        void reset(double pid_pos_gainP, double pid_pos_gainI, double pid_pos_gainD);

        void debug();

        void getPosPID(double& storeP, double& storeI, double& storeD);
    private:
        PIDController positionController;
        double maxVelocity;
        double maxAcceleration;
        double maxJerk;
        double velocity;
        double acceleration;
    };

}


#endif
