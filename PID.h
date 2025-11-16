// SPDX-FileCopyrightText: 2025 David Chirme Sisa ([dff-laoise](https://github.com/bulb-light)) 
// SPDX-License-Identifier: MIT

/**
 * @file PID.h
 * @brief Discrete PID controller class for Arduino/embedded systems.
 * @author David Chirme
 * @version 1.0
 * @date 2025
 */
#ifndef __PID_H__
#define __PID_H__

// #include <Arduino.h>

/**
 * @class PID
 * @brief Implements a discrete PID controller using trapezoidal approximation
 * for integral and backward‐difference for derivative.
 * 
 * This class provides a flexible and efficient implementation of a discrete PID controller
 * for use in control systems. It supports dynamic reconfiguration of parameters,
 * anti-windup mechanisms, and output clamping.
 * 
 * Example usage:
 * @code
 * PID myPID(Kp, Ti, Td, sampleTimeSec, minOuput, maxOutput);
 * float output = myPID.computePIDOut(error);
 * @endcode
 * @details Reference: "Discrete PID controller approximation" by David Chirme (bulb-light):
 *          https://github.com/bulb-light/ArduinoProjects_dff/blob/main/DiscreteTimePID.md
 *          Note: This implementation assumes that computePIDOut() is called
 *          at a fixed interval defined by mSampleTimeSec.
 */
class PID {
    public:
        /**
         * @brief Construct a new PID controller.
         * @param Kp Proportional gain (continuous).
         * @param Ti Integral time constant in seconds (continuous, must be != 0).
         *        If Ti == 0, it will be set to a small value (0.0001) to avoid division by zero.
         * @param Td Derivative time constant in seconds (continuous).
         * @param sampleTimeSec Sampling time in seconds (must be > 0).
         *        If value <= 0, defaults to 0.01s.
         * @param minOutput Minimum output limit.
         * @param maxOutput Maximum output limit.
         *        If minOutput >= maxOutput, defaults to (0.0, 255.0)
         * 
         * @note The constructor initializes the PID controller and validates the input parameters.
         * If invalid values are provided, defaults are used.
         */
        PID(float Kp, float Ti, float Td, float sampleTimeSec, float minOutput, float maxOutput);

        ~PID();

        /**
         * @brieft Update the sampling time for the PID controller.
         * @param sampleTimeSec Sampling time in seconds (must be > 0).
         *        If value <= 0, defaults to 0.01s.
         * 
         * This method updates the sampling time dynamically and recalculates
         * the PID coefficients to reflect the new sampling interval.
         */
        void setSampleTime(float sampleTimeSec);

        /**
         * @brief Update the PID controller parameters.
         * @param Kp Proportional gain (continuous).
         * @param Ti Integral time constant in seconds (continuous, must be != 0).
         *        If Ti == 0, it will be set to a small value (0.0001) to avoid division by zero.
         * @param Td Derivative time constant in seconds (continuous).
         * 
         * This method updates the PID parameters dynamically and recalculates
         * the PID coefficients accordingly.
         */
        void setPIDParams(float Kp, float Ti, float Td);

        /**
         * @brief Update the ouput limits.
         * @param minOutput Minimum output limit.
         * @param maxOutput Maximum output limit.
         *        If minOutput >= maxOutput, defaults to (0.0, 255.0)
         * 
         * This method sets new output limits dynamically. The output of the
         * PID controller will be clamped to these limits during computation.
         */
        void setOutputLimits(float minOutput, float maxOutput);

        /**
         * @brief Compute the discrete PID controller output based on the current error.
         * @param error The current error value (setpoint - measurement).
         * @return The computed PID controller output.
         * 
         * This method calculates the discrete PID output using the discrete PID formula.
         * It applies output clamping and anti-windup mechanisms as needed.
         * @note This method should be called at a fixed interval defined by the sampliing time.
         */
        float computePIDOut(float error);

        // inline float getIntegralAccumulator() const { return this->integralAccumulator; }

        /**
         * @brief Reset the internal states of the PID controller.
         * 
         * This method clears the internal error history and output states, effectively resetting
         * the PID controller to its initial state.
         */
        void resetStates();

    private:
        // Member variables
        float mKp; // proportional gain
        float mTi; // integral time constant
        float mTd; // derivative time constant

        float mSampleTimeSec; // sampling time in seconds

        float mMinOutput; // minimum output limit
        float mMaxOutput;  // maximum output limit

        bool mWasSaturated; // flag to indicate if last output was saturated

        // Precomputed discrete PID coeffcients
        float Kpd, Kid, Kdd; // discrete equivalents

        // Internal state for discrete implementation
        float lastError1;      // error at k-1
        // float lastError2;      // error at k-2
        float lastOutput1;      // output at k-1
        float integralAccumulator; // integral accumulator

        /** 
         * @brief Update PID coefficients based on current parameters.
         * Kpd, Kid, Kdd are updated accordingly (discrete equivalents).
         */
        void updatePIDCoefficients();
};

#endif